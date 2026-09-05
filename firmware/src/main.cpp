/*
 * ESP32-S3 + OV2640 PlatformIO 工程
 *
 * 目标：从源码编译 esp32-camera，解锁 JPEG 模式（Arduino 预编译库在 JPEG 下返空帧 FB-OVF，
 * 源码编译可绕过）。JPEG 直播 + UDP 高帧率 / 高分辨率，拍照走可靠 TCP。
 *
 * 架构（v3, 双任务）：
 *   loop()/loopTask(核1): TCP accept、命令解析、抓帧、摄像头生命周期(deinit/init/切分辨率)
 *   senderTask(核0):      从 PSRAM 拷贝队列取帧发送（UDP 分片重试/TCP 整帧）
 *   抓帧侧只做 fb->PSRAM 拷贝即归还缓冲，TCP 慢客户端或 UDP 重试不再拖慢采集流水线。
 *   cam_client 由互斥锁保护；所有摄像头重建/切换前 wait_tx_idle() 排空队列防 UAF。
 *
 * 接线（OV2640 -> ESP32-S3）：
 *   3.3V-3V3  GND-GND  SCL-5  SDA-4  VSYNC-6  HREF-7  PCLK-13  XCLK-15
 *   D7-16 D6-17 D5-18 D4-2  D3-10 D2-8  D1-9   D0-1
 *   PWDN -> GND（拉低使能）, RST 悬空或接 3.3V
 *   （本板 GPIO11/12 损坏，D0 已改接 GPIO1，D4 改接 GPIO2）
 *
 * 编译/烧录：pio run && pio run -t upload（COM7）
 */
#include "globals.h"        // 常量 + 跨模块共享全局(见 globals.h 拆分说明)
#include "diag.h"           // 诊断统计 / 自适应画质
#include "discovery.h"      // 局域网广播发现
#include "frame_pool.h"     // PSRAM 帧内存池
#include "camera.h"         // 相机管理
#include "frame_io.h"       // 帧收发(OVU/TCP/串口)
#include "web_cfg.h"        // Web 配置页 + NVS
#include "wifi_sta.h"       // WiFi STA 接入
#include "led.h"            // 板载 RGB 状态灯
#include "ai_detect.h"      // 人体检测(Person Detection)

// ---- 函数原型（.ino 由 Arduino 自动生成，.cpp 需手动声明）----
void handle_cmd(int c, Stream &in);
void enqueue_frame(camera_fb_t *fb);
void wait_tx_idle(uint32_t timeout_ms);
void sender_task(void *arg);
void relay_task(void *arg);
bool send_frame_relay(camera_fb_t *fb);
bool send_frame_udp_via_turn(camera_fb_t *fb);
void p2p_punch();
void ice_poll();
void turn_keepalive();
static void p2p_socks_init();
static void p2p_reset();

// UART0 = CH343 转串口所在通道（TX=GPIO43, RX=GPIO44），避免默认 Serial 走 USB-CDC。
HardwareSerial Dbg(0);

// ---- WiFi 传输（纯 STA 连入局域网 + TCP 服务端；SoftAP 仅在无凭据时兜底配网）----
#ifdef ENABLE_WIFI   // 凭据/secrets/引脚宏/PIX_FORMAT 定义见 globals.h
WiFiServer  cam_server(TCP_PORT);
WiFiClient  cam_client;
// accept 抢不到 g_cli_mutex 时暂存的新连接: 绝不能让局部变量析构(那会关掉已完成
// 握手的连接, 而上位机侧显示"已连"并已下发 'U', 固件却无 cam_client -> 永久无流)。
static WiFiClient g_pending_client;
WiFiUDP     cam_udp;            // UDP 预览通道(高帧率, 丢帧不重传)   [globals.h extern]
bool        udp_mode  = false;  // 上位机发 'U' 开启                  [globals.h extern]
uint16_t    udp_seq   = 0;      // [globals.h extern]
bool        g_auto_quality = true;   // [globals.h extern]
uint8_t     g_udp_backoff_frames = 0;   // [globals.h extern]
volatile bool g_cam_ready = false;   // esp_camera_init 成功置 true [globals.h extern]
// 诊断/自适应画质统计(g_diag_*/g_adapt_*)在 diag.cpp, 声明见 globals.h

// ---- 局域网自动发现 + 跨网中继客户端 ----
char            g_dev_name[32]   = AP_SSID;  // 广播/注册用设备名 [globals.h extern]
char            g_dev_id[40]     = "";       // 注册 id(默认 cam-<MAC后4字节>) [extern]
char            g_relay_host[80] = "";       // 中继服务端 "host:port", 空=关闭 [extern]
char            g_relay_token[40]= "";       // 中继鉴权口令(reg/watch 必须携带) [extern]
static WiFiClient   relay_cli;
static SemaphoreHandle_t g_relay_tx  = NULL;     // 保护 relay 发送(信令与帧交错用)
volatile bool    g_relay_viewer  = false;    // 有观众经中继在观看 [globals.h extern]
volatile bool    g_relay_online  = false;    // 已连上中继服务端 [globals.h extern]
volatile uint16_t  g_relay_stall_ms = 0;        // 中继 TCP 写阻塞滚动量(ms), 0=无阻塞 [globals.h extern]

// ---- 可靠 P2P 直连: 多候选交叉打洞 + 双向保活 + 三层降级闭环 ----
// 可靠性设计(对照旧版"单候选单端口, 3s 失败永久放弃"的缺陷):
//   1. 本端 3 个候选 socket(主口 cam_udp + 2 个随机口)全部 STUN 上报, 对端同理;
//      打洞交叉探测 3xN 组合 —— 端口受限/对称 NAT 下单候选几乎必败。
//   2. 打洞/保活包带 magic, 回包校验 IP+端口都在候选表内才认定成功(防误判)。
//   3. 直连后双向 500ms keepalive, 双向维持 NAT 映射(旧版只有观众单向发)。
//   4. 降级链闭环: 直连失活->TURN; TURN 失活->TCP 中继; 期间周期重试打洞恢复直连。
//      旧版 TURN 优先级高于 TCP 且失败不可检测, 观众退出后会一直往死端点发帧。
// P2P 常量(P2P_NSOCK/P2P_NPEER_MAX/P2P_MAGIC/...) 定义见 globals.h
static WiFiUDP*      g_p2p_udp[P2P_NSOCK] = {NULL, NULL, NULL};  // [0]=&cam_udp
static IPAddress     g_p2p_peer_ipN[P2P_NPEER_MAX];   // 对端候选表(打洞目标)
static uint16_t      g_p2p_peer_portN[P2P_NPEER_MAX] = {0};
static int           g_p2p_npeer = 0;            // 有效对端候选数
static volatile int8_t g_p2p_lock_sock = -1;     // 打洞成功锁定的本端 socket 索引
static IPAddress     g_p2p_lock_ip;              // 锁定的对端端点(视频发往此处)
static uint16_t      g_p2p_lock_port = 0;
volatile bool        g_p2p_active = false;       // P2P 直连已建立 [globals.h extern]
static uint32_t      g_p2p_punch_ms = 0;         // 上次打洞/保活发包时间
static uint32_t      g_p2p_punch_start = 0;      // 本轮打洞开始时间
static uint32_t      g_p2p_next_retry = 0;       // 下次重试打洞时刻(降级后周期恢复)
static uint32_t      g_p2p_last_rx = 0;          // 最近收到对端包的时刻(存活判定)
volatile bool        g_turn_mode    = false;     // TURN 兜底 [globals.h extern]
static uint32_t      g_turn_last_stun = 0;       // 上次 STUN 保活刷新时间
static bool          g_p2p_socks_ready = false;  // 候选 socket 是否已创建

// 中继链路收到的一整条命令数据; 用轻量 Stream 喂给 handle_cmd
class ByteStream : public Stream {
  public:
    const uint8_t *p; size_t n, i;
    ByteStream(const uint8_t *_p, size_t _n) : p(_p), n(_n), i(0) {}
    int available()                       override { return (int)(n - i); }
    int read()                            override { return i < n ? (unsigned char)p[i++] : -1; }
    int peek()                            override { return i < n ? (unsigned char)p[i] : -1; }
    size_t write(uint8_t)                 override { return 0; }
    size_t write(const uint8_t *, size_t) override { return 0; }
    void flush()                          override {}
};

// ---- 双任务：跨任务共享句柄与握手标志 ----
struct TxFrame {
  uint8_t *buf;        // 帧数据(帧池槽位 或 PSRAM 堆分配), 发送侧释放
  size_t   len;
  int      fmt;        // pixformat_t 原值(JPEG/RGB)
  uint16_t w, h;
  int8_t   slot;       // >=0: 来自帧池, 归还时放回空闲队列; -1: heap_caps_free
};
// 最新帧快照(替代 FIFO 队列): 采集与抽帧/发送解耦后不再排队积压。
// 双槽乒乓: 生产侧(核1 采集)把最新帧写入空闲槽并标 ready; 发送侧(核0)每轮
// 拉取最新 ready 槽。发送侧正读的槽被占用时生产侧直接跳过 memcpy(抽帧节拍在
// 发送侧), 不为注定被丢的帧浪费时间; 发送侧永远发最新帧, 杜绝旧帧滞后。
#define SNAP_SLOTS    2
static TxFrame        g_snap[SNAP_SLOTS];
static SemaphoreHandle_t g_snap_mutex = NULL;   // 保护 ready/inuse 标志切换(跨核)
static SemaphoreHandle_t g_new_frame  = NULL;   // 发布信号: 生产侧发布新帧时 give
static volatile uint8_t g_snap_ready[SNAP_SLOTS] = {0, 0};  // 槽内是最新未发帧
static volatile uint8_t g_snap_inuse[SNAP_SLOTS] = {0, 0};  // 发送侧正读该槽
static volatile uint32_t g_pub_seq   = 0;    // 单调发布序号(全局)
static volatile uint32_t g_slot_seq[SNAP_SLOTS] = {0, 0};  // 每槽最近一次发布序号
// 保护 cam_udp: sender_task(核0 优先级3) 发视频分片、relay_task(核0 优先级1) 打洞/STUN/
// 排空都会操作同一个 WiFiUDP 对象。lwIP socket 与 WiFiUDP 内部状态均非线程安全,
// 且两者同核, 高优先级 sender 可在 relay 执行 beginPacket 期间抢占 -> 状态错乱。
// TURN 提前开启后并发窗口显著变大, 必须串行化。
SemaphoreHandle_t    g_udp_mutex = NULL;   // [globals.h extern]

// ---- 帧内存池：消除每帧 PSRAM malloc/free 的抖动与碎片 ----
// frame_pool_init/frame_alloc/frame_free 已拆到 frame_pool.cpp
SemaphoreHandle_t    g_cli_mutex = NULL;   // 保护 cam_client 的 accept/swap/stop/send [globals.h extern]
static volatile int  g_inflight   = 0;     // 发送方正持有的帧数(已出队未释放)
volatile bool        g_switching  = false; // 摄像头重建窗口, 生产侧暂停入队 [globals.h extern]
volatile uint32_t   g_loop_tick  = 0;     // loop 心跳: sender_task 监控, 卡死则 ESP.restart 自愈

// ---- 人体检测调度(阶段2+3) ----
static uint32_t g_ai_last_infer_ms = 0;    // 上次推理时刻(节流)
static uint8_t  g_ai_confirm_cnt   = 0;    // 连续判定"有人"的帧数(防抖)
static volatile bool g_ai_trigger_photo = false;   // 推理命中, 待下轮 loop 不持锁时触发拍照
static uint32_t g_ai_last_trigger_ms = 0;  // 上次触发拍照时刻(冷却)
static uint32_t g_ai_cooldown_ms    = 10000;  // 触发冷却: 两次拍照/报警最小间隔(默认 10s)
static bool     g_ai_relay_alert    = true;    // 命中后经中继发报警 JSON(开关)
static uint32_t g_ai_hit_total      = 0;    // 累计命中次数(状态查询用)
static uint32_t g_ai_infer_interval_ms = 1000;  // 推理节流间隔(默认 1s 一次)
static uint32_t g_ai_last_infer_us   = 0;   // 最近一次推理耗时(us, 状态查询用)
static uint32_t g_ai_infer_total     = 0;   // 累计推理次数

// 发送成功统一记账（DIAG 周期窗口 + 自适应累计两套都递增）
#endif

// ---- 引脚配置（esp32-camera 官方 ESP32-S3 示例，含损坏脚位移位）----
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    15
#define SIOD_GPIO_NUM    4    // SDA
#define SIOC_GPIO_NUM    5    // SCL
#define D7_GPIO_NUM      16
#define D6_GPIO_NUM      17
#define D5_GPIO_NUM      18
#define D4_GPIO_NUM      2    // 原12损坏 -> GPIO2
#define D3_GPIO_NUM      10
#define D2_GPIO_NUM      8
#define D1_GPIO_NUM      9
#define D0_GPIO_NUM      1    // 原11损坏 -> GPIO1
#define VSYNC_GPIO_NUM   6
#define HREF_GPIO_NUM    7
#define PCLK_GPIO_NUM    13

// ---- UART0 调试口（CH343 TX/RX）----
#define TX_GPIO_NUM    43
#define RX_GPIO_NUM    44

// ---- 运行时可切换的分辨率 [均 globals.h extern] ----
framesize_t PREVIEW_FRAME = FRAMESIZE_QVGA;   // 预览 320x240（30fps，OV2640 CIF 模式甜点）
framesize_t HI_FRAME      = FRAMESIZE_UXGA;   // 拍照 1600x1200
framesize_t cur_frame     = FRAMESIZE_QVGA;
int         g_quality     = 16;   // JPEG 质量(0-63, 值越大压缩越强文件越小).
// 帧率优先策略: 默认升至16在 QVGA 下每帧量约减半, 跨网带宽受限时帧率近翻倍, QVGA画质损失轻微。
// 注意: QVGA(CIF模式)下 quality 太低(如2)会卡死无帧(传感器编码bug), 16 在此安全区间内且比10更抗带宽。
//       VGA(SVGA模式)下可运行时 'q2' 提画质(实测 15fps + 55KB/帧) 不损失帧率.

// g_config(摄像头配置)定义在 camera.cpp; 帧头/帧尾(FRAME_BEGIN/END)定义在 frame_io.cpp;
// BYTES_PER_PIXEL/MAX_FB_BYTES 见 globals.h

// 帧收发(TCP/串口/OVU 分片)已拆到 frame_io.cpp; fb_count_for/framesize_from_code 在 camera.cpp

// 中继 JSON 下行命令 {"t":"cmd","res":"Vx"} / {"t":"cmd","res":"Hx"} 解析。
// 返回命令首字母('V'/'H', 兼容小写), code 回填分辨率码; 格式非法返回 0。
// 兼容 json.dumps 默认格式("res": "Vx" 冒号后可能有空格)。JSON 小消息在中继链路走
// 阻塞入队(reliable), 相比原始单字节透传不丢帧、不漂移。
static char relay_cmd_lead(const char *s, char &code) {
  if (!s) return 0;
  const char *r = strstr(s, "\"res\"");
  if (!r) return 0;
  r += 5;
  while (*r && *r != ':') r++;                 // 跳过键名到冒号
  if (*r != ':') return 0;
  r++;
  while (*r && (*r == ' ' || *r == '\t')) r++; // 冒号后可能有空格
  if (*r != '"') return 0;
  r++;
  if (!r[0] || !r[1]) return 0;
  char lead = r[0];
  if (lead != 'V' && lead != 'v' && lead != 'H' && lead != 'h') return 0;
  code = r[1];
  return lead;
}

// apply_color_calib / cam_lock / cam_unlock 已拆到 camera.cpp
// g_cam_mutex: 相机互斥锁对象, 由 setup 创建, 声明见 globals.h
SemaphoreHandle_t g_cam_mutex = NULL;
#ifdef ENABLE_WIFI
// 等待发送侧不再持有任何帧且在途帧归零。必须在任何 esp_camera_deinit/改分辨率前调用：
// 快照里存的是已从相机驱动拷出的 PSRAM 内存，切换本身安全，
// 但若在途帧还被 sender 引用就 deinit 会改变后续 fb 语义 —— 排空保证无竞争窗口。
void wait_tx_idle(uint32_t timeout_ms) {
  uint32_t t0 = millis();
  for (;;) {
    bool busy = (g_inflight > 0);
    if (!busy && g_snap_mutex) {
      // 尚未被发送侧取走的 ready 帧也属未排空; inuse 槽(生产写中)同样占位。
      for (int i = 0; i < SNAP_SLOTS; i++) {
        if (g_snap_ready[i] || g_snap_inuse[i]) { busy = true; break; }
      }
    }
    if (!busy) break;
    if (millis() - t0 >= timeout_ms) break;
    delay(2);
  }
}
#endif

// init_camera_as / camera_reconnect / camera_change_framesize 已拆到 camera.cpp
// (wait_tx_idle 保留在本文件: 依赖快照双槽状态, 供 camera 模块经 camera.h 调用)

#ifdef ENABLE_WIFI
// 抓帧 -> PSRAM 副本写入"最新帧快照"双槽之一，立即归还驱动缓冲。
// 采集与抽帧解耦核心: 发送侧抽取节拍, 本函数只把【最新】帧留在快照里,
// 发送慢于采集(两槽都被占)时跳过 memcpy —— 不为注定被丢的帧浪费时间。
// 只做 memcpy 级工作(~7KB@QVGA), 不做任何阻塞发送。
//
// 跨核握手(与 sender_task 在 g_snap_mutex 下互斥):
//   槽状态机: 空闲 ->(生产写中, inuse=1)-> ready=1/不使用inuse ->(发送侧取走, inuse=1)
//            -> 发送完释放 -> 空闲。
//   生产侧: 只在"非 inuse"(空闲或已 ready 未取)槽上写; 写前释放槽内旧 buf(丢旧保新);
//          memcpy 期间标 inuse=1 防发送侧同时读, 写毕置 ready 并 give 信号量唤醒发送。
//   发送侧: 只取 "ready 且非 inuse" 的最新槽, 转移 buf 后置 inuse=1 防生产覆盖。
void enqueue_frame(camera_fb_t *fb) {
  int slot = -1;
  if (!xSemaphoreTake(g_snap_mutex, pdMS_TO_TICKS(10))) return;  // 忙/切换期放弃本帧
  for (int i = 0; i < SNAP_SLOTS; i++) {
    if (!g_snap_inuse[i]) { slot = i; break; }   // 空闲或已 ready 未取 -> 可写
  }
  if (slot < 0) {                                 // 两槽都在用(发送慢于采集)
    xSemaphoreGive(g_snap_mutex);
    g_diag_evict_frames++; g_adapt_total_evict++; g_diag_skip_frames++;
    return;
  }
  TxFrame &t = g_snap[slot];
  if (t.buf) { frame_free(t.buf, t.slot); t.buf = NULL; }   // 覆盖旧帧(丢旧保新)
  g_snap_inuse[slot] = 1;                          // 生产写中, 占槽
  xSemaphoreGive(g_snap_mutex);

  uint8_t *buf = frame_alloc(fb->len, &t.slot);
  memcpy(buf, fb->buf, fb->len);
  t.buf = buf;
  t.len = fb->len;
  t.fmt = fb->format;
  t.w = fb->width;
  t.h = fb->height;

  xSemaphoreTake(g_snap_mutex, portMAX_DELAY);
  g_slot_seq[slot] = ++g_pub_seq;                  // 标记为最新发布
  g_snap_ready[slot] = 1;
  g_snap_inuse[slot] = 0;                          // 发布完成, 供发送侧取走
  xSemaphoreGive(g_snap_mutex);
  if (g_new_frame) xSemaphoreGive(g_new_frame);    // 唤醒发送侧
  g_diag_cap_bytes += fb->len;
}

// 发送任务(核0)：拉取最新帧快照 -> 按当前模式选通道 -> 记账 -> 释放。
// 局域网客户端(TCP/UDP)路径持 g_cli_mutex 保护 cam_client;
// P2P/TURN/中继路径不碰 cam_client, 不持 g_cli_mutex —— 原实现所有路径都持该锁,
// 导致跨网模式下 loop() 的 accept/命令解析被阻塞最坏 200ms。
void sender_task(void *arg) {
  TxFrame t;
  camera_fb_t shim;
  IPAddress dst;
  for (;;) {
    // 等待新帧; 超时(500ms)醒来检查心跳, 避免 loopTask 卡死时 sender 也永久阻塞
    if (g_new_frame) xSemaphoreTake(g_new_frame, pdMS_TO_TICKS(500));
    // loop 心跳看门狗(独立任务监控, 不依赖 task watchdog 配置, 可靠):
    // loopTask 卡在 esp_camera_fb_get(永久等帧)等处时 g_loop_tick 停更, 5s 触发重启自愈。
    // 实测 task watchdog 默认不订阅 loopTask 且手动 init 会与框架冲突失效, 改用此方案。
    if (g_loop_tick && (millis() - g_loop_tick > 5000)) {
      Dbg.println("[wdt] loopTask 卡死, 重启自愈");
      delay(100);
      ESP.restart();
    }
    if (!xSemaphoreTake(g_snap_mutex, pdMS_TO_TICKS(50))) continue;  // snap 被占(切分辨率/入队中), 跳过本轮
    // 取最新 ready 槽(释放该槽所有权后本地持有发送), 其余 ready 旧帧丢弃。
    int pick = -1; uint32_t best = 0;
    for (int i = 0; i < SNAP_SLOTS; i++) {
      if (g_snap_ready[i] && g_slot_seq[i] > best) { best = g_slot_seq[i]; pick = i; }
    }
    t.buf = NULL; t.slot = -1; t.len = 0;
    if (pick >= 0) {
      t = g_snap[pick];
      g_snap[pick].buf = NULL;       // 转移所有权: 发送完成后由本任务释放
      g_snap[pick].slot = -1;
      g_snap_ready[pick] = 0;
    } else {
      // 无新帧(被生产覆盖前未发布临时窗口)或信号量被唤醒但无 ready, 软跳。
      for (int i = 0; i < SNAP_SLOTS; i++) {
        if (g_snap_ready[i]) { frame_free(g_snap[i].buf, g_snap[i].slot); g_snap[i].buf = NULL; g_snap_ready[i] = 0; }
      }
    }
    xSemaphoreGive(g_snap_mutex);
    if (!t.buf) continue;            // 无帧可发
    g_inflight++;

    memset(&shim, 0, sizeof(shim));
    shim.buf = t.buf; shim.len = t.len;
    shim.width = t.w; shim.height = t.h; shim.format = (pixformat_t)t.fmt;
    bool handled = false;

    // ---- 局域网客户端路径: 持 g_cli_mutex 保护 cam_client ----
    if (xSemaphoreTake(g_cli_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
      if (cam_client && cam_client.connected()) {
        dst = cam_client.remoteIP();
        bool ok;
        if (udp_mode) {
          if (g_udp_backoff_frames > 0) {
            g_udp_backoff_frames--;
            g_diag_skip_frames++;
            ok = false;
          } else {
            ok = send_frame_udp(&shim, cam_udp, dst, TCP_PORT);
          }
        } else {
          ok = send_frame_locked(&shim);
          if (!ok && !(cam_client && cam_client.connected())) {
            /* 客户端已被 send_frame_locked 判死并 stop, 仅记账 */ ;
          }
        }
        if (ok) record_sent(t.len);
        handled = true;
      }
      xSemaphoreGive(g_cli_mutex);
    }

    // ---- 跨网路径(P2P/TURN/中继): 不需要 g_cli_mutex ----
    // 原实现这些路径也在 g_cli_mutex 内, 跨网帧率受限于 loop() 的锁竞争。
    // 分离后, UDP/TURN 发送不再阻塞 loop() 的 TCP accept 和命令解析。
    if (!handled) {
      if (g_p2p_active) {
        // P2P 直连已建立: 视频经打洞锁定的候选 socket 直发观众, 不再走中继。
        // 锁定的是独立 socket([1..2])时不经 g_udp_mutex, 不与打洞/局域网发送竞争。
        if (g_udp_backoff_frames > 0) {
          g_udp_backoff_frames--;
          g_diag_skip_frames++;
        } else if (g_p2p_lock_sock >= 0 && g_p2p_udp[g_p2p_lock_sock]
                   && send_frame_udp(&shim, *g_p2p_udp[g_p2p_lock_sock],
                                     g_p2p_lock_ip, g_p2p_lock_port)) {
          record_sent(t.len);
        } else {
          g_diag_skip_frames++;
        }
      } else if (g_turn_mode) {
        // 打洞失败, TURN 兜底: 帧经服务端 UDP 转向观众(不依赖 NAT 类型, 必达)
        if (g_udp_backoff_frames > 0) {
          g_udp_backoff_frames--;
          g_diag_skip_frames++;
        } else if (send_frame_udp_via_turn(&shim)) {
          record_sent(t.len);
        } else {
          g_diag_skip_frames++;
        }
      } else if (g_relay_viewer) {
        // 无本地客户端但有观众经中继在观看 -> 整帧(含 FRAME 头)上行到中继
        // I1(大帧防 HOL): XGA/UXGA 大帧(~22KB, 常 >RELAY_BIG_FRAME_LEN)在带宽不足时
        // 会独占/阻塞整条 TCP 流(见 SOLUTIONS N1), 且一个大帧写不进去时仍占着帧缓冲,
        // 队列满即顶掉其后的所有小帧 -> 帧率骤降 + 周期断连。这里在**入队阻塞前**就
        // 预丢弃: 大帧按"背压分片"处理，不整帧占流，留给队列更多并发余量。
        // 信号: 上次中继发送阻塞时长(g_relay_stall_ms)。上次阻塞 >1s 视为拥塞,
        // 丢弃当前大帧(计数 bigdrop)，但**小帧照常发送**(小帧单发不产生 HOL)。
        const uint32_t RELAY_BIG_FRAME_LEN = 8 * 1024;
        const uint32_t RELAY_STALL_CONGEST_MS = 1000;
        const bool relay_congested = (g_relay_stall_ms >= RELAY_STALL_CONGEST_MS);
        if (shim.len > RELAY_BIG_FRAME_LEN && relay_congested) {
          g_diag_bigdrop_frames++;
          g_diag_skip_frames++;                  // 与大帧一起计入真丢帧
        } else if (send_frame_relay(&shim)) {
          record_sent(t.len);
        } else {
          g_diag_skip_frames++;
        }
      } else {
        g_diag_nc_frames++;
      }
    }

    frame_free(t.buf, t.slot);
    g_inflight--;
    adapt_quality();                   // 反馈环贴近真实发送速率
  }
}
#endif

void setup() {
  Dbg.begin(2000000, SERIAL_8N1, RX_GPIO_NUM, TX_GPIO_NUM);
  Dbg.setTimeout(100);
  delay(500);
  led_init();

  camera_config_t &config = g_config;
  memset(&config, 0, sizeof(config));
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = D0_GPIO_NUM;
  config.pin_d1       = D1_GPIO_NUM;
  config.pin_d2       = D2_GPIO_NUM;
  config.pin_d3       = D3_GPIO_NUM;
  config.pin_d4       = D4_GPIO_NUM;
  config.pin_d5       = D5_GPIO_NUM;
  config.pin_d6       = D6_GPIO_NUM;
  config.pin_d7       = D7_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 24000000;          // 必须≤规格上限24MHz: 30MHz下DSP色度通路失效,JPEG输出近灰度(实测饱和度5.8/255)
  config.pixel_format = PIX_FORMAT;        // JPEG(本工程目标) 或 RGB565
  config.frame_size   = PREVIEW_FRAME;
  config.fb_count     = fb_count_for(PREVIEW_FRAME);
  config.fb_location  = CAMERA_FB_IN_PSRAM; // 8MB PSRAM 帧缓冲

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Dbg.printf("摄像头初始化失败, err=0x%x\n", err);
    led_show_error();
    while (true) { delay(100); }
  }
  Dbg.println("摄像头初始化成功 (JPEG)");
  g_cam_ready = true;
  apply_color_calib();
  log_i("CAM init ok");

  // 人体检测模型: 相机就绪后加载一次。阶段1只验证"模型能加载+真实占用",
  // 采集/推理调度在阶段2接入。
  if (ai_detect_init())
    Dbg.println("[AI] 人体检测模型就绪");
  else
    Dbg.println("[AI] 人体检测模型初始化失败(不影响推流)");

#ifdef ENABLE_WIFI
  setup_wifi();
  log_i("CAM setup_wifi returned");
  g_snap_mutex = xSemaphoreCreateMutex();
  g_new_frame  = xSemaphoreCreateCounting(SNAP_SLOTS * 4, 0);   // 新帧发布信号量
  memset(g_snap, 0, sizeof(g_snap));
  g_cam_mutex = xSemaphoreCreateMutex();
  g_cli_mutex = xSemaphoreCreateMutex();
  g_udp_mutex = xSemaphoreCreateMutex();
  p2p_socks_init();    // P2P 候选 socket(主口 cam_udp + 2 个随机口, 用于交叉打洞)
  // 帧池(定长块)优先: 消除每帧 PSRAM malloc/free 抖动; 初始化失败自动回退逐帧 malloc
  if (frame_pool_init())
    Dbg.printf("[PSRAM] 帧池就绪 %d x %uB\n", FB_POOL_BLOCKS, (unsigned)FB_POOL_BLOCK);
  else
    Dbg.println("[PSRAM] 帧池初始化失败, 回退逐帧 malloc");
  // 核0 与 lwIP/WiFi 任务共核但优先级低本任务; 发送是大块 memcpy+socket 写,
  // 放核0 让它和采集(DMA中断)抢的同时彻底不碰 loopTask(核1) 的控制响应。
  // 栈 4096 -> 6144: 分片拼包缓冲(OVU_HDR+OVU_CHUNK=1400B)落在发送栈上。
  xTaskCreatePinnedToCore(sender_task, "tx", 6144, NULL, 3, NULL, 0);
  g_relay_tx = xSemaphoreCreateMutex();
  // 网络型任务统一 pin 核0(与 WiFi/lwIP 协议栈同核): relay 常驻收发/可能等待,
  // 若放核1 会抢走 loopTask 的采集与命令实时性。核1 只留给采集+控制, 最大化双核并行。
  xTaskCreatePinnedToCore(disc_task,  "disc",  2048, NULL, 1, NULL, 0); // 局域网发现广播
  xTaskCreatePinnedToCore(relay_task, "relay", 3072, NULL, 1, NULL, 0); // 跨网中继(常驻)
  xTaskCreatePinnedToCore(diag_beat_task, "diagb", 3072, NULL, 1, NULL, 0); // UDP 诊断心跳(核0)
  g_diag_last_ms = millis();
  g_adapt_last_ms = millis();
  log_i("CAM setup done, tasks up");
  // 初始灯态: 已入网=绿(短暂亮后灭), 否则若开了兜底热点=配网蓝呼吸
  if (WiFi.status() == WL_CONNECTED) {
    led_show_online();
  } else if (g_softap_on) {
    led_show_config();
  }
#endif
}

#ifdef ENABLE_WIFI
// handle_web_root / handle_web_save / default_dev_id 已拆到 web_cfg.cpp

// 局域网自动发现: 周期 UDP 广播(255.255.255.255:9999)设备名 + STA IP。
// 上位机 --auto 监听同端口即可自动发现并直连, 无需手动填 IP。
// 中继: 发送一条"4字节大端长度+payload"消息; 加锁, 防止与帧上行交错。
// 三段式 scatter-gather: FRAME头/帧体/FRAME尾 可零拷贝拼成一条消息,
// 省掉原先"整帧再 malloc 一份 + 两次 memcpy"的开销(每帧一次 PSRAM 分配与整帧拷贝)。
static bool relay_send_iov(const uint8_t *p0, size_t n0,
                           const uint8_t *p1, size_t n1,
                           const uint8_t *p2, size_t n2) {
  size_t total = n0 + n1 + n2;
  if (total == 0) return false;
  if (!relay_cli || !relay_cli.connected()) return false;
  const uint8_t *seg[3] = { p0, p1, p2 };
  size_t         segn[3] = { n0, n1, n2 };
  xSemaphoreTake(g_relay_tx, portMAX_DELAY);
  uint8_t hdr[4] = { (uint8_t)(total >> 24), (uint8_t)(total >> 16),
                     (uint8_t)(total >> 8),  (uint8_t)total };

  // 关键: 协议是"4字节大端长度 + 净荷"的长度前缀流。任何一条消息都不允许
  // 在中间被放弃(长度头已发出但净荷没发完), 否则服务端会把后续所有帧的字节
  // 误读成这一条的巨型净荷 -> 整个中继流永久错乱(帧率骤降/垃圾字节/卡死),
  // 只有服务端 45s 超时或摄像头重连 TCP 才能恢复。此前 write()==0(背压)时
  // 直接 break 放弃, 正是造成间歇性垃圾字节 + 帧率不稳的根因。
  bool ok = true;
  uint32_t stall = 0;                 // 连续无进展的毫秒数
  uint32_t total_stall = 0;           // 本条消息累计"写缓冲满等待"时长(ms), 喂给拥塞判定
  // 发送一段(含 4B 长度头)。s==0 表示 TCP 发送缓冲满/对端慢, 等待再重试;
  // 阻塞超过 ~2s 判定链路已死, 主动断开让上层重连重建(干净复位, 而非留下错位流)。
  auto flush_seg = [&](const uint8_t *p, size_t n) {
    size_t off = 0;
    while (off < n) {
      size_t s = relay_cli.write(p + off, n - off);
      if (s > 0) {
        off += s;
        stall = 0;
        continue;
      }
      // s == 0: 缓冲满, 等腾出再发(只在该段未发完时阻塞, 不忙转)。
      delay(1);
      total_stall++;                  // I1: 统计最近写阻塞量(供大帧拥塞回避)
      if (++stall >= 5000) {          // 5s: 跨网经云中继, 延迟/抖动远大于局域网,
        ok = false;                   // 2s 太激进导致频繁断连。5s 才判定链路真死。
        relay_cli.stop();             // 复位连接, 强制重建干净的中继流
        break;
      }
    }
  };
  if (ok)  flush_seg(hdr, 4);
  for (int i = 0; i < 3 && ok; i++) flush_seg(seg[i], segn[i]);
  xSemaphoreGive(g_relay_tx);
  // I1: 汇聚"近期中继写阻塞"给 sender_task 的大帧预丢弃 —— 按时间平滑衰减后叠加本次
  // 阻塞量。send_frame_relay 在拥塞期持续被调, 阻塞量会滚高; 溢出上限 65535 防回绕。
  uint32_t _semi = ((uint32_t)g_relay_stall_ms * 2u) / 3u + total_stall;
  g_relay_stall_ms = (_semi > 65535u) ? 65535u : (uint16_t)_semi;
  return ok;
}

void relay_send_payload(const uint8_t *data, size_t len) {
  relay_send_iov(data, len, NULL, 0, NULL, 0);
}

// ---- 可靠 P2P: 候选 socket 初始化 ----
// [0]=cam_udp(主口, 与局域网分片共用); [1..2]=独立随机端口。
// 多候选的意义: 端口受限锥形 NAT 只允许"本端曾向该目标发过包"的端口收包,
// 对端回包源端口稍有偏差即失败; 3 个候选 x 对端 N 候选交叉探测成倍提高命中率。
static void p2p_socks_init() {
  if (g_p2p_socks_ready) return;
  g_p2p_udp[0] = &cam_udp;
  for (int i = 1; i < P2P_NSOCK; i++) {
    WiFiUDP *u = new WiFiUDP();
    if (!u) break;
    if (u->begin(0) != 1) { delete u; break; }   // 0 = lwIP 分配随机端口
    g_p2p_udp[i] = u;
    Dbg.printf("[P2P] 候选 socket[%d] 就绪\n", i);
  }
  g_p2p_socks_ready = true;
}

// 对端候选是否匹配(回包校验: IP+端口都在候选表内, 防无关流量误判直连成功)
static bool p2p_is_peer(IPAddress ip, uint16_t port) {
  for (int j = 0; j < g_p2p_npeer; j++)
    if (g_p2p_peer_portN[j] == port && g_p2p_peer_ipN[j] == ip) return true;
  return false;
}

static void p2p_punch_locked(WiFiUDP *u) {
  for (int j = 0; j < g_p2p_npeer; j++) {
    u->beginPacket(g_p2p_peer_ipN[j], g_p2p_peer_portN[j]);
    u->write((const uint8_t *)P2P_MAGIC, 3);
    u->endPacket();
  }
}

// 收包检测: 只认带 magic 且端点在候选表内的包; 命中即锁定该 (socket, 端点) 通道。
static void p2p_detect_locked(WiFiUDP *u, int idx) {
  while (u->parsePacket() > 0) {
    uint8_t m[3] = {0, 0, 0};
    int n = u->read(m, 3);
    IPAddress sip = u->remoteIP();
    uint16_t sp = u->remotePort();
    u->flush();                        // 丢弃包体, 防残留在接收队列
    if (n == 3 && memcmp(m, P2P_MAGIC, 3) == 0 && p2p_is_peer(sip, sp)) {
      g_p2p_last_rx = millis();
      if (!g_p2p_active) {
        g_p2p_lock_sock = (int8_t)idx;
        g_p2p_lock_ip = sip;
        g_p2p_lock_port = sp;
        g_p2p_active = true;
        g_turn_mode = false;           // 直连恢复, 撤 TURN
        Dbg.printf("[P2P] 打洞成功 sock[%d] -> %s:%u\n", idx,
                   sip.toString().c_str(), sp);
      }
    }
  }
}

// 交叉打洞: 本端全部 socket x 对端全部候选组合发包 + 收包检测。
// 与 sender_task 共用 cam_udp([0]), 需持 g_udp_mutex; 拿不到就跳过本轮。
void p2p_punch() {
  if (g_p2p_npeer == 0 || !g_udp_mutex) return;
  if (xSemaphoreTake(g_udp_mutex, pdMS_TO_TICKS(20)) != pdTRUE) return;
  for (int i = 0; i < P2P_NSOCK; i++)
    if (g_p2p_udp[i]) p2p_punch_locked(g_p2p_udp[i]);
  for (int i = 0; i < P2P_NSOCK; i++)
    if (g_p2p_udp[i]) p2p_detect_locked(g_p2p_udp[i], i);
  xSemaphoreGive(g_udp_mutex);
}

// 排空全部候选 socket 的接收队列。带 magic 的包视为对端活性信号
// (直连与 TURN 模式的降级判定都依赖它); 必须排空防 pbuf 积压。
static void p2p_drain() {
  if (!g_udp_mutex) return;
  if (xSemaphoreTake(g_udp_mutex, pdMS_TO_TICKS(20)) != pdTRUE) return;
  for (int i = 0; i < P2P_NSOCK; i++) {
    WiFiUDP *u = g_p2p_udp[i];
    if (!u) continue;
    while (u->parsePacket() > 0) {
      uint8_t m[3] = {0, 0, 0};
      int n = u->read(m, 3);
      u->flush();
      if (n == 3 && memcmp(m, P2P_MAGIC, 3) == 0) g_p2p_last_rx = millis();
    }
  }
  xSemaphoreGive(g_udp_mutex);
}

// 直连保活: 持续向锁定端点发 magic。双向保活(旧版仅观众单向发,
// 摄像头侧 NAT 映射过期后对端再也打不进来, 直连悄悄变死)。
static void p2p_keepalive() {
  if (!g_p2p_active || g_p2p_lock_sock < 0 || !g_udp_mutex) return;
  WiFiUDP *u = g_p2p_udp[g_p2p_lock_sock];
  if (!u) return;
  if (xSemaphoreTake(g_udp_mutex, pdMS_TO_TICKS(20)) != pdTRUE) return;
  u->beginPacket(g_p2p_lock_ip, g_p2p_lock_port);
  u->write((const uint8_t *)P2P_MAGIC, 3);
  u->endPacket();
  xSemaphoreGive(g_udp_mutex);
}

// 清空 P2P 会话状态(中继断开/观众更换时调用)
static void p2p_reset() {
  g_p2p_npeer = 0;
  g_p2p_active = false;
  g_p2p_lock_sock = -1;
  g_p2p_lock_port = 0;
  g_turn_mode = false;
  g_p2p_last_rx = 0;
  g_p2p_next_retry = 0;
}

// STUN 上报: 本端全部候选 socket 各报一次 -> 服务器为每个端口记录一个反射候选。
// 与 sender_task 共用 cam_udp, 需持 g_udp_mutex。
void p2p_stun() {
  if (g_relay_host[0] == 0 || !g_udp_mutex) return;
  char h[64] = {0}; int port = 8000;
  sscanf(g_relay_host, "%63[^:]:%d", h, &port);
  IPAddress srv;
  if (!srv.fromString(h)) return;
  if (xSemaphoreTake(g_udp_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return;
  char msg[96];
  int n = snprintf(msg, sizeof(msg), "{\"t\":\"stun\",\"id\":\"%s\",\"role\":\"cam\"}",
                   g_dev_id);
  for (int i = 0; i < P2P_NSOCK; i++) {
    WiFiUDP *u = g_p2p_udp[i];
    if (!u) continue;
    u->beginPacket(srv, port + 1);     // UDP STUN 端口 = 中继 TCP 端口 + 1
    u->write((const uint8_t*)msg, n);
    u->endPacket();
  }
  xSemaphoreGive(g_udp_mutex);
}

// ICE 候选刷新: 周期上报 STUN, 保持服务器上 cam 的 server-reflexive 候选新鲜。
// host 候选 = WiFi.localIP(建连即固定), relayed 候选 = 服务器转发的映射。
void turn_keepalive() {
  if (!g_relay_online) return;
  uint32_t now = millis();
  if (now - g_turn_last_stun < 15000) return;   // 15s 一次即可维持 NAT 映射
  g_turn_last_stun = now;
  p2p_stun();                                   // 服务器按 STUN 刷新 cam 端点
}
void ice_poll() {
  // 三层候选协商中的轮询: 主要工作是保活刷新 server-reflexive 候选。
  // 打洞阶段与 TURN 兜底阶段都依赖该候选的新鲜度, 统一由 turn_keepalive 维护。
  turn_keepalive();
}

// 中继: 读一条完整消息; 返回 false 表示断连/出错。
bool relay_read_msg() {
  relay_cli.setTimeout(1500);
  uint8_t hdr[4];
  if (relay_cli.readBytes(hdr, 4) != 4) return false;
  uint32_t len = ((uint32_t)hdr[0] << 24) | ((uint32_t)hdr[1] << 16)
               | ((uint32_t)hdr[2] << 8) | hdr[3];
  if (len == 0 || len > (1 << 20)) return false;      // 非法长度防御
  static uint8_t sbuf[2048];   // 信令/命令通常 <1KB; relay_task 单线程, 静态缓冲安全
  uint8_t *buf = (len <= sizeof(sbuf)) ? sbuf : (uint8_t*)malloc(len);
  if (!buf) return false;
  bool ok = (relay_cli.readBytes(buf, len) == (int)len);
  if (ok && buf[0] == '{' && memchr(buf, '}', len)) {
    char s[160]; int cn = len; if (cn > 159) cn = 159;
    memcpy(s, buf, cn); s[cn] = 0;
    if      (strstr(s, "viewer_attached")) g_relay_viewer = true;
    else if (strstr(s, "viewer_left")) {
      g_relay_viewer = false;
      p2p_reset();                         // 观众离开, 撤销 UDP 通道与过时候选
    }
    else if (strstr(s, "viewer_udp")) {
      // 观众已完成 STUN 上报(服务器持有其 UDP 公网端点) -> 视频立刻走 TURN UDP,
      // 不再先经过数秒 TCP 中继。TCP 中继是队头阻塞的, 跨网帧率受单帧 RTT 限制。
      // 注意: 未 STUN 的观众(如 deploy/relay_fps.py)不会收到此信令, 行为保持原样。
      g_turn_mode = true;
      g_turn_last_stun = millis();
      g_relay_viewer = true;
      Dbg.println("[P2P] 观众 UDP 端点就绪, 视频改走 TURN");
    }
    else if (strstr(s, "p2p_peer")) {
      // {"t":"p2p_peer","ip":"x","port":N,"cands":["ip:port",...]}
      // -> 对端(观众)UDP 候选列表。可靠 P2P: 优先解析 cands 全部候选做交叉打洞;
      //    无 cands(旧服务器)时回退单 ip/port, 行为与旧版一致。
      g_p2p_npeer = 0;
      char *c = strstr(s, "\"cands\":[");
      if (c) {
        c += 9;
        while (g_p2p_npeer < P2P_NPEER_MAX && (c = strchr(c, '"')) != NULL) {
          c++;
          char *e = strchr(c, '"');
          if (!e) break;
          *e = 0;
          char ipb[40]; int p = 0;
          if (sscanf(c, "%39[^:]:%d", ipb, &p) == 2 && p > 0 && p < 65536
              && g_p2p_peer_ipN[g_p2p_npeer].fromString(ipb)) {
            g_p2p_peer_portN[g_p2p_npeer] = (uint16_t)p;
            g_p2p_npeer++;
          }
          c = e + 1;
        }
      }
      if (g_p2p_npeer == 0) {              // 旧服务器兜底: 单 ip/port
        char *ip = strstr(s, "\"ip\":\"");
        char *pt = strstr(s, "\"port\":");
        if (ip && pt) {
          ip += 6;
          char ipbuf[40]; int k = 0;
          while (*ip && *ip != '"' && k < 39) ipbuf[k++] = *ip++;
          ipbuf[k] = 0;
          int p = atoi(pt + 7);
          if (p > 0 && p < 65536 && g_p2p_peer_ipN[0].fromString(ipbuf)) {
            g_p2p_peer_portN[0] = (uint16_t)p;
            g_p2p_npeer = 1;
          }
        }
      }
      if (g_p2p_npeer > 0) {
        g_p2p_active = false;
        g_p2p_lock_sock = -1;
        g_p2p_punch_ms = millis();
        g_p2p_punch_start = millis();
        g_p2p_next_retry = 0;
        Dbg.printf("[P2P] 对端 %u 个候选, 开始交叉打洞\n", g_p2p_npeer);
      }
    }
    else if (strstr(s, "\"cmd\"")) {
      // 观众下行命令(可靠 JSON 通道): {"t": "cmd", "res": "Vx"/"Hx"} -> 执行并 ACK。
      // 匹配 "\"cmd\"" 以兼容 json.dumps 冒号后空格("t": "cmd")。
      char code = 0;
      char lead = relay_cmd_lead(s, code);
      char ack[96]; int ok = 0; int fs = -1;
      if (lead) {
        if (lead == 'V' || lead == 'v') {
          framesize_t t = framesize_from_code(code);
          if (t == FRAMESIZE_VGA) {
            // 运行时切 VGA 触发 esp32-camera EV-VSYNC-OVF 死循环(库残bug), 拒绝防卡死
            Dbg.println("[中继] VGA 运行中切换会死循环, 已拒绝");
          } else {
            PREVIEW_FRAME = t;
            ok = camera_change_framesize(t) ? 1 : 0;
            fs = (int)t;
            Dbg.printf("[中继] 预览分辨率 -> %c (%s)\n", code, ok ? "OK" : "FAIL");
          }
        } else if (lead == 'H' || lead == 'h') {
          HI_FRAME = framesize_from_code(code);
          ok = 1; fs = (int)HI_FRAME;
          Dbg.printf("[中继] 拍照分辨率 -> %c\n", code);
        }
      }
      snprintf(ack, sizeof(ack),
               "{\"t\":\"cmd_ack\",\"res\":\"%c%c\",\"ok\":%d,\"fs\":%d}",
               (lead ? lead : '?'), (code ? code : '?'), ok, fs);
      relay_send_payload((const uint8_t*)ack, strlen(ack));
    }
    // 其余 JSON(reg_ok/error)忽略
  } else if (ok && buf[0] != '{') {
    // 非 JSON = 观众下行命令, 作为字节流喂给命令处理器
    ByteStream bs(buf, len);
    while (bs.available() > 0) {
      int c = bs.read();
      if (c >= 0) handle_cmd(c, bs);
    }
  }
  if (buf != sbuf) free(buf);
  return ok;
}

// 中继客户端常驻任务: 连接服务端 -> 注册 -> 心跳保活 -> 观众接入/断开的控制消息。
void relay_task(void *arg) {
  uint32_t last_hb = 0;
  for (;;) {
    if (g_relay_host[0] == 0) { delay(5000); continue; }   // 未配置中继
    char h[64] = {0}; int port = 8000;
    sscanf(g_relay_host, "%63[^:]:%d", h, &port);
    Dbg.printf("[中继] 连接 %s:%d ...\n", h, port);
    if (relay_cli.connect(h, port, 5000)) {
      relay_cli.setNoDelay(true);
      g_relay_online = true;
      last_hb = millis();
      // 注册携带鉴权口令: 服务端若设了 token, 不带或不匹配会被拒绝并断连
      char reg[200];
      snprintf(reg, sizeof(reg),
               "{\"t\":\"reg\",\"id\":\"%s\",\"name\":\"%s\",\"token\":\"%s\"}",
               g_dev_id, g_dev_name, g_relay_token);
      relay_send_payload((const uint8_t*)reg, strlen(reg));
      Dbg.printf("[中继] 已注册 id=%s (token=%s)\n", g_dev_id,
                 g_relay_token[0] ? "已配置" : "无");
      p2p_stun();   // 上报自身 UDP 公网端点, 供服务器 P2P 撮合
      bool conn = true;
      while (conn) {
        if (relay_cli.available()) {
          if (!relay_read_msg()) conn = false;
        } else {
          uint32_t now = millis();
          if (now - last_hb >= 20000) {              // 心跳保活/感知链路
            relay_send_payload((const uint8_t*)"{\"t\":\"hb\"}", 10);
            last_hb = now;
          }
          // P2P 状态机(可靠版: 降级链闭环 + 周期重试恢复):
          //   打洞中  -> 每 100ms 交叉打洞; 3s 未通 -> TURN, 但保留候选, 30s 后重试
          //   直连中  -> 每 500ms 保活+排空; 3s 无对端包 -> 失活, 立即重打洞+降 TURN
          //   TURN 中 -> 5s 无对端包(观众 socket 已关, TURN 也是黑洞) -> 降回 TCP 中继;
          //              期间仍每 30s 重试打洞, 网络恢复即无缝切回直连
          //   判定信号统一为"多久没收到观众的 magic 包"(观众 _p2p_loop 每 500ms 发)
          if (g_p2p_npeer > 0) {
            if (g_p2p_active) {
              if (now - g_p2p_punch_ms >= P2P_KEEPALIVE_MS) {
                g_p2p_punch_ms = now;
                p2p_keepalive();               // 双向保活, 维持两侧 NAT 映射
                p2p_drain();
                if (now - g_p2p_last_rx > P2P_DEAD_MS) {
                  Dbg.println("[P2P] 直连失活, 重新打洞并降 TURN");
                  g_p2p_active = false;
                  g_p2p_lock_sock = -1;
                  g_turn_mode  = true;
                  g_p2p_punch_start = now;
                  g_p2p_next_retry = now + P2P_RETRY_MS;
                }
              }
            } else if (now - g_p2p_punch_ms >= P2P_PUNCH_MS) {
              g_p2p_punch_ms = now;
              p2p_punch();
              if (now - g_p2p_punch_start >= P2P_PUNCH_TMO
                  && !g_p2p_active && !g_turn_mode) {
                Dbg.println("[P2P] 打洞超时, 切换 TURN 兜底(候选保留, 周期重试)");
                g_turn_mode = true;            // 帧走 send_frame_udp_via_turn
                g_turn_last_stun = now;
                g_p2p_next_retry = now + P2P_RETRY_MS;
              }
            }
          }
          // 降级后的周期恢复: TURN/TCP 期间每 30s 再试一轮打洞。
          // 网络状况会变(NAT 映射刷新/观众换网), 恢复直连可卸掉服务器带宽。
          if (!g_p2p_active && g_p2p_npeer > 0 && g_p2p_next_retry != 0
              && now >= g_p2p_next_retry) {
            Dbg.println("[P2P] 周期重试打洞...");
            g_p2p_next_retry = now + P2P_RETRY_MS;
            g_p2p_punch_start = now;
            g_p2p_punch_ms = 0;
          }
          // TURN 失活检测: 观众已停流/退出时其 socket 关闭, TURN 转向收不到,
          // 此时必须降回 TCP 中继 —— 旧版 TURN 优先级高于 TCP 且失败不可感知,
          // 会一直往死端点发帧, TCP 兜底永远轮不到。
          if (g_turn_mode && !g_p2p_active
              && now - g_p2p_last_rx > P2P_TURN_DEAD_MS) {
            Dbg.println("[P2P] TURN 无观众应答, 降回 TCP 中继");
            g_turn_mode = false;
          }
          // ICE 候选刷新/TURN 保活: 维持服务器上 cam 的反射映射, 对称 NAT 也能收到转向
          ice_poll();
          if (!relay_cli.connected()) conn = false;
          if (conn) delay(10);
        }
      }
      g_relay_online = false;
      g_relay_viewer = false;
      p2p_reset();
      Dbg.println("[中继] 连接断开");
      relay_cli.stop();
    } else {
      Dbg.println("[中继] 连接失败");
    }
    delay(4000);   // 重连退避
  }
}

// 中继上行一帧: 把 FRAME_BEGIN/SIZE/数据/FRAME_END 打包成一条长度前缀消息。
// 观众端拿到的是与局域网流一致的字节流, 复用上位机现有帧解析。
// 零拷贝: 三段直接喂给 relay_send_iov, 不再为此额外分配 PSRAM 并整帧拷贝。
bool send_frame_relay(camera_fb_t *fb) {
  if (!g_relay_viewer || !relay_cli || !relay_cli.connected()) return false;
  char head[64];
  int hn = snprintf(head, sizeof(head), "FRAME_BEGIN\nSIZE %u %u %s\n",
                    fb->width, fb->height,
                    (fb->format == PIXFORMAT_JPEG) ? "JPG" : "RGB");
  if (hn <= 0 || hn >= (int)sizeof(head)) return false;
  return relay_send_iov((const uint8_t*)head, (size_t)hn,
                        fb->buf, fb->len,
                        (const uint8_t*)FRAME_END, strlen(FRAME_END));
}

// TURN 兜底发送一帧: 复用 OVU 分片布局, 但每片套上 "C2V" 信封发往服务器 UDP TURN 端口。
// 服务器把 payload(即完整 OVU 数据报)转向观众的真实公网端点 —— 不依赖 NAT 类型, 对称 NAT 也能必达。
// 信封: magic(3) "C2V" + cam_id_len(4, 大端) + cam_id + OVU 数据报。
static bool send_frame_udp_via_turn_locked(camera_fb_t *fb) {
  if (g_relay_host[0] == 0 || !g_relay_online) return false;
  char h[64] = {0}; int port = 8000;
  sscanf(g_relay_host, "%63[^:]:%d", h, &port);
  IPAddress srv;
  if (!srv.fromString(h)) return false;
  if (port + 1 > 65535) return false;

  const char *cid = g_dev_id;
  size_t cid_len = strlen(cid);
  if (cid_len == 0) return false;

  // 与 send_frame_udp 完全相同的 OVU 分片(观众端复用现有 UDP 重组逻辑)
  const uint8_t *buf = fb->buf;
  size_t len = fb->len;
  size_t nchunk = ovu_chunk_count(len);
  uint8_t pkt[OVU_HDR + OVU_CHUNK];
  ovu_build_header(pkt, (pixformat_t)fb->format, udp_seq, nchunk, fb->width, fb->height);

  // 信封固定头: magic(3) + cam_id_len(4, 大端)
  uint8_t env_head[7] = {
    'C', '2', 'V',
    (uint8_t)(cid_len >> 24), (uint8_t)(cid_len >> 16),
    (uint8_t)(cid_len >> 8),  (uint8_t)cid_len
  };

  bool allok = true;
  size_t off = 0;
  for (size_t i = 0; i < nchunk && off < len; i++) {
    size_t n = len - off;
    if (n > OVU_CHUNK) n = OVU_CHUNK;
    pkt[7] = (uint8_t)i;
    memcpy(pkt + OVU_HDR, buf + off, n);
    // OVU 数据报整块一次写入(此前 hdr + payload 两次 write, 多一次 pbuf 分配+拷贝)
    cam_udp.beginPacket(srv, port + 1);
    cam_udp.write(env_head, 7);
    cam_udp.write((const uint8_t*)cid, cid_len);
    cam_udp.write(pkt, OVU_HDR + n);
    int rc = cam_udp.endPacket();
    if (rc != 1) {
      allok = false;
      g_diag_udp_err++;
      g_adapt_total_err++;
      g_diag_last_err = -1;
      if (nchunk > 4) g_udp_backoff_frames = 3;   // 大帧失败退避, 别冲爆 TX 队列
      break;
    }
    off += n;
  }
  if (allok) udp_seq++;
  return allok;
}

// 加锁入口: 与 sender_task 的局域网分片发送共用同一 cam_udp
bool send_frame_udp_via_turn(camera_fb_t *fb) {
  if (!g_udp_mutex) return false;
  if (xSemaphoreTake(g_udp_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return false;
  bool ok = send_frame_udp_via_turn_locked(fb);
  xSemaphoreGive(g_udp_mutex);
  return ok;
}

// setup_wifi / log_wifi_ip / check_sta_link 已拆到 wifi_sta.cpp

#endif

// 阻塞等待期间让出 Web 配置页与日志(仅限 loop 任务: WebServer 非线程安全,
// 中继命令在核0 relay_task 上调用 handle_cmd 时不得碰 httpServer)。
static void yield_web(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
#ifdef ENABLE_WIFI
    if (strcmp(pcTaskGetTaskName(NULL), "loopTask") == 0) httpServer.handleClient();
#endif
    delay(1);
  }
}

// 上位机命令统一处理（串口与 TCP 共用协议）
void handle_cmd(int c, Stream &in) {
  if (c == 'P' || c == 'p') {
    Dbg.println("CMD_PHOTO");
    camera_change_framesize(HI_FRAME);
    yield_web(500);   // 原 delay(500): 拍照期间 Web 配置页冻结、DIAG 停摆
    int attempt = 0;
    camera_fb_t *fb = NULL;
    bool locked = false;
#if defined(ENABLE_WIFI)
    bool cam_grab_ok;
    locked = cam_lock(1500);   // 与采集双核互斥后再抓帧
    cam_grab_ok = locked;
#else
    bool cam_grab_ok = true;   // 纯串口无并发, 无需锁
#endif
    while (cam_grab_ok && !fb && attempt < 10) {
      fb = esp_camera_fb_get();
      if (!fb) { yield_web(50); attempt++; }
    }
    if (!fb) {
#ifdef ENABLE_WIFI
      if (locked) cam_unlock();
#endif
      Dbg.println("拍照失败：esp_camera_fb_get 返回空");
      camera_reconnect();
    } else {
      bool sent_via_tcp = false;
#ifdef ENABLE_WIFI
      sent_via_tcp = send_frame(fb);   // 内部持锁走可靠 TCP, 上位机在 TCP 流中解照片帧
#endif
      if (!sent_via_tcp) send_frame_serial(fb);   // 无客户端或 TCP 中断则串口兜底
      esp_camera_fb_return(fb);
#ifdef ENABLE_WIFI
      if (locked) cam_unlock();
#endif
    }
    camera_change_framesize(PREVIEW_FRAME);
    Dbg.println("CMD_PHOTO_DONE");
  } else if (c == 'v' || c == 'V') {
    long t0 = millis();
    while (millis() - t0 < 200 && !in.available()) delay(1);
    if (!in.available()) { Dbg.println("V 缺分辨率码, 已忽略"); return; }
    char code = (char)in.read();
    framesize_t target = framesize_from_code(code);
    // 相同分辨率直接跳过, 避免重连时 _resync 反复触发全量重建
    if (target == cur_frame) {
      Dbg.printf("预览分辨率未变(%c), 跳过\n", code);
      return;
    }
    PREVIEW_FRAME = target;
    if (camera_change_framesize(PREVIEW_FRAME)) {
      Dbg.printf("预览分辨率已设为: %c\n", code);
    } else {
      // 切换失败(锁忙等): 回退 PREVIEW_FRAME, 防止重连 _resync 反复触发死循环
      PREVIEW_FRAME = cur_frame;
      Dbg.printf("预览分辨率切换失败, 回退到当前(%c)\n", code);
    }
  } else if (c == 'h' || c == 'H') {
    long t0 = millis();
    while (millis() - t0 < 200 && !in.available()) delay(1);
    if (!in.available()) { Dbg.println("H 缺分辨率码, 已忽略"); return; }
    char code = (char)in.read();
    HI_FRAME = framesize_from_code(code);
    Dbg.printf("拍照分辨率已设为: %c (下次拍照生效)\n", code);
  } else if (c == 'q') {
    long t0 = millis();
    while (millis() - t0 < 200 && !in.available()) delay(1);
    int q = in.read();
    if (q >= 0) {
      // QVGA(CIF模式) 下过低画质易触发传感器编码卡死无帧(见 g_quality 初始化注释)，小图钳下限
      const int floor_q = (cur_frame < FRAMESIZE_VGA) ? 8 : 2;
      if (q < floor_q) {
        Dbg.printf("quality %d 过低, 当前分辨率钳制到 %d\n", q, floor_q);
        q = floor_q;
      }
      g_quality = q;
      sensor_t *s = esp_camera_sensor_get();
      if (s) s->set_quality(s, g_quality);
      Dbg.printf("JPEG quality=%d\n", g_quality);
    }
  } else if (c == 'a' || c == 'A') {
    g_auto_quality = !g_auto_quality;
    Dbg.printf("AUTOQ %s\n", g_auto_quality ? "ON" : "OFF");
  } else if (c == 'd') {
#ifdef ENABLE_WIFI
    bool dly = cam_lock(500);
#else
    bool dly = true;
#endif
    if (!dly) { Dbg.println("DIAG: 相机锁忙, 跳过"); }
    else {
      camera_fb_t *fb = esp_camera_fb_get();
      if (fb) {
        Dbg.printf("DIAG fmt=%d len=%u %ux%u\n", (int)fb->format, (unsigned)fb->len,
                   fb->width, fb->height);
        esp_camera_fb_return(fb);
      }
#ifdef ENABLE_WIFI
      cam_unlock();
#endif
    }
  } else if (c == 'e' || c == 'w') {
    // 色彩调试: e+档位(-2..2)=饱和度, w+模式(0自动/1晴天/2阴天/3办公室/4家居)
    long t0 = millis();
    while (millis() - t0 < 200 && !in.available()) delay(1);
    int lvl = in.read();
    sensor_t *s = esp_camera_sensor_get();
    if (!s) { Dbg.println("e/w: no sensor"); }
    else if (lvl < 0) {
      Dbg.printf("%c 状态: sat=%d wb=%d\n", c, (int)s->status.saturation, (int)s->status.wb_mode);
    } else if (c == 'e') {
      s->set_saturation(s, lvl > 2 ? 2 : lvl);
      Dbg.printf("sat=%d\n", (int)s->status.saturation);
    } else {
      s->set_wb_mode(s, lvl > 4 ? 4 : lvl);
      Dbg.printf("wb=%d\n", (int)s->status.wb_mode);
    }
  } else if (c == 'k') {
    // 时钟寄存器诊断实验: 验证 CLKRC/COM8 读回, 测试不同写入组合
    sensor_t *s = esp_camera_sensor_get();
    if (!s) { Dbg.println("k: no sensor"); }
    else {
      int v;
      v = s->get_reg(s, 0x0013, 0xFF); Dbg.printf("k COM8=0x%02X\n", (uint8_t)v);
      v = s->get_reg(s, 0x0011, 0xFF); Dbg.printf("k CLKRC init=0x%02X\n", (uint8_t)v);
      s->set_reg(s, 0x0011, 0xFF, 0x01);
      v = s->get_reg(s, 0x0011, 0xFF); Dbg.printf("k CLKRC=0x01 -> 0x%02X\n", (uint8_t)v);
      s->set_reg(s, 0x0011, 0xFF, 0x80);
      v = s->get_reg(s, 0x0011, 0xFF); Dbg.printf("k CLKRC=0x80 -> 0x%02X\n", (uint8_t)v);
      s->set_reg(s, 0x0011, 0xFF, 0x00);   // 还原
      v = s->get_reg(s, 0x0011, 0xFF); Dbg.printf("k CLKRC=0x00 -> 0x%02X\n", (uint8_t)v);
    }
  } else if (c == 'r' || c == 'R') {
    camera_reconnect();
#ifdef ENABLE_WIFI
  } else if (c == 'U') {
    udp_mode = true;
    Dbg.println("UDP 预览模式开启");
  } else if (c == 'N') {
    udp_mode = false;
    Dbg.println("UDP 预览模式关闭(TCP)");
#endif
  } else if (c == 'T' || c == 't') {
    // AI 人体检测配置(阶段3): T+子命令
    //   Ts      查询状态
    //   Tt<0-100> 人置信度阈值(%)
    //   Tf<1-10>  连续确认帧数(防抖)
    //   Tc<0-255> 触发冷却(秒)
    //   Ti<200-60000> 推理间隔(ms)
    //   Tm      切换中继报警
    long t0 = millis();
    while (millis() - t0 < 200 && !in.available()) delay(1);
    if (!in.available()) {
      Dbg.println("T 缺子命令 (s/t/f/c/i/m)");
      return;
    }
    char sub = (char)in.read();
    if (sub == 's' || sub == 'S') {
      Dbg.printf("AI: 阈值=%.0f%% 确认帧=%u 冷却=%us 推理间隔=%ums 中继报警=%s\n",
                 g_ai_person_thresh * 100, (unsigned)g_ai_confirm_frames,
                 (unsigned)(g_ai_cooldown_ms / 1000),
                 (unsigned)g_ai_infer_interval_ms,
                 g_ai_relay_alert ? "ON" : "OFF");
      Dbg.printf("AI: 累计推理=%u 命中=%u 上次推理耗时=%ums\n",
                 (unsigned)g_ai_infer_total, (unsigned)g_ai_hit_total,
                 (unsigned)(g_ai_last_infer_us / 1000));
    } else if (sub == 'i' || sub == 'I') {
      long t1 = millis();
      while (millis() - t1 < 200 && !in.available()) delay(1);
      if (!in.available()) { Dbg.println("Ti 缺间隔ms"); return; }
      int v = in.read();
      if (v >= 200 && v <= 60000) {
        g_ai_infer_interval_ms = (uint32_t)v;
        Dbg.printf("AI 推理间隔=%ums\n", v);
      } else Dbg.println("Ti 间隔须 200-60000ms");
    } else if (sub == 't' || sub == 'T') {
      long t1 = millis();
      while (millis() - t1 < 200 && !in.available()) delay(1);
      if (!in.available()) { Dbg.println("Tt 缺阈值%%"); return; }
      int pct = in.read();
      if (pct >= 0 && pct <= 100) {
        g_ai_person_thresh = pct / 100.0f;
        Dbg.printf("AI 阈值=%.0f%%\n", g_ai_person_thresh * 100);
      } else Dbg.println("Tt 阈值须 0-100");
    } else if (sub == 'f' || sub == 'F') {
      long t1 = millis();
      while (millis() - t1 < 200 && !in.available()) delay(1);
      if (!in.available()) { Dbg.println("Tf 缺帧数"); return; }
      int n = in.read();
      if (n >= 1 && n <= 10) {
        g_ai_confirm_frames = (uint8_t)n;
        Dbg.printf("AI 确认帧=%u\n", (unsigned)g_ai_confirm_frames);
      } else Dbg.println("Tf 帧数须 1-10");
    } else if (sub == 'c' || sub == 'C') {
      long t1 = millis();
      while (millis() - t1 < 200 && !in.available()) delay(1);
      if (!in.available()) { Dbg.println("Tc 缺冷却秒"); return; }
      int s = in.read();
      if (s >= 0 && s <= 255) {
        g_ai_cooldown_ms = (uint32_t)s * 1000;
        Dbg.printf("AI 冷却=%us\n", s);
      } else Dbg.println("Tc 冷却须 0-255 秒");
    } else if (sub == 'm' || sub == 'M') {
      g_ai_relay_alert = !g_ai_relay_alert;
      Dbg.printf("AI 中继报警 %s\n", g_ai_relay_alert ? "ON" : "OFF");
    } else {
      Dbg.println("T 子命令未知 (s/t/f/c/i/m)");
    }
  }
}

void loop() {
  g_loop_tick = millis();   // 心跳: sender_task 监控, 卡死(如 fb_get 永久等帧)则重启自愈
  log_wifi_ip();
#ifdef ENABLE_WIFI
  httpServer.handleClient();   // Web 配置页：通过局域网 IP 访问(SoftAP 已关闭)
#endif
  // ---- WiFi：accept 最新 TCP 客户端；断开及时重置；旧客户端半开时抢占接管 ----
#ifdef ENABLE_WIFI
  {
    bool dead = false;
    if (xSemaphoreTake(g_cli_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      if (cam_client && !cam_client.connected()) {
        cam_client.stop();
        cam_client = WiFiClient();
        dead = true;
      }
      xSemaphoreGive(g_cli_mutex);
    }
    // 抢占式 accept: 即使旧客户端 connected() 仍为 true(半开/粘滞), 新连接也接管
    WiFiClient s = cam_server.available();
    if (s) {
      if (xSemaphoreTake(g_cli_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        if (cam_client) cam_client.stop();
        cam_client = s;
        cam_client.setNoDelay(true);
        cam_client.setTimeout(500);
        xSemaphoreGive(g_cli_mutex);
        Dbg.println("WiFi 客户端接入(抢占)");
        log_i("NET accept(抢占) cam_ready=%d", (int)g_cam_ready);
      } else {
        // 抢锁失败: sender_task 每帧都持该锁发送, 撞上发送窗口很常见。
        // 原实现让 s 在此析构 -> 关闭新连接, 上位机"已连但无流"(实测 0 fps)。
        // 暂存后下轮重试接管, 保证新连接不丢。
        if (g_pending_client) g_pending_client.stop();
        g_pending_client = s;
      }
    }
    // 接管上一轮暂存的新连接(持续重试直到成功)
    if (g_pending_client) {
      if (xSemaphoreTake(g_cli_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        if (cam_client) cam_client.stop();
        cam_client = g_pending_client;
        g_pending_client = WiFiClient();
        cam_client.setNoDelay(true);
        cam_client.setTimeout(500);
        xSemaphoreGive(g_cli_mutex);
        Dbg.println("WiFi 客户端接入(延迟接管)");
        log_i("NET accept(延迟) cam_ready=%d", (int)g_cam_ready);
      }
    }
    check_sta_link();
    test_tick();   // 测试连接状态机(异步推进)
    // 运行期灯态跟随: 已入网=绿, 否则开了兜底热点=配网蓝呼吸
    if (WiFi.status() == WL_CONNECTED) {
      if (!g_softap_on) led_show_online();
    } else if (g_softap_on) {
      led_show_config();
    }
    led_update();
  }
#endif

  // ---- 人体检测触发: 消费推理命中的标志, 在“不持相机锁”时执行拍照 ----
  if (g_ai_trigger_photo) {
    g_ai_trigger_photo = false;
    Dbg.println("[AI] 触发拍照");
    handle_cmd('P', Dbg);
  }

  // ---- 命令：串口 + TCP 都读 ----
  while (Dbg.available()) handle_cmd(Dbg.read(), Dbg);
#ifdef ENABLE_WIFI
  {
    if (xSemaphoreTake(g_cli_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      WiFiClient snap = cam_client;   // 读命令期间锁定同一 socket, 防中途被抢占
      xSemaphoreGive(g_cli_mutex);
      if (snap && snap.connected()) {
        while (snap.available()) handle_cmd(snap.read(), snap);
      }
    }
  }
#endif

  // ---- 采集：连续抓帧入队(发送交由 senderTask)；失锁自动重连 ----
#ifdef ENABLE_WIFI
  if (!cam_lock(200)) return;   // 正在重建/换分辨率(他核持锁)则跳过本帧
#endif
  camera_fb_t *fb = NULL;
  int attempt = 0;
  while (!fb && attempt < 3) {
    fb = esp_camera_fb_get();
    if (!fb) { delay(30); attempt++; }
  }
  if (!fb) {
#ifdef ENABLE_WIFI
    cam_unlock();               // 先释放锁再重连(reconnect 内部会重新取锁)
#endif
    static uint32_t fail_count = 0;
    fail_count++;
    if (fail_count >= 2) {
      fail_count = 0;
      camera_reconnect();
    }
    return;
  }
#ifdef ENABLE_WIFI
  g_diag_cap_frames++;
  if (!g_switching) enqueue_frame(fb);
#else
  send_frame(fb);
#endif

  // ---- 人体检测: 节流借帧推理(不影响主推流, 软解 JPEG 旁路) ----
  {
    uint32_t now = millis();
    if (now - g_ai_last_infer_ms >= g_ai_infer_interval_ms) {
      g_ai_last_infer_ms = now;
      uint32_t _t0 = micros();
      ai_result_t r = ai_detect_frame(fb);
      uint32_t _t1 = micros();
      g_ai_last_infer_us = _t1 - _t0;
      g_ai_infer_total++;
      if (r.ok) {
        if (r.person) {
          if (++g_ai_confirm_cnt >= g_ai_confirm_frames) {
            g_ai_confirm_cnt = 0;
            uint32_t now2 = millis();
            if (now2 - g_ai_last_trigger_ms < g_ai_cooldown_ms) {
              Dbg.printf("[AI] 检测到人(%.2f) 但冷却中, 跳过\n", r.person_score);
            } else {
              g_ai_last_trigger_ms = now2;
              g_ai_hit_total++;
              Dbg.printf("[AI] 检测到人(置信度 %.2f), 待触发拍照(累计命中 %u)\n",
                         r.person_score, (unsigned)g_ai_hit_total);
              // 中继报警: 复用现有 cmd_ack 可靠 JSON 通道(服务端原样转发给观众), 
              // 不引入服务端不识别的 t 类型。观众端收到 {"t":"cmd_ack","res":"AI",...}
              // 即“检测到人”事件。res 前缀 AI 与摄像头命令回执区分。
              if (g_ai_relay_alert && g_relay_viewer) {
                char _al[96];
                int _n = snprintf(_al, sizeof(_al),
                                  "{\"t\":\"cmd_ack\",\"res\":\"AI\",\"ok\":1,"
                                  "\"score\":%d,\"hit\":%u}",
                                  (int)(r.person_score * 100),
                                  (unsigned)g_ai_hit_total);
                relay_send_payload((const uint8_t*)_al, (size_t)_n);
                Dbg.println("[AI] 中继报警已发送");
              }
              g_ai_trigger_photo = true;   // 只置标志, 由下轮 loop 在不持锁时消费
            }
          }
        } else {
          g_ai_confirm_cnt = 0;
        }
      }
    }
  }

  esp_camera_fb_return(fb);   // 无论是否入队都只归还一次
#ifdef ENABLE_WIFI
  cam_unlock();
  log_diag();
#endif
  delay(1);
}
