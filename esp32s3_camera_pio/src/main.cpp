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
#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiServer.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <Preferences.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"

// ---- 像素格式：本工程专为解锁 JPEG ----
#define PIX_FORMAT PIXFORMAT_JPEG   // JPEG（源库编译），改回 PIXFORMAT_RGB565 即旧基线

// ---- 函数原型（.ino 由 Arduino 自动生成，.cpp 需手动声明）----
int fb_count_for(framesize_t fs);
bool send_frame(camera_fb_t *fb);
void send_frame_serial(camera_fb_t *fb);
framesize_t framesize_from_code(char code);
void apply_color_calib();
bool init_camera_as(framesize_t target, framesize_t fallback);
bool camera_reconnect();
bool camera_change_framesize(framesize_t fs);
void setup_wifi();
void log_wifi_ip();
void check_sta_link();
void log_diag();
void adapt_quality();
void handle_cmd(int c, Stream &in);
void enqueue_frame(camera_fb_t *fb);
void wait_tx_idle(uint32_t timeout_ms);
void sender_task(void *arg);
void handle_web_root();
void handle_web_save();
void disc_task(void *arg);
void relay_task(void *arg);
bool send_frame_relay(camera_fb_t *fb);
bool send_frame_udp_via_turn(camera_fb_t *fb);
void p2p_punch();
void ice_poll();
void turn_keepalive();
String default_dev_id();

// UART0 = CH343 转串口所在通道（TX=GPIO43, RX=GPIO44），避免默认 Serial 走 USB-CDC。
HardwareSerial Dbg(0);

// ---- WiFi 传输（纯 STA 连入局域网 + TCP 服务端；SoftAP 仅在无凭据时兜底配网）----
#define ENABLE_WIFI 1
#ifdef ENABLE_WIFI
// 凭据不入库：建本地文件 src/secrets.h 覆盖（已在 .gitignore）。
//   #define STA_SSID "你的SSID"
//   #define STA_PASS "你的密码"
// 未定义时留空 -> setup_wifi() 检测无凭据会开 SoftAP 兜底，保证配网页(:80)可达。
// 正常运行凭据来自 NVS：配网页保存过一次后即持久化，重启自动重连。
#if __has_include("secrets.h")
#include "secrets.h"
#endif
#ifndef STA_SSID
#define STA_SSID   ""
#endif
#ifndef STA_PASS
#define STA_PASS   ""
#endif
#define AP_SSID    "OV2640Cam"         // 兜底热点(仅在无 STA 凭据时开启)
#define AP_PASS    ""
#define TCP_PORT   3333
static WiFiServer  cam_server(TCP_PORT);
static WiFiClient  cam_client;
static uint32_t    g_last_ip_log = 0;
static bool        g_softap_on   = false;   // 兜底热点是否已开(保存凭据后需关闭)

// ---- Web 配置页 + 目标 WiFi(NVS) ----
static char        g_sta_ssid[33] = STA_SSID;   // 运行时 STA 目标(可在配网页覆盖并持久化)
static char        g_sta_pass[64] = STA_PASS;
static Preferences g_prefs;
static WebServer   httpServer(80);

static WiFiUDP     cam_udp;            // UDP 预览通道(高帧率, 丢帧不重传)
static bool        udp_mode  = false;  // 上位机发 'U' 开启
static uint16_t    udp_seq   = 0;
static bool        g_auto_quality = true;
static uint8_t     g_udp_backoff_frames = 0;
static uint32_t    g_diag_last_ms = 0;
static uint32_t    g_diag_cap_frames = 0;
static uint32_t    g_diag_sent_frames = 0;
static uint32_t    g_diag_skip_frames = 0;
// 队列满时主动顶掉的最旧帧(背压), 与 g_diag_skip_frames(真丢帧)分开统计。
// 此前两者混用一个计数器, 且无条件出队导致每帧+1, DIAG 的 skp 完全失去诊断价值。
static uint32_t    g_diag_evict_frames = 0;
static uint32_t    g_diag_nc_frames = 0;
static uint32_t    g_diag_udp_err = 0;
static uint32_t    g_diag_udp_retry_ok = 0;
static int         g_diag_last_err = 0;
static uint64_t    g_diag_sent_bytes = 0;
static uint64_t    g_diag_cap_bytes = 0;
static uint32_t    g_adapt_last_ms = 0;
static uint64_t    g_adapt_last_sent = 0;
static uint64_t    g_adapt_last_bytes = 0;
// 自适应画质用独立累计计数：log_diag 每5s清零 g_diag_*，统计窗不同不能共用，
// 否则差分遇到清零会产生下溢/0值，画质调节半失效。
// 注意：UDP 错误计数同样必须独立累计（此前漏了，adapt_quality 直接读 g_diag_udp_err，
// 该值被 log_diag 每5s清零，判定结果取决于采样点落在清零前后，时灵时不灵）。
static uint64_t    g_adapt_total_frames = 0;
static uint64_t    g_adapt_total_bytes  = 0;
static uint32_t    g_adapt_total_err    = 0;
static uint32_t    g_adapt_last_err     = 0;

// ---- 局域网自动发现 + 跨网中继客户端 ----
#define DISCOVERY_PORT  9999          // 局域网广播发现端口
#define RELAY_HOST_DEFAULT "8.148.79.85:8001"   // 未配置时默认连接的公网中继服务端(空=关闭)
static WiFiUDP      disc_udp;
static uint32_t     g_disc_last_ms = 0;

static char         g_dev_name[32]   = AP_SSID;  // 广播/注册用设备名(可在配网页改)
static char         g_dev_id[40]     = "";       // 注册 id(默认 cam-<MAC后4字节>)
static char         g_relay_host[80] = "";       // 中继服务端 "host:port", 空=关闭
static WiFiClient   relay_cli;
static SemaphoreHandle_t g_relay_tx  = NULL;     // 保护 relay 发送(信令与帧交错用)
static volatile bool g_relay_viewer  = false;    // 有观众经中继在观看
static volatile bool g_relay_online  = false;    // 已连上中继服务端

// ---- P2P 直连(尽力而为, 失败回退中继) ----
static IPAddress     g_p2p_peer_ip;              // P2P 对端(观众)公网 IP
static volatile uint16_t g_p2p_peer_port = 0;    // P2P 对端公网端口(0=未配置)
static volatile bool g_p2p_active = false;       // P2P 直连已建立(视频走 UDP 直连)
static uint32_t      g_p2p_punch_ms = 0;         // 上次打洞发包时间
static uint32_t      g_p2p_punch_start = 0;      // 打洞开始时间(超时放弃用)
static uint32_t      g_p2p_last_rx = 0;          // 最近收到对端包的时刻(直连存活判定)
static volatile bool g_turn_mode    = false;     // 打洞超时已切换 TURN 兜底(帧经服务器转向观众)
static uint32_t      g_turn_last_stun = 0;       // 上次 STUN 保活刷新时间

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
#define TXQ_DEPTH      4          // ~XGA 50KB/帧 -> 最坏积压 200KB PSRAM
static QueueHandle_t       g_txq       = NULL;
// 保护 cam_udp: sender_task(核0 优先级3) 发视频分片、relay_task(核0 优先级1) 打洞/STUN/
// 排空都会操作同一个 WiFiUDP 对象。lwIP socket 与 WiFiUDP 内部状态均非线程安全,
// 且两者同核, 高优先级 sender 可在 relay 执行 beginPacket 期间抢占 -> 状态错乱。
// TURN 提前开启后并发窗口显著变大, 必须串行化。
static SemaphoreHandle_t   g_udp_mutex = NULL;

// ---- 帧内存池：消除每帧 PSRAM malloc/free 的抖动与碎片 ----
// 改造前采集侧每帧一次 heap_caps_malloc、发送侧一次 heap_caps_free，30fps 即每秒
// 30 次 PSRAM 分配/释放，且帧长随场景变化极易产生碎片。定长块池化后分配退化为
// 一次队列出队（跨任务安全、零碎片）；超过块长的特大帧回退堆分配，行为不变。
#define FB_POOL_BLOCKS 6
#define FB_POOL_BLOCK  (64 * 1024)    // 覆盖 QVGA~UXGA 全部 JPEG 帧(48K 不够覆盖 UXGA 极端帧)
static uint8_t      *g_pool_mem  = NULL;
static QueueHandle_t g_pool_free = NULL;   // 空闲块索引

static bool frame_pool_init() {
  g_pool_mem = (uint8_t*)heap_caps_malloc((size_t)FB_POOL_BLOCKS * FB_POOL_BLOCK,
                                          MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!g_pool_mem) return false;
  g_pool_free = xQueueCreate(FB_POOL_BLOCKS, sizeof(int));
  if (!g_pool_free) { heap_caps_free(g_pool_mem); g_pool_mem = NULL; return false; }
  for (int i = 0; i < FB_POOL_BLOCKS; i++) {
    int idx = i;
    xQueueSend(g_pool_free, &idx, 0);
  }
  return true;
}
// 取一块；无空闲/未初始化返回 -1(调用方回退堆分配)
static int frame_pool_take() {
  int idx;
  if (!g_pool_free) return -1;
  return (xQueueReceive(g_pool_free, &idx, 0) == pdTRUE) ? idx : -1;
}
static void frame_pool_give(int idx) {
  if (idx >= 0 && g_pool_free) xQueueSend(g_pool_free, &idx, 0);
}
// 分配一帧缓冲：优先池化块，超大帧回退 PSRAM 堆
static uint8_t *frame_alloc(size_t len, int8_t *slot) {
  if (g_pool_mem && len <= FB_POOL_BLOCK) {
    int idx = frame_pool_take();
    if (idx >= 0) { *slot = (int8_t)idx; return g_pool_mem + (size_t)idx * FB_POOL_BLOCK; }
  }
  *slot = -1;
  return (uint8_t*)heap_caps_malloc(len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}
static void frame_free(uint8_t *p, int8_t slot) {
  if (!p) return;
  if (slot >= 0) frame_pool_give((int)slot);
  else heap_caps_free(p);
}
static SemaphoreHandle_t   g_cli_mutex = NULL;   // 保护 cam_client 的 accept/swap/stop/send
static volatile int        g_inflight   = 0;     // 发送方正持有的帧数(已出队未释放)
static volatile bool       g_switching  = false; // 摄像头重建窗口, 生产侧暂停入队

// 发送成功统一记账（DIAG 周期窗口 + 自适应累计两套都递增）
static void record_sent(size_t len) {
  g_diag_sent_frames++;
  g_diag_sent_bytes   += len;
  g_adapt_total_frames++;
  g_adapt_total_bytes += len;
}
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

// ---- 帧头/帧尾（上位机识别帧边界）----
static const char FRAME_BEGIN[] = "FRAME_BEGIN\n";
static const char FRAME_END[]   = "FRAME_END\n";

// 运行时可切换的分辨率
static framesize_t PREVIEW_FRAME = FRAMESIZE_QVGA;   // 预览 320x240（30fps，OV2640 CIF 模式甜点）
static framesize_t HI_FRAME      = FRAMESIZE_UXGA;   // 拍照 1600x1200
static framesize_t cur_frame     = FRAMESIZE_QVGA;
static int         g_quality     = 16;   // JPEG 质量(0-63, 值越大压缩越强文件越小).
// 帧率优先策略: 默认升至16在 QVGA 下每帧量约减半, 跨网带宽受限时帧率近翻倍, QVGA画质损失轻微。
// 注意: QVGA(CIF模式)下 quality 太低(如2)会卡死无帧(传感器编码bug), 16 在此安全区间内且比10更抗带宽。
//       VGA(SVGA模式)下可运行时 'q2' 提画质(实测 15fps + 55KB/帧) 不损失帧率.

// 摄像头配置全局保存，供失锁后 deinit+reinit 复用
static camera_config_t g_config;

// JPEG 模式下每帧字节数不定，缓冲预算按 RGB 等价量估算，防 PSRAAM OOM
#define BYTES_PER_PIXEL 2
#define MAX_FB_BYTES (6 * 1024 * 1024)   // 6MB

// 依据分辨率自适应双缓冲数量（大图降为1，防 PSRAM OOM 卡死）
int fb_count_for(framesize_t fs) {
  if (fs < 0 || fs >= FRAMESIZE_INVALID) return 1;
  int w = resolution[fs].width;
  int h = resolution[fs].height;
  int64_t one = (int64_t)w * h * BYTES_PER_PIXEL;
  int64_t two = one * 2;
  if (two <= MAX_FB_BYTES) return 2;
  return 1;
}

// 发送一帧原始数据（带尺寸头）。仅在持有 g_cli_mutex 时调用。
// 失败返回 false 并重置客户端（P 拍照路径据此回退串口补发）。
static bool send_frame_locked(camera_fb_t *fb) {
  if (!(cam_client && cam_client.connected())) { g_diag_nc_frames++; return false; }
  cam_client.write((const uint8_t*)FRAME_BEGIN, sizeof(FRAME_BEGIN) - 1);
  char head[48];
  snprintf(head, sizeof(head), "SIZE %u %u %s\n", fb->width, fb->height,
           (fb->format == PIXFORMAT_JPEG) ? "JPG" : "RGB");
  cam_client.write((const uint8_t*)head, strlen(head));
  const uint8_t *p = fb->buf; size_t left = fb->len;
  bool tcp_ok = true;
  while (left) {
    size_t n = left > 16384 ? 16384 : left;
    size_t s = cam_client.write(p, n);
    if (s == 0) { tcp_ok = false; break; }
    p += s; left -= s;
    if (left) yield();
  }
  if (tcp_ok) {
    cam_client.write((const uint8_t*)FRAME_END, sizeof(FRAME_END) - 1);
    return true;
  }
  Dbg.println("TCP 发送失败, 丢帧并重置客户端");
  cam_client.stop();
  g_diag_skip_frames++;
  g_diag_last_err = -2;
  return false;
}

// 对外包装：拍照等 loop 侧调用加锁后复用同一实现
bool send_frame(camera_fb_t *fb) {
#ifdef ENABLE_WIFI
  if (xSemaphoreTake(g_cli_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) return false;
  bool ok = send_frame_locked(fb);
  xSemaphoreGive(g_cli_mutex);
  return ok;
#else
  send_frame_serial(fb);
  return true;
#endif
}

// 强制通过串口发送帧（不受 WiFi 客户端状态影响）。用于 P 拍照的兜底路径。
void send_frame_serial(camera_fb_t *fb) {
  Dbg.write(FRAME_BEGIN, sizeof(FRAME_BEGIN) - 1);
  Dbg.printf("SIZE %u %u %s\n", fb->width, fb->height,
             (fb->format == PIXFORMAT_JPEG) ? "JPG" : "RGB");
  Dbg.write(fb->buf, fb->len);
  Dbg.write(FRAME_END, sizeof(FRAME_END) - 1);
}

#ifdef ENABLE_WIFI
// ---- OVU 分片协议公共定义 ----
// 局域网直发(send_frame_udp) 与 TURN 转向(send_frame_udp_via_turn) 共用，
// 此前两处各写一份分片逻辑且已发生漂移(turn 路径漏了逐片重试)，改协议极易漏改。
//   头: "OVU"(3) fmt(1) seq(2) cnt_minus1(1) idx(1) w(2) h(2)  fmt:0=RGB565 1=JPEG
#define OVU_HDR         12
#define OVU_CHUNK       (1400 - OVU_HDR)   // 单报文净荷, 整包 1400B 避开 MTU 分片
#define OVU_MAX_CHUNKS  250                // idx/cnt 是 u8

// 组装分片头；hdr[7](idx) 由调用方逐片填
static inline void ovu_build_header(uint8_t *hdr, pixformat_t fmt, uint16_t seq,
                                    size_t nchunk, uint16_t w, uint16_t h) {
  hdr[0] = 'O'; hdr[1] = 'V'; hdr[2] = 'U';
  hdr[3] = (fmt == PIXFORMAT_JPEG) ? 1 : 0;
  hdr[4] = (uint8_t)(seq >> 8);
  hdr[5] = (uint8_t)(seq & 0xFF);
  hdr[6] = (uint8_t)(nchunk - 1);
  hdr[7] = 0;
  hdr[8]  = (uint8_t)(w >> 8);
  hdr[9]  = (uint8_t)(w & 0xFF);
  hdr[10] = (uint8_t)(h >> 8);
  hdr[11] = (uint8_t)(h & 0xFF);
}
static inline size_t ovu_chunk_count(size_t len) {
  size_t n = (len + OVU_CHUNK - 1) / OVU_CHUNK;
  if (n == 0) n = 1;
  return n > OVU_MAX_CHUNKS ? OVU_MAX_CHUNKS : n;
}

// 通过 UDP 把一整帧分片成多个数据报发送（预览用，丢帧不重传）。
// 优化1: 头+净荷先在栈上拼成连续报文再单次 write。此前两次 write 会让 lwIP
//        生成链式 pbuf(两次分配+拷贝)，每帧 4~36 片 = 每帧数十次额外开销。
// 优化2: 重试改为整帧共享预算。此前小帧 6 次重试×2ms×最多4片 = 最坏阻塞 48ms
//        (注释写 8ms 与实际不符)，期间 sender_task 停止消费队列。
static bool send_frame_udp_locked(camera_fb_t *fb, IPAddress dst, uint16_t port) {
  const uint8_t *buf = fb->buf;
  size_t len = fb->len;
  size_t nchunk = ovu_chunk_count(len);
  uint8_t pkt[OVU_HDR + OVU_CHUNK];          // 1400B 拼包缓冲(单次 write)
  ovu_build_header(pkt, (pixformat_t)fb->format, udp_seq, nchunk, fb->width, fb->height);

  const bool small_frame     = (nchunk <= 4);
  const int  max_retry       = small_frame ? 6 : 1;
  const int  retry_delay_ms  = small_frame ? 2 : 1;
  const int  retry_budget_ms = small_frame ? 6 : 3;    // 整帧重试总预算(ms), 小帧 10->6 减少队列阻塞
  int budget_used = 0;
  size_t off = 0;
  for (size_t i = 0; i < nchunk && off < len; i++) {
    size_t n = len - off;
    if (n > OVU_CHUNK) n = OVU_CHUNK;
    pkt[7] = (uint8_t)i;
    memcpy(pkt + OVU_HDR, buf + off, n);
    bool sent = false;
    for (int t = 0; t <= max_retry; t++) {
      cam_udp.beginPacket(dst, port);
      cam_udp.write(pkt, OVU_HDR + n);
      int rc = cam_udp.endPacket();
      if (rc == 1) {
        if (t > 0) g_diag_udp_retry_ok++;
        sent = true;
        break;
      }
      g_diag_udp_err++;
      g_adapt_total_err++;
      g_diag_last_err = -1;
      if (t >= max_retry || budget_used >= retry_budget_ms) break;
      delay(retry_delay_ms);
      budget_used += retry_delay_ms;
    }
    if (!sent) {
      g_diag_skip_frames++;
      if (!small_frame) g_udp_backoff_frames = 3;
      return false;
    }
    off += n;
  }
  udp_seq++;
  return true;
}

// 加锁入口: cam_udp 与 relay_task 的打洞/STUN 共用, 必须串行化
bool send_frame_udp(camera_fb_t *fb, IPAddress dst, uint16_t port = TCP_PORT) {
  if (!g_udp_mutex) return false;
  if (xSemaphoreTake(g_udp_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
    g_diag_skip_frames++;              // 拿不到锁(打洞中), 宁丢一帧不并发
    return false;
  }
  bool ok = send_frame_udp_locked(fb, dst, port);
  xSemaphoreGive(g_udp_mutex);
  return ok;
}
#endif

// 分辨率字符码 -> framesize_t。q=QQVGA,t=QVGA,v=VGA,s=SVGA,x=XGA,u=UXGA
framesize_t framesize_from_code(char code) {
  switch (code) {
    case 'q': return FRAMESIZE_QQVGA;
    case 't': return FRAMESIZE_QVGA;
    case 'v': return FRAMESIZE_VGA;
    case 's': return FRAMESIZE_SVGA;
    case 'x': return FRAMESIZE_XGA;
    case 'u': return FRAMESIZE_UXGA;
    default:  return FRAMESIZE_QQVGA;
  }
}

// 每次 reinit 后重设传感器画质参数（deinit+init 会复位寄存器）
void apply_color_calib() {
  sensor_t *s = esp_camera_sensor_get();
  if (!s) { Dbg.println("警告：获取 sensor 失败"); return; }
  s->set_whitebal(s, 1);      // AWB
  s->set_gain_ctrl(s, 1);     // AGC
  s->set_exposure_ctrl(s, 1);
  // JPEG 质量(0-63, 越小画质越高文件越大), 运行时 'q' 命令可调
  s->set_quality(s, g_quality);
  uint8_t clkrc_init = (uint8_t)s->get_reg(s, 0x0011, 0xFF);
  Dbg.printf("CLKRC init=0x%02X\n", clkrc_init);
  delay(200);                 // 给 AWB/AGC 收敛时间
}

#ifdef ENABLE_WIFI
// 相机驱动互斥锁: deinit/init 重建与采集(fb_get/fb_return) 分处不同核(中继命令在核0、
// 采集 loop 在核1), 双核并发访问 cam_obj 会在 deinit 释放后仍在 fb_get/return 引用
// → use-after-free 卡死(与分辨率无关, 切任意都卡)。所有 esp_camera_* 调用必须以它串行化。
static SemaphoreHandle_t g_cam_mutex = NULL;
static bool cam_lock(uint32_t timeout_ms) {
  return g_cam_mutex && xSemaphoreTake(g_cam_mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}
static void cam_unlock() { if (g_cam_mutex) xSemaphoreGive(g_cam_mutex); }
// 等待发送队列排空且没有在途帧。必须在任何 esp_camera_deinit/改分辨率前调用：
// 队列里存的是已从相机驱动拷出的堆内存，切换本身安全，
// 但若在途帧还被 sender 引用就 deinit 会改变后续 fb 语义 —— 排空保证无竞争窗口。
void wait_tx_idle(uint32_t timeout_ms) {
  uint32_t t0 = millis();
  while ((uxQueueMessagesWaiting(g_txq) > 0 || g_inflight > 0) && millis() - t0 < timeout_ms) {
    delay(2);
  }
}
#endif

// 以目标分辨率重建摄像头（失败自动降级：先单缓冲，再回退 fallback）
bool init_camera_as(framesize_t target, framesize_t fallback) {
  g_config.frame_size = target;
  g_config.fb_count   = fb_count_for(target);
  esp_err_t err = esp_camera_init(&g_config);
  if (err == ESP_OK) {
    cur_frame = target;
    Dbg.printf("摄像头初始化: fs=%u fb=%d\n", (unsigned)target, g_config.fb_count);
    apply_color_calib();
    return true;
  }
  Dbg.printf("init fs=%u 失败 err=0x%x, 尝试单缓冲...\n", (unsigned)target, err);
  // 失败的 init 可能残留传感器/DMA/SCCB 状态，必须先 deinit 再重试，
  // 否则二次 init 大概率同样失败，会被误判为内存不足而错误降级
  esp_camera_deinit();
  delay(100);
  g_config.fb_count = 1;
  err = esp_camera_init(&g_config);
  if (err == ESP_OK) {
    cur_frame = target;
    Dbg.printf("摄像头初始化(单缓冲): fs=%u fb=1\n", (unsigned)target);
    apply_color_calib();
    return true;
  }
  Dbg.printf("单缓冲也失败 err=0x%x, 回退 fs=%u\n", err, (unsigned)fallback);
  esp_camera_deinit();
  delay(100);
  g_config.frame_size = fallback;
  g_config.fb_count   = fb_count_for(fallback);
  err = esp_camera_init(&g_config);
  if (err == ESP_OK) {
    cur_frame = fallback;
    Dbg.printf("已回退到 fs=%u\n", (unsigned)fallback);
    apply_color_calib();
    return true;
  }
  Dbg.printf("回退也失败 err=0x%x\n", err);
  return false;
}

// 失锁后重新初始化（先 deinit 释放 DMA/缓冲，再重建为当前分辨率）
bool camera_reconnect() {
  Dbg.println("摄像头失锁, 正在重连...");
#ifdef ENABLE_WIFI
  g_switching = true;
  wait_tx_idle(200);
  if (!cam_lock(1500)) {        // 与采集/切换互斥; 拿不到锁则放弃本次重连, 下轮采集自动重试
    g_switching = false;
    return false;
  }
  esp_camera_deinit();
  delay(300);
  g_switching = false;
  bool ok = init_camera_as(cur_frame, PREVIEW_FRAME);
  cam_unlock();
  return ok;
#else
  esp_camera_deinit();
  delay(300);
  return init_camera_as(cur_frame, PREVIEW_FRAME);
#endif
}

// 真正的分辨率切换：deinit + init 全量重配，失败自动回退
bool camera_change_framesize(framesize_t fs) {
  if (fs == cur_frame) { return true; }
#ifdef ENABLE_WIFI
  g_switching = true;
  wait_tx_idle(500);            // 大分辨率下最后一帧可能近100ms, 留足排空时间
  if (!cam_lock(1000)) {        // 与核1采集 fb_get/fb_return 双核互斥, 防 deinit 释放后仍被引用
    Dbg.println("切换取消: 相机锁忙(采集阻塞), 稍后重试");
    g_switching = false;
    return false;
  }
  esp_camera_deinit();
  delay(200);
  bool ok = init_camera_as(fs, cur_frame);
  cam_unlock();
  if (!ok) {
    // 三层重配全失败: 传感器/驱动可能已进入不可恢复状态, 直接复位保证不永久卡死
    Dbg.println("FSW FAIL -> restart");
    delay(100);
    g_switching = false;
    ESP.restart();
  }
  g_switching = false;
#else
  esp_camera_deinit();
  delay(200);
  bool ok = init_camera_as(fs, cur_frame);
#endif
  return ok;
}

#ifdef ENABLE_WIFI
// 抓帧 -> PSRAM 副本入队，立即归还驱动缓冲。队列满/拷贝失败按丢帧计数背压。
// 在 loop 任务执行, 只做 memcpy 级工作(~7KB@QVGA), 不做任何阻塞发送。
void enqueue_frame(camera_fb_t *fb) {
  TxFrame t;
  t.buf = frame_alloc(fb->len, &t.slot);
  if (!t.buf) { g_diag_skip_frames++; g_diag_last_err = -3; return; }
  memcpy(t.buf, fb->buf, fb->len);
  t.len = fb->len;
  t.fmt = fb->format;
  t.w = fb->width;
  t.h = fb->height;
  // 队列满=网络消费不动。策略: 丢【最旧】帧腾位让进【最新】帧(丢旧保新),
  // 保证发送端永远取到当前最新画面, 观众看到实时而非积压的旧帧。
  // 采集侧始终满帧率入队, 帧率不被通信延迟拖慢。
  // 注意必须判满: 此前无条件出队, 队列退化为深度1(多分配的槽位白占 PSRAM),
  // 且 skp 每帧+1 完全淹没真实丢帧 —— DIAG 的 skp 恒等于帧数, 无法再用于诊断。
  if (uxQueueSpacesAvailable(g_txq) == 0) {
    TxFrame victim;
    if (xQueueReceive(g_txq, &victim, 0) == pdTRUE) {
      frame_free(victim.buf, victim.slot);
      g_diag_evict_frames++;             // 背压主动顶帧, 与链路丢帧分开统计
    }
  }
  if (xQueueSend(g_txq, &t, 0) != pdTRUE) {
    frame_free(t.buf, t.slot);
    g_diag_skip_frames++;                // 真丢帧: 入队失败
    return;
  }
  g_diag_cap_bytes += fb->len;
}

// 发送任务(核0)：出队 -> 按当前模式选通道 -> 记账 -> 释放。
// 局域网客户端(TCP/UDP)路径持 g_cli_mutex 保护 cam_client;
// P2P/TURN/中继路径不碰 cam_client, 不持 g_cli_mutex —— 原实现所有路径都持该锁,
// 导致跨网模式下 loop() 的 accept/命令解析被阻塞最坏 200ms。
void sender_task(void *arg) {
  TxFrame t;
  camera_fb_t shim;
  IPAddress dst;
  for (;;) {
    if (xQueueReceive(g_txq, &t, portMAX_DELAY) != pdTRUE) continue;
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
            ok = send_frame_udp(&shim, dst);
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
        // P2P 直连已建立: 视频经 UDP 直发对端(观众), 不再走中继
        if (g_udp_backoff_frames > 0) {
          g_udp_backoff_frames--;
          g_diag_skip_frames++;
        } else if (send_frame_udp(&shim, g_p2p_peer_ip, g_p2p_peer_port)) {
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
        if (send_frame_relay(&shim)) record_sent(t.len);
        else g_diag_skip_frames++;
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
    while (true) { delay(100); }
  }
  Dbg.println("摄像头初始化成功 (JPEG)");
  apply_color_calib();

#ifdef ENABLE_WIFI
  setup_wifi();
  g_txq = xQueueCreate(TXQ_DEPTH, sizeof(TxFrame));
  g_cam_mutex = xSemaphoreCreateMutex();
  g_cli_mutex = xSemaphoreCreateMutex();
  g_udp_mutex = xSemaphoreCreateMutex();
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
  g_diag_last_ms = millis();
  g_adapt_last_ms = millis();
#endif
}

#ifdef ENABLE_WIFI
// Web 配置页：通过设备所在局域网 IP（或上位机 --auto 发现）访问。
void handle_web_root() {
  String html = String("<!DOCTYPE html><html lang='zh'><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>OV2640 摄像头配置</title>"
    "<style>body{font-family:sans-serif;max-width:520px;margin:40px auto;padding:0 16px}"
    "input{width:100%;padding:10px;margin:6px 0 14px;box-sizing:border-box;font-size:16px}"
    "button{width:100%;padding:12px;background:#2f6fed;color:#fff;border:0;font-size:17px;border-radius:6px}"
    "table{width:100%;border-collapse:collapse}td{padding:6px 0}label{color:#555}</style></head><body>"
    "<h2>OV2640 摄像头配置</h2>"
    "<table>")
    + "<tr><td>STA 状态</td><td>" + String(WiFi.status() == WL_CONNECTED ? "已连接" : "未连接")
    + " / " + String(g_sta_ssid) + "</td></tr>"
    + "<tr><td>设备ID</td><td>" + String(g_dev_id) + "</td></tr>"
    + "<tr><td>中继</td><td>" + String(g_relay_host[0] ? String(g_relay_host) : String("未配置"))
    + (g_relay_online ? " <b>(在线)</b>" : "") + "</td></tr></table>"
    + "<hr><h3>设置摄像头要连接的目标 WiFi</h3>"
    + "<form action='/save' method='get'>"
    + "<label>WiFi 名称 (SSID)</label>"
    + "<input name='ssid' value='" + String(g_sta_ssid) + "'>"
    + "<label>密码</label>"
    + "<input type='password' name='pass' value='" + String(g_sta_pass) + "'>"
    + "<hr><h3>入网标识 / 跨网中继(可选)</h3>"
    + "<label>设备ID (局域网发现与中继注册用, 留空自动)</label>"
    + "<input name='id' value='" + String(g_dev_id) + "'>"
    + "<label>中继服务端 IP:端口 (如 1.2.3.4:8000, 留空=仅局域网)</label>"
    + "<input name='relay' value='" + String(g_relay_host) + "'>"
    + "<button type='submit'>保存并连接</button></form>"
    + "<p style='color:#888'>保存后摄像头将尝试连接该 WiFi。配置页通过目标网络的局域网 IP 访问"
    + "(可在路由器管理页或上位机 --auto 发现)，SoftAP 已彻底关闭。</p></body></html>";
  httpServer.send(200, "text/html; charset=utf-8", html);
}

void handle_web_save() {
  String _ssid = httpServer.hasArg("ssid") ? httpServer.arg("ssid") : "";
  String _pass = httpServer.hasArg("pass") ? httpServer.arg("pass") : "";
  String _name = httpServer.hasArg("id")   ? httpServer.arg("id")   : "";
  String _relay= httpServer.hasArg("relay")? httpServer.arg("relay"): "";
  _ssid.trim();
  if (_ssid.isEmpty()) {
    httpServer.send(200, "text/html; charset=utf-8",
        "<html><meta charset='utf-8'><body>SSID 不能为空，<a href='/'>返回</a></body></html>");
    return;
  }
  if (_name.isEmpty()) _name = default_dev_id();
  _relay.trim();
  g_prefs.begin("wifi", false);
  g_prefs.putString("ssid", _ssid);
  g_prefs.putString("pass", _pass);
  g_prefs.putString("id",   _name);
  g_prefs.putString("relay", _relay);
  g_prefs.end();

  _ssid.toCharArray(g_sta_ssid, sizeof(g_sta_ssid));
  _pass.toCharArray(g_sta_pass, sizeof(g_sta_pass));
  _name.toCharArray(g_dev_id,   sizeof(g_dev_id));
  if (_name != String(g_dev_name)) _name.toCharArray(g_dev_name, sizeof(g_dev_name));
  _relay.toCharArray(g_relay_host, sizeof(g_relay_host));

  httpServer.send(200, "text/html; charset=utf-8",
      String("<html><meta charset='utf-8'><meta http-equiv='refresh' content='2;url=/'>"
             "<body>已保存目标 WiFi <b>" + _ssid + "</b>，正在连接，2 秒后刷新…</body></html>"));

  // 立即按新目标重连 STA；成功则摄像头入网。
  // 凭据写入后关掉兜底热点把射频还给 STA; 若连不上, 兜底热点会保留以便再次配置。
  if (g_softap_on) {
    WiFi.softAPdisconnect(true);
    g_softap_on = false;
    WiFi.mode(WIFI_STA);
    Dbg.println("[WiFi] 已写入凭据, 兜底热点关闭");
  }
  WiFi.disconnect();
  delay(50);
  WiFi.begin(g_sta_ssid, g_sta_pass);
}

// 默认设备注册 id: cam-<MAC 尾 4 字节>
String default_dev_id() {
  uint8_t m[6]; WiFi.macAddress(m);
  char b[32];
  snprintf(b, sizeof(b), "cam-%02X%02X%02X%02X", m[2], m[3], m[4], m[5]);
  return String(b);
}

// 局域网自动发现: 周期 UDP 广播(255.255.255.255:9999)设备名 + STA IP。
// 上位机 --auto 监听同端口即可自动发现并直连, 无需手动填 IP。
void disc_announce() {
  if (WiFi.status() != WL_CONNECTED) return;
  IPAddress ip = WiFi.localIP();
  char msg[128];
  snprintf(msg, sizeof(msg), "OV2640|%s|%d.%d.%d.%d",
           g_dev_name, ip[0], ip[1], ip[2], ip[3]);
  disc_udp.beginPacket(IPAddress(255, 255, 255, 255), DISCOVERY_PORT);
  disc_udp.write((const uint8_t*)msg, strlen(msg));
  disc_udp.endPacket();
}
void disc_task(void *arg) {
  for (;;) {
    if (WiFi.status() == WL_CONNECTED) disc_announce();
    delay(3000);
  }
}

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
  relay_cli.write(hdr, 4);
  bool ok = true;
  for (int i = 0; i < 3 && ok; i++) {
    size_t off = 0;
    while (off < segn[i]) {
      size_t s = relay_cli.write(seg[i] + off, segn[i] - off);
      if (s == 0) { ok = false; break; }
      off += s;
      if (off < segn[i]) {
        // 仅在发送缓冲区满时延迟; 原实现每次部分写都 delay(1),
        // 跨网大帧(50KB+)可累积 10-20ms 额外延迟
        if (relay_cli.availableForWrite() == 0) delay(1);
      }
    }
  }
  xSemaphoreGive(g_relay_tx);
  return ok;
}

void relay_send_payload(const uint8_t *data, size_t len) {
  relay_send_iov(data, len, NULL, 0, NULL, 0);
}

// P2P 打洞: 向对端公网端点发包, 并检测对端回包(打洞成功则置 g_p2p_active)。
// 与 sender_task 共用 cam_udp, 需持 g_udp_mutex; 拿不到就跳过本轮(下轮再试)。
void p2p_punch() {
  if (g_p2p_peer_port == 0 || !g_udp_mutex) return;
  if (xSemaphoreTake(g_udp_mutex, pdMS_TO_TICKS(20)) != pdTRUE) return;
  // 向对端发打洞包(内容任意, 目的让双方 NAT 建立映射)
  cam_udp.beginPacket(g_p2p_peer_ip, g_p2p_peer_port);
  cam_udp.write((const uint8_t *)"P2P", 3);
  cam_udp.endPacket();
  // 检测对端回包: 收到来自对端 IP 的 UDP 包 => 打洞成功
  if (cam_udp.parsePacket() > 0) {
    IPAddress src = cam_udp.remoteIP();
    cam_udp.flush();                 // 丢弃该包内容, 避免残留在接收队列
    if (src == g_p2p_peer_ip) {
      g_p2p_last_rx = millis();
      if (!g_p2p_active) {
        g_p2p_active = true;
        Dbg.printf("[P2P] 打洞成功, 直连 %s:%u\n",
                   g_p2p_peer_ip.toString().c_str(), g_p2p_peer_port);
      }
    }
  }
  xSemaphoreGive(g_udp_mutex);
}

// P2P 存活维护: 排空对端持续发来的保活/打洞包并刷新存活时间。
// 必须排空, 否则包会一直积压在 lwIP 接收队列占用 pbuf。
static void p2p_drain() {
  if (!g_udp_mutex) return;
  if (xSemaphoreTake(g_udp_mutex, pdMS_TO_TICKS(20)) != pdTRUE) return;
  while (cam_udp.parsePacket() > 0) {
    if (cam_udp.remoteIP() == g_p2p_peer_ip) g_p2p_last_rx = millis();
    cam_udp.flush();
  }
  xSemaphoreGive(g_udp_mutex);
}

// 向服务器 UDP STUN 端口上报自身公网端点(打洞辅助)。服务器据此撮合双方。
void p2p_stun() {
  if (g_relay_host[0] == 0 || !g_udp_mutex) return;
  char h[64] = {0}; int port = 8000;
  sscanf(g_relay_host, "%63[^:]:%d", h, &port);
  IPAddress srv;
  if (!srv.fromString(h)) return;
  if (xSemaphoreTake(g_udp_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return;
  cam_udp.beginPacket(srv, port + 1);   // UDP STUN 端口 = 中继 TCP 端口 + 1
  char msg[96];
  int n = snprintf(msg, sizeof(msg), "{\"t\":\"stun\",\"id\":\"%s\",\"role\":\"cam\"}",
                   g_dev_id);
  cam_udp.write((const uint8_t*)msg, n);
  cam_udp.endPacket();
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
      g_turn_mode = false;                 // 观众离开, 撤销 UDP 转发通道
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
      // {"t":"p2p_peer","ip":"x.x.x.x","port":N} -> 对端(观众)公网端点
      char *ip = strstr(s, "\"ip\":\"");
      char *pt = strstr(s, "\"port\":");
      if (ip && pt) {
        ip += 6;
        char ipbuf[32]; int k = 0;
        while (*ip && *ip != '"' && k < 31) ipbuf[k++] = *ip++;
        ipbuf[k] = 0;
        int p = atoi(pt + 7);
        if (p > 0 && p < 65536 && g_p2p_peer_ip.fromString(ipbuf)) {
          g_p2p_peer_port = (uint16_t)p;
          g_p2p_active = false;
          g_p2p_punch_ms = millis();
          g_p2p_punch_start = millis();
          Dbg.printf("[P2P] 对端 %s:%d, 开始打洞\n", ipbuf, p);
        }
      }
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
      char reg[160];
      snprintf(reg, sizeof(reg), "{\"t\":\"reg\",\"id\":\"%s\",\"name\":\"%s\"}",
               g_dev_id, g_dev_name);
      relay_send_payload((const uint8_t*)reg, strlen(reg));
      Dbg.printf("[中继] 已注册 id=%s\n", g_dev_id);
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
          // P2P 状态机:
          //   未建立 -> 每 100ms 打洞, 超时(3s)放弃, 切换 TURN 兜底
          //   已建立 -> 每 500ms 排空对端保活包刷新存活; 3s 无包判定直连失活,
          //             回退 TURN 并重新尝试打洞(修复: 此前 g_p2p_active 一旦置位
          //             就永不清零, 对端 NAT 映射失效后摄像头会一直往黑洞发流)
          if (g_p2p_peer_port != 0) {
            if (g_p2p_active) {
              if (now - g_p2p_punch_ms >= 500) {
                g_p2p_punch_ms = now;
                p2p_drain();
                if (now - g_p2p_last_rx > 3000) {
                  Dbg.println("[P2P] 直连失活, 回退 TURN");
                  g_p2p_active = false;
                  g_turn_mode  = true;
                  g_p2p_punch_start = now;
                }
              }
            } else if (now - g_p2p_punch_ms >= 100) {
              p2p_punch();
              g_p2p_punch_ms = now;
              if (now - g_p2p_punch_start >= 3000) {
                Dbg.println("[P2P] 打洞超时, 切换 TURN 兜底");
                g_p2p_peer_port = 0;
                g_turn_mode = true;          // 帧走 send_frame_udp_via_turn
                g_turn_last_stun = now;
              }
            }
          }
          // ICE 候选刷新/TURN 保活: 维持服务器上 cam 的反射映射, 对称 NAT 也能收到转向
          ice_poll();
          if (!relay_cli.connected()) conn = false;
          if (conn) delay(10);
        }
      }
      g_relay_online = false;
      g_relay_viewer = false;
      g_p2p_active = false;
      g_p2p_peer_port = 0;
      g_turn_mode = false;
      g_p2p_last_rx = 0;
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
                        (const uint8_t*)FRAME_END, sizeof(FRAME_END) - 1);
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

void setup_wifi() {
  WiFi.setSleep(false);

  // 从 NVS 读取上次保存的目标 WiFi；没有记录则用编译期默认
  g_prefs.begin("wifi", true);
  String _ssid = g_prefs.getString("ssid", String(STA_SSID));
  String _pass = g_prefs.getString("pass", String(STA_PASS));
  String _name = g_prefs.getString("name", String(AP_SSID));
  String _id   = g_prefs.getString("id",   default_dev_id());
  String _relay= g_prefs.getString("relay", RELAY_HOST_DEFAULT);   // "host:port", 空=关闭中继
  g_prefs.end();
  _ssid.toCharArray(g_sta_ssid, sizeof(g_sta_ssid));
  _pass.toCharArray(g_sta_pass, sizeof(g_sta_pass));
  _name.toCharArray(g_dev_name, sizeof(g_dev_name));
  _id.toCharArray(g_dev_id,   sizeof(g_dev_id));
  _relay.toCharArray(g_relay_host, sizeof(g_relay_host));
  if (g_relay_host[0] != 0) Dbg.printf("[中继] 目标: %s (id=%s)\n", g_relay_host, g_dev_id);

  // 凭据加载完再定模式: 无 STA 凭据(NVS 与编译期都没有)时开 SoftAP 兜底,
  // 保证配网页可达、设备不会变成"连不上也配不了"的死机;
  // 有凭据则保持纯 STA —— SoftAP 与 STA 时分复用同一射频, 会明显拉低视频吞吐。
  g_softap_on = (g_sta_ssid[0] == 0);
  if (g_softap_on) {
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(AP_SSID, AP_PASS);
    Dbg.printf("[WiFi] 无 STA 凭据, 已开兜底热点 '%s' 供配网(配好后自动关闭)\n", AP_SSID);
  } else {
    WiFi.mode(WIFI_STA);   // 纯 STA: 彻底关闭 SoftAP, 释放射频给 STA
    Dbg.println("[WiFi] 纯 STA 模式, SoftAP 已彻底关闭");
  }

  cam_server.begin();
  cam_udp.begin(TCP_PORT);
  Dbg.printf("[WiFi] TCP监听: %u\n", TCP_PORT);

  // Web 配置页(通过局域网 IP 访问)
  httpServer.on("/",         HTTP_GET, handle_web_root);
  httpServer.on("/save",     HTTP_GET, handle_web_save);
  httpServer.begin();
  Dbg.println("[WiFi] 配置页已开(局域网 IP:80)");

  if (g_sta_ssid[0] == 0) {
    Dbg.println("[WiFi] 未配置目标 WiFi, 等待通过配置页写入");
    return;
  }
  Dbg.printf("[WiFi] STA 连接 %s ...\n", g_sta_ssid);
  WiFi.begin(g_sta_ssid, g_sta_pass);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < 12000) {
    delay(200);
    Dbg.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    IPAddress ip = WiFi.localIP();
    Dbg.printf("\n[WiFi] STA已连: %s ip=%d.%d.%d.%d\n",
               g_sta_ssid, ip[0], ip[1], ip[2], ip[3]);
    if (g_softap_on) {           // 已入网, 关掉兜底热点把射频还给 STA
      WiFi.softAPdisconnect(true);
      g_softap_on = false;
      Dbg.println("[WiFi] 已入网, 兜底热点关闭");
    }
  } else {
    Dbg.printf("\n[WiFi] STA 连接超时, 设备离线%s\n",
               g_softap_on ? "(兜底热点已开, 可连入配网)" : "(需重连/重新配置)");
  }
}

// 周期打印本机 STA IP（上位机找板子局域网地址用）
void log_wifi_ip() {
  if (WiFi.status() == WL_CONNECTED) {
    uint32_t now = millis();
    if (now - g_last_ip_log >= 5000) {
      g_last_ip_log = now;
      IPAddress ip = WiFi.localIP();
      Dbg.printf("[WiFi] ip=%d.%d.%d.%d\n", ip[0], ip[1], ip[2], ip[3]);
    }
  }
}

// STA 断线自愈：setup_wifi 只在开机时 connect 一次，掉线后无 AP 兜底，
// 这里周期性检测并重连（非阻塞，连接结果由后续状态轮询确认）。
static uint32_t g_sta_last_try = 0;
void check_sta_link() {
  uint32_t now = millis();
  if (WiFi.status() == WL_CONNECTED) { g_sta_last_try = now; return; }
  if (g_sta_ssid[0] == 0) return;            // 未配置凭据: 由配网页触发连接
  if (now - g_sta_last_try < 15000) return;
  g_sta_last_try = now;
  Dbg.printf("[WiFi] STA 掉线(status=%d), 尝试重连...\n", WiFi.status());
  WiFi.disconnect();
  delay(50);
  WiFi.begin(g_sta_ssid, g_sta_pass);
}

void log_diag() {
  uint32_t now = millis();
  if (g_diag_last_ms == 0 || now - g_diag_last_ms < 5000) return;
  float dt = (now - g_diag_last_ms) / 1000.0f;
  float cap_fps = g_diag_cap_frames / dt;
  float sent_fps = g_diag_sent_frames / dt;
  float kbps = (g_diag_sent_bytes / 1024.0f) / dt;
  uint32_t avg_len = (g_diag_sent_frames > 0) ? (uint32_t)(g_diag_sent_bytes / g_diag_sent_frames) : 0;
  int rssi = WiFi.RSSI();
  // skp=真丢帧(入队失败/发送失败), evc=队列满主动顶掉的最旧帧(背压, 非故障)
  Dbg.printf("DIAG cap=%.1f/s sent=%.1f/s kb=%.0f/s avg=%uB uerr=%u rty=%u skp=%u evc=%u nc=%u err=%d q=%d auto=%d rssi=%d\n",
             cap_fps, sent_fps, kbps, avg_len,
             g_diag_udp_err, g_diag_udp_retry_ok, g_diag_skip_frames, g_diag_evict_frames,
             g_diag_nc_frames,
             g_diag_last_err, g_quality, g_auto_quality ? 1 : 0, rssi);
  g_diag_last_ms = now;
  g_diag_cap_frames = 0;
  g_diag_sent_frames = 0;
  g_diag_skip_frames = 0;
  g_diag_evict_frames = 0;
  g_diag_nc_frames = 0;
  g_diag_udp_err = 0;
  g_diag_udp_retry_ok = 0;
  g_diag_last_err = 0;
  g_diag_sent_bytes = 0;
  g_diag_cap_bytes = 0;
}

void adapt_quality() {
  // 原实现仅 udp_mode 时生效, 导致跨网(P2P/TURN/中继)路径下自适应画质完全不工作——
  // 跨网恰恰最需要动态码率调节。改为检测任意活跃发送通道。
  bool has_stream = udp_mode || g_p2p_active || g_turn_mode || g_relay_viewer;
  if (!g_auto_quality || !has_stream) return;
  // 原来 cur_frame < FRAMESIZE_VGA 直接 return, 导致默认 QVGA 下自适应画质完全不生效,
  // 与 README 描述不符。改为按分辨率选目标帧率; 下限保留 8(QVGA/CIF 模式下
  // quality 过低会触发传感器编码卡死无帧, 见 g_quality 初始化注释)。
  uint32_t now = millis();
  // 首次调用: 设基线后直接返回, 等下一个采样窗再评估。
  // 原实现 g_adapt_last_ms==0 时 return 但基线设置在函数末尾(return 之后),
  // 导致 g_adapt_last_ms 永远为 0, adapt_quality 实际从未生效。
  if (g_adapt_last_ms == 0) {
    g_adapt_last_ms    = now;
    g_adapt_last_sent  = g_adapt_total_frames;
    g_adapt_last_bytes = g_adapt_total_bytes;
    g_adapt_last_err   = g_adapt_total_err;
    return;
  }
  if (now - g_adapt_last_ms < 2500) return;

  float dt = (now - g_adapt_last_ms) / 1000.0f;
  uint32_t sent_delta = (uint32_t)(g_adapt_total_frames - g_adapt_last_sent);
  uint64_t bytes_delta = g_adapt_total_bytes - g_adapt_last_bytes;
  uint32_t err_delta   = g_adapt_total_err - g_adapt_last_err;
  float fps = sent_delta / dt;
  float kbps = (bytes_delta / 1024.0f) / dt;
  float target_fps;
  if (g_turn_mode || g_relay_viewer) {
    // 跨网路径 RTT 高、带宽受限, 目标帧率降低, 避免画质频繁震荡
    target_fps = (cur_frame >= FRAMESIZE_VGA) ? 5.0f : 12.0f;
  } else {
    target_fps = (cur_frame >= FRAMESIZE_SVGA) ? 8.0f
                 : (cur_frame >= FRAMESIZE_VGA)  ? 12.0f
                 : 25.0f;   // QQVGA/QVGA: 传感器上限 30fps
  }

  int new_q = g_quality;
  // 用独立累计的 err_delta 判定: 直接读 g_diag_udp_err 会被 log_diag 每 5s 清零,
  // 判定结果取决于采样点落在清零前后(时灵时不灵)。
  if (fps < target_fps - 2.0f && err_delta > 0 && g_quality < 35) {
    new_q = g_quality + 2;
  } else if (fps > target_fps + 1.5f && err_delta == 0 && kbps < 650.0f && g_quality > 8) {
    new_q = g_quality - 1;
  }

  if (new_q != g_quality) {
    g_quality = new_q;
    sensor_t *s = esp_camera_sensor_get();
    if (s) s->set_quality(s, g_quality);
    Dbg.printf("AUTOQ quality=%d fps=%.1f kb=%.0f/s err=%u\n", g_quality, fps, kbps, err_delta);
  }
  g_adapt_last_ms = now;
  g_adapt_last_sent = g_adapt_total_frames;
  g_adapt_last_bytes = g_adapt_total_bytes;
  g_adapt_last_err = g_adapt_total_err;
}
#endif

// 上位机命令统一处理（串口与 TCP 共用协议）
void handle_cmd(int c, Stream &in) {
  if (c == 'P' || c == 'p') {
    Dbg.println("CMD_PHOTO");
    camera_change_framesize(HI_FRAME);
    delay(500);
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
      if (!fb) { delay(50); attempt++; }
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
    PREVIEW_FRAME = framesize_from_code(code);
    camera_change_framesize(PREVIEW_FRAME);
    Dbg.printf("预览分辨率已设为: %c\n", code);
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
  }
}

void loop() {
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
      }
    }
    check_sta_link();
  }
#endif

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
  esp_camera_fb_return(fb);   // 无论是否入队都只归还一次
#ifdef ENABLE_WIFI
  cam_unlock();
  log_diag();
#endif
  delay(1);
}
