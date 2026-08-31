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
void update_ap_stream_state();
void handle_cmd(int c, Stream &in);
void enqueue_frame(camera_fb_t *fb);
void wait_tx_idle(uint32_t timeout_ms);
void sender_task(void *arg);
void handle_web_root();
void handle_web_save();
void disc_task(void *arg);
void relay_task(void *arg);
bool send_frame_relay(camera_fb_t *fb);
void p2p_punch();
void p2p_stun();
String default_dev_id();

// UART0 = CH343 转串口所在通道（TX=GPIO43, RX=GPIO44），避免默认 Serial 走 USB-CDC。
HardwareSerial Dbg(0);

// ---- WiFi 传输（STA 连入局域网 + AP 兜底 + TCP 服务端）----
#define ENABLE_WIFI 1
#ifdef ENABLE_WIFI
#define STA_SSID   "CMCC-Dr6M-2.4G"    // 家庭路由器
#define STA_PASS   "swc670143"
#define AP_SSID    "OV2640Cam"         // 兜底热点
#define AP_PASS    ""
#define TCP_PORT   3333
static WiFiServer  cam_server(TCP_PORT);
static WiFiClient  cam_client;
static uint32_t    g_last_ip_log = 0;

// ---- Web 配置页 + 目标 WiFi(NVS) ----
static char        g_sta_ssid[33] = STA_SSID;   // 运行时 STA 目标(可在配网页覆盖并持久化)
static char        g_sta_pass[64] = STA_PASS;
static Preferences g_prefs;
static WebServer   httpServer(80);

static WiFiUDP     cam_udp;            // UDP 预览通道(高帧率, 丢帧不重传)
static bool        udp_mode  = false;  // 上位机发 'U' 开启
static uint16_t    udp_seq   = 0;
static bool        g_ap_suspended = false;
static bool        g_auto_quality = true;
static uint8_t     g_udp_backoff_frames = 0;
static uint32_t    g_diag_last_ms = 0;
static uint32_t    g_diag_cap_frames = 0;
static uint32_t    g_diag_sent_frames = 0;
static uint32_t    g_diag_skip_frames = 0;
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
// 否则差分遇到清零会产生下溢/0值，画质调节半失效
static uint64_t    g_adapt_total_frames = 0;
static uint64_t    g_adapt_total_bytes  = 0;

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
  uint8_t *buf;        // PSRAM 堆拷贝，发送侧释放
  size_t   len;
  int      fmt;        // pixformat_t 原值(JPEG/RGB)
  uint16_t w, h;
};
#define TXQ_DEPTH      4          // ~XGA 50KB/帧 -> 最坏积压 200KB PSRAM
static QueueHandle_t       g_txq       = NULL;
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
static int         g_quality     = 10;   // JPEG 质量(0-63, 越小画质越高文件越大).
// 注意: QVGA(CIF模式)下 quality 太低(如2)会卡死无帧(传感器编码bug), 默认10最稳(30fps).
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
// 通过 UDP 把一整帧分片成多个数据报发送（预览用，丢帧不重传）。
// 数据报布局: 12B头 + 分片。
//   头: "OVU"(3) fmt(1) seq(2) cnt_minus1(1) idx(1) w(2) h(2)  fmt:0=RGB565 1=JPEG
bool send_frame_udp(camera_fb_t *fb, IPAddress dst, uint16_t port = TCP_PORT) {
  const size_t HDR   = 12;
  const size_t CHUNK = 1400 - HDR;     // 单报文净荷, 避开 MTU 分片
  const uint8_t *buf = fb->buf;
  size_t len = fb->len;
  size_t nchunk = (len + CHUNK - 1) / CHUNK;
  if (nchunk > 250) nchunk = 250;      // idx/cnt 是 u8
  uint8_t hdr[HDR];
  hdr[0] = 'O'; hdr[1] = 'V'; hdr[2] = 'U';
  hdr[3] = (fb->format == PIXFORMAT_JPEG) ? 1 : 0;  // 格式: 0=RGB565 1=JPEG
  hdr[4] = (uint8_t)(udp_seq >> 8);
  hdr[5] = (uint8_t)(udp_seq & 0xFF);
  hdr[6] = (uint8_t)(nchunk - 1);
  // hdr[7] = idx
  hdr[8]  = (uint8_t)(fb->width >> 8);
  hdr[9]  = (uint8_t)(fb->width & 0xFF);
  hdr[10] = (uint8_t)(fb->height >> 8);
  hdr[11] = (uint8_t)(fb->height & 0xFF);
  const bool small_frame = (nchunk <= 4);
  const int max_retry = small_frame ? 6 : 1;
  const int retry_delay_ms = small_frame ? 2 : 1;
  size_t off = 0;
  for (size_t i = 0; i < nchunk && off < len; i++) {
    size_t n = len - off;
    if (n > CHUNK) n = CHUNK;
    hdr[7] = (uint8_t)i;
    bool sent = false;
    for (int t = 0; t <= max_retry; t++) {
      cam_udp.beginPacket(dst, port);
      cam_udp.write(hdr, HDR);
      cam_udp.write(buf + off, n);
      int rc = cam_udp.endPacket();
      if (rc == 1) {
        if (t > 0) g_diag_udp_retry_ok++;
        sent = true;
        break;
      }
      g_diag_udp_err++;
      g_diag_last_err = -1;
      if (t < max_retry) delay(retry_delay_ms);
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
#endif
  esp_camera_deinit();
  delay(300);
#ifdef ENABLE_WIFI
  g_switching = false;
#endif
  return init_camera_as(cur_frame, PREVIEW_FRAME);
}

// 真正的分辨率切换：deinit + init 全量重配，失败自动回退
bool camera_change_framesize(framesize_t fs) {
  if (fs == cur_frame) { return true; }
#ifdef ENABLE_WIFI
  g_switching = true;
  wait_tx_idle(500);            // 大分辨率下最后一帧可能近100ms, 留足排空时间
#endif
  esp_camera_deinit();
  delay(200);
  bool ok = init_camera_as(fs, cur_frame);
#ifdef ENABLE_WIFI
  g_switching = false;
#endif
  return ok;
}

#ifdef ENABLE_WIFI
// 抓帧 -> PSRAM 副本入队，立即归还驱动缓冲。队列满/拷贝失败按丢帧计数背压。
// 在 loop 任务执行, 只做 memcpy 级工作(~7KB@QVGA), 不做任何阻塞发送。
void enqueue_frame(camera_fb_t *fb) {
  TxFrame t;
  t.buf = (uint8_t*)heap_caps_malloc(fb->len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!t.buf) { g_diag_skip_frames++; g_diag_last_err = -3; return; }
  memcpy(t.buf, fb->buf, fb->len);
  t.len = fb->len;
  t.fmt = fb->format;
  t.w = fb->width;
  t.h = fb->height;
  if (xQueueSend(g_txq, &t, 0) != pdTRUE) {
    heap_caps_free(t.buf);
    g_diag_skip_frames++;              // 队列满=网络消费不动, 主动丢新帧保采集节拍
    return;
  }
  g_diag_cap_bytes += fb->len;
}

// 发送任务(核0)：出队 -> 按当前模式选通道 -> 记账 -> 释放。
// 全程持 g_cli_mutex 与 loop 的 accept/stop/P 路径互斥; 单帧最长阻塞被天然限制在一帧内。
void sender_task(void *arg) {
  TxFrame t;
  camera_fb_t shim;
  IPAddress dst;
  for (;;) {
    if (xQueueReceive(g_txq, &t, portMAX_DELAY) != pdTRUE) continue;
    g_inflight++;

    if (xSemaphoreTake(g_cli_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
      memset(&shim, 0, sizeof(shim));
      shim.buf = t.buf; shim.len = t.len;
      shim.width = t.w; shim.height = t.h; shim.format = (pixformat_t)t.fmt;
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
      } else if (g_p2p_active) {
        // P2P 直连已建立: 视频经 UDP 直发对端(观众), 不再走中继
        if (g_udp_backoff_frames > 0) {
          g_udp_backoff_frames--;
          g_diag_skip_frames++;
        } else if (send_frame_udp(&shim, g_p2p_peer_ip, g_p2p_peer_port)) {
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
      xSemaphoreGive(g_cli_mutex);
    } else {
      g_diag_skip_frames++;            // 拿不到锁(P 正占用客户端), 宁丢一帧不死等
    }

    heap_caps_free(t.buf);
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
  g_cli_mutex = xSemaphoreCreateMutex();
  // 核0 与 lwIP/WiFi 任务共核但优先级低本任务; 发送是大块 memcpy+socket 写,
  // 放核0 让它和采集(DMA中断)抢的同时彻底不碰 loopTask(核1) 的控制响应。
  xTaskCreatePinnedToCore(sender_task, "tx", 4096, NULL, 3, NULL, 0);
  g_relay_tx = xSemaphoreCreateMutex();
  xTaskCreate(disc_task,  "disc",  2048, NULL, 1, NULL);   // 局域网自动发现广播
  xTaskCreate(relay_task, "relay", 3072, NULL, 1, NULL);   // 跨网中继客户端(常驻在线)
  g_diag_last_ms = millis();
  g_adapt_last_ms = millis();
#endif
}

#ifdef ENABLE_WIFI
// Web 配置页：连上 OV2640Cam 热点后浏览器打开 192.168.4.1 使用。
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
    + "<tr><td>热点</td><td>" + String(AP_SSID) + " (192.168.4.1)</td></tr>"
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
    + "<p style='color:#888'>保存后摄像头将尝试连接该 WiFi。连接成功后既可通过本热点(192.168.4.1)，"
    + "也可通过目标网络的局域网 IP 访问摄像头。</p></body></html>";
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

  // 立即按新目标重连 STA；若成功则摄像头入网，热点在流媒体时自动让位(ST双链路)
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
void relay_send_payload(const uint8_t *data, size_t len) {
  if (!relay_cli || !relay_cli.connected()) return;
  xSemaphoreTake(g_relay_tx, portMAX_DELAY);
  uint8_t hdr[4] = { (uint8_t)(len >> 24), (uint8_t)(len >> 16),
                     (uint8_t)(len >> 8),  (uint8_t)len };
  relay_cli.write(hdr, 4);
  size_t off = 0;
  while (off < len) {
    size_t s = relay_cli.write(data + off, len - off);
    if (s == 0) break;
    off += s;
    if (off < len) delay(1);
  }
  xSemaphoreGive(g_relay_tx);
}

// P2P 打洞: 向对端公网端点发包, 并检测对端回包(打洞成功则置 g_p2p_active)。
void p2p_punch() {
  if (g_p2p_peer_port == 0) return;
  // 向对端发打洞包(内容任意, 目的让双方 NAT 建立映射)
  cam_udp.beginPacket(g_p2p_peer_ip, g_p2p_peer_port);
  cam_udp.write((const uint8_t *)"P2P", 3);
  cam_udp.endPacket();
  // 检测对端回包: 收到来自对端 IP 的 UDP 包 => 打洞成功
  int n = cam_udp.parsePacket();
  if (n > 0) {
    IPAddress src = cam_udp.remoteIP();
    if (src == g_p2p_peer_ip) {
      g_p2p_active = true;
      Dbg.printf("[P2P] 打洞成功, 直连 %s:%u\n",
                 g_p2p_peer_ip.toString().c_str(), g_p2p_peer_port);
    }
  }
}

// 向服务器 UDP STUN 端口上报自身公网端点(打洞辅助)。服务器据此撮合双方。
void p2p_stun() {
  if (g_relay_host[0] == 0) return;
  char h[64] = {0}; int port = 8000;
  sscanf(g_relay_host, "%63[^:]:%d", h, &port);
  IPAddress srv;
  if (!srv.fromString(h)) return;
  cam_udp.beginPacket(srv, port + 1);   // UDP STUN 端口 = 中继 TCP 端口 + 1
  char msg[96];
  int n = snprintf(msg, sizeof(msg), "{\"t\":\"stun\",\"id\":\"%s\",\"role\":\"cam\"}",
                   g_dev_id);
  cam_udp.write((const uint8_t*)msg, n);
  cam_udp.endPacket();
}

// 中继: 读一条完整消息; 返回 false 表示断连/出错。
bool relay_read_msg() {
  relay_cli.setTimeout(1500);
  uint8_t hdr[4];
  if (relay_cli.readBytes(hdr, 4) != 4) return false;
  uint32_t len = ((uint32_t)hdr[0] << 24) | ((uint32_t)hdr[1] << 16)
               | ((uint32_t)hdr[2] << 8) | hdr[3];
  if (len == 0 || len > (1 << 20)) return false;      // 非法长度防御
  uint8_t *buf = (uint8_t*)malloc(len);
  if (!buf) return false;
  bool ok = (relay_cli.readBytes(buf, len) == (int)len);
  if (ok && buf[0] == '{' && memchr(buf, '}', len)) {
    char s[160]; int cn = len; if (cn > 159) cn = 159;
    memcpy(s, buf, cn); s[cn] = 0;
    if      (strstr(s, "viewer_attached")) g_relay_viewer = true;
    else if (strstr(s, "viewer_left"))     g_relay_viewer = false;
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
  free(buf);
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
          // P2P 打洞: 有对端但未建立时周期性发包; 超时(5s)放弃保持中继
          if (g_p2p_peer_port != 0 && !g_p2p_active) {
            if (now - g_p2p_punch_ms >= 100) {
              p2p_punch();
              g_p2p_punch_ms = now;
              if (now - g_p2p_punch_start >= 5000) {
                Dbg.println("[P2P] 打洞超时, 保持中继");
                g_p2p_peer_port = 0;
              }
            }
          }
          if (!relay_cli.connected()) conn = false;
          if (conn) delay(10);
        }
      }
      g_relay_online = false;
      g_relay_viewer = false;
      g_p2p_active = false;
      g_p2p_peer_port = 0;
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
bool send_frame_relay(camera_fb_t *fb) {
  if (!g_relay_viewer || !relay_cli || !relay_cli.connected()) return false;
  char head[64];
  int hn = snprintf(head, sizeof(head), "FRAME_BEGIN\nSIZE %u %u %s\n",
                    fb->width, fb->height,
                    (fb->format == PIXFORMAT_JPEG) ? "JPG" : "RGB");
  size_t endn = sizeof(FRAME_END) - 1;
  size_t total = hn + fb->len + endn;
  uint8_t *blob = (uint8_t*)heap_caps_malloc(total, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!blob) return false;
  memcpy(blob, head, hn);
  memcpy(blob + hn, fb->buf, fb->len);
  memcpy(blob + hn + fb->len, FRAME_END, endn);
  relay_send_payload(blob, total);
  heap_caps_free(blob);
  return true;
}

void setup_wifi() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);
  WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
  WiFi.softAP(AP_SSID, AP_PASS);
  Dbg.printf("[WiFi] 兜底AP已开: %s\n", AP_SSID);

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

  cam_server.begin();
  cam_udp.begin(TCP_PORT);
  Dbg.printf("[WiFi] TCP监听: %u\n", TCP_PORT);

  // Web 配置页(连上热点后浏览器打开 192.168.4.1)
  httpServer.on("/",         HTTP_GET, handle_web_root);
  httpServer.on("/save",     HTTP_GET, handle_web_save);
  httpServer.begin();
  Dbg.println("[WiFi] 配置页已开: http://192.168.4.1/");

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
  } else {
    Dbg.printf("\n[WiFi] STA 连接超时, 仅 AP 可用 ip=192.168.4.1\n");
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

// STA 断线自愈：setup_wifi 只在开机时 connect 一次，掉线后原本仅靠 AP 兜底。
// 这里周期性检测并重连（非阻塞，连接结果由后续状态轮询确认）。
static uint32_t g_sta_last_try = 0;
void check_sta_link() {
  uint32_t now = millis();
  if (WiFi.status() == WL_CONNECTED) { g_sta_last_try = now; return; }
  if (now - g_sta_last_try < 15000) return;
  g_sta_last_try = now;
  Dbg.printf("[WiFi] STA 掉线(status=%d), 尝试重连...\n", WiFi.status());
  WiFi.disconnect();
  delay(50);
  WiFi.begin(g_sta_ssid, g_sta_pass);
}

void update_ap_stream_state() {
  const bool has_sta = (WiFi.status() == WL_CONNECTED);
  bool streaming = false;
  if (xSemaphoreTake(g_cli_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    streaming = (cam_client && cam_client.connected());
    xSemaphoreGive(g_cli_mutex);
  }
  if (has_sta && streaming && !g_ap_suspended) {
    WiFi.softAPdisconnect(true);
    g_ap_suspended = true;
    Dbg.println("[WiFi] 流媒体中暂停 AP, 优先 STA 带宽");
  } else if ((!streaming || !has_sta) && g_ap_suspended) {
    WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
    WiFi.softAP(AP_SSID, AP_PASS);
    g_ap_suspended = false;
    Dbg.println("[WiFi] 已恢复 AP");
  }
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
  Dbg.printf("DIAG cap=%.1f/s sent=%.1f/s kb=%.0f/s avg=%uB uerr=%u rty=%u skp=%u nc=%u err=%d q=%d auto=%d ap=%d rssi=%d\n",
             cap_fps, sent_fps, kbps, avg_len,
             g_diag_udp_err, g_diag_udp_retry_ok, g_diag_skip_frames, g_diag_nc_frames,
             g_diag_last_err, g_quality, g_auto_quality ? 1 : 0, g_ap_suspended ? 0 : 1, rssi);
  g_diag_last_ms = now;
  g_diag_cap_frames = 0;
  g_diag_sent_frames = 0;
  g_diag_skip_frames = 0;
  g_diag_nc_frames = 0;
  g_diag_udp_err = 0;
  g_diag_udp_retry_ok = 0;
  g_diag_last_err = 0;
  g_diag_sent_bytes = 0;
  g_diag_cap_bytes = 0;
}

void adapt_quality() {
  if (!g_auto_quality || !udp_mode) return;
  if (cur_frame < FRAMESIZE_VGA) return;
  uint32_t now = millis();
  if (g_adapt_last_ms == 0 || now - g_adapt_last_ms < 2500) return;

  float dt = (now - g_adapt_last_ms) / 1000.0f;
  uint32_t sent_delta = (uint32_t)(g_adapt_total_frames - g_adapt_last_sent);
  uint64_t bytes_delta = g_adapt_total_bytes - g_adapt_last_bytes;
  float fps = sent_delta / dt;
  float kbps = (bytes_delta / 1024.0f) / dt;
  float target_fps = (cur_frame >= FRAMESIZE_VGA) ? 12.0f : 20.0f;

  int new_q = g_quality;
  if (fps < target_fps - 2.0f && g_diag_udp_err > 0 && g_quality < 35) {
    new_q = g_quality + 2;
  } else if (fps > target_fps + 1.5f && g_diag_udp_err == 0 && kbps < 650.0f && g_quality > 8) {
    new_q = g_quality - 1;
  }

  if (new_q != g_quality) {
    g_quality = new_q;
    sensor_t *s = esp_camera_sensor_get();
    if (s) s->set_quality(s, g_quality);
    Dbg.printf("AUTOQ quality=%d fps=%.1f kb=%.0f/s\n", g_quality, fps, kbps);
  }
  g_adapt_last_ms = now;
  g_adapt_last_sent = g_adapt_total_frames;
  g_adapt_last_bytes = g_adapt_total_bytes;
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
    while (!fb && attempt < 10) {
      fb = esp_camera_fb_get();
      if (!fb) { delay(50); attempt++; }
    }
    if (!fb) {
      Dbg.println("拍照失败：esp_camera_fb_get 返回空");
      camera_reconnect();
    } else {
      bool sent_via_tcp = false;
#ifdef ENABLE_WIFI
      sent_via_tcp = send_frame(fb);   // 内部持锁走可靠 TCP, 上位机在 TCP 流中解照片帧
#endif
      if (!sent_via_tcp) send_frame_serial(fb);   // 无客户端或 TCP 中断则串口兜底
      esp_camera_fb_return(fb);
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
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
      Dbg.printf("DIAG fmt=%d len=%u %ux%u\n", (int)fb->format, (unsigned)fb->len,
                 fb->width, fb->height);
      esp_camera_fb_return(fb);
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
  httpServer.handleClient();   // Web 配置页：连上热点后浏览器打开 192.168.4.1
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
    update_ap_stream_state();
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
  camera_fb_t *fb = NULL;
  int attempt = 0;
  while (!fb && attempt < 3) {
    fb = esp_camera_fb_get();
    if (!fb) { delay(30); attempt++; }
  }
  if (!fb) {
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
  log_diag();
#endif
  delay(1);
}
