/*
 * ESP32-S3 + OV2640 最小验证工程（Arduino）
 *
 * 功能：初始化摄像头 -> 拍一帧 JPEG -> 通过串口把整张 JPEG 发给电脑。
 * 电脑端用 host/capture_serial.py 接收并存成 jpg 文件查看。
 *
 * 接线（OV2640 -> ESP32-S3）：
 *   3.3V -> 3V3,  GND -> GND,
 *   SCL -> GPIO5,  SDA -> GPIO4,
 *   VSYNC -> GPIO6, HREF -> GPIO7, PCLK(DCLK) -> GPIO13, XCLK -> GPIO15,
 *   D7 -> 16, D6 -> 17, D5 -> 18, D4 -> 2, D3 -> 10, D2 -> 8, D1 -> 9, D0 -> 1,
 *   PWDN -> GND（必须拉低使能）, RST 悬空或接 3.3V 均可。
 *
 * 串口协议：
 *   每帧输出两行文本 + JPEG 二进制：
 *     FRAME_BEGIN
 *     <JPEG 原始字节>
 *     FRAME_END
 */

#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiServer.h>
#include <WiFiUdp.h>

// UART0 = CH343 转串口所在通道（TX=GPIO43, RX=GPIO44）。
// 用 HardwareSerial 强制输出到 UART0，避免默认 Serial 走了 USB-CDC。
HardwareSerial Dbg(0);

// ---- WiFi 传输（STA 连入局域网 + AP 兜底 + TCP 服务端，摆脱串口带宽瓶颈）----
#define ENABLE_WIFI 1
#ifdef ENABLE_WIFI
#define STA_SSID   "CMCC-Dr6M-2.4G"    // 家庭路由器（STA 模式连入，电脑可直接访问局域网 IP）
#define STA_PASS   "swc670143"
#define AP_SSID   "OV2640Cam"      // 兜底热点名（STA 连不上时，手机/电脑可连此热点）
#define AP_PASS   ""               // 空密码（裸机调试用，注意安全）
#define TCP_PORT  3333
static WiFiServer  cam_server(TCP_PORT);
static WiFiClient  cam_client;     // 当前接入的 TCP 客户端
static uint32_t    g_last_ip_log = 0;   // 上次打印本机 IP 的时间(ms)
#endif

#ifdef ENABLE_WIFI
static WiFiUDP     cam_udp;            // UDP 预览通道(高帧率, 丢帧不重传)
static bool        udp_mode  = false;  // 由上位机发 'U' 开启 UDP 预览(默认关, 不破坏 TCP/串口基线)
static uint16_t    udp_seq   = 0;      // UDP 帧序号(丢帧/乱序检测)
#endif

// ---- 引脚配置（esp32-camera 官方 ESP32-S3 示例脚位）----
#define PWDN_GPIO_NUM    -1   // PWDN 已硬件拉低，代码不控制
#define RESET_GPIO_NUM   -1   // 不控制复位
#define XCLK_GPIO_NUM    15
#define SIOD_GPIO_NUM    4    // SDA
#define SIOC_GPIO_NUM    5    // SCL
#define D7_GPIO_NUM      16
#define D6_GPIO_NUM      17
#define D5_GPIO_NUM      18
#define D4_GPIO_NUM      2    // 原12, 疑似引脚损坏, 换到GPIO2
#define D3_GPIO_NUM      10
#define D2_GPIO_NUM      8
#define D1_GPIO_NUM      9
#define D0_GPIO_NUM      1    // 原11, 疑似引脚损坏, 换到GPIO1
#define VSYNC_GPIO_NUM   6
#define HREF_GPIO_NUM    7
#define PCLK_GPIO_NUM    13

// ---- UART0 调试口（对应 CH343 的 TX/RX）----
#define TX_GPIO_NUM    43
#define RX_GPIO_NUM    44

// ---- 帧头/帧尾（供上位机脚本识别一帧的边界）----
static const char FRAME_BEGIN[] = "FRAME_BEGIN\n";
static const char FRAME_END[]   = "FRAME_END\n";

// 运行时可切换的分辨率
static framesize_t PREVIEW_FRAME = FRAMESIZE_QQVGA; // 预览 160x120
static framesize_t HI_FRAME      = FRAMESIZE_UXGA;  // 拍照 1600x1200
static framesize_t cur_frame     = FRAMESIZE_QQVGA;

// 摄像头配置全局保存，供失锁后 deinit+reinit 复用
static camera_config_t g_config;

// RGB565 每字节
#define BYTES_PER_PIXEL 2
// PSRAM 缓冲预算上限（留足余量，避免 8MB 被双缓冲或系统开销挤爆）
#define MAX_FB_BYTES (6 * 1024 * 1024)   // 6MB

// 依据分辨率自适应双缓冲数量：
// 每帧太大时(AUXGA/UXGA)只分配单缓冲，否则双缓冲。防 PSRAM OOM 卡死。
int fb_count_for(framesize_t fs) {
  if (fs < 0 || fs >= FRAMESIZE_INVALID) return 1;
  int w = resolution[fs].width;
  int h = resolution[fs].height;
  int64_t one = (int64_t)w * h * BYTES_PER_PIXEL;
  int64_t two = one * 2;
  if (two <= MAX_FB_BYTES) return 2;
  return 1;
}

// 发送一帧原始数据(带尺寸头)。优先走 WiFi TCP(客户端已连接)，失败则断开并回退串口。
void send_frame(camera_fb_t *fb) {
#ifdef ENABLE_WIFI
  if (cam_client && cam_client.connected()) {
    cam_client.write((const uint8_t*)FRAME_BEGIN, sizeof(FRAME_BEGIN) - 1);
    char head[48];
    snprintf(head, sizeof(head), "SIZE %u %u %s\n", fb->width, fb->height,
             (fb->format == PIXFORMAT_JPEG) ? "JPG" : "RGB");
    cam_client.write((const uint8_t*)head, strlen(head));
    // 分块发送避免单次大 write 撑爆 lwIP；任一分块失败则整帧回退串口（不丢帧）
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
      return;
    }
    Dbg.println("TCP 发送失败, 回退串口并重置客户端");
    cam_client.stop();
  }
#endif
  Dbg.write(FRAME_BEGIN, sizeof(FRAME_BEGIN) - 1);
  Dbg.printf("SIZE %u %u %s\n", fb->width, fb->height,
             (fb->format == PIXFORMAT_JPEG) ? "JPG" : "RGB");
  Dbg.write(fb->buf, fb->len);
  Dbg.write(FRAME_END, sizeof(FRAME_END) - 1);
}

#ifdef ENABLE_WIFI
// 通过 UDP 把一整帧分片成多个数据报发送(预览用, 丢帧不重传 => 高帧率)。
// 数据报布局: 12B 头 + 分片。
//   头: "OVU"(3) ver(1) seq_hi(1) seq_lo(1) cnt_minus1(1) idx(1) w_hi(1) w_lo(1) h_hi(1) h_lo(1)
//   对应上位机 host/udp_capture.py 的 udp 解析。
void send_frame_udp(camera_fb_t *fb, IPAddress dst) {
  const size_t HDR   = 12;
  const size_t CHUNK = 1400 - HDR;     // 单报文净荷, 避开 MTU 分片
  const uint8_t *buf = fb->buf;
  size_t len = fb->len;
  size_t nchunk = (len + CHUNK - 1) / CHUNK;
  if (nchunk > 250) nchunk = 250;      // 头里 idx/cnt 是 u8, 0..FF
  uint8_t hdr[HDR];
  hdr[0] = 'O'; hdr[1] = 'V'; hdr[2] = 'U';
  hdr[3] = 0;                          // 版本
  hdr[4] = (uint8_t)(udp_seq >> 8);
  hdr[5] = (uint8_t)(udp_seq & 0xFF);
  hdr[6] = (uint8_t)(nchunk - 1);
  // hdr[7] = idx, 每片覆写
  hdr[8]  = (uint8_t)(fb->width >> 8);
  hdr[9]  = (uint8_t)(fb->width & 0xFF);
  hdr[10] = (uint8_t)(fb->height >> 8);
  hdr[11] = (uint8_t)(fb->height & 0xFF);
  size_t off = 0;
  for (size_t i = 0; i < nchunk && off < len; i++) {
    size_t n = len - off;
    if (n > CHUNK) n = CHUNK;
    hdr[7] = (uint8_t)i;
    cam_udp.beginPacket(dst, TCP_PORT);
    cam_udp.write(hdr, HDR);
    cam_udp.write(buf + off, n);
    cam_udp.endPacket();
    off += n;
  }
  udp_seq++;
}
#endif

// 分辨率字符码 -> framesize_t。支持的码：q=QQVGA,t=QVGA,v=VGA,s=SVGA,x=XGA,u=UXGA
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

// 每次 esp_camera_init 后调用：deinit+init 会复位传感器寄存器，需重设画质参数
void apply_color_calib() {
  sensor_t *s = esp_camera_sensor_get();
  if (!s) { Dbg.println("警告：获取 sensor 失败，无法校准"); return; }
  s->set_whitebal(s, 1);      // 自动白平衡 AWB
  s->set_gain_ctrl(s, 1);     // 自动增益
  s->set_exposure_ctrl(s, 1);
  delay(200);                 // 给自动曝光/白平衡收敛时间
}

// 用全局配置以目标分辨率重建摄像头；返回是否成功
// 失败时自动降级：高分辨率先强制单缓冲，再失败则回退到给定 fallback 分辨率
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
  // 降级：单缓冲重试
  g_config.fb_count = 1;
  err = esp_camera_init(&g_config);
  if (err == ESP_OK) {
    cur_frame = target;
    Dbg.printf("摄像头初始化(单缓冲): fs=%u fb=1\n", (unsigned)target);
    apply_color_calib();
    return true;
  }
  // 再失败：回退到 fallback 分辨率，保证相机不处于死锁状态
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

// 失锁后重新初始化摄像头（先 deinit 释放 DMA/缓冲，再重建为当前分辨率）
bool camera_reconnect() {
  Dbg.println("摄像头失锁, 正在重连...");
  esp_camera_deinit();
  delay(300);
  return init_camera_as(cur_frame, PREVIEW_FRAME);
}

// 真正的分辨率切换：deinit + init 全量重配 DMA 缓冲，失败自动回退
bool camera_change_framesize(framesize_t fs) {
  if (fs == cur_frame) { return true; }  // 同分辨率无需切换
  esp_camera_deinit();
  delay(200);
  // 目标失败则回退到当前已生效(或预览)分辨率，避免卡死在坏状态
  return init_camera_as(fs, cur_frame);
}

void setup() {
  Dbg.begin(2000000, SERIAL_8N1, RX_GPIO_NUM, TX_GPIO_NUM);  // 2M波特率: UXGA大图传输(~19s/帧)
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
  config.xclk_freq_hz = 24000000;           // 24MHz(OV2640上限): 提高采集帧率上限
  config.pixel_format = PIXFORMAT_RGB565;    // RGB565 稳定基线（预编译库 JPEG 空帧，不可用）
  config.frame_size   = PREVIEW_FRAME;     // 初始：QQVGA 预览
  config.fb_count     = fb_count_for(PREVIEW_FRAME);  // 自适应双缓冲(高分辨率降为1)
  config.fb_location  = CAMERA_FB_IN_PSRAM; // 8MB PSRAM 帧缓冲(UXGA 3.84MB)

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Dbg.printf("摄像头初始化失败, err=0x%x\n", err);
    while (true) { delay(100); }
  }
  Dbg.println("摄像头初始化成功");
  Dbg.println("画质校准(最小)：仅 AWB+AGC");
  apply_color_calib();

#ifdef ENABLE_WIFI
  setup_wifi();
#endif
}

#ifdef ENABLE_WIFI
// 启动 WiFi：STA 连局域网(家庭路由) + 兜底 AP 热点，再开 TCP 服务端。
void setup_wifi() {
  WiFi.mode(WIFI_AP_STA);
  // 兜底热点(STA 连不上或想直连时可连它)
  WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
  WiFi.softAP(AP_SSID, AP_PASS);
  Dbg.printf("[WiFi] 兜底AP已开: %s\n", AP_SSID);
  cam_server.begin();
  cam_udp.begin(TCP_PORT);
  Dbg.printf("[WiFi] TCP监听: %u\n", TCP_PORT);

  // STA 连入局域网（家庭路由），非阻塞等待几秒
  Dbg.printf("[WiFi] STA 连接 %s ...\n", STA_SSID);
  WiFi.begin(STA_SSID, STA_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < 12000) {
    delay(200);
    Dbg.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    IPAddress ip = WiFi.localIP();
    Dbg.printf("\n[WiFi] STA已连: %s ip=%d.%d.%d.%d\n",
               STA_SSID, ip[0], ip[1], ip[2], ip[3]);
  } else {
    Dbg.printf("\n[WiFi] STA 连接超时, 仅 AP 可用 ip=192.168.4.1\n");
  }
}

// 周期打印本机 STA IP（上位机需知道板子局域网地址；上电后串口即可读到）
void log_wifi_ip() {
#ifdef ENABLE_WIFI
  if (WiFi.status() == WL_CONNECTED) {
    uint32_t now = millis();
    if (now - g_last_ip_log >= 5000) {
      g_last_ip_log = now;
      IPAddress ip = WiFi.localIP();
      Dbg.printf("[WiFi] ip=%d.%d.%d.%d\n", ip[0], ip[1], ip[2], ip[3]);
    }
  }
#endif
}
#endif

// 上位机命令统一处理（串口与 TCP 共用同一套协议）
void handle_cmd(int c, Stream &in) {
  if (c == 'P' || c == 'p') {
    // 拍照：切换到 HI_FRAME 抓一帧高清图发送，再切回预览
    Dbg.println("CMD_PHOTO");
    camera_change_framesize(HI_FRAME);
    delay(500); // 高分辨率全量重配稳定时间更长
    int attempt = 0;
    camera_fb_t *fb = NULL;
    while (!fb && attempt < 10) {   // 拍照时也带重试
      fb = esp_camera_fb_get();
      if (!fb) { delay(50); attempt++; }
    }
    if (!fb) {
      Dbg.println("拍照失败：esp_camera_fb_get 返回空");
      camera_reconnect();
    } else {
      send_frame(fb);
      esp_camera_fb_return(fb);
    }
    camera_change_framesize(PREVIEW_FRAME);
    Dbg.println("CMD_PHOTO_DONE");
  } else if (c == 'v' || c == 'V') {
    long t0 = millis();
    while (millis() - t0 < 200 && !in.available()) delay(1);
    char code = (char)in.read();   // 从同一数据源读第二个字节: V<code>
    PREVIEW_FRAME = framesize_from_code(code);
    camera_change_framesize(PREVIEW_FRAME);
    Dbg.printf("预览分辨率已设为: %c\n", code);
  } else if (c == 'h' || c == 'H') {
    long t0 = millis();
    while (millis() - t0 < 200 && !in.available()) delay(1);
    char code = (char)in.read();   // 从同一数据源读第二个字节: H<code>
    HI_FRAME = framesize_from_code(code);
    Dbg.printf("拍照分辨率已设为: %c (下次拍照生效)\n", code);
  } else if (c == 'd') {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
      if (fb->format == PIXFORMAT_RGB565) {
        uint32_t rD = 0, gD = 0, bD = 0, n = 0;
        uint16_t *p = (uint16_t*)fb->buf;
        uint32_t px = fb->width * fb->height;
        uint32_t step = px / 6000; if (step < 1) step = 1;
        for (uint32_t i = 0; i < px; i += step) {
          uint16_t v = p[i];
          uint8_t r = (v >> 11) & 0x1F, g = (v >> 5) & 0x3F, b = v & 0x1F;
          if (r > g && r > b) rD++; else if (g > r && g > b) gD++; else if (b > r && b > g) bD++;
          n++;
        }
        Dbg.printf("DIAG px=%u R=%u G=%u B=%u\n", n, rD, gD, bD);
      } else {
        Dbg.printf("DIAG(len=%u) 仅支持 RGB565, 当前 %s\n", (unsigned)fb->len,
                   (fb->format == PIXFORMAT_JPEG) ? "JPEG" : "other");
      }
      esp_camera_fb_return(fb);
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
  log_wifi_ip();   // 周期打印 STA IP，便于找到板子局域网地址
  // ---- WiFi：无有效客户端时就尝试 accept 最新连接；已断开的连接及时重置 ----
#ifdef ENABLE_WIFI
  if (cam_client && !cam_client.connected()) {
    cam_client.stop();
    cam_client = WiFiClient();   // 置回无效，允许重新 accept
  }
  if (!cam_client) {
    WiFiClient s = cam_server.available();
    if (s) {
      cam_client = s;
      cam_client.setNoDelay(true);
      cam_client.setTimeout(500);
      Dbg.println("WiFi 客户端已接入");
    }
  }
#endif

  // ---- 命令：串口 + TCP 都读，统一处理 ----
  while (Dbg.available()) handle_cmd(Dbg.read(), Dbg);
#ifdef ENABLE_WIFI
  if (cam_client && cam_client.connected()) {
    while (cam_client.available()) handle_cmd(cam_client.read(), cam_client);
  }
#endif

  // ---- 默认：实时预览（连续抓帧发送；失锁自动重连）----
  camera_fb_t *fb = NULL;
  int attempt = 0;
  while (!fb && attempt < 3) {
    fb = esp_camera_fb_get();
    if (!fb) { delay(30); attempt++; }
  }
  if (!fb) {
    static uint32_t fail_count = 0;
    fail_count++;
    if (fail_count >= 2) {   // 连续多次失败才重连，避免频繁打断
      fail_count = 0;
      camera_reconnect();
    }
    return;
  }
  send_frame_wrapper(fb);
  esp_camera_fb_return(fb);
  delay(1);  // 预览帧率由采集/传输决定，尽量少让出 CPU
}

// 发送一帧预览：打开 UDP 时走 UDP(高帧率)，否则回退 TCP/串口(稳定)。
void send_frame_wrapper(camera_fb_t *fb) {
#ifdef ENABLE_WIFI
  if (udp_mode && cam_client && cam_client.connected())
    send_frame_udp(fb, cam_client.remoteIP());
  else
#endif
  send_frame(fb);
}