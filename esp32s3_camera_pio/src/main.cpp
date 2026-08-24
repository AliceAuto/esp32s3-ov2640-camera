/*
 * ESP32-S3 + OV2640 PlatformIO 工程
 *
 * 目标：从源码编译 esp32-camera，解锁 JPEG 模式（Arduino 预编译库在 JPEG 下返空帧 FB-OVF，
 * 源码编译可绕过）。JPEG 直播 + UDP 高帧率 / 高分辨率，拍照走可靠 TCP。
 *
 * 接线（OV2640 -> ESP32-S3）：
 *   3.3V-3V3  GND-GND  SCL-5  SDA-4  VSYNC-6  HREF-7  PCLK-13  XCLK-15
 *   D7-16 D6-17 D5-18 D4-2  D3-10 D2-8  D1-9   D0-1
 *   PWDN -> GND（拉低使能）, RST 悬空或接 3.3V
 *   （本板 GPIO11/12 损坏，D0 已改接 GPIO1，D4 改接 GPIO2）
 *
 * 传输：
 *   - WiFi STA(家庭路由)+AP(OV2640Cam 兜底)，TCP 服务端 :3333
 *   - 预览：默认走 UDP（上位机发 'U' 开启），丢帧不重传 -> 高帧率
 *   - 拍照：走 TCP（可靠），命令 V/H/P/U/N 与旧工程一致
 *
 * 编译/烧录：pio run && pio run -t upload（COM7）
 */
#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiServer.h>
#include <WiFiUdp.h>

// ---- 像素格式：本工程专为解锁 JPEG ----
#define PIX_FORMAT PIXFORMAT_JPEG   // JPEG（源库编译），改回 PIXFORMAT_RGB565 即旧基线

// ---- 函数原型（.ino 由 Arduino 自动生成，.cpp 需手动声明）----
int fb_count_for(framesize_t fs);
void send_frame(camera_fb_t *fb);
void send_frame_udp(camera_fb_t *fb, IPAddress dst);
void send_frame_wrapper(camera_fb_t *fb);
framesize_t framesize_from_code(char code);
void apply_color_calib();
bool init_camera_as(framesize_t target, framesize_t fallback);
bool camera_reconnect();
bool camera_change_framesize(framesize_t fs);
void setup_wifi();
void log_wifi_ip();
void handle_cmd(int c, Stream &in);

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
#endif

#ifdef ENABLE_WIFI
static WiFiUDP     cam_udp;            // UDP 预览通道(高帧率, 丢帧不重传)
static bool        udp_mode  = false;  // 上位机发 'U' 开启
static uint16_t    udp_seq   = 0;
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

// 发送一帧原始数据（带尺寸头）。优先 WiFi TCP（已连接），失败回退串口（不丢帧）。
void send_frame(camera_fb_t *fb) {
#ifdef ENABLE_WIFI
  if (cam_client && cam_client.connected()) {
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
// 通过 UDP 把一整帧分片成多个数据报发送（预览用，丢帧不重传）。
// 数据报布局: 12B头 + 分片。
//   头: "OVU"(3) fmt(1) seq(2) cnt_minus1(1) idx(1) w(2) h(2)  fmt:0=RGB565 1=JPEG
void send_frame_udp(camera_fb_t *fb, IPAddress dst) {
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
  // 极致优化(诊断): 打印当前 CLKRC。运行时写入被传感器忽略(见 'k' 命令实验),
  // 要改 CLKRC 需从源码编译修改 set_window
  uint8_t clkrc_init = (uint8_t)s->get_reg(s, 0x0011, 0xFF);
  Dbg.printf("CLKRC init=0x%02X\n", clkrc_init);
  delay(200);                 // 给 AWB/AGC 收敛时间
}

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
  esp_camera_deinit();
  delay(300);
  return init_camera_as(cur_frame, PREVIEW_FRAME);
}

// 真正的分辨率切换：deinit + init 全量重配，失败自动回退
bool camera_change_framesize(framesize_t fs) {
  if (fs == cur_frame) { return true; }
  esp_camera_deinit();
  delay(200);
  return init_camera_as(fs, cur_frame);
}

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
  config.xclk_freq_hz = 30000000;          // 极致优化: 24->30MHz 超频(OV2640标称上限24, 30通常可跑, 不稳则回退)
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
#endif
}

#ifdef ENABLE_WIFI
void setup_wifi() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
  WiFi.softAP(AP_SSID, AP_PASS);
  Dbg.printf("[WiFi] 兜底AP已开: %s\n", AP_SSID);
  cam_server.begin();
  cam_udp.begin(TCP_PORT);
  Dbg.printf("[WiFi] TCP监听: %u\n", TCP_PORT);

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
      send_frame(fb);
      esp_camera_fb_return(fb);
    }
    camera_change_framesize(PREVIEW_FRAME);
    Dbg.println("CMD_PHOTO_DONE");
  } else if (c == 'v' || c == 'V') {
    long t0 = millis();
    while (millis() - t0 < 200 && !in.available()) delay(1);
    char code = (char)in.read();
    PREVIEW_FRAME = framesize_from_code(code);
    camera_change_framesize(PREVIEW_FRAME);
    Dbg.printf("预览分辨率已设为: %c\n", code);
  } else if (c == 'h' || c == 'H') {
    long t0 = millis();
    while (millis() - t0 < 200 && !in.available()) delay(1);
    char code = (char)in.read();
    HI_FRAME = framesize_from_code(code);
    Dbg.printf("拍照分辨率已设为: %c (下次拍照生效)\n", code);
  } else if (c == 'q') {
    long t0 = millis();
    while (millis() - t0 < 200 && !in.available()) delay(1);
    int q = in.read();
    if (q >= 0) {
      g_quality = q;
      sensor_t *s = esp_camera_sensor_get();
      if (s) s->set_quality(s, g_quality);
      Dbg.printf("JPEG quality=%d\n", g_quality);
    }
  } else if (c == 'd') {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
      Dbg.printf("DIAG fmt=%d len=%u %ux%u\n", (int)fb->format, (unsigned)fb->len,
                 fb->width, fb->height);
      esp_camera_fb_return(fb);
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

// 发送一帧预览：打开 UDP 时走 UDP(高帧率)，否则回退 TCP/串口(稳定)。
// 无客户端时跳过发送(只保帧采集)，避免串口被帧淹没、干扰命令/日志通道。
void send_frame_wrapper(camera_fb_t *fb) {
#ifdef ENABLE_WIFI
  if (cam_client && cam_client.connected()) {
    if (udp_mode)
      send_frame_udp(fb, cam_client.remoteIP());
    else
      send_frame(fb);
    return;
  }
  return;   // 无客户端：丢弃帧，串口仅作命令/日志
#else
  send_frame(fb);
#endif
}

void loop() {
  log_wifi_ip();
  // ---- WiFi：accept 最新 TCP 客户端；断开及时重置；旧客户端半开时抢占接管 ----
#ifdef ENABLE_WIFI
  if (cam_client && !cam_client.connected()) {
    cam_client.stop();
    cam_client = WiFiClient();
  }
  // 抢占式 accept: 即使旧客户端 connected() 仍为 true(半开/粘滞), 新连接也接管
  {
    WiFiClient s = cam_server.available();
    if (s) {
      if (cam_client) cam_client.stop();
      cam_client = s;
      cam_client.setNoDelay(true);
      cam_client.setTimeout(500);
      Dbg.println("WiFi 客户端接入(抢占)");
    }
  }
#endif

  // ---- 命令：串口 + TCP 都读 ----
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
    if (fail_count >= 2) {
      fail_count = 0;
      camera_reconnect();
    }
    return;
  }
  send_frame_wrapper(fb);
  esp_camera_fb_return(fb);
  delay(1);
}