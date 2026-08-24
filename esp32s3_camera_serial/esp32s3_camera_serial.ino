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

// UART0 = CH343 转串口所在通道（TX=GPIO43, RX=GPIO44）。
// 用 HardwareSerial 强制输出到 UART0，避免默认 Serial 走了 USB-CDC。
HardwareSerial Dbg(0);

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

// 发送一帧原始数据(带尺寸头)
void send_frame(camera_fb_t *fb) {
  Dbg.write(FRAME_BEGIN, sizeof(FRAME_BEGIN) - 1);
  Dbg.printf("SIZE %u %u\n", fb->width, fb->height);
  Dbg.write(fb->buf, fb->len);
  Dbg.write(FRAME_END, sizeof(FRAME_END) - 1);
}

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

// 用全局配置以目标分辨率重建摄像头；返回是否成功
// 失败时自动降级：高分辨率先强制单缓冲，再失败则回退到给定 fallback 分辨率
bool init_camera_as(framesize_t target, framesize_t fallback) {
  g_config.frame_size = target;
  g_config.fb_count   = fb_count_for(target);
  esp_err_t err = esp_camera_init(&g_config);
  if (err == ESP_OK) {
    cur_frame = target;
    Dbg.printf("摄像头初始化: fs=%u fb=%d\n", (unsigned)target, g_config.fb_count);
    return true;
  }
  Dbg.printf("init fs=%u 失败 err=0x%x, 尝试单缓冲...\n", (unsigned)target, err);
  // 降级：单缓冲重试
  g_config.fb_count = 1;
  err = esp_camera_init(&g_config);
  if (err == ESP_OK) {
    cur_frame = target;
    Dbg.printf("摄像头初始化(单缓冲): fs=%u fb=1\n", (unsigned)target);
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
  config.xclk_freq_hz = 20000000;           // 20MHz
  config.pixel_format = PIXFORMAT_RGB565;    // RGB565
  config.frame_size   = PREVIEW_FRAME;     // 初始：QQVGA 预览
  config.fb_count     = fb_count_for(PREVIEW_FRAME);  // 自适应双缓冲(高分辨率降为1)
  config.fb_location  = CAMERA_FB_IN_PSRAM; // 8MB PSRAM 帧缓冲(UXGA 3.84MB)

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Dbg.printf("摄像头初始化失败, err=0x%x\n", err);
    while (true) { delay(100); }
  }
  Dbg.println("摄像头初始化成功");

  // ---- 画质校准（最小化测试）：排查导致偏绿缺蓝的异常增益 ----
  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_whitebal(s, 1);   // 自动白平衡 AWB(最小配置)
    s->set_gain_ctrl(s, 1);  // 自动增益
    s->set_exposure_ctrl(s, 1);
    // 注释掉 awb_gain/aec2/lenc/bpc/wpc/dcw，怀疑它们导致 B 通道增益失效
    Dbg.println("画质校准(最小)：仅 AWB+AGC");
    delay(500); // 给自动曝光/白平衡收敛时间
  } else {
    Dbg.println("警告：获取 sensor 失败，无法校准");
  }
}

void loop() {
  // ---- 上位机命令（单字节或 "R码" 字符）----
  while (Dbg.available()) {
    int c = Dbg.read();
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
      // 等待下一个字节作为分辨率码: V<code> 设置预览分辨率
      char code = (char)Dbg.read();   // 阻塞读第二个字节(天然等待)
      PREVIEW_FRAME = framesize_from_code(code);
      camera_change_framesize(PREVIEW_FRAME);
      Dbg.printf("预览分辨率已设为: %c\n", code);
    } else if (c == 'h' || c == 'H') {
      // H<code> 设置拍照(高清)分辨率
      char code = (char)Dbg.read();   // 阻塞读第二个字节
      HI_FRAME = framesize_from_code(code);
      Dbg.printf("拍照分辨率已设为: %c (下次拍照生效)\n", code);
    } else if (c == 'd') {
      // 板内诊断
      camera_fb_t *fb = esp_camera_fb_get();
      if (fb) {
        uint32_t rD=0, gD=0, bD=0, n=0;
        uint16_t *p = (uint16_t*)fb->buf;
        uint32_t px = fb->width * fb->height;
        uint32_t step = px / 6000; if (step < 1) step = 1;
        for (uint32_t i = 0; i < px; i += step) {
          uint16_t v = p[i];
          uint8_t r = (v >> 11) & 0x1F, g = (v >> 5) & 0x3F, b = v & 0x1F;
          if (r>g && r>b) rD++; else if (g>r && g>b) gD++; else if (b>r && b>g) bD++;
          n++;
        }
        Dbg.printf("DIAG px=%u R=%u G=%u B=%u\n", n, rD, gD, bD);
        esp_camera_fb_return(fb);
      }
    } else if (c == 'r' || c == 'R') {
      // 手动重连（调试用）
      camera_reconnect();
    }
  }

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
  send_frame(fb);
  esp_camera_fb_return(fb);
  delay(30);  // 预览 ~20fps@QQVGA
}