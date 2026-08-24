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

void setup() {
  Dbg.begin(2000000, SERIAL_8N1, RX_GPIO_NUM, TX_GPIO_NUM);  // 2M波特率: UXGA大图传输(~19s/帧)
  Dbg.setTimeout(100);
  delay(500);

  camera_config_t config;
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
  config.xclk_freq_hz = 20000000;          // 20MHz
  config.pixel_format = PIXFORMAT_RGB565;    // 对照：RGB565
  config.frame_size   = FRAMESIZE_UXGA;    // 1600x1200 OV2640最高分辨率
  config.fb_count     = 1;                 // 单缓冲(拍照模式)
  config.fb_location  = CAMERA_FB_IN_PSRAM; // 8MB PSRAM 帧缓冲(3.84MB/帧)

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Dbg.printf("摄像头初始化失败, err=0x%x\n", err);
    while (true) { delay(100); }
  }
  Dbg.println("摄像头初始化成功");

  // ---- 画质校准：解决偏绿黄(蓝通道低)+高噪点 ----
  // 依据正点原子 ov2640cfg.h 的参数思路，用 esp32-camera 官方 API 配置，
  // 避免与库内置初始化表寄存器冲突。
  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_whitebal(s, 1);   // 自动白平衡 AWB 使能(CTRL1 bit3)
    s->set_awb_gain(s, 1);   // AWB 增益使能(CTRL1 bit2)——必须和 AWB 一起开，否则白平衡不生效
    s->set_gain_ctrl(s, 1);  // 自动增益 AGC：避免增益过高喷出的噪点
    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 1);       // 增强自动曝光
    s->set_saturation(s, 0); // 饱和度 0(默认)
    s->set_brightness(s, 0); // 亮度 0(默认)
    s->set_contrast(s, 0);   // 对比度 0(默认)
    s->set_dcw(s, 1);        // 降噪
    s->set_bpc(s, 1);        // 坏点消除，压制雪花噪点
    s->set_wpc(s, 1);
    s->set_lenc(s, 1);       // 镜头校正（边缘暗角），进一步降噪
    s->set_special_effect(s, 0);
    Dbg.println("画质校准完成：AWB/AGC/降噪已开启");
    delay(300); // 给自动曝光/白平衡收敛时间
  } else {
    Dbg.println("警告：获取 sensor 失败，无法校准");
  }
}

void loop() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Dbg.println("抓帧失败：esp_camera_fb_get 返回空");
    delay(100);
    return;
  }

  // 全速串流：有帧就原样发出去，用 marker 切分
  Dbg.write(FRAME_BEGIN, sizeof(FRAME_BEGIN) - 1);
  Dbg.write(fb->buf, fb->len);   // 原样发送帧数据
  Dbg.write(FRAME_END, sizeof(FRAME_END) - 1);
  esp_camera_fb_return(fb);
  delay(3000);  // 拍照模式: 3s一张, 给上位机接收处理时间
}