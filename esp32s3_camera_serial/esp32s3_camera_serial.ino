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
 *   D7 -> 16, D6 -> 17, D5 -> 18, D4 -> 12, D3 -> 10, D2 -> 8, D1 -> 9, D0 -> 11,
 *   PWDN -> GND（必须拉低使能）, RST 悬空或接 3.3V 均可。
 *
 * 串口协议：
 *   每帧输出两行文本 + JPEG 二进制：
 *     FRAME_BEGIN
 *     <JPEG 原始字节>
 *     FRAME_END
 */

#include "esp_camera.h"

// ---- 引脚配置（esp32-camera 官方 ESP32-S3 示例脚位）----
#define PWDN_GPIO_NUM    -1   // PWDN 已硬件拉低，代码不控制
#define RESET_GPIO_NUM   -1   // 不控制复位
#define XCLK_GPIO_NUM    15
#define SIOD_GPIO_NUM    4    // SDA
#define SIOC_GPIO_NUM    5    // SCL
#define D7_GPIO_NUM      16
#define D6_GPIO_NUM      17
#define D5_GPIO_NUM      18
#define D4_GPIO_NUM      12
#define D3_GPIO_NUM      10
#define D2_GPIO_NUM      8
#define D1_GPIO_NUM      9
#define D0_GPIO_NUM      11
#define VSYNC_GPIO_NUM   6
#define HREF_GPIO_NUM    7
#define PCLK_GPIO_NUM    13

// ---- 帧头/帧尾（供上位机脚本识别一帧的边界）----
static const char FRAME_BEGIN[] = "FRAME_BEGIN\n";
static const char FRAME_END[]   = "FRAME_END\n";

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);
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
  config.pixel_format = PIXFORMAT_JPEG;    // 直接出 JPEG 压缩流，省内存
  config.frame_size   = FRAMESIZE_VGA;     // 640x480，验证接线足够
  config.jpeg_quality = 12;                // 0-63，越小越清晰（12 为常用值）
  config.fb_count     = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("摄像头初始化失败, err=0x%x\n", err);
    while (true) { delay(100); }
  }
  Serial.println("摄像头初始化成功");
}

void loop() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("抓帧失败：esp_camera_fb_get 返回空");
    delay(1000);
    return;
  }

  if (fb->format == PIXFORMAT_JPEG) {
    Serial.print(FRAME_BEGIN);
    Serial.write(fb->buf, fb->len);   // 原样发送 JPEG 数据
    Serial.print(FRAME_END);
    Serial.printf("  #len=%u\n", fb->len);
  } else {
    Serial.println("当前不是 JPEG 输出，请检查 pixel_format");
  }

  esp_camera_fb_return(fb);
  delay(1000);   // 每秒一帧，方便调试观察
}