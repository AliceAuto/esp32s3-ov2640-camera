/*
 * 板载 RGB 状态灯实现。
 * 灯状态机:
 *   CONFIG  -> 蓝色呼吸(提示用户连热点配网)
 *   ONLINE  -> 绿色短暂亮起后熄灭(避免常亮刺眼)
 *   ERROR   -> 红色常亮
 * led_update 在 loop 周期调用, 用非阻塞 millis 计时驱动呼吸/闪烁。
 */
#include "led.h"
#include <Adafruit_NeoPixel.h>

static Adafruit_NeoPixel strip(LED_NUMPIX, LED_PIN, NEO_GRB + NEO_KHZ800);

// 全局亮度上限(0-255): 板载 WS2812 满亮很刺眼, 统一压到 ~25% 让状态灯柔和
#define LED_BRIGHTNESS 64

typedef enum { LED_OFF, LED_CONFIG, LED_ONLINE, LED_ERROR } led_state_t;
static led_state_t s_state = LED_OFF;
static uint32_t s_last_ms = 0;
static uint32_t s_online_done_ms = 0;
static bool     s_online_shown = false;   // 绿灯是否已亮过(防止 loop 反复触发导致闪烁)

void led_init() {
  strip.begin();
  strip.setBrightness(LED_BRIGHTNESS);   // 全局压暗, 避免刺眼
  strip.clear();
  strip.show();
  s_state = LED_OFF;
  s_last_ms = millis();
}

void led_set(uint8_t r, uint8_t g, uint8_t b) {
  strip.setPixelColor(0, strip.Color(r, g, b));
  strip.show();
}

// 呼吸: 用三角波让亮度 0..255..0 周期 1.6s
static void breathe(uint32_t now, uint8_t r, uint8_t g, uint8_t b) {
  uint32_t t = (now % 1600);
  int v = (t < 800) ? (t * 255 / 800) : ((1600 - t) * 255 / 800);
  strip.setPixelColor(0, strip.Color(r * v / 255, g * v / 255, b * v / 255));
  strip.show();
}

void led_show_config() {
  s_state = LED_CONFIG;
  s_online_shown = false;   // 进入配网态时复位, 下次重连再亮一次绿灯
}
void led_show_online() {
  // 一次性: 只在"从未亮过绿灯"时才触发一次, 之后即使 loop 反复调用也不再亮,
  // 避免"亮 600ms -> 灭 -> 再亮"的闪烁。
  if (!s_online_shown) {
    s_online_shown = true;
    s_state = LED_ONLINE;
    s_online_done_ms = millis() + 600;   // 绿灯亮 600ms 后熄灭
  }
}
void led_show_error()  { s_state = LED_ERROR; }

void led_update() {
  uint32_t now = millis();
  switch (s_state) {
    case LED_CONFIG:
      breathe(now, 0, 0, 255);   // 蓝呼吸
      break;
    case LED_ONLINE:
      if (now < s_online_done_ms) {
        led_set(0, 255, 0);      // 绿
      } else {
        led_set(0, 0, 0);        // 熄灭
        s_state = LED_OFF;
      }
      break;
    case LED_ERROR:
      led_set(255, 0, 0);        // 红常亮
      break;
    case LED_OFF:
    default:
      break;
  }
}
