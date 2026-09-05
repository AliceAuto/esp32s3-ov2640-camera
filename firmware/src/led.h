/*
 * 板载 RGB 状态灯 (ESP32-S3-DevKitC-1 GPIO48 WS2812)。
 * 用灯色表示设备状态: 蓝闪=配网模式(已开兜底热点), 绿=已入网, 红=初始化失败。
 */
#pragma once
#include <Arduino.h>

#define LED_PIN        48     // ESP32-S3-DevKitC-1 板载 WS2812 RGB
#define LED_NUMPIX     1

void led_init();
void led_set(uint8_t r, uint8_t g, uint8_t b);   // 立即设定并刷新
void led_update();                                // loop 周期调用, 处理闪烁
void led_show_config();                           // 配网模式: 蓝色呼吸
void led_show_online();                           // 已入网: 绿色常亮后熄灭
void led_show_error();                            // 初始化失败: 红色
