/*
 * Web 配置页 + 目标 WiFi(NVS) 持久化。
 * 归属: httpServer/g_prefs/g_sta_ssid 等/g_softap_on 定义 + 配置页处理。
 */
#pragma once
#include "globals.h"

void handle_web_root();
void handle_web_save();
void handle_web_scan();
void handle_web_test();       // 启动测试连接(异步, 立即返回)
void handle_web_test_status();// 查询测试连接结果(前端轮询)
void test_tick();             // 测试连接状态机(loop 周期推进)
void wifi_scan_and_cache();   // 开热点前扫描附近 WiFi 并缓存(避免 AP 抢射频导致扫不到)
String default_dev_id();
