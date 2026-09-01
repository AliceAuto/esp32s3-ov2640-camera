/*
 * Web 配置页 + 目标 WiFi(NVS) 持久化。
 * 归属: httpServer/g_prefs/g_sta_ssid 等/g_softap_on 定义 + 配置页处理。
 */
#pragma once
#include "globals.h"

void handle_web_root();
void handle_web_save();
String default_dev_id();
