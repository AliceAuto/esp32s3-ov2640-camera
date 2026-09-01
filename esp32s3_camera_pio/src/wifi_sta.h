/*
 * WiFi STA 接入: NVS 加载凭据 / 连接 / 兜底 SoftAP / 断线自愈。
 */
#pragma once
#include "globals.h"

void setup_wifi();
void log_wifi_ip();
void check_sta_link();
