/*
 * 全局共享头: 常量宏 + 跨模块共享全局变量声明。
 *
 * 拆分说明: 原 main.cpp 1672 行单文件, 大量 static 全局被多个功能域交叉引用,
 * 无法直接拆分。本头把"被两个及以上模块使用的全局"统一声明为 extern,
 * 定义留在各自归属的 .cpp:
 *   diag 统计    -> diag.cpp
 *   相机配置/锁  -> camera.cpp (g_config / 引脚)
 *   帧收发/OVU   -> frame_io.cpp (FRAME_BEGIN/END)
 *   配置页/NVS   -> web_cfg.cpp (g_sta_ssid 等/g_prefs/httpServer/g_softap_on)
 *   WiFi STA     -> wifi_sta.cpp (g_last_ip_log/g_sta_last_try)
 *   其余共享     -> main.cpp
 * 模块化演进: 新增模块只需在本头补 extern 声明 + 在归属文件定义。
 */
#pragma once
#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiServer.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <Preferences.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"

#define ENABLE_WIFI 1

// ---- 凭据(不入库): 建本地文件 src/secrets.h 覆盖(已在 .gitignore) ----
// 未定义时留空 -> setup_wifi() 检测无凭据会开 SoftAP 兜底, 保证配网页(:80)可达。
#if __has_include("secrets.h")
#include "secrets.h"
#endif
#ifndef STA_SSID
#define STA_SSID   ""
#endif
#ifndef STA_PASS
#define STA_PASS   ""
#endif
#define AP_SSID     "OV2640Cam"        // 兜底热点(仅在无 STA 凭据时开启)
#define AP_PASS     ""
#define TCP_PORT    3333
#define DISCOVERY_PORT  9999
#define RELAY_HOST_DEFAULT ""                  // 公网中继地址(如 "host:8001"; 空=关闭), 也可在配网页填写
#ifndef RELAY_TOKEN
#define RELAY_TOKEN ""                          // 中继鉴权口令, secrets.h 可覆盖(与服务器一致)
#endif

// ---- 像素格式：本工程专为解锁 JPEG ----
#define PIX_FORMAT PIXFORMAT_JPEG   // JPEG（源库编译），改回 PIXFORMAT_RGB565 即旧基线

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
// UART0 调试口（CH343 TX/RX）
#define TX_GPIO_NUM    43
#define RX_GPIO_NUM    44

// ---- 帧头/帧尾（上位机识别帧边界; 定义在 frame_io.cpp）----
extern const char FRAME_BEGIN[];
extern const char FRAME_END[];

// ---- 可靠 P2P 常量 ----
#define P2P_NSOCK        3
#define P2P_NPEER_MAX    4
#define P2P_MAGIC        "P2P"
#define P2P_PUNCH_MS     100
#define P2P_PUNCH_TMO    3000
#define P2P_KEEPALIVE_MS 500
#define P2P_DEAD_MS      3000
#define P2P_TURN_DEAD_MS 5000
#define P2P_RETRY_MS     30000

// ---- 帧池 / 双槽乒乓 ----
#define FB_POOL_BLOCKS 6
#define FB_POOL_BLOCK  (64 * 1024)
#define SNAP_SLOTS     2
#define BYTES_PER_PIXEL 2
#define MAX_FB_BYTES   (6 * 1024 * 1024)

// ---- 跨模块共享全局(定义处见注释) ----
extern HardwareSerial Dbg;                              // main.cpp
extern framesize_t PREVIEW_FRAME, HI_FRAME, cur_frame;  // main.cpp
extern int   g_quality;            // JPEG 质量 0-63     // main.cpp
extern bool  g_auto_quality;                            // main.cpp
extern WiFiUDP cam_udp;            // UDP 预览通道       // main.cpp
extern bool  udp_mode;                                  // main.cpp
extern volatile bool g_p2p_active, g_turn_mode, g_relay_viewer;  // main.cpp
extern char  g_dev_name[32], g_dev_id[40], g_relay_host[80], g_relay_token[40];  // main.cpp
extern WiFiServer  cam_server;                          // main.cpp
extern WiFiClient  cam_client;                          // main.cpp
extern uint16_t    udp_seq;                             // main.cpp
extern uint8_t     g_udp_backoff_frames;                // main.cpp
extern SemaphoreHandle_t g_cli_mutex, g_udp_mutex, g_cam_mutex;  // main.cpp(setup 创建)
extern volatile bool g_switching;                       // main.cpp
extern volatile bool g_relay_online;                    // main.cpp
extern volatile uint16_t g_relay_stall_ms;              // main.cpp: 中继 TCP 写阻塞量(可靠拥塞信号)
extern char  g_sta_ssid[33], g_sta_pass[64];            // web_cfg.cpp
extern Preferences g_prefs;                             // web_cfg.cpp
extern WebServer httpServer;                            // web_cfg.cpp
extern bool  g_softap_on;                               // web_cfg.cpp
extern camera_config_t g_config;                        // camera.cpp

// ---- 诊断统计(定义在 diag.cpp) ----
extern uint32_t g_diag_last_ms, g_diag_cap_frames, g_diag_sent_frames, g_diag_skip_frames;
extern uint32_t g_diag_evict_frames, g_diag_nc_frames, g_diag_bigdrop_frames;
extern uint32_t g_diag_udp_err, g_diag_udp_retry_ok;
extern int      g_diag_last_err;
extern uint64_t g_diag_sent_bytes, g_diag_cap_bytes;
extern uint32_t g_adapt_last_ms;
extern uint64_t g_adapt_last_sent, g_adapt_last_bytes;
extern uint64_t g_adapt_total_frames, g_adapt_total_bytes;
extern uint32_t g_adapt_total_err, g_adapt_last_err;
extern uint64_t g_adapt_total_evict, g_adapt_last_evict;
extern volatile bool g_cam_ready;   // main.cpp: esp_camera_init 成功置 true
