/*
 * 相机管理: 初始化/失锁重连/分辨率切换/画质校准。
 * 归属: g_config(全局配置) + 相机互斥锁访问 + 全部 esp_camera_* 调用。
 * wait_tx_idle 实现在 main.cpp(依赖快照双槽状态), 此处仅声明。
 */
#pragma once
#include "globals.h"

int  fb_count_for(framesize_t fs);
framesize_t framesize_from_code(char code);
void apply_color_calib();
bool init_camera_as(framesize_t target, framesize_t fallback);
bool camera_reconnect();
bool camera_change_framesize(framesize_t fs);
bool cam_lock(uint32_t timeout_ms);   // 相机互斥锁(所有 esp_camera_* 调用须持锁)
void cam_unlock();
void wait_tx_idle(uint32_t timeout_ms);   // main.cpp 实现
