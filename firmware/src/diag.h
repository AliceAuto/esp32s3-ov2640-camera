/*
 * 诊断统计模块: 帧计数 / 自适应画质。
 * 归属: g_diag_* 与 g_adapt_* 全局 + record_sent/log_diag/adapt_quality。
 */
#pragma once
#include "globals.h"

// 发送成功一帧的记账(UDP/TURN/P2P/TCP 各发送路径共用)
void record_sent(size_t len);
// 每 5s 打印 DIAG 统计行(清窗口计数)
void log_diag();
// 自适应画质: 检测拥塞(err/evict)按偏差步进调节 JPEG 质量
void adapt_quality();
// UDP 诊断心跳: 周期上报关键状态到最近客户端(独立任务, 不依赖相机产帧)
void send_diag_beat();
// 诊断心跳任务入口(核0低优先级, 与 sender 抢锁让路)
void diag_beat_task(void *arg);
