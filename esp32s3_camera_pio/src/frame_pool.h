/*
 * PSRAM 帧内存池: 消除每帧 malloc/free 抖动与碎片。
 * 定长块队列出队/归还; 超过块长的特大帧回退 PSRAM 堆(行为不变)。
 */
#pragma once
#include "globals.h"

// 初始化帧池(失败返回 false, 调用方回退逐帧 malloc)
bool frame_pool_init();
// 分配一帧缓冲: 优先池化块, 超大帧回退堆; slot>=0 表示来自池, -1 为堆
uint8_t *frame_alloc(size_t len, int8_t *slot);
// 释放一帧缓冲(池槽归队 / 堆 free)
void frame_free(uint8_t *p, int8_t slot);
