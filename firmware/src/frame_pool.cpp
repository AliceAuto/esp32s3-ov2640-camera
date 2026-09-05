/*
 * PSRAM 帧内存池实现(自 main.cpp 拆出)。
 */
#include "frame_pool.h"

static uint8_t      *g_pool_mem  = NULL;
static QueueHandle_t g_pool_free = NULL;   // 空闲块索引

bool frame_pool_init() {
  g_pool_mem = (uint8_t*)heap_caps_malloc((size_t)FB_POOL_BLOCKS * FB_POOL_BLOCK,
                                          MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!g_pool_mem) return false;
  g_pool_free = xQueueCreate(FB_POOL_BLOCKS, sizeof(int));
  if (!g_pool_free) { heap_caps_free(g_pool_mem); g_pool_mem = NULL; return false; }
  for (int i = 0; i < FB_POOL_BLOCKS; i++) {
    int idx = i;
    xQueueSend(g_pool_free, &idx, 0);
  }
  return true;
}

// 取一块；无空闲/未初始化返回 -1(调用方回退堆分配)
static int frame_pool_take() {
  int idx;
  if (!g_pool_free) return -1;
  return (xQueueReceive(g_pool_free, &idx, 0) == pdTRUE) ? idx : -1;
}

static void frame_pool_give(int idx) {
  if (idx >= 0 && g_pool_free) xQueueSend(g_pool_free, &idx, 0);
}

// 分配一帧缓冲：优先池化块，超大帧回退 PSRAM 堆
uint8_t *frame_alloc(size_t len, int8_t *slot) {
  if (g_pool_mem && len <= FB_POOL_BLOCK) {
    int idx = frame_pool_take();
    if (idx >= 0) { *slot = (int8_t)idx; return g_pool_mem + (size_t)idx * FB_POOL_BLOCK; }
  }
  *slot = -1;
  return (uint8_t*)heap_caps_malloc(len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}

void frame_free(uint8_t *p, int8_t slot) {
  if (!p) return;
  if (slot >= 0) frame_pool_give((int)slot);
  else heap_caps_free(p);
}
