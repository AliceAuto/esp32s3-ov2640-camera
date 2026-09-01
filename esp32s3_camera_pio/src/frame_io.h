/*
 * 帧收发: TCP 整帧(带尺寸头) / 串口兜底 / UDP 分片(OVU 协议)。
 * 归属: FRAME_BEGIN/END 常量 + OVU 分片公共定义 + 发送实现。
 */
#pragma once
#include "globals.h"

// ---- OVU 分片协议公共定义 ----
// 局域网直发(send_frame_udp) 与 TURN 转向(send_frame_udp_via_turn) 共用，
// 此前两处各写一份分片逻辑且已发生漂移(turn 路径漏了逐片重试)，改协议极易漏改。
//   头: "OVU"(3) fmt(1) seq(2) cnt_minus1(1) idx(1) w(2) h(2)  fmt:0=RGB565 1=JPEG
#define OVU_HDR         12
#define OVU_CHUNK       (1400 - OVU_HDR)   // 单报文净荷, 整包 1400B 避开 MTU 分片
#define OVU_MAX_CHUNKS  250                // idx/cnt 是 u8

// 组装分片头；hdr[7](idx) 由调用方逐片填
static inline void ovu_build_header(uint8_t *hdr, pixformat_t fmt, uint16_t seq,
                                    size_t nchunk, uint16_t w, uint16_t h) {
  hdr[0] = 'O'; hdr[1] = 'V'; hdr[2] = 'U';
  hdr[3] = (fmt == PIXFORMAT_JPEG) ? 1 : 0;
  hdr[4] = (uint8_t)(seq >> 8);
  hdr[5] = (uint8_t)(seq & 0xFF);
  hdr[6] = (uint8_t)(nchunk - 1);
  hdr[7] = 0;
  hdr[8]  = (uint8_t)(w >> 8);
  hdr[9]  = (uint8_t)(w & 0xFF);
  hdr[10] = (uint8_t)(h >> 8);
  hdr[11] = (uint8_t)(h & 0xFF);
}

static inline size_t ovu_chunk_count(size_t len) {
  size_t n = (len + OVU_CHUNK - 1) / OVU_CHUNK;
  if (n == 0) n = 1;
  return n > OVU_MAX_CHUNKS ? OVU_MAX_CHUNKS : n;
}

// 发送一帧原始数据（带尺寸头）。仅在调用方已持有 g_cli_mutex 时调用，
// 失败返回 false 并重置客户端（P 拍照路径据此回退串口补发）。
bool send_frame_locked(camera_fb_t *fb);
// 对外包装: 拍照等 loop 侧调用加锁后复用同一实现(TCP 整帧)
bool send_frame(camera_fb_t *fb);
// 强制通过串口发送帧（不受 WiFi 客户端状态影响）。用于 P 拍照的兜底路径。
void send_frame_serial(camera_fb_t *fb);
// UDP 分片发送。socket 参数化: 局域网/TURN 用 cam_udp(与打洞互斥),
// P2P 直连用打洞锁定的候选 socket([1..2] 独占口, 不加锁)。
bool send_frame_udp(camera_fb_t *fb, WiFiUDP &u, IPAddress dst, uint16_t port = TCP_PORT);
