/*
 * 帧收发实现(自 main.cpp 拆出)。
 */
#include "frame_io.h"
#include "diag.h"   // record_sent 未用, 但 g_diag_* 计数见 globals.h

// ---- 帧头/帧尾（上位机识别帧边界）----
const char FRAME_BEGIN[] = "FRAME_BEGIN\n";
const char FRAME_END[]   = "FRAME_END\n";

// 发送一帧原始数据（带尺寸头）。仅在持有 g_cli_mutex 时调用。
// 失败返回 false 并重置客户端（P 拍照路径据此回退串口补发）。
bool send_frame_locked(camera_fb_t *fb) {
  if (!(cam_client && cam_client.connected())) { g_diag_nc_frames++; return false; }
  cam_client.write((const uint8_t*)FRAME_BEGIN, sizeof(FRAME_BEGIN) - 1);
  char head[48];
  snprintf(head, sizeof(head), "SIZE %u %u %s\n", fb->width, fb->height,
           (fb->format == PIXFORMAT_JPEG) ? "JPG" : "RGB");
  cam_client.write((const uint8_t*)head, strlen(head));
  const uint8_t *p = fb->buf; size_t left = fb->len;
  bool tcp_ok = true;
  while (left) {
    size_t n = left > 16384 ? 16384 : left;
    size_t s = cam_client.write(p, n);
    if (s == 0) { tcp_ok = false; break; }
    p += s; left -= s;
    if (left) yield();
  }
  if (tcp_ok) {
    cam_client.write((const uint8_t*)FRAME_END, sizeof(FRAME_END) - 1);
    return true;
  }
  Dbg.println("TCP 发送失败, 丢帧并重置客户端");
  cam_client.stop();
  g_diag_skip_frames++;
  g_diag_last_err = -2;
  return false;
}

// 对外包装：拍照等 loop 侧调用加锁后复用同一实现
bool send_frame(camera_fb_t *fb) {
#ifdef ENABLE_WIFI
  if (xSemaphoreTake(g_cli_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) return false;
  bool ok = send_frame_locked(fb);
  xSemaphoreGive(g_cli_mutex);
  return ok;
#else
  send_frame_serial(fb);
  return true;
#endif
}

// 强制通过串口发送帧（不受 WiFi 客户端状态影响）。用于 P 拍照的兜底路径。
void send_frame_serial(camera_fb_t *fb) {
  Dbg.write(FRAME_BEGIN, sizeof(FRAME_BEGIN) - 1);
  Dbg.printf("SIZE %u %u %s\n", fb->width, fb->height,
             (fb->format == PIXFORMAT_JPEG) ? "JPG" : "RGB");
  Dbg.write(fb->buf, fb->len);
  Dbg.write(FRAME_END, sizeof(FRAME_END) - 1);
}

#ifdef ENABLE_WIFI
// OVU 分片协议定义(OVU_HDR/OVU_CHUNK/ovu_build_header/ovu_chunk_count)见 frame_io.h

// 通过 UDP 把一整帧分片成多个数据报发送（预览用，丢帧不重传）。
// 优化1: 头+净荷先在栈上拼成连续报文再单次 write。此前两次 write 会让 lwIP
//        生成链式 pbuf(两次分配+拷贝)，每帧 4~36 片 = 每帧数十次额外开销。
// 优化2: 重试改为整帧共享预算。此前小帧 6 次重试×2ms×最多4片 = 最坏阻塞 48ms
//        (注释写 8ms 与实际不符)，期间 sender_task 停止消费队列。
static bool send_frame_udp_locked(camera_fb_t *fb, WiFiUDP &u, IPAddress dst, uint16_t port) {
  const uint8_t *buf = fb->buf;
  size_t len = fb->len;
  size_t nchunk = ovu_chunk_count(len);
  uint8_t pkt[OVU_HDR + OVU_CHUNK];          // 1400B 拼包缓冲(单次 write)
  ovu_build_header(pkt, (pixformat_t)fb->format, udp_seq, nchunk, fb->width, fb->height);

  const bool small_frame     = (nchunk <= 4);
  // 重试预算按帧大小成比例: 大帧分片多, 单片瞬时 TX-full(ERR_MEM) 不应整帧作废。
  // 小帧 6 片*2ms; 大帧给每片 1 次即时重试即可, 不做长阻塞(阻塞越久采集流水线越饿)。
  const int  max_retry       = small_frame ? 6 : 1;
  const int  retry_delay_ms  = small_frame ? 2 : 1;
  const int  retry_budget_ms = small_frame ? 6 : 2;    // 大帧重试预算放宽到 2ms(单片瞬时重试), 不加长阻塞
  int budget_used = 0;
  size_t off = 0;
  for (size_t i = 0; i < nchunk && off < len; i++) {
    size_t n = len - off;
    if (n > OVU_CHUNK) n = OVU_CHUNK;
    pkt[7] = (uint8_t)i;
    memcpy(pkt + OVU_HDR, buf + off, n);
    bool sent = false;
    for (int t = 0; t <= max_retry; t++) {
      u.beginPacket(dst, port);
      u.write(pkt, OVU_HDR + n);
      int rc = u.endPacket();
      if (rc == 1) {
        if (t > 0) g_diag_udp_retry_ok++;
        sent = true;
        break;
      }
      g_diag_udp_err++;
      g_adapt_total_err++;
      g_diag_last_err = -1;
      if (t >= max_retry || budget_used >= retry_budget_ms) break;
      delay(retry_delay_ms);
      budget_used += retry_delay_ms;
    }
    if (!sent) {
      g_diag_skip_frames++;
      // 背压信号不要夸张: 一次 TX 失败就整帧丢弃会令 >4 片的大帧(UXGA ~37 片)在
      // 拥塞瞬间成片触发级联——每次失败再吞掉此后 3 帧, 帧率会掉到 1 帧量级。
      // 改为最多退避 1 帧 + 立刻让自适应画质降档压缩(减小后续单帧/分片数)。
      if (!small_frame) g_udp_backoff_frames = 1;
      return false;
    }
    off += n;
  }
  udp_seq++;
  return true;
}

// 发送入口。socket 参数化: 局域网/TURN 用 cam_udp(需与 relay_task 的打洞互斥),
// P2P 直连用打洞锁定的候选 socket([1..2] 为独占口, 无并发, 不加锁不阻塞)。
bool send_frame_udp(camera_fb_t *fb, WiFiUDP &u, IPAddress dst, uint16_t port) {
  bool shared = (&u == &cam_udp);
  if (shared) {
    if (!g_udp_mutex) return false;
    if (xSemaphoreTake(g_udp_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
      g_diag_skip_frames++;              // 拿不到锁(打洞中), 宁丢一帧不并发
      return false;
    }
  }
  bool ok = send_frame_udp_locked(fb, u, dst, port);
  if (shared) xSemaphoreGive(g_udp_mutex);
  return ok;
}
#endif
