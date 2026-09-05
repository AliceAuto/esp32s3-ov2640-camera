/*
 * 诊断统计实现(自 main.cpp 拆出):
 *   - 帧/字节计数(供 DIAG 与自适应画质差分使用)
 *   - log_diag: 每 5s 打印 cap/sent/kb/丢帧/背压/错误/RSSI
 *   - adapt_quality: 拥塞自动降画质保帧率, 余量回升
 */
#include "diag.h"
#include <ESP.h>   // ESP.getFreeHeap

// ---- 计数与统计窗口 ----
uint32_t g_diag_last_ms = 0;
uint32_t g_diag_cap_frames = 0;
uint32_t g_diag_sent_frames = 0;
uint32_t g_diag_skip_frames = 0;
// 队列满时主动顶掉的最旧帧(背压), 与 g_diag_skip_frames(真丢帧)分开统计。
// 此前两者混用一个计数器, 且无条件出队导致每帧+1, DIAG 的 skp 完全失去诊断价值。
uint32_t g_diag_evict_frames = 0;
uint32_t g_diag_nc_frames = 0;
uint32_t g_diag_bigdrop_frames = 0;   // 大帧预丢弃(防中继 HOL)
uint32_t g_diag_udp_err = 0;
uint32_t g_diag_udp_retry_ok = 0;
int      g_diag_last_err = 0;
uint64_t g_diag_sent_bytes = 0;
uint64_t g_diag_cap_bytes = 0;

// ---- 自适应画质用独立累计计数：log_diag 每5s清零 g_diag_*，统计窗不同不能共用，
// 否则差分遇到清零会产生下溢/0值，画质调节半失效。
// 注意：UDP 错误计数同样必须独立累计（此前漏了，adapt_quality 直接读 g_diag_udp_err，
// 该值被 log_diag 每5s清零，判定结果取决于采样点落在清零前后，时灵时不灵）。
uint32_t g_adapt_last_ms = 0;
uint64_t g_adapt_last_sent = 0;
uint64_t g_adapt_last_bytes = 0;
uint64_t g_adapt_total_frames = 0;
uint64_t g_adapt_total_bytes  = 0;
uint32_t g_adapt_total_err    = 0;
uint32_t g_adapt_last_err     = 0;
// 队列背压(顶掉最旧帧)独立累计: TCP 中继路径没有 UDP 丢包, 帧率上不去的信号是
// "发送队列持续被顶"(evict), 用它替代 err 触发加大压缩 —— 否则中继帧率永远提不上去。
uint64_t g_adapt_total_evict  = 0;
uint64_t g_adapt_last_evict   = 0;

void record_sent(size_t len) {
  g_diag_sent_frames++;
  g_diag_sent_bytes   += len;
  g_adapt_total_frames++;
  g_adapt_total_bytes += len;
}

void log_diag() {
  uint32_t now = millis();
  if (g_diag_last_ms == 0 || now - g_diag_last_ms < 5000) return;
  float dt = (now - g_diag_last_ms) / 1000.0f;
  float cap_fps = g_diag_cap_frames / dt;
  float sent_fps = g_diag_sent_frames / dt;
  float kbps = (g_diag_sent_bytes / 1024.0f) / dt;
  uint32_t avg_len = (g_diag_sent_frames > 0) ? (uint32_t)(g_diag_sent_bytes / g_diag_sent_frames) : 0;
  int rssi = WiFi.RSSI();
  // skp=真丢帧(入队失败/发送失败), evc=队列满主动顶掉的最旧帧(背压, 非故障)
  Dbg.printf("DIAG cap=%.1f/s sent=%.1f/s kb=%.0f/s avg=%uB uerr=%u rty=%u skp=%u evc=%u bgd=%u nc=%u err=%d q=%d auto=%d rssi=%d\n",
             cap_fps, sent_fps, kbps, avg_len,
             g_diag_udp_err, g_diag_udp_retry_ok, g_diag_skip_frames, g_diag_evict_frames,
             g_diag_bigdrop_frames, g_diag_nc_frames,
             g_diag_last_err, g_quality, g_auto_quality ? 1 : 0, rssi);
  g_diag_last_ms = now;
  g_diag_cap_frames = 0;
  g_diag_sent_frames = 0;
  g_diag_skip_frames = 0;
  g_diag_evict_frames = 0;
  g_diag_bigdrop_frames = 0;
  g_diag_nc_frames = 0;
  g_diag_udp_err = 0;
  g_diag_udp_retry_ok = 0;
  g_diag_last_err = 0;
  g_diag_sent_bytes = 0;
  g_diag_cap_bytes = 0;
}

void adapt_quality() {
  // 原实现仅 udp_mode 时生效, 导致跨网(P2P/TURN/中继)路径下自适应画质完全不工作——
  // 跨网恰恰最需要动态码率调节。改为检测任意活跃发送通道。
  bool has_stream = udp_mode || g_p2p_active || g_turn_mode || g_relay_viewer;
  if (!g_auto_quality || !has_stream) return;
  uint32_t now = millis();
  // 首次调用: 设基线后直接返回, 等下一个采样窗再评估。
  if (g_adapt_last_ms == 0) {
    g_adapt_last_ms    = now;
    g_adapt_last_sent  = g_adapt_total_frames;
    g_adapt_last_bytes = g_adapt_total_bytes;
    g_adapt_last_err   = g_adapt_total_err;
    g_adapt_last_evict = g_adapt_total_evict;
    return;
  }
  if (now - g_adapt_last_ms < 2500) return;

  float dt = (now - g_adapt_last_ms) / 1000.0f;
  uint32_t sent_delta = (uint32_t)(g_adapt_total_frames - g_adapt_last_sent);
  uint64_t bytes_delta = g_adapt_total_bytes - g_adapt_last_bytes;
  uint32_t err_delta   = g_adapt_total_err - g_adapt_last_err;
  uint32_t evict_delta = (uint32_t)(g_adapt_total_evict - g_adapt_last_evict);
  float fps = sent_delta / dt;
  float kbps = (bytes_delta / 1024.0f) / dt;
  float target_fps;
  if (g_turn_mode || g_relay_viewer) {
    // 跨网路径 RTT 高、带宽受限, 目标帧率低于局域网, 但仍要尽力拉高
    target_fps = (cur_frame >= FRAMESIZE_VGA) ? 7.0f : 20.0f;
  } else {
    // 局域网: 目标要贴着传感器/信道可达的甜点, 而非传感器名义上限 ——
    // UXGA 传感器编码墙 15fps, 但 2.4GHz + UDP 分片信道墙更低(~5-8fps), 见 SOLUTIONS N7/C5。
    // 定太高会导致永远"落后目标"而无限加大压缩到画质崩坏仍追不上, 反而更糊且更不稳。
    target_fps = (cur_frame == FRAMESIZE_UXGA)  ? 10.0f
                 : (cur_frame >= FRAMESIZE_SVGA) ? 10.0f
                 : (cur_frame >= FRAMESIZE_VGA)  ? 12.0f
                 : 25.0f;   // QQVGA/QVGA: 传感器上限 30fps
  }

  int new_q = g_quality;
  // 拥塞判定: UDP 路径看发送丢包(err_delta>0); 跨网 TCP 中继/P2P 没有 UDP 丢包,
  // 且双槽快照下发送慢不会明显抬 evict(生产总能找到空槽) —— 这两者触发不到。
  // 跨网 TCP 的正确拥塞信号是"中继写阻塞量" g_relay_stall_ms(relay_send_iov 对每条
  // 消息的 write 超时阻塞做滚动累加): 带宽撑不住单帧时写缓冲持续满 -> 持续阻塞。
  // 写入越多压缩越强, 这正是 I2"拥塞自动降画质保帧率"在中继路径缺失的一环。
  const uint16_t RELAY_CONGE_STALL_MS = 300;   // 中继写阻塞累计 0.3s 即视为拥塞
  bool congested = (err_delta > 0) || (evict_delta > 0)
                || (g_relay_stall_ms > RELAY_CONGE_STALL_MS);
  // 走公网中继/P2P/TURN 时带宽受限是常态(本机实测跨网上行仅 ~50KB/s, 局域网却
  // 149KB/s)。自适应"有余量就回画质"会在流畅(已压低)时又把画质拉高 -> 帧率再掉,
  // 形成画质/帧率反复震荡。跨网路径改为"只降不升": 一旦为流畅压过画质就不再回弹,
  // 收敛到偏低固定值, 换取稳定帧率。局域网路径(UDP/TCP 直连)保留原回弹逻辑。
  const bool cross_net = g_turn_mode || g_relay_viewer || g_p2p_active;
  // I2: 细化步进 —— 按"距目标帧率的偏差"取步进, 偏差大时大步快速追拢, 接近时
  // 只微调, 避免低带宽链路下固定 ±2 造成的画质锯齿崩摆。
  float gap = target_fps - fps;                 // >0 落后于目标
  int step = (gap > 6.0f) ? 6 : (gap > 3.0f) ? 4 : 2;
  // 大分辨率画质上限放宽: UXGA 单帧本就大, 压缩上限 35 在信道拥塞时往往压不够,
  // 分片数仍高达 ~25-37 片而显著亏帧。允许压到 40(仍高于钳制下限 8, 画质可接受)以
  // 把单帧缩小到能让分片流水线跟上的程度 —— 画质损失换帧率, 保"能看到连续画面"。
  const int quality_cap = (cur_frame >= FRAMESIZE_SVGA) ? 40 : 35;
  if (fps < target_fps - 2.0f && congested && g_quality < quality_cap) {
    new_q = g_quality + step;   // 加大压缩 -> 单帧更小 -> 带宽不变时帧率提升
  } else if (!cross_net && fps > target_fps + 1.5f && !congested
             && kbps < 650.0f && g_quality > 8) {
    new_q = g_quality - 1;   // 有余量才回画质(逐档, 防过冲) —— 仅局域网
  }

  if (new_q != g_quality) {
    g_quality = new_q;
    sensor_t *s = esp_camera_sensor_get();
    if (s) s->set_quality(s, g_quality);
    Dbg.printf("AUTOQ quality=%d fps=%.1f kb=%.0f/s err=%u evc=%u\n",
               g_quality, fps, kbps, err_delta, evict_delta);
  }
  g_adapt_last_ms = now;
  g_adapt_last_sent = g_adapt_total_frames;
  g_adapt_last_bytes = g_adapt_total_bytes;
  g_adapt_last_err = g_adapt_total_err;
  g_adapt_last_evict = g_adapt_total_evict;
}

// ---- UDP 诊断心跳 ----
// 独立于相机产帧: 即使 esp_camera_fb_get 持续失败、sender 无帧可发(0 fps),
// 也能通过 UDP 把状态上报给最近客户端, 精确定位是"采集失败"还是"发送失败"。
// 用专用 socket(不与 cam_udp 分帧冲突), 发往最近 cam_client 的 IP:TCP_PORT。
// 包格式: ASCII 明文, 首字节 'D' 前缀 "DIAG:" 便于上位机区分帧流与诊断包。
static WiFiUDP diag_udp;

void diag_beat_task(void *arg) {
  (void)arg;
  diag_udp.begin(TCP_PORT + 1);   // 独立端口(避与 cam_udp 的 TCP_PORT 冲突)
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(3000));   // 3s 心跳
    send_diag_beat();
  }
}

void send_diag_beat() {
  if (xSemaphoreTake(g_cli_mutex, pdMS_TO_TICKS(100)) != pdTRUE) return;
  WiFiClient snap = cam_client;
  IPAddress dst;
  bool have = snap && snap.connected();
  if (have) dst = snap.remoteIP();
  xSemaphoreGive(g_cli_mutex);
  if (!have) return;

  char buf[256];
  int rssi = WiFi.RSSI();
  int n = snprintf(
      buf, sizeof(buf),
      "DIAG:cam_ready=%d cap=%u sent=%u uerr=%u rty=%u skp=%u evc=%u "
      "nc=%u bgd=%u err=%d q=%d auto=%d rssi=%d udp=%d heap=%u\n",
      g_cam_ready ? 1 : 0,
      g_diag_cap_frames, g_diag_sent_frames, g_diag_udp_err, g_diag_udp_retry_ok,
      g_diag_skip_frames, g_diag_evict_frames, g_diag_nc_frames, g_diag_bigdrop_frames,
      g_diag_last_err, g_quality, g_auto_quality ? 1 : 0, rssi,
      udp_mode ? 1 : 0, (unsigned)ESP.getFreeHeap());
  if (n < 0 || n >= (int)sizeof(buf)) return;
  log_i("DIAG %s", buf);   // esp_log 控制台(COM10)无条件可见, 便于定位心跳/采集状态
  // 双通道上报: UDP 专属诊断口(发往客户端 IP) + cam_client 的 TCP 通道。
  // TCP 通道一定能到达(除非 TCP 接收方不读会被写缓冲, 但诊断稀疏不阻塞预览)。
  if (snap && snap.connected()) {
    snap.write((const uint8_t *)buf, n);
  }
  diag_udp.beginPacket(dst, TCP_PORT);
  diag_udp.write((const uint8_t *)buf, n);
  diag_udp.endPacket();
}
