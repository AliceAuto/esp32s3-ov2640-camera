/*
 * 相机管理实现(自 main.cpp 拆出)。
 * 摄像头配置全局 g_config 定义在此(setup 直接操作其成员, 声明见 globals.h)。
 * g_cam_mutex 由 setup(main.cpp) 创建, 此处通过 globals.h extern 访问。
 */
#include "camera.h"

// 摄像头配置全局保存，供失锁后 deinit+reinit 复用
camera_config_t g_config;

// 相机驱动互斥锁访问: deinit/init 重建与采集(fb_get/fb_return) 分处不同核(中继命令在核0、
// 采集 loop 在核1), 双核并发访问 cam_obj 会在 deinit 释放后仍在 fb_get/return 引用
// → use-after-free 卡死(与分辨率无关, 切任意都卡)。所有 esp_camera_* 调用必须以它串行化。
bool cam_lock(uint32_t timeout_ms) {
  return g_cam_mutex && xSemaphoreTake(g_cam_mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}
void cam_unlock() { if (g_cam_mutex) xSemaphoreGive(g_cam_mutex); }

// 依据分辨率自适应双缓冲数量（大图降为1，防 PSRAM OOM 卡死）
int fb_count_for(framesize_t fs) {
  if (fs < 0 || fs >= FRAMESIZE_INVALID) return 1;
  int w = resolution[fs].width;
  int h = resolution[fs].height;
  int64_t one = (int64_t)w * h * BYTES_PER_PIXEL;
  int64_t two = one * 2;
  if (two <= MAX_FB_BYTES) return 2;
  return 1;
}

// 分辨率字符码 -> framesize_t。q=QQVGA,t=QVGA,v=VGA,s=SVGA,x=XGA,u=UXGA
framesize_t framesize_from_code(char code) {
  switch (code) {
    case 'q': return FRAMESIZE_QQVGA;
    case 't': return FRAMESIZE_QVGA;
    case 'v': return FRAMESIZE_VGA;
    case 's': return FRAMESIZE_SVGA;
    case 'x': return FRAMESIZE_XGA;
    case 'u': return FRAMESIZE_UXGA;
    default:  return FRAMESIZE_QQVGA;
  }
}

// 每次 reinit 后重设传感器画质参数（deinit+init 会复位寄存器）
void apply_color_calib() {
  sensor_t *s = esp_camera_sensor_get();
  if (!s) { Dbg.println("警告：获取 sensor 失败"); return; }
  s->set_whitebal(s, 1);      // AWB
  s->set_gain_ctrl(s, 1);     // AGC
  s->set_exposure_ctrl(s, 1);
  // JPEG 质量(0-63, 越小画质越高文件越大), 运行时 'q' 命令可调
  s->set_quality(s, g_quality);
  uint8_t clkrc_init = (uint8_t)s->get_reg(s, 0x0011, 0xFF);
  Dbg.printf("CLKRC init=0x%02X\n", clkrc_init);
  delay(200);                 // 给 AWB/AGC 收敛时间
}

// 以目标分辨率重建摄像头（失败自动降级：先单缓冲，再回退 fallback）
bool init_camera_as(framesize_t target, framesize_t fallback) {
  g_config.frame_size = target;
  g_config.fb_count   = fb_count_for(target);
  esp_err_t err = esp_camera_init(&g_config);
  if (err == ESP_OK) {
    cur_frame = target;
    Dbg.printf("摄像头初始化: fs=%u fb=%d\n", (unsigned)target, g_config.fb_count);
    apply_color_calib();
    return true;
  }
  Dbg.printf("init fs=%u 失败 err=0x%x, 尝试单缓冲...\n", (unsigned)target, err);
  // 失败的 init 可能残留传感器/DMA/SCCB 状态，必须先 deinit 再重试，
  // 否则二次 init 大概率同样失败，会被误判为内存不足而错误降级
  esp_camera_deinit();
  delay(100);
  g_config.fb_count = 1;
  err = esp_camera_init(&g_config);
  if (err == ESP_OK) {
    cur_frame = target;
    Dbg.printf("摄像头初始化(单缓冲): fs=%u fb=1\n", (unsigned)target);
    apply_color_calib();
    return true;
  }
  Dbg.printf("单缓冲也失败 err=0x%x, 回退 fs=%u\n", err, (unsigned)fallback);
  esp_camera_deinit();
  delay(100);
  g_config.frame_size = fallback;
  g_config.fb_count   = fb_count_for(fallback);
  err = esp_camera_init(&g_config);
  if (err == ESP_OK) {
    cur_frame = fallback;
    Dbg.printf("已回退到 fs=%u\n", (unsigned)fallback);
    apply_color_calib();
    return true;
  }
  Dbg.printf("回退也失败 err=0x%x\n", err);
  return false;
}

// 失锁后重新初始化（先 deinit 释放 DMA/缓冲，再重建为当前分辨率）
bool camera_reconnect() {
  Dbg.println("摄像头失锁, 正在重连...");
#ifdef ENABLE_WIFI
  g_switching = true;
  wait_tx_idle(200);
  if (!cam_lock(1500)) {        // 与采集/切换互斥; 拿不到锁则放弃本次重连, 下轮采集自动重试
    g_switching = false;
    return false;
  }
  esp_camera_deinit();
  delay(300);
  g_switching = false;
  bool ok = init_camera_as(cur_frame, PREVIEW_FRAME);
  cam_unlock();
  return ok;
#else
  esp_camera_deinit();
  delay(300);
  return init_camera_as(cur_frame, PREVIEW_FRAME);
#endif
}

// 真正的分辨率切换：deinit + init 全量重配，失败自动回退
bool camera_change_framesize(framesize_t fs) {
  if (fs == cur_frame) { return true; }
  // 快路径: 同为 CIF 系小图(≤QVGA 320x240)、fb_count 均为 2 时, 缓冲区在开机按大图
  // 已分配、足以容纳两者(含 QCIF/HQVGA 等)。用寄存器级 set_framesize 毫秒切换,
  // 避开 deinit+init 全量重建造成的 ~0.5-1s 黑屏/冻结。
  // 必须避开 VGA(触发库 EV-VSYNC-OVF 死循环 bug)与 SVGA/XGA/UXGA(fb_count=1,
  // 必须重分配缓冲) —— 这些仍走下方慢路径。
  if (fs <= FRAMESIZE_QVGA && cur_frame <= FRAMESIZE_QVGA
      && fb_count_for(fs) == 2 && fb_count_for(cur_frame) == 2) {
    sensor_t *s = esp_camera_sensor_get();
    if (s && s->set_framesize(s, fs) == 0) {
      cur_frame = fs;
      g_config.frame_size = fs;
      Dbg.printf("预览分辨率快速切换: fs=%d\n", (int)fs);
      return true;
    }
    Dbg.println("快速切换失败 -> 回退全量重建");
  }
#ifdef ENABLE_WIFI
  g_switching = true;
  wait_tx_idle(500);            // 大分辨率下最后一帧可能近100ms, 留足排空时间
  if (!cam_lock(1000)) {        // 与核1采集 fb_get/fb_return 双核互斥, 防 deinit 释放后仍被引用
    Dbg.println("切换取消: 相机锁忙(采集阻塞), 稍后重试");
    g_switching = false;
    return false;
  }
  esp_camera_deinit();
  delay(200);
  bool ok = init_camera_as(fs, cur_frame);
  cam_unlock();
  if (!ok) {
    // 三层重配全失败: 传感器/驱动可能已进入不可恢复状态, 直接复位保证不永久卡死
    Dbg.println("FSW FAIL -> restart");
    delay(100);
    g_switching = false;
    ESP.restart();
  }
  g_switching = false;
#else
  esp_camera_deinit();
  delay(200);
  bool ok = init_camera_as(fs, cur_frame);
#endif
  return ok;
}
