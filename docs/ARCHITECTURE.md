# OV2640 + ESP32-S3 无线摄像头系统 — 体系结构

> 本文档描述系统的总体架构、模块划分、帧流水线、传输通道与关键机制，是论文与技术交接的体系级参考。
> 器件选型依据（OV2640 / ESP32-S3）见 [SELECTION.md](./SELECTION.md)；问题根因与解决见 [SOLUTIONS.md](./SOLUTIONS.md)；已识别取舍的分类（可落地改进 / 需换约束）见 [SOLUTIONS.md 六、局限性 / 未来工作](./SOLUTIONS.md)。

---

## 1. 系统概述

基于 **ESP32-S3 + OV2640** 的低时延无线视频采集与传输系统。

- **采集**：OV2640 DVP 接口，硬件 JPEG 编码，PSRAM 帧缓冲，RGB565 可由编译开关切换。
- **主控**：ESP32-S3（双核 240MHz，8MB OPI PSRAM），FreeRTOS（Arduino 框架）。
- **传输**：四类通道并存 —— 串口 UART0 2Mbaud、局域网 WiFi TCP、局域网 WiFi UDP（高帧率）、公网中继（跨网，含 P2P/TURN）。
- **上位机**：Python3 + tkinter + Pillow，`udp_capture.py` 实时预览/分辨率切换/一键拍照。

```
                    ┌────────────────────────────────────────────┐
   OV2640 ──DVP──▶ │ ESP32-S3                                    │
   (JPEG/RGB565)   │  cam/loop(核1)  ── enqueue_frame ──▶ g_txq ─┼──▶ sender_task(核0)
                   │                                            │        │
                   │                                            │        ├─▶ 串口 UART0 (2M)
                   │  relay_task / disc_task / p2p (核0) ────────┼────────▶ Wi-Fi TCP/UDP
                   └────────────────────────────────────────────┘        └─▶ 公网中继 / P2P/TURN
                                              │ (中继命令下行)
                                              ▼
                                   relay_server.py (云)
                                              │
                                              ▼
                                   host/udp_capture.py (终端，含单独诊断脚本)
```

---

## 2. 目录结构

- `esp32s3_camera_pio/` — **当前固件**（PlatformIO，从源码编译 esp32-camera，解锁 JPEG）
  - `platformio.ini` — 编译配置；`src/main.cpp` — 固件主体
- `esp32s3_camera_serial/host/` — **Python 上位机工具集**
  - `udp_capture.py` — 主预览程序（串口 / WiFi / UDP / 中继均可）
  - `diagnostics/` — 位级/色彩/结构诊断脚本（曾定位花屏=坏数据线）
  - `archive/` — 旧 Arduino `.ino` 固件与历史维护文档（归档存证）
- `relay_server.py` — **跨网信令 + 中继服务端**（云服务器运行）
- `deploy/` — 中继服务部署与量化测试工具（`relay_fps.py`、`test_switch.py` 等）
- `docs/` — 本体系结构与解决方案文档
- `tools/` — 本地工具链（arduino-cli 等，不入库）

---

## 3. 固件模块（`src/main.cpp`）

### 3.1 任务划分与核亲和

| 任务 | 核 | 优先级 | 职责 |
|------|----|--------|------|
| `loop()` + 采集（Arduino loopTask） | 1 | — | 抓帧 `esp_camera_fb_get`，拷入 PSRAM 队列；处理串口/TCP 命令；Web 配置页 |
| `sender_task` | 0 | 3 | 出队 → 按通道发送帧 → 记账 → `adapt_quality()` |
| `relay_task` | 0 | 1 | 中继链路：连接/注册/心跳，P2P 打洞状态机，TURN 降级，命令下行透传 |
| `disc_task` | 0 | 1 | 局域网 UDP 广播自动发现 |

**约束**：
- 采集必须固定在核1（`-DCONFIG_CAMERA_CORE1`）。默认 NO_AFFINITY 时，切换分辨率 `deinit` 跨核与 `cam_task` 拷贝并发，释放 `cam_obj` 会产生 use-after-free 卡死。
- 网络任务（sender/relay/disc）固定在核0，避免抢占核1 的实时采集/控制。

### 3.2 帧流水线（核心）

```
capture(核1)                    sender(核0)
  │  esp_camera_fb_get()          │  xQueueReceive(g_txq, portMAX_DELAY)
  ▼                               ▼
enqueue_frame(fb):             按当前模式选通道：
  1. frame_alloc(PSRAM 池)         │ UDP 局域网 → send_frame_udp
  2. memcpy 拷帧, 立即 fb_return   │ TCP 局域网 → send_frame_locked
  3. g_txq 判满 → 丢【最旧】帧    │ P2P/TURN  → send_frame_udp*
  4. xQueueSend 最新帧入队         │ 中继     → send_frame_relay
                                   ▼ frame_free + adapt_quality()
```

- **丢旧保新队列**：`g_txq`（FreeRTOS 队列，深度 `TXQ_DEPTH`）+ PSRAM 帧池。满时 pop `xQueueReceive` 最旧帧腾位给最新帧，保证观众看到的是实时而非积压旧帧。采集侧始终满帧率入队，通信延迟不拖慢采集。
- **背压信号**：队列被顶帧数 `g_adapt_total_evict` 作为拥塞信号，喂给 `adapt_quality()` 动态调 JPEG 压缩，跨网带宽不足时自动降画质保帧率。
- **竞态安全**：所有发送路径均非阻塞入队、仅接收阻塞；多核共享的 `cam_client`/UDP 由 `g_cli_mutex` 保护；中继发送由 `g_relay_tx` 信号量串行化。

### 3.3 帧协议

局域网/串口帧（JPEG 或 RGB565，帧头携带格式与尺寸）：

```
FRAME_BEGIN\n
SIZE <w> <h> <JPG|RGB>\n
<len 字节净荷>
FRAME_END\n
```

- 上位机按正则 `FRAME_BEGIN\nSIZE ...\n(.*?)FRAME_END\n` 切帧；`JPG` 校验 `FF D8`/`FF D9` 后解码，`RGB` 按 big-endian 解析 RGB565。
- 中继（`relay_server.py`）采用 **4 字节大端长度前缀 + payload** 的透传协议，不解析视频帧，因此与局域网帧格式解耦。**任何消息不允许中途放弃**，否则整条流错位（见 SOLUTIONS）。

### 3.4 分辨率与自适应缓冲

- 6 档分辨率码：`q` QQVGA 160×120 / `t` QVGA 320×240 / `v` VGA / `s` SVGA / `x` XGA / `u` UXGA 1600×1200。
- `fb_count_for(fs)`：单帧字节 ×2 ≤ 6MB → 双缓冲，否则单缓冲。UXGA RGB565 单帧 3.84MB×2≈7.7MB 逼近 PSRAM 上限，因此大图必须单缓冲防 OOM。
- `init_camera_as(target, fallback)` 失败自动降级：先强制单缓冲重试，再回退 fallback 分辨率，绝不死锁。
- **快速切换（2026-09 新增）**：QQVGA↔QVGA（CIF 系、fb_count=2、缓冲在开机按大图分配）走寄存器级 `set_framesize`，毫秒级切换；涉及 VGA 及以上的仍走 `deinit+init` 全量重建（VGA 触发库 EV-VSYNC-OVF bug，XGA/UXGA 需 fb_count=1 重分配缓冲）。

### 3.5 命令协议

| 命令 | 功能 |
|------|------|
| `P` | 拍照：切拍照分辨率抓一帧，完成后切回预览 |
| `V<码>` / `H<码>` | 设预览/拍照分辨率（立即/下次生效） |
| `U` / `N` | 开/关 UDP 预览模式 |
| `q<n>` | 调 JPEG 质量（0-63） |
| `a` | 开/关自适应画质 |
| `r` | 手动重连摄像头 |

---

## 4. 传输通道

### 4.1 串口（基线，2Mbaud）
数据经 UART0（CH343，GPIO43/44），`Debug Serial Dbg(0)`。做帧率/可靠性基准，同时是裸板无网环境的兜底。

### 4.2 局域网 WiFi
- STA 连路由器（STA 优先，SoftAP `OV2640Cam` 兜底），TCP 监听 `0.0.0.0:3333`。
- **UDP 高帧率预览**：12B 自定义头 + 1388B 分片，丢片即丢帧不重传。小帧（≤4 片）"等队列排空"轮询重试 12ms，丢包率 2%→0。
- **TCP 可靠**：大帧拍照与命令。16KB/块分块写。
- **空口优化**：`WiFi.setSleep(false)` 关 modem sleep；流媒体期间 `softAPdisconnect()` 释放空口给 STA（同信道），VGA 容量近乎翻倍。

### 4.3 局域网自动发现（`--auto`）
摄像头每 3s 向 `255.255.255.255:9999` UDP 广播 `OV2640|名字|IP`，上位机监听几秒自动直连。

### 4.4 跨网中继（`--relay`）
- 摄像头常驻 TCP 连接公网 `relay_server.py` 并注册 id（默认 `cam-<MAC尾>`）；有观众 `watch` 时上行帧、透传下行命令。
- 无本地客户端时服务中继观众；一台摄像头同一时刻服务一个观看端。
- 服务端只做 `4B 长度前缀 + payload` 字节透传，与局域网帧协议兼容。

### 4.5 P2P 打洞 / TURN 降级
- 双端各开多个 UDP 候选端口并 STUN 上报公网端点，服务器交叉撮合打洞；对称/端口受限 NAT 下打洞失败，5s 超时降级 **TURN** UDP 转发，再失败回退纯 TCP 中继。
- 状态机以"多久没收到观众的 magic 包"为判活信号，支持直连/TURN/TCP 三态平滑迁移。

---

## 5. 上位机架构（`host/udp_capture.py`）

- **传输通道抽象**：位置参数 + 开关选择串口 / `--wifi` / `--auto` / `--relay`，共用同一 `FRAME/SIZE/END` 帧解析。
- **最新帧优先队列**：`queue.Queue(maxsize=2)` + `put_latest()`，满则丢最旧，渲染永不滞后。
- **实时帧率统计**：每秒结算窗口，界面底部显示当前 fps。
- **三级自动重连**：串口异常重开 → 5s 无帧强制重连 → 重连首帧后 `resync_board()` 重新下发分辨率，兜底裸板触碰短路复位。
- **色彩看门狗**：B 通道显著偏蓝持续 3s 自动下发 `r` 完整重建（应对源码版 esp32-camera 的 AWB 表兼容问题，约 5s 自愈）。
- **远端解析约束**：RGB565 必须 big-endian：`v = (b[i]<<8) | b[i+1]`；JPG 校验 SOI/EOI。切勿改回 little-endian（整体花屏）。

---

## 6. 性能基准（实测）

UDP 传输，JPEG quality=16，2.4GHz 安静信道（见 SOLUTIONS 的"信道拥挤"说明）：

| 分辨率 | 传感器上限 | 局域网 UDP | 跨网 TCP 中继 |
|--------|-----------|-----------|--------------|
| QQVGA 160×120 | 30fps | ~29fps | ~29fps |
| QVGA 320×240  | 30fps | ~29fps | **19~20fps**（90KB/s 自适应画质）|
| VGA 640×480   | 15fps | ~15fps | — |

- 拍照：UXGA 1600×1200 JPEG，本地 WiFi 约 0.3s/帧，串口 2M 约 22s/帧。
- 中继下 QVGA 帧率受公网带宽与上下行限制在 ~19-20fps；带宽不足时自适应压缩压到目标帧率。