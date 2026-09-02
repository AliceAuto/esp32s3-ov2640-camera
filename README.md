# ESP32-S3 + OV2640 无线摄像头系统

基于 **ESP32-S3 + OV2640** 的低时延无线视频采集系统。支持串口 / 局域网 WiFi（TCP/UDP）/ 跨网中继（含 P2P/TURN）四类传输，QVGA 预览跨网稳定 **~19fps**、局域网满帧率，UXGA(1600×1200) 一键拍照。

---

## 文档索引

| 文档 | 内容 |
|------|------|
| [docs/SELECTION.md](docs/SELECTION.md) | 选型论证：OV2640 + ESP32-S3 选型依据与竞品对比 |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | 体系结构：模块划分、帧流水线、传输通道、任务调度、性能基准 |
| [docs/SOLUTIONS.md](docs/SOLUTIONS.md) | 问题与解决方案：全部踩坑的根因、诊断与解决（论文级工程复盘），含[局限性/未来工作](docs/SOLUTIONS.md)分类 |
| [INTERVIEW.md](INTERVIEW.md) | 10 道关键技术决策的深度 QA |
| [esp32s3_camera_serial/host/diagnostics/](esp32s3_camera_serial/host/diagnostics/) | 开发期诊断脚本索引（位级/色彩/结构） |

---

## 仓库结构

```
ov2640/
├── esp32s3_camera_pio/          # 当前固件（PlatformIO，源码编译 esp32-camera，解锁 JPEG）
│   └── src/main.cpp             # 固件主体
├── esp32s3_camera_serial/host/  # Python 上位机工具集
│   ├── udp_capture.py           # 主预览程序（串口/WiFi/UDP/中继）
│   └── diagnostics/             # 开发期诊断脚本（归档索引）
├── relay_server.py              # 跨网信令 + 中继服务端（云服务器）
├── deploy/                      # 部署与量化测试工具（relay_fps.py、test_switch.py 等）
├── docs/                        # 体系结构与解决方案文档
└── archive/                     # 旧 Arduino .ino 固件 + 历史维护文档（存证）
```

> `esp32s3_camera_serial` 虽沿用旧名，现仅承载上位机 host 工具；旧的 Arduino 固件与维护文档已归档至 `archive/esp32s3_camera_serial/`。

---

## 快速开始

### 编译 / 烧录固件（PlatformIO）

```bash
cd esp32s3_camera_pio
pio run                 # 编译
pio run -t upload       # 烧录（platformio.ini: upload_port = COM10）
```

### 运行预览

```bash
cd esp32s3_camera_serial/host

# 跨网中继观看（推荐，需云中继在线）
python udp_capture.py --relay 8.148.79.85:8001 --id cam-1DCD23D8

# 局域网 WiFi（TCP 可靠 / UDP 高帧率）
python udp_capture.py --wifi 192.168.10.118 3333

# 局域网优先，失败自动回退中继
python udp_capture.py --lanrelay

# 串口（2Mbaud 基线）
python udp_capture.py COM10
```

### 中继服务端（云服务器）

```bash
python relay_server.py 8001 [token]   # TCP 8001 放行；token 可选鉴权
```

---

## 技术要点速览

- **JPEG 空帧绕过**：Arduino 预编译库对 JPEG SOI 校验过严 → 改用 PlatformIO 从源码编译 `espressif/esp32-camera`。
- **实时性**：UDP 分片丢帧不重传 + 最新帧优先队列，保证低时延；拍照走可靠 TCP。
- **帧率优化**：关 modem sleep、流媒体期间关软 AP、分帧大小差异化重试、自适应 JPEG 压缩（拥塞自动降质保帧率）。
- **稳定性**：采集固定核1、网络固定核0；分辨率 QQVGA↔QVGA 寄存器级快速切换（~1s→~130ms）；中继长度前缀流严禁中途放弃。
- **诊断**：板载 DIAG + 位级激活率统计，快速区分编码/链路/曝光瓶颈。

> 注意：**一台摄像头同一时刻只用一个观看进程**（多客户端会"抢占式"互顶）。OV2640 定焦、无长曝光，不具备拍星能力。

完整设计决策与性能数据见 [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)，全部踩坑复盘见 [docs/SOLUTIONS.md](docs/SOLUTIONS.md)。