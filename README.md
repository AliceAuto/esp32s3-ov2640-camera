# ESP32-S3 + OV2640 无线摄像头系统

基于 **ESP32-S3 + OV2640** 的低时延无线视频采集系统。支持串口 / 局域网 WiFi（TCP/UDP）/ 跨网中继（含 P2P/TURN）四类传输，QVGA 预览跨网稳定 **\~19fps**、局域网满帧率，UXGA(1600×1200) 一键拍照。

***

## 文档索引

| 文档                                           | 内容                                                              |
| -------------------------------------------- | --------------------------------------------------------------- |
| [docs/SELECTION.md](docs/SELECTION.md)       | 选型论证：OV2640 + ESP32-S3 选型依据与竞品对比                                |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | 体系结构：模块划分、帧流水线、传输通道、任务调度、性能基准                                   |
| [docs/SOLUTIONS.md](docs/SOLUTIONS.md)       | 问题与解决方案：全部踩坑的根因、诊断与解决（论文级工程复盘），含[局限性/未来工作](docs/SOLUTIONS.md)分类 |
| [INTERVIEW.md](INTERVIEW.md)                 | 10 道关键技术决策的深度 QA                                                |

***

## 仓库结构

```
ov2640/
├── firmware/                    # 固件（PlatformIO，源码编译 esp32-camera，解锁 JPEG）
│   └── src/main.cpp             # 固件主体
├── host/                        # 上位机（入口 main.py + 模块库 lib/）
│   ├── main.py                  # 主预览程序（WiFi TCP/UDP / 跨网中继 / 自动发现）
│   └── lib/                     # 模块库（core 状态/reader 收流/gui 界面/protocol/切帧/中继/主题等）
├── server/                      # 云中继服务端（跨网信令 + P2P/TURN 转发）
│   └── relay_server.py          # 中继服务端主体（云服务器运行）
├── deploy/                      # 部署与量化测试工具（relay_fps.py、test_switch.py 等）
├── docs/                        # 体系结构与解决方案文档
├── tools/                       # 工具脚本 + 本地工具链
│   ├── preview_serial.py        # 串口轮询拍照预览
│   ├── capture_serial.py        # 串口取图保存
│   ├── fps_test.py              # 局域网 UDP/TCP 帧率对比测试
│   └── (arduino-cli 等工具链，不入库)
└── vendor/                      # 厂商资料（选型/手册/参考代码，不入库）
```

***

## 快速开始

### 初始化开发环境

仓库已提供 VS Code 工作区配置。首次使用时，在仓库根目录执行：

```powershell
py -3.11 -m venv .venv
.venv\Scripts\python.exe -m pip install -r requirements.txt
```

推荐安装扩展：PlatformIO IDE、Python、Pylance、C/C++。打开仓库后，可直接运行 VS Code 任务“主机端: 协议自检”或“固件: 编译”。

### 编译 / 烧录固件（PlatformIO）

```bash
cd firmware
pio run                 # 编译
pio run -t upload       # 自动探测并烧录已连接的开发板
pio run -t upload --upload-port COM7  # 需要时手动指定串口
```

在 VS Code 中选择“固件: ESP32-S3 调试”可启动 PlatformIO 调试。该功能需要支持 JTAG 的 ESP32-S3 调试器或板载 USB-JTAG；仅连接普通串口时可烧录和看日志，但不能断点调试。

### 运行预览

```bash
cd host

# 跨网中继观看（推荐，需云中继在线）
python main.py --relay <你的中继地址>:8001 --id cam-0001

# 局域网 WiFi（TCP 可靠 / UDP 高帧率）
python main.py --wifi <摄像头IP> 3333

# 局域网优先，失败自动回退中继
python main.py --lanrelay

# 串口轮询拍照预览（2Mbaud 基线，见 tools/preview_serial.py）
python ../tools/preview_serial.py COM7
```

### 中继服务端（云服务器）

```bash
python server/relay_server.py 8001 [token]   # TCP 8001 放行；token 可选鉴权
```

***

## 技术要点速览

- **JPEG 空帧绕过**：Arduino 预编译库对 JPEG SOI 校验过严 → 改用 PlatformIO 从源码编译 `espressif/esp32-camera`。

- **实时性**：UDP 分片丢帧不重传 + 最新帧优先队列，保证低时延；拍照走可靠 TCP。

- **帧率优化**：关 modem sleep、流媒体期间关软 AP、分帧大小差异化重试、自适应 JPEG 压缩（拥塞自动降质保帧率）。

- **稳定性**：采集固定核1、网络固定核0；分辨率 QQVGA↔QVGA 寄存器级快速切换（\~1s→\~130ms）；中继长度前缀流严禁中途放弃。

- **诊断**：板载 DIAG + 位级激活率统计，快速区分编码/链路/曝光瓶颈。

> 注意：**一台摄像头同一时刻只用一个观看进程**（多客户端会"抢占式"互顶）。OV2640 定焦、无长曝光，不具备拍星能力。

完整设计决策与性能数据见 [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)，全部踩坑复盘见 [docs/SOLUTIONS.md](docs/SOLUTIONS.md)。
