# ESP32-S3 + OV2640 高帧率无线摄像头系统

> 基于 ESP32-S3 + OV2640 的局域网实时视频流系统，实现 UDP/TCP 双通道传输，QVGA 预览达 **30fps 满帧率**，UXGA(1600×1200) 拍照走可靠 TCP。

---

## 系统架构

```
┌─────────────────────────────────────────────────┐
│                 ESP32-S3 主控                     │
│                                                   │
│  OV2640 (DVP)  ──DMA──>  PSRAM 帧缓冲            │
│  24MHz XCLK               (自适应 1~2 缓冲)       │
│  SCCB(I2C)                          │             │
│                                     ▼             │
│                          ┌──────────────────┐     │
│                          │   JPEG 编码器     │     │
│                          │  (OV2640 硬件)    │     │
│                          └────────┬─────────┘     │
│                                   │               │
│                    ┌──────────────┼──────────┐    │
│                    ▼              ▼          ▼    │
│              ┌──────────┐  ┌──────────┐  ┌────┐  │
│              │ UDP 预览  │  │ TCP 拍照  │  │UART│  │
│              │ (高帧率)  │  │ (可靠)    │  │2M  │  │
│              │ 1388B/片  │  │ 16KB/块  │  │baud│  │
│              └─────┬────┘  └────┬─────┘  └──┬─┘  │
│                    │            │            │     │
│              WiFi STA/AP    WiFi STA/AP   CH343   │
└────────────────────┼────────────┼────────────┼────┘
                     │            │            │
                     ▼            ▼            ▼
┌────────────────────────────────────────────────┐
│              Python 上位机 (host/)               │
│                                                  │
│  udp_capture.py   live_capture.py  preview.py    │
│  (UDP+TCP GUI)    (TCP GUI+拍照)   (串口 GUI)   │
│                                                  │
│  tkinter 窗口: 实时画面 + 分辨率切换 + 一键拍照   │
│  自动重连 + 分辨率重同步 + 帧率统计               │
└────────────────────────────────────────────────┘
```

## 核心特性

| 特性 | 实现 |
|------|------|
| **JPEG 源码编译** | 绕过 Arduino 预编译库 JPEG 空帧 bug (FB-OVF)，从源码编译 esp32-camera |
| **UDP/TCP 双通道** | UDP 丢帧不重传→高帧率预览；TCP 可靠传输→拍照/命令 |
| **自适应帧缓冲** | 按分辨率动态 1~2 缓冲（UXGA 单缓冲防 PSRAM OOM，小图双缓冲提升吞吐） |
| **失锁自动恢复** | 连续 2 次抓帧失败→deinit+reinit 重建，失败自动降级（双缓冲→单缓冲→fallback 分辨率） |
| **AP/STA 双模 WiFi** | STA 连局域网（推荐）+ AP 热点兜底，流媒体期间关 AP 释放空口 |
| **损坏引脚容错** | GPIO11/12 物理损坏→D0 改接 GPIO1，D4 改接 GPIO2，位级诊断定位 |
| **板载 DIAG** | 每 5s 输出采集/发送速率、丢片率、WiFi RSSI、错误码，一眼区分瓶颈 |
| **自适应 JPEG 质量** | UDP 模式自动调节 quality(0-63)，大分辨率降低质量换帧率 |

## 性能数据

UDP 传输，JPEG quality=10，2.4GHz 信道安静时段实测：

| 分辨率 | 传感器上限 | 实测帧率 | 丢片 | 帧长 |
|--------|-----------|---------|------|------|
| QQVGA 160×120 | 30fps | **29.4fps** | 0 | 1.7KB |
| QVGA 320×240 | 30fps | **29.1fps** | 0 | 5.0KB |
| VGA 640×480 | 15fps | **14.9fps** | 0 | 12.7KB |
| SVGA 800×600 | 15fps | ~2-6fps | 信道受限 | 20KB |

拍照模式（TCP 可靠传输）：UXGA 1600×1200 JPEG，WiFi ~0.3s/帧。

## 硬件平台

| 组件 | 型号 | 说明 |
|------|------|------|
| 主控 | ESP32-S3-DevKitC-1 | 8MB OPI PSRAM，240MHz 双核 |
| 摄像头 | OV2640 红版模块 | 200万像素 DVP，定焦，硬件 JPEG 编码 |
| USB转串口 | CH343 | UART0 (GPIO43/44)，2Mbaud 调试口 |

### 接线（OV2640 → ESP32-S3）

```
OV2640         ESP32-S3        说明
─────         ─────────       ────
3.3V     →    3V3             供电
GND      →    GND             共地
PWDN     →    GND             拉低使能
SCC_SCL  →    GPIO5           I2C 时钟
SCC_SDA  →    GPIO4           I2C 数据
VSYNC    →    GPIO6           帧同步
HREF     →    GPIO7           行参考
PCLK     →    GPIO13          像素时钟
XCLK     →    GPIO15          传感器时钟
D7       →    GPIO16          数据位7
D6       →    GPIO17          数据位6
D5       →    GPIO18          数据位5
D4       →    GPIO2*          数据位4 (原GPIO12损坏→改接)
D3       →    GPIO10          数据位3
D2       →    GPIO8           数据位2
D1       →    GPIO9           数据位1
D0       →    GPIO1*          数据位0 (原GPIO11损坏→改接)
```

> *GPIO11/12 物理损坏，通过位级诊断定位后改接 GPIO1/GPIO2。

## 项目结构

```
ov2640/
├── esp32s3_camera_pio/          # PlatformIO 工程（JPEG 源码编译版，主推）
│   ├── platformio.ini
│   └── src/main.cpp             # 固件主体（~650行）
├── esp32s3_camera_serial/       # Arduino IDE 工程（RGB565 基线 + 全量文档）
│   ├── esp32s3_camera_serial.ino
│   ├── host/                    # Python 上位机工具集
│   │   ├── udp_capture.py       # UDP 高帧率预览 GUI
│   │   ├── live_capture.py      # TCP 实时预览 + 拍照 GUI
│   │   ├── preview_serial.py    # 串口预览 GUI
│   │   ├── verify_wifi.py       # WiFi 无头验证
│   │   ├── diag_*.py            # 位级/色彩/结构诊断工具集
│   │   └── photos/              # 拍照输出目录
│   └── README.md                # 详细维护文档（含开发历程）
├── tools/                       # arduino-cli + 本地包索引
└── 资料/                        # OV2640 数据手册
```

## 快速开始

### 编译烧录（PlatformIO）

```bash
cd esp32s3_camera_pio
pio run              # 编译
pio run -t upload    # 烧录到 COM7
```

### 运行上位机

```bash
cd esp32s3_camera_serial/host

# WiFi UDP 高帧率预览（推荐）
python udp_capture.py --wifi <板子IP> 3333

# WiFi TCP 实时预览 + 拍照
python live_capture.py --wifi <板子IP> 3333

# 串口预览
python preview_serial.py COM7 2000000
```

### 串口命令协议

| 命令 | 功能 |
|------|------|
| `P` | 拍照：切 UXGA 抓一帧 JPEG 发送，完成后切回预览 |
| `V<码>` | 设置预览分辨率（立即生效） |
| `H<码>` | 设置拍照分辨率（下次 P 命令生效） |
| `U` / `N` | 开启/关闭 UDP 预览模式 |
| `q<0-63>` | 调节 JPEG 质量（0=最高画质，63=最小体积） |
| `r` | 手动重连摄像头 |

分辨率码：`q`=QQVGA, `t`=QVGA, `v`=VGA, `s`=SVGA, `x`=XGA, `u`=UXGA

## 关键技术深度

### 1. JPEG 空帧 bug 的根因与绕过

Arduino 预编译库 `libespressif__esp32-camera.a` 的 JPEG 编码路径对 SOI(0xFFD8) 起始标记校验过严，OV2640 输出的 JPEG 帧头不满足其内部断言，导致 `esp_camera_fb_get()` 返回空帧并报 `FB-OVF`。从源码编译 esp32-camera（PlatformIO `lib_deps = espressif/esp32-camera`）后，可修改 SOI 校验逻辑，彻底绕过此问题。

### 2. PSRAM 自适应帧缓冲策略

8MB OPI PSRAM 是有限资源。UXGA RGB565 单帧 = 1600×1200×2 = 3.84MB，双缓冲需 7.7MB，逼近上限。设计 `fb_count_for()` 按分辨率动态分配：

```
单帧字节 × 2 ≤ 6MB → 双缓冲（提升采集吞吐）
否则       → 单缓冲（防 OOM）
```

初始化失败自动降级：双缓冲→单缓冲→fallback 分辨率，**绝不死锁**。

### 3. UDP 分片协议设计

单帧 JPEG 可达 50KB+，超过 MTU。自定义 12 字节帧头 + 1388B 分片：

```
┌─────┬─────┬───────┬──────┬─────┬───────┬─────┬──────────┐
│OVU  │fmt  │seq    │cnt-1 │idx  │width  │height│ payload  │
│3B   │1B   │2B     │1B    │1B   │2B     │2B   │≤1388B    │
└─────┴─────┴───────┴──────┴─────┴───────┴─────┴──────────┘
```

接收端按 `(seq, idx)` 重组，整帧到齐后解码。丢片即整帧丢弃，不重传。

### 4. WiFi 空口优化

- **关 modem sleep**：`WiFi.setSleep(false)` 消除每包唤醒延迟
- **流媒体期间关 AP**：`WiFi.softAPdisconnect()` 释放空口给 STA，VGA 帧率翻倍
- **等队列排空重试**：小帧(≤4片)每 2ms 重试同分片，丢包率 2%→0
- **AP/STA 统一信道**：避免单射频双接口时隙切换丢包

### 5. 硬件故障诊断方法论

GPIO11/12 损坏导致 DVP 数据位恒 0，画面花屏。诊断路径：

1. **位级激活率统计**（`diag_stripes.py`）：采样每个数据位的 0/1 翻转率，发现 D0/D4 位 0% 激活
2. **硬件改线验证**：D0→GPIO1，D4→GPIO2，花屏消失
3. **字节序独立验证**：OV2640 big-endian 输出，上位机需 `(buf[i]<<8)|buf[i+1]` 解析 RGB565

### 6. 色彩异常诊断与自愈（2026-08 实测）

| 症状 | 根因 | 修复 |
|------|------|------|
| 整体欠饱和接近灰度（中心饱和度 5.8/255） | `xclk_freq_hz=30MHz` 超 OV2640 规格上限(24MHz)，DSP 色度通路失效 | 固化 24MHz；实测色彩恢复且 QVGA@30fps 无回退 |
| 突发整幅蓝紫色偏（白桌 R185/G142/B254），持续不恢复 | 源码版 esp32-camera 的 `set_wb_mode()` 写入的 DSP 寄存器表与该模块不兼容，UV 增益损坏且 API 无法回写恢复 | 上位机内置看门狗：B 比 R/G 高 >40 持续 3s 自动下发串口/TCP `'r'` 完整重建，~5s 自愈 |

- 串口调试命令 `e+档位`(饱和度) / `w+模式`(白平衡)：**本模块上实测均为无效/危险操作**，仅保留用于更换模组后复测
- 上位机"停流超时重连"曾被对端硬拆导致的 `G["tcp"]=None` 状态卡死（静默假死不重连），已修复为无条件超时重建

## 面试准备

详见 [INTERVIEW.md](./INTERVIEW.md) — 10 道项目深度理解问题与参考答案。
