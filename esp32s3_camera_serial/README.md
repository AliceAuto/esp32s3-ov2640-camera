# OV2640 + ESP32-S3 相机工程（维护文档）

> 通过串口实时预览 + 一键拍照（最高 UXGA 1600×1200）的裸机验证工程。本文档仅供后续维护参考。

---

## 1. 项目概述

| 项 | 值 |
| --- | --- |
| 传感器 | OV2640（200万像素 DVP 摄像头） |
| 主控 | ESP32-S3（8MB PSRAM，片外帧缓冲） |
| 图像格式 | **JPEG（默认，高压缩）/ RGB565（big-endian）可切换** |
| 数据传输 | WiFi TCP/UDP（局域网 IP:3333）或 串口 UART0 2,000,000 baud |
| 预览分辨率 | 默认 QQVGA 160×120（可切 QVGA/VGA） |
| 拍照分辨率 | 默认 **UXGA 1600×1200**（可切 VGA/SVGA/XGA/UXGA） |
| 帧缓冲 | PSRAM，按分辨率自适应 1/2 缓冲 |

**重要**：OV2640 为**定焦镜头，无法调节焦距**。近距离（<20cm）或预览分辨率过低会被明显放大导致"模糊"，属正常现象，不是对焦故障。

---

## 2. 硬件接线（OV2640 → ESP32-S3）

> ⚠️ 本板 **GPIO11、GPIO12 已损坏**（DVP 输入读到恒 0/无法翻转）。因此：
> - **D0 原接 GPIO11 → 改接到 GPIO1**
> - **D4 原接 GPIO12 → 改接到 GPIO2**
> 若更换新板恢复原件，请改回解释的默认脚位，否则颜色错乱。

| OV2640 | ESP32-S3 | 说明 |
| ------ | -------- | ---- |
| 3.3V | 3V3 | 供电 |
| GND | GND | 共地 |
| PWDN | **GND** | 必须拉低使能，否则不工作 |
| SCCB_SCL (SCL) | GPIO5 | I2C 时钟 |
| SCCB_SDA (SDA) | GPIO4 | I2C 数据 |
| VSYNC | GPIO6 | 帧同步 |
| HREF | GPIO7 | 行参考 |
| PCLK (DCLK) | GPIO13 | 像素时钟 |
| XCLK | GPIO15 | 时钟输入（本模块自带晶振，可留空） |
| D0 | **GPIO1** | 数据位0（原11，已损坏） |
| D1 | GPIO9 | 数据位1 |
| D2 | GPIO8 | 数据位2 |
| D3 | GPIO10 | 数据位3 |
| D4 | **GPIO2** | 数据位4（原12，已损坏） |
| D5 | GPIO18 | 数据位5 |
| D6 | GPIO17 | 数据位6 |
| D7 | GPIO16 | 数据位7 |

---

## 3. 固件 `esp32s3_camera_serial.ino`

### 3.1 关键初始化参数

```cpp
config.xclk_freq_hz = 20000000;           // 20MHz
config.pixel_format = PIXFORMAT_RGB565;   // RGB565（JPEG 模式有缺陷，见第5节）
config.frame_size   = FRAMESIZE_QQVGA;    // 初始预览
config.fb_location  = CAMERA_FB_IN_PSRAM; // 8MB PSRAM
config.fb_count     = fb_count_for(...);  // 自适应缓冲
```

- **自适应缓冲 `fb_count_for()`**：`MAX_FB_BYTES=6MB`。单帧 ×2 ≤ 6MB 用双缓冲，否则单缓冲。关键原因是 **PSRAM 双缓冲会导致高分辨率（XGA/UXGA，RGB565 3.84MB×2≈7.7MB）OOM → 初始化失败 → 失锁卡死**。
- **画质校准（最小配置）**：`set_whitebal(1)` + `set_gain_ctrl(1)` + `set_exposure_ctrl(1)`。其余 AWB 增强/降噪（awb_gain/aec2/lenc/bpc/wpc/dcw）已被注释掉，怀疑它们会干扰 B 通道增益导致偏绿缺蓝。

### 3.2 自动恢复机制（关键，勿删）

- `init_camera_as(target, fallback)`：初始化失败自动**降级**——先强制单缓冲重试，仍失败则**回退**到 `fallback` 分辨率，保证相机不进入死锁态。
- `camera_reconnect()`：失锁后 deinit + 重建（含分辨率切换降级）。预览连续 2 次抓帧失败即触发。
- `camera_change_framesize(fs)`：**deinit + init 全量重配**实现切换（应对预编译库 set_framesize 对 PSRAM 的缺陷），失败自动回退到当前分辨率。

### 3.3 串口帧协议

发送一帧原始 RGB565（big-endian）：

```
FRAME_BEGIN\n
SIZE <w> <h>\n
<w*h*2 字节 RGB565 big-endian>
FRAME_END\n
```

### 3.4 上位机命令

| 命令 | 作用 |
| ---- | ---- |
| `P` / `p` | 拍照：切到 HI_FRAME 抓一帧高清发送，完成后切回预览 |
| `V<码>` / `v<码>` | 设置预览分辨率（立即生效） |
| `H<码>` / `h<码>` | 设置拍照分辨率（下次拍照生效） |
| `d` | 板内诊断：采样输出 R/G/B 主色统计 |
| `r` / `R` | 手动重连摄像头（调试用） |

### 3.5 分辨率字符码

| 码 | 分辨率 | 尺寸 |
| -- | ------ | ---- |
| `q` | QQVGA | 160×120 |
| `t` | QVGA | 320×240 |
| `v` | VGA | 640×480 |
| `s` | SVGA | 800×600 |
| `x` | XGA | 1024×768 |
| `u` | UXGA | 1600×1200 |

---

## 4. 构建与烧录（arduino-cli）

工具链：`d:\Projects\ov2640\tools\arduino-cli.exe`（esp32 core 3.3.11，已装好）。

```powershell
$cli = "d:\Projects\ov2640\tools\arduino-cli.exe"

# 编译
& $cli compile --fqbn "esp32:esp32:esp32s3:PSRAM=opi" `
    "d:\Projects\ov2640\esp32s3_camera_serial"

# 上传（COM7）
& $cli upload -p COM7 --fqbn "esp32:esp32:esp32s3:PSRAM=opi" `
    "d:\Projects\ov2640\esp32s3_camera_serial"
```

- **必须启用 `PSRAM=opi`**（8MB OPI PSRAM），否则大分辨率帧缓冲 malloc 失败，初始化报错。
- 设备使用 **UART0（CH343）** 而非 USB-CDC，端口号为 `COM7`，硬件复位重刷固件可重启板子。

> **⚠️ arduino-cli 路径无法解锁 JPEG**：它会链接预编译的 `libespressif__esp32-camera.a`，JPEG 模式下出空帧（FB-OVF）。**JPEG 版本改用 PlatformIO 源码编译**（见 4.2）。

---

## 4.2 构建与烧录（PlatformIO，JPEG 解锁路线）

> PlatformIO 从源码编译 `espressif/esp32-camera`，彻底绕过预编译库的 JPEG 空帧缺陷。工程在 `d:\Projects\ov2640\esp32s3_camera_pio\`。

```powershell
$pio = "C:\Users\27427\AppData\Local\Programs\Python\Python311\Scripts\pio.exe"

# 编译（首次会自动下载 espressif32 平台 + 工具链 + 源码编译 esp32-camera）
& $pio run -d d:\Projects\ov2640\esp32s3_camera_pio

# 上传到 COM7
& $pio run -t upload -d d:\Projects\ov2640\esp32s3_camera_pio
```

- 关键配置（`esp32s3_camera_pio\platformio.ini`）：
  - `board = esp32-s3-devkitc-1`，`board_build.arduino.memory_type = qio_opi`（8MB OPI PSRAM）。
  - `lib_deps = espressif/esp32-camera`：**从源码编译**（非预编译 `.a`）。
  - `build_flags`：`-DBOARD_HAS_PSRAM`、`-DARDUINO_USB_CDC_ON_BOOT=0`。
- 源码结构：`src/main.cpp` = 原 `.ino` 迁移，`#define PIX_FORMAT PIXFORMAT_JPEG` 启用 JPEG；改回 `PIXFORMAT_RGB565` 即 RGB 基线。`apply_color_calib()` 里 `set_quality(s,10)` 把 JPEG 画质拉满。
- **第一台机器安装 PlatformIO 首次编译需下载约 700MB 工具链，务必开代理/镜像加速**，否则耗时极长。
- 若报缺 `freertos/FreeRTOS.h` 等 IDF 头文件：是 `framework-arduinoespressif32` 包下载不完整，删 `~/.platformio/packages/framework-arduinoespressif32` 后重跑 `pio run` 让其重下。

---

## 5. 上位机脚本（`host/`）

| 脚本 | 用途 |
| ---- | ---- |
| **`live_capture.py`** | **主程序**：实时预览 GUI + 下拉切换预览/拍照分辨率 + 一键拍照存图 + **自动重连**。用法：`python live_capture.py COM7 [baud]` 或 `--wifi <IP> 3333` |
| **`udp_capture.py`** | **JPEG 高帧率预览**：走 UDP 通道（丢帧不重传）+ JPEG 解码，帧率明显高于 TCP。用法：`python udp_capture.py --wifi <IP> 3333` |
| `verify_switch.py` | 自动验证：预览切 QVGA、拍照切 UXGA、切回 QQVGA 的无头验证脚本 |
| `hi_res_capture.py` | UXGA 无头抓拍教程脚本 |
| `rgb_capture.py` / `raw_save.py` / `raw_monitor.py` | RGB565 / 原始字节采集诊断 |
| `board_diag.py` / `diag_*.py` / `bitcheck.py` / `stripe_check.py` | 位级/色彩/结构诊断工具（曾用于定位花屏=数据线损坏） |
| `test_convert.py` / `rgb2png*.py` / `analyze_frame.py` | RGB565→PNG 转换与画质分析 |
| `photos/` | 拍照输出目录（`photo_YYYYMMDD_HHMMSS_WxH.png`） |

### live_capture.py 关键解析规则（维护者必读）

- **RGB565 必须 big-endian 解析**：`v = (frame[o] << 8) | frame[o+1]`。用 little-endian 会整体花屏。切勿改回。
- **格式随帧头携带**：`SIZE <w> <h> <JPG|RGB>`。`JPG` 直接 `Image.open(BytesIO).convert("RGB")`（校验 `FF D8`/`FF D9`）；`RGB` 按 `w*h*2` 取帧体做 big-endian 转换。预览队列元素为 `(PIL.Image, w, h)` 三元组。
- 按帧头 `FRAME_BEGIN/SIZE/FRAME_END\n` 正则切帧，`PRE=160×120` 进预览队列，其余尺寸判为照片帧。
- **自动重连**（应对裸机触碰短路复位）：
  1. 串口读异常 → 关闭并自动重开。
  2. 心跳保护：超过 `RECONNECT_AFTER=5s` 无任何帧 → 强制重开串口。
  3. 重开/重启后读到**首帧**自动 `resync_board()`，重新下发 `V<预览码>H<拍照码>`，使板端分辨率与界面选择对齐（MCU 复位会丢运行时设置回到默认 QQVGA）。
- 依赖：`pyserial`、`Pillow`、`tkinter`

---

## 6. 常见问题排查

| 现象 | 原因与处理 |
| ---- | ---- |
| 整幅花屏彩带 | 上位机没按 big-endian 解析 RGB565；排查坏数据位（GPIO1/2 方案） |
| 画面糊 | 预览 QQVGA 被放大显示；切 QVGA 即可；定焦镜头近距离本身糊 |
| 高分辨率切换卡死 | PSRAM 双缓冲 OOM → 已用 `fb_count_for()` 自适应 + 失败回退解决 |
| JPEG 模式抓空帧 | **预编译 esp32-camera 库**在 JPEG 下返空帧（FB-OVF）→ 已用 **PlatformIO 源码编译**解决（见 4.2）。保持源码编译即可稳定出 JPEG |
| 串口“拒绝访问” | 残留 python 进程占用 COM7，用任务管理器结束 `python.exe`（命令行匹配 `live_capture`） |
| 偏绿缺蓝 | 检查是否只开 AWB+AGC（见 3.1）；必要时松开被注释的 awb_gain |
| 裸机触碰死机 | 已在 live_capture 实现自动重连，数秒内自动恢复 |

---

## 7. WiFi 传输（局域网 / 热点，高帧率）

串口 2M 波特率约 200KB/s 是帧率瓶颈（QQVGA 预览约 5fps）。固件已内置 **WiFi 传输**，且经过一轮"极致工程优化"（见 9.13），现在有 **TCP（可靠）** 与 **UDP（高帧率预览）** 双通道。

### UDP 预览模式（默认高帧率通道）

- 上位机 TCP 连接后发 **`U`** 开启 UDP 预览，板端改为按 **UDP 分片**发帧（丢帧不重传 → 高帧率）；再次连接新客户端默认回到 TCP。
- **UDP 帧协议**：12B 头 + 分片。头 = `"OVU"(3) fmt(1) seq(2) cnt-1(1) idx(1) w(2) h(2)`，`fmt:0=RGB565 1=JPEG`；单片净荷 **1388B**（避开 MTU 分片）。上位机脚本 `host/udp_fps.py` 按此重组并统计帧率/丢片。
- 综合实测（UDP，JPEG quality=10，当前 2.4GHz 信道安静时段）：

| 分辨率 | 传感器上限 | 实测帧率 | 丢片 | 帧长 |
| ---- | ---- | ---- | ---- | ---- |
| QQVGA 160×120 | 30fps | **29.4fps** | 0 | 1.7KB |
| QVGA 320×240 | 30fps | **29.1fps** | 0 | 5.0KB |
| VGA 640×480 | 15fps | **14.9fps** | 0 | 12.7KB |
| SVGA 800×600 | 15fps | ~2-6fps | 受信道容量限制 | 20KB(q10)/11KB(q50) |

> 注意：以上为**信道安静**时的结果。2.4GHz 信道（当前 ch=1）受邻居路由器/设备争用波动，拥挤时段 UDP 会丢片降帧（`udp_fps.py` 有丢片统计可观测）。传感器上限：QQVGA/QVGA=30fps，VGA/SVGA=15fps —— 实测 UDP 在小分辨率已跑满传感器上限。**UDP 是尽力而为通道，丢帧属设计行为**；需要每帧可靠时切 TCP。

### 模式一：STA 连入局域网（推荐，电脑不断网）

固件默认以 STA 身份连接家庭路由器（`STA_SSID`/`STA_PASS`，见 `.ino` 顶部），DHCP 拿到局域网 IP；**电脑直接连板子局域网 IP 即可，无需切热点**。保留 AP 兜底热点 `OV2640Cam`。

- **获取板子 IP**：上电后串口每 5 秒打印 `[WiFi] ip=x.x.x.x`。改写入 `.ino` 顶部的 `STA_SSID`/`STA_PASS` 即可换网。
- **上位机连接**：`python live_capture.py --wifi <板子IP> 3333`，例如 `python live_capture.py --wifi 192.168.10.118 3333`。
- 无头验证：`python verify_wifi.py <板子IP> 3333`（测预览帧率 + 一键 UXGA 拍照存 PNG）。
- 帧/命令协议与串口完全一致，仅传输通道换成 TCP。

### 模式二：AP 热点直连（兜底）

当不在局域网内（现场无路由）时，手机/电脑连接板子热点 `OV2640Cam`（无密码），上位机连 `192.168.4.1:3333`：`python live_capture.py --wifi 192.168.4.1 3333`。

### 注意事项

- 若局域网内 ping 不通板子，大概率路由器开了 **AP 隔离**，需到路由器后台关闭（限制同网段设备互访）。
- 板子 IP 由 DHCP 分配可能变化，以串口 `[WiFi] ip=` 打印为准；重烧/复位后重新读取。
- 若 STA 连不上，固件自动退回纯热点模式，电脑连 `OV2640Cam` 走模式二。

---

## 8. 遗留事项

- ~~JPEG 模式不可用~~ → **已解决**：PlatformIO 源码编译 esp32-camera（`esp32s3_camera_pio/`）正常出 JPEG，配合 UDP 预览帧率大幅提升。
- GPIO11/12 损坏是板级硬件缺陷，换板需恢复默认脚位映射并更新文档第2节。
- WiFi 上传带宽更高，拍大图(UXGA)帧率接近实时；拍照脚本若偶取到残留的预览帧（尺寸小于目标），属解析时序，需等待下一帧。
- 高分辨率（UXGA）是 JPEG 单缓冲大帧，UDP 分片数可能逼近上限，必要时调大固件的分片许可（`nchunk>N` 处）。

---

## 9. 开发历程：遇到的问题与解决（按时间顺序）

> 这一节是实际调试中逐个踩过、解决的坑，供将来复现/维护时对照。

### 8.1 编译环境搭建
- **问题**：需用命令行动手装好 Arduino 工具链，esptool 核心包下载慢。
- **解决**：安装 `arduino-cli`，esp32 core **3.3.11** 经**国内镜像加速**安装，装上 `esp32-camera` 库，配好 FQBN.最终用 `PSRAM=opi` 编译烧录.

### 8.2 固件串口无任何输出
- **现象**：上电后串口一片空白，怀疑程序没跑。
- **原因**：默认 `Serial` 走的是 USB-CDC，而调试口实际在 **UART0（CH343，GPIO43/44）**。
- **解决**：改用 `HardwareSerial Dbg(0)` 强制指定 UART0，后续所有输出走 `Dbg.*`。

### 8.3 摄像头初始化失败 `frame buffer malloc failed`
- **原因**：默认没启用片外 PSRAM，大分辨率帧缓冲在内部 DRAM 分配失败。
- **解决**：编译时指定 **`PSRAM=opi`**，启用 8MB OPI PSRAM（`fb_location=CAMERA_FB_IN_PSRAM`）。

### 8.4 JPEG 模式抓帧为空 `esp_camera_fb_get 返回空`
- **现象**：RGB565 能出图，切 JPEG 就空帧。
- **原因**：**官方预编译库**对 JPEG 的 SOI 起始标记校验过严，OV2640 输出不满足 → 整帧被丢弃。
- **解决**：**切 RGB565 模式**验证硬件链路，确认 DVP/VSYNC/DMA 采集全部正常；JPEG 留待将来源码重编译库（遗留事项）。

### 8.5 串口“拒绝访问” PermissionError(13)
- **原因**：多个 `capture_serial.py` 残留进程同时占用 COM7。
- **解决**：通过 `Get-CimInstance Win32_Process` 按命令行匹配，`Stop-Process` 强制结束占用进程释放串口。

### 8.6 图像大片偏绿黄、噪点多
- **现象**：画面整体偏绿、蓝色通道弱，且雪花噪点明显。
- **原因**：预编译库默认未正确开自动白平衡，或增益设置不当，B 通道增益失效。
- **解决**：初始化后调用 `set_whitebal(1)` + `set_gain_ctrl(1)` + `set_exposure_ctrl(1)`；并确认 OV2640 的 **AWB 需同时置 CTRL1 的 bit3+bit2** 才生效。随后把存疑的 awb_gain/aec2/lenc/bpc/wpc/dcw 注释掉，用**最小配置(AWB+AGC)** 得到稳定中性色。

### 8.7 花屏彩带（原因之一）——字节序
- **现象**：RGB565 画面被解码成大片色带、伪彩。
- **原因**：**OV2640 按 big-endian 输出**（高字节在前），而上位机当时用 little-endian 解析，像素 R/G/B 位被打乱。
- **解决**：上位机改为 `v = (frame[o] << 8) | frame[o+1]`（big-endian）解析。（这是维护中**最常被误改回**的一处，见第5节）

### 8.8 花屏彩带（另一独立原因）——DVP 数据线损坏
- **现象**：字节序修正后，某些画面仍有伪彩/结构完整但颜色坏。
- **成因定位**：这说明还有第二个**独立**的根因——**字节序 + 坏数据线**必须两项都修好才出正常图。
- **原因**：这块开发板 **GPIO11、GPIO12 已物理损坏**，DVP 输入读到恒 0。
- **定位**：用 `host/diag_stripes.py` 做**位级诊断**——统计每条数据位激活率，发现某几位 **0% 激活（always 0）**，确认正是断线。
- **解决**：**D0 改接到 GPIO1、D4 改接到 GPIO2**，并更新固件脚位宏。教训：**“花屏/彩带先查数据位激活率，再动寄存器”**。
- 附带：板内 `DIAG` 算法曾因取样用 6bit 而误判偏色，后统一改用**标准 8bit** 采样分析。

### 8.9 高分辨率切换卡死（VGA/SVGA/UXGA）
- **现象**：从 QQVGA/QVGA 切到高分辨率，`esp_camera_fb_get` 一直返回空、相机失锁。
- **原因**：
  1. 预编译库在 PSRAM 下 **`set_framesize` 未真正重配 DMA 缓冲**，传感器失锁；
  2. 写死的 `fb_count=2` 在 **UXGA(3.84MB×2≈7.7MB)** 时逼近 8MB PSRAM 上限 → **OOM**。
- **解决**：
  1. 分辨率切换一律走 **`deinit + init` 全量重配**（`camera_change_framesize`）；
  2. 用 `fb_count_for()` **按分辨率自适应 1/2 缓冲**（大图单缓冲），预留 6MB 上限；
  3. 初始化失败**自动降级**：先强制单缓冲重试，再失败**回退**到原分辨率，绝不死锁；
  4. 预览连续抓帧失败即自动 `camera_reconnect()` 重建。

### 8.10 画面“模糊”（误判为对焦问题）
- **现象**：预览看起来很糊，怀疑没对焦。
- **结论**：**OV2640 是定焦镜头，无对焦电机**。糊的真因是预览用 **QQVGA(160×120) 被放大显示**。
- **解决**：预览切到 **QVGA** 明显更清晰；拍照用 UXGA 才是真高清。近距离(<20cm)物理模糊属正常，拍摄主体放 30cm 以上更锐。

### 8.11 裸机触碰短路导致 MCU 复位死机
- **现象**：裸板无外壳，手一碰可能瞬时短路 → MCU 复位，串流中断。
- **解决**：在 `live_capture.py` 实现**三级自动恢复**（见第5节）——串口异常重开、5 秒无帧强制重连、重连首帧后重同步下发分辨率，数秒内自动恢复，无需手动干预。

### 8.12 极致优化一：UDP 直连 lwIP + 关 modem sleep + 统一信道
- **动机**：QQVGA 预览仅 ~21fps / VGA ~7fps，瓶颈在传输路径。
- **做法**（`esp32s3_camera_pio/src/main.cpp`）：
  1. **lwIP 原始 `udp_pcb` 发送**：绕开 `WiFiUdp` 每包套接字开销；并**移除 `WiFiUdp` 绑定**（曾与原始 pcb 同时绑 3333 导致 bind 失败、发送走未绑定 pcb）。
  2. **`WiFi.setSleep(false)`** 关 modem sleep，降每包唤醒延迟/抖动。
  3. **AP/STA 统一信道**（`esp_wifi_set_channel`）：单射频双接口若分处不同信道会时隙切换丢包。
  4. **帧缓冲按分辨率自适应 3/2/1**：小图 3 缓冲，发送慢时采集不停顿。
  5. 新客户端接入**默认重置为 TCP**（`udp_mode=false`），避免 UDP 模式跨连接粘滞。
- **板载 DIAG**：每 5s 打印 `cap/sent 速率、平均帧长、uerr(udp发送失败)、pbf(pbuf分配失败)、rty(重试成功)、skp(背压丢帧)、nc(无客户端丢帧)、err(最近错误码)、rssi、AEC/AGC 曝光增益寄存器` —— 据此一眼区分"编码瓶颈 / 链路瓶颈 / 曝光拉长"。

### 8.13 极致优化二：等队列排空重试 + 流媒体期间关软AP
- **现象**：DIAG 显示 `err=-1`（lwIP **ERR_MEM = WiFi netif TX 队列满**），QQVGA 丢片 2%、VGA 掉到 5fps。1ms×3 短重试几乎从不恢复（`rty≈0`），说明队列满持续 >3ms。
- **根因**：WiFi TX 队列被灌满 → udp_sendto 静默失败 → 整帧被放弃。**这是链路/射频争用问题，不是编码或曝光问题**（`pbf=0` 排除内存不足，AEC/AGC 正常）。
- **解决**（`send_frame_udp` 按帧大小分策略）：
  1. **小帧（≤4 片，QQVGA/QVGA）**："等队列排空"式轮询重试——每 2ms 重试同分片，最长等 12ms。分片少、阻塞代价低，实测丢包率从 2% **降为 0**，QQVGA 25→29.4fps。
  2. **大帧（>4 片，VGA/SVGA）**：只 1ms 短重试，**不阻塞采集流水线**（否则每片 12ms 阻塞 × 多片反而饿死发送、加剧丢包）；持续失败即丢帧 + 背压 3 帧让队列排空。
  3. **流媒体期间关闭兜底软AP**（`WiFi.softAPdisconnect()`，无客户端时恢复）：AP/STA 单射频时分共存会抢占 STA 空口，关闭后 VGA 容量约翻倍（4.9→8.1fps 起，安静时段达 14.9fps 满速零丢）。
- **结论**：当前 RF 环境下小分辨率已达**传感器上限**（QQVGA/QVGA 30fps、VGA 15fps）且**零丢片**；SVGA（8-15 片/帧）受 2.4GHz 信道容量限制只能 ~2-6fps。UDP 丢片为信道瞬时拥塞（同配置两次结果差异大），属外部射频波动，非固件缺陷。

---

> 最终基线（稳定）：JPEG（PlatformIO 源码编译）+ QQVGA/QVGA 预览 / UXGA 拍照，WiFi TCP/UDP 双通道（UDP 预览小分辨率跑满传感器上限零丢片，见第7节），PSRAM 自适应缓冲，失锁自动重连，上位机自动重连+分辨率重同步。