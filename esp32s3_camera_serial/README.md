# OV2640 + ESP32-S3 相机工程（维护文档）

> 通过串口实时预览 + 一键拍照（最高 UXGA 1600×1200）的裸机验证工程。本文档仅供后续维护参考。

---

## 1. 项目概述

| 项 | 值 |
| --- | --- |
| 传感器 | OV2640（200万像素 DVP 摄像头） |
| 主控 | ESP32-S3（8MB PSRAM，片外帧缓冲） |
| 图像格式 | RGB565（big-endian 输出） |
| 数据传输 | 串口 UART0（GPIO43/44），波特率 **2,000,000** |
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

---

## 5. 上位机脚本（`host/`）

| 脚本 | 用途 |
| ---- | ---- |
| **`live_capture.py`** | **主程序**：实时预览 GUI + 下拉切换预览/拍照分辨率 + 一键拍照存图 + **自动重连**。用法：`python live_capture.py COM7 [baud]` |
| `verify_switch.py` | 自动验证：预览切 QVGA、拍照切 UXGA、切回 QQVGA 的无头验证脚本 |
| `hi_res_capture.py` | UXGA 无头抓拍教程脚本 |
| `rgb_capture.py` / `raw_save.py` / `raw_monitor.py` | RGB565 / 原始字节采集诊断 |
| `board_diag.py` / `diag_*.py` / `bitcheck.py` / `stripe_check.py` | 位级/色彩/结构诊断工具（曾用于定位花屏=数据线损坏） |
| `test_convert.py` / `rgb2png*.py` / `analyze_frame.py` | RGB565→PNG 转换与画质分析 |
| `photos/` | 拍照输出目录（`photo_YYYYMMDD_HHMMSS_WxH.png`） |

### live_capture.py 关键解析规则（维护者必读）

- **RGB565 必须 big-endian 解析**：`v = (frame[o] << 8) | frame[o+1]`。用 little-endian 会整体花屏。切勿改回。
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
| JPEG 模式抓空帧 | **预编译 esp32-camera 库**对 SOI 校验过严，OV2640 不满足 → 改用 RGB565 稳定基线。真需 JPEG 须从源码重编译库（`lib/esp32-camera` 已存在但仍链接预编译 `.a`） |
| 串口“拒绝访问” | 残留 python 进程占用 COM7，用任务管理器结束 `python.exe`（命令行匹配 `live_capture`） |
| 偏绿缺蓝 | 检查是否只开 AWB+AGC（见 3.1）；必要时松开被注释的 awb_gain |
| 裸机触碰死机 | 已在 live_capture 实现自动重连，数秒内自动恢复 |

---

## 7. 遗留事项

- **JPEG 模式**：当前强制 RGB565。要从源码编译 esp32-camera 替换预编译库（`board 链接的是 libespressif__esp32-camera.a`，需覆盖使其链接 `lib/esp32-camera` 源码产物），才能启用 JPEG 高压缩输出。
- GPIO11/12 损坏是板级硬件缺陷，换板需恢复默认脚位映射并更新文档第2节。

---

## 8. 开发历程：遇到的问题与解决（按时间顺序）

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

---

> 最终基线（稳定）：RGB565 + QQVGA 预览 / UXGA 拍照，PSRAM 自适应缓冲，失锁自动重连，上位机自动重连+分辨率重同步。