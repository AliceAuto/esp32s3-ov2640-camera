# 诊断工具集（diagnostics）

> 位级 / 色彩 / 结构 / 抓帧诊断脚本，曾在开发期定位花屏（DVP 坏数据线）、色彩异常与帧解析问题。
> 改为 **通过命令行参数直接运行**，相互独立、无库依赖，多数需串口 COM 口或原始字节文件。

## 常用工具

| 脚本 | 用途 | 用法 |
|------|------|------|
| `diag_stripes.py` | **位级激活率统计**（定位 DVP 坏数据线：0% 激活位=断线） | `python diag_stripes.py <raw.bin> [W] [H]` |
| `board_diag.py` | 板内 R/G/B 主色统计（诊断偏色） | `python board_diag.py COM7 [baud]` |
| `hi_res_capture.py` | UXGA 无头抓拍存图（验证拍照链路） | - |
| `raw_save.py` / `raw_monitor.py` / `rgb_capture.py` | RGB565 / 原始字节采集 | - |
| `rgb2png.py` / `rgb2png_big.py` | RGB565 → PNG 转换 | - |
| `analyze_frame.py` / `bitcheck.py` | 单帧结构与字节位检查 | - |
| `sniff_frames.py` / `sniff_com.py` / `cmd_*.py` | 帧/串口/命令嗅探调试 | - |
| `_diag_*.py` | 一次性诊断脚本（`_` 前缀=临时，保留作取证） | - |

> 说明：这些脚本多为开发期一次性诊断，已不随主程序维护；**生产链路请使用 `../live_capture.py`**。保留于此为论文取证与故障复现参考。原始字节样本 `dump_boot.txt` 亦存档于此。