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
| `gui_style_check.py` | **GUI 深色主题渲染自检**（截图核对配色/滑杆/徽章） | `python gui_style_check.py [输出.png]` |

> 说明：这些脚本多为开发期一次性诊断，已不随主程序维护；**生产链路请使用 `../live_capture.py`**。
> 维护记录（2026-09）：`_diag_*.py`（中继/P2P 排障临时脚本）与重复的 `diag_structure.py`、废弃的 `run_preview.bat` 已清理删除；帧协议解析 / RGB565 转换等公共逻辑已收敛到 `../lib/`。
> 原始字节样本 `dump_boot.txt` 存档于此。