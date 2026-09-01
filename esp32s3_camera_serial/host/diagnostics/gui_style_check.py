# -*- coding: utf-8 -*-
"""真实 GUI 渲染自检: 用共享主题库构建与客户端一致的界面, 截图保存验证观感。"""
import sys, os
os.environ.pop("TCL_LIBRARY", None)
os.environ.pop("TK_LIBRARY", None)
os.environ.pop("TCL_ROOT", None)
_HOST = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _HOST)
import tkinter as tk
from tkinter import ttk
from PIL import ImageGrab
from lib import gui_style as Gs

root = tk.Tk()
root.title("GUI 风格自检 (渲染 2.5s 后自动关闭)")
Gs.apply_theme(root)
root.geometry("760x620+60+60")

# ---- 模拟 udp_capture 工具栏 ----
bar = ttk.Frame(root, style="Bar.TFrame"); bar.pack(fill="x")
tb = ttk.Frame(bar, style="Bar.TFrame"); tb.pack(fill="x", padx=8, pady=6)
Gs.section_label(tb, "传输:").pack(side="left")
m = ttk.Combobox(tb, values=["UDP", "TCP"], state="readonly", width=6); m.set("UDP")
m.pack(side="left", padx=4)
Gs.section_label(tb, "预览:").pack(side="left", padx=(10, 0))
r = ttk.Combobox(tb, values=["QQVGA 160x120", "QVGA 320x240", "VGA 640x480"], state="readonly", width=14)
r.set("QVGA 320x240"); r.pack(side="left", padx=4)
Gs.section_label(tb, "画质:").pack(side="left", padx=(10, 0))
qv = tk.IntVar(value=10); sl = ttk.Scale(tb, from_=8, to=40, variable=qv, length=110)
sl.pack(side="left", padx=4)
ql = ttk.Label(tb, text="10", style="Bar.TLabel", width=3); ql.pack(side="left")
pbtn = ttk.Button(tb, text="拍照", style="Photo.TButton"); pbtn.pack(side="left", padx=(10, 4))
sbtn = ttk.Button(tb, text="存图", style="Accent.TButton"); sbtn.pack(side="left")

# ---- 视频区 ----
cv = Gs.video_canvas(root, 640, 400); cv.pack(padx=8, pady=(0, 6))

# ---- 模拟状态栏 + 徽章 ----
foot = ttk.Frame(root, style="Bar.TFrame"); foot.pack(fill="x")
bf = tk.Frame(foot, bg=Gs.PANEL); bf.pack(side="left", padx=8, pady=6)
b = Gs.badge(bf, " 连接中 "); b.pack(side="left")
b.config(text=" ● 实时 ", bg=Gs.OK)
info = ttk.Label(foot, text="UDP QVGA 320x240  链路 25.0 | 上屏 24.0 fps  120 KB/s  帧计 1230 丢帧 0",
                 style="Bar.TLabel"); info.pack(side="left", padx=8)

root.update_idletasks(); root.update()
root.after(700, root.update)

# 等窗口稳定后抓图
import time
root.lift(); root.attributes("-topmost", True)
root.update()
time.sleep(0.9)
root.update()
root.update_idletasks()
x0 = root.winfo_rootx(); y0 = root.winfo_rooty()
x1 = x0 + root.winfo_width(); y1 = y0 + root.winfo_height()
img = ImageGrab.grab(bbox=(x0, y0, x1, y1))
out = os.path.join(_HOST, "_style_preview.png")
img.save(out)
print("SCREENSHOT_SAVED:", out, img.size)
# 滑块精确验证: 定位"画质"标签附近滑杆区域(工具栏 y ~ 20-60px, 画质标签在横向约 55% 处)
from collections import Counter
RGB = img.convert("RGB")
# 统计整图里的 ACCENT 蓝 & FIELD 轨道色像素
ACC=(63,169,245); FIELD=(49,54,63)
def near(col, t, tol=30): return all(abs(a-b)<=tol for a,b in zip(col,t))
cnt=Counter(RGB.getdata())
acc=sum(v for k,v in cnt.items() if near(k,ACC)); fld=sum(v for k,v in cnt.items() if near(k,FIELD))
print("ACCENT-blue px:", acc, " FIELD-trough px:", fld)
root.destroy()