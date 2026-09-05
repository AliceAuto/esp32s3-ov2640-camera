# -*- coding: utf-8 -*-
"""
OV2640 上位机统一深色主题库。

main.py(网络预览) / tools/preview_serial.py(串口预览) 两个 Tkinter 上位机共用同一套现代化深色风格,
保证观感一致。本模块只负责"样式", 不涉及任何业务/协议逻辑。

用法:
    from lib import gui_style as Gs
    Gs.apply_theme(root)                    # 应用全局 ttk + option 深色主题
    Gs.badge(parent, " 连接中 ")            # 状态徽章(后面可 config bg/text 变色)
    Gs.option_menu(parent, var, values, command)   # 深色 OptionMenu
    Gs.video_canvas(parent, w, h)          # 深色视频区 Canvas(自带细边框)
"""
import tkinter as tk
from tkinter import ttk

# ---- 配色(现代暗色, 参考 Nord/One Dark 系, 高对比且克制) ----
BG        = "#14161c"   # 窗口背景(近黑蓝)
PANEL     = "#1c1f27"   # 工具栏/状态栏背景
PANEL2    = "#262a34"   # 控件/内嵌面板背景
FIELD     = "#2f3442"   # 下拉/输入框背景
FG        = "#eceff5"   # 主文字
FG_DIM    = "#9aa3b5"   # 次要文字
FG_HINT   = "#6b7284"   # 提示文字
BORDER    = "#343a47"   # 边框
ACCENT    = "#4f8cff"   # 主强调色(亮蓝)
ACCENT_DK = "#2f6fe0"   # 强调深(按下/悬停)
OK        = "#34d399"   # 实时(绿)
WARN      = "#f5b04f"   # 等帧(橙)
DANGER    = "#ef5b60"   # 未连(红)
PHOTO     = "#22d3a7"   # 拍照按钮(青绿)
PHOTO_DK  = "#17a085"
VIDEO_BG  = "#0a0b0e"   # 视频区
SELECT    = "#3d6fb5"   # 下拉选中项

FONT = "Microsoft YaHei UI"


def font(size=10, bold=False):
    """统一字体: 无粗体则普通; 字体缺失时 Tk 会自动回退, 不报错。"""
    return (FONT, size, "bold" if bold else "normal")


def apply_theme(root):
    """应用全局深色主题: 根背景 + ttk(clam) + tk 原生控件 option 默认值。

    ttk 控件(Button/Combobox/Scale/Label/Frame)用 Style 统一;
    tk 原生控件(Label/OptionMenu/Menu)用 option_add 统一默认值。
    """
    root.configure(bg=BG)
    root.option_add("*Background", BG)
    root.option_add("*Foreground", FG)
    root.option_add("*Font", font(10))
    # tk 原生控件深色默认
    root.option_add("*Label*Background", BG)
    root.option_add("*Label*Foreground", FG)
    root.option_add("*Button*Background", PANEL2)
    root.option_add("*Button*Foreground", FG)
    root.option_add("*Button*ActiveBackground", ACCENT_DK)
    root.option_add("*Button*ActiveForeground", "#ffffff")
    root.option_add("*Menubutton*Background", PANEL2)
    root.option_add("*Menubutton*Foreground", FG)
    root.option_add("*Menubutton*ActiveBackground", ACCENT_DK)
    root.option_add("*Menubutton*ActiveForeground", "#ffffff")
    root.option_add("*Menu*Background", PANEL2)
    root.option_add("*Menu*Foreground", FG)
    root.option_add("*Menu*ActiveBackground", SELECT)
    root.option_add("*Menu*ActiveForeground", "#ffffff")

    style = ttk.Style(root)
    try:
        style.theme_use("clam")          # clam 支持逐项自定义颜色
    except tk.TclError:
        pass
    # 基础样式: 所有 ttk 控件默认深色
    style.configure(".",
                    background=BG, foreground=FG,
                    bordercolor=BORDER, lightcolor=BORDER, darkcolor=BORDER,
                    troughcolor=BG, selectbackground=SELECT,
                    selectforeground="#ffffff", fieldbackground=FIELD,
                    arrowcolor=FG_DIM, focuscolor=ACCENT)
    style.configure("TFrame", background=BG)
    style.configure("TLabel", background=BG, foreground=FG)
    style.configure("Bar.TFrame", background=PANEL)     # 工具栏/状态栏底

    # 普通按钮
    style.configure("TButton",
                    background=PANEL2, foreground=FG, bordercolor=BORDER,
                    padding=(12, 5), focuscolor=BG)
    style.map("TButton",
              background=[("pressed", ACCENT_DK), ("active", ACCENT)],
              foreground=[("pressed", "#ffffff"), ("active", "#ffffff")])
    # 主操作按钮(蓝色强调)
    style.configure("Accent.TButton",
                    background=ACCENT, foreground="#ffffff",
                    bordercolor=ACCENT, padding=(14, 5), font=font(10, True))
    style.map("Accent.TButton",
              background=[("pressed", ACCENT_DK), ("active", "#59b6ff")])
    # 拍照按钮(青绿强调)
    style.configure("Photo.TButton",
                    background=PHOTO, foreground="#0a1411",
                    bordercolor=PHOTO, padding=(14, 5), font=font(10, True))
    style.map("Photo.TButton",
              background=[("pressed", PHOTO_DK), ("active", "#4fe0b8")])

    # 下拉框/输入框
    style.configure("TCombobox",
                    background=FIELD, foreground=FG, fieldbackground=FIELD,
                    bordercolor=BORDER, arrowcolor=FG_DIM, padding=4)
    style.map("TCombobox",
              fieldbackground=[("readonly", FIELD)],
              foreground=[("readonly", FG)],
              selectbackground=[("readonly", SELECT)],
              selectforeground=[("readonly", "#ffffff")])
    style.configure("TEntry", background=FIELD, foreground=FG, fieldbackground=FIELD)
    style.configure("TSpinbox", background=FIELD, foreground=FG, fieldbackground=FIELD)

    # 滑杆: 轨道提亮到 FIELD, 滑块用强调蓝, 在深色工具栏上清晰可见。
    # 注: clam 的 TScale thumb 与轨道都用 style 的 background/troughcolor 上色;
    # 布局方向(横/纵)由 Horizontal./Vertical. 前缀区分。
    for _name in ("TScale", "Horizontal.TScale", "Vertical.TScale"):
        style.configure(_name, background=ACCENT, troughcolor=FIELD,
                        bordercolor=BORDER, sliderlength=18, sliderrelief="flat",
                        sliderthickness=10)
        style.map(_name, background=[("active", ACCENT_DK)],
                  troughcolor=[("disabled", PANEL2)])

    # 下拉列表深色(ttk.Combobox 的弹出 listbox)
    try:
        root.option_add("*TCombobox*Listbox.background", FIELD)
        root.option_add("*TCombobox*Listbox.foreground", FG)
        root.option_add("*TCombobox*Listbox.selectBackground", SELECT)
        root.option_add("*TCombobox*Listbox.selectForeground", "#ffffff")
        root.option_add("*TCombobox*Listbox.borderWidth", 0)
    except tk.TclError:
        pass
    return style


def badge(parent, text=" 连接中 ", bg=WARN):
    """状态徽章 Label: 胶囊观感(大内边距+粗体+深色字提升彩底对比), 可 config 变色。"""
    return tk.Label(parent, text=text, fg="#0b0d10", bg=bg,
                    padx=10, pady=3, font=font(9, True))


def option_menu(parent, variable, values, command=None, width=None):
    """深色 OptionMenu: 统一面板底色/文字/选中高亮, 保持原功能签名兼容。"""
    om = tk.OptionMenu(parent, variable, *values, command=command)
    om.configure(bg=PANEL2, fg=FG, activebackground=ACCENT_DK,
                 activeforeground="#ffffff", highlightthickness=0,
                 bd=0, relief="flat", width=width,
                 font=font(10), cursor="hand2")
    try:
        om["menu"].configure(bg=PANEL2, fg=FG, activebackground=SELECT,
                             activeforeground="#ffffff", bd=0, relief="flat",
                             tearoff=0)
    except tk.TclError:
        pass
    return om


def video_canvas(parent, width=640, height=480, bg=VIDEO_BG):
    """深色视频区 Canvas: 自带宽 1px 深色细边框, 画面在其中居中显示。"""
    return tk.Canvas(parent, bg=bg, highlightthickness=1,
                     highlightbackground=BORDER, highlightcolor=BORDER,
                     width=width, height=height)


def section_label(parent, text):
    """工具栏分组标签(次要文字色, 视觉上弱于控件)。"""
    return ttk.Label(parent, text=text, foreground=FG_DIM)


def hdivider(parent, color=BORDER):
    """水平分隔线(1px): 区分工具栏/视频区/状态栏等区块。"""
    return tk.Frame(parent, bg=color, height=1, bd=0, highlightthickness=0)


def vdivider(parent, color=BORDER, padx=8):
    """工具栏内的竖向细分隔(分组之间的视觉边界)。"""
    f = tk.Frame(parent, bg=color, width=1)
    f.pack(side="left", fill="y", padx=padx, pady=2)
    return f


def big_value(parent, text="--", color=FG, size=16, bg=PANEL):
    """大号数值标签(fps 等关键指标, 粗体+亮色, 一目了然)。"""
    return tk.Label(parent, text=text, fg=color, bg=bg, font=font(size, True))


def brand(parent, title, sub=""):
    """品牌/标题区: 应用名(强调蓝) + 副标题(连接目标, 次要色)。"""
    f = tk.Frame(parent, bg=BG)
    tk.Label(f, text=title, fg=ACCENT, bg=BG, font=font(12, True)).pack(side="left")
    if sub:
        tk.Label(f, text=sub, fg=FG_HINT, bg=BG, font=font(10)).pack(side="left", padx=(8, 0))
    return f
