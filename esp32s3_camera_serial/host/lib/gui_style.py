# -*- coding: utf-8 -*-
"""
OV2640 上位机统一深色主题库。

live_capture / udp_capture 两个 Tkinter 上位机共用同一套现代化深色风格,
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

# ---- 配色 ----
BG        = "#1b1d22"   # 窗口背景
PANEL     = "#23262c"   # 工具栏/状态栏背景
PANEL2    = "#2b2f37"   # 控件/内嵌面板背景
FIELD     = "#31363f"   # 下拉/输入框背景
FG        = "#e9ecf2"   # 主文字
FG_DIM    = "#aab0bd"   # 次要文字
FG_HINT   = "#767d8a"   # 提示文字
BORDER    = "#3b4049"   # 边框
ACCENT    = "#3fa9f5"   # 主强调色(蓝)
ACCENT_DK = "#1f7fd6"   # 强调深(按下/悬停)
OK        = "#2fbf71"   # 实时(绿)
WARN      = "#e09a3e"   # 等帧(橙)
DANGER    = "#e5534d"   # 未连(红)
PHOTO     = "#26d1a1"   # 拍照按钮(青绿)
PHOTO_DK  = "#1aa684"
VIDEO_BG  = "#0c0d10"   # 视频区
SELECT    = "#3d6ea5"   # 下拉选中项

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

    # 滑杆
    style.configure("TScale", background=BG, troughcolor=PANEL2, bordercolor=BORDER)
    style.configure("Vertical.TScale", background=BG, troughcolor=PANEL2,
                    bordercolor=BORDER)

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
    """状态徽章 Label: 圆角胶囊观感(padding 撑出), 后续可 config(text=, bg=) 变色。"""
    return tk.Label(parent, text=text, fg="#ffffff", bg=bg,
                    padx=8, pady=2, font=font(9, True))


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
