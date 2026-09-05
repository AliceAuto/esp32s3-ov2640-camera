# -*- coding: utf-8 -*-
"""Tkinter 界面层: 工具栏/视频区/状态栏, 后台解码线程与渲染循环, 拍照/存图。

状态与命令均经 lib.core 访问; 本模块只做 UI 构建、帧上屏、拍照交互与色彩看门狗回调。
"""
import io
import os
import queue
import time
import tkinter as tk
from tkinter import ttk

from PIL import Image, ImageTk

from lib import core
from lib import gui_style as Gs
from lib import watchdog as _watchdog
from lib.protocol import RES_LABELS, res_label, code_of, label_of_code

ui = {}          # 控件句柄/渲染状态集中存放
display_q = queue.Queue(maxsize=2)


def _on_watchdog_violation():
    if not core.G.get("photo_pending") and core.send_cmd(b"r"):
        print("[watchdog] 检测到蓝紫色偏, 已下发传感器重建", flush=True)
        info = ui.get("info")
        if info is not None:
            info.config(text="检测到色彩异常(紫屏), 已自动重建摄像头")


watchdog = _watchdog.ColorWatchdog(_on_watchdog_violation)


def build_ui(root):
    ui["root"] = root
    root.title(f"OV2640 实时预览 [{core.HOST}:{core.PORT}]")
    Gs.apply_theme(root)
    root.minsize(720, 540)

    # ---- 工具栏(深色面板底, 通栏) ----
    bar = ttk.Frame(root, style="Bar.TFrame"); bar.pack(fill="x")
    tb = ttk.Frame(bar, style="Bar.TFrame"); tb.pack(fill="x", padx=8, pady=6)

    Gs.section_label(tb, "传输:").pack(side="left")
    mode_cb = ttk.Combobox(tb, values=["UDP", "TCP"], state="readonly", width=6)
    mode_cb.set("UDP"); mode_cb.pack(side="left", padx=(4, 12))
    ui["mode_cb"] = mode_cb

    Gs.section_label(tb, "预览:").pack(side="left")
    # width 需容纳 "UXGA 1600x1200" 这类带尺寸标签, 否则选中框显示被截断
    res_cb = ttk.Combobox(tb, values=RES_LABELS, state="readonly", width=14)
    res_cb.set(label_of_code(core.G["res_code"])); res_cb.pack(side="left", padx=(4, 12))
    ui["res_cb"] = res_cb

    # AUTOQ 与手动互斥的 UI 同步(先定义, 画质栏初始化时立即调用)
    def _sync_auto_ui():
        ui["q_scale"].state(["disabled"] if core.G["auto_q"] else ["!disabled"])
        ui["q_lbl"].state(["disabled"] if core.G["auto_q"] else ["!disabled"])

    Gs.section_label(tb, "画质:").pack(side="left")
    # AUTOQ 复选框: 勾选=固件自适应接管(滑杆禁用); 取消=滑杆手动接管, 两者严格互斥
    auto_var = tk.BooleanVar(value=core.G["auto_q"])
    auto_cb = ttk.Checkbutton(tb, text="自动", variable=auto_var,
                              command=lambda: on_auto_q(auto_var.get()))
    auto_cb.pack(side="left", padx=(4, 0))
    ui["auto_var"], ui["auto_cb"] = auto_var, auto_cb
    q_var = tk.IntVar(value=core.G["quality"])
    q_scale = ttk.Scale(tb, from_=8, to=40, variable=q_var, length=110)
    q_scale.pack(side="left", padx=(4, 2))
    q_lbl = ttk.Label(tb, text=str(q_var.get()), width=3)
    q_lbl.pack(side="left", padx=(0, 12))
    ui["q_var"], ui["q_lbl"], ui["q_scale"] = q_var, q_lbl, q_scale
    _sync_auto_ui()

    Gs.section_label(tb, "拍照:").pack(side="left")
    photo_cb = ttk.Combobox(tb, values=RES_LABELS, state="readonly", width=14)
    photo_cb.set(res_label("UXGA")); photo_cb.pack(side="left", padx=(4, 4))
    ui["photo_cb"] = photo_cb

    photo_btn = ttk.Button(tb, text="拍照", style="Photo.TButton",
                           command=lambda: take_photo())
    photo_btn.pack(side="left", padx=(6, 4))
    ui["photo_btn"] = photo_btn

    snap_btn = ttk.Button(tb, text="存图(Ctrl+S)", command=snapshot)
    snap_btn.pack(side="left")

    # ---- 视频区(固定 Canvas: 画面居中适配, 不随帧变化撑动窗口) ----
    # 原用 Label 且每帧 config(width=画面宽, height=画面高): Label 的 requested
    # size 随图像变化 -> 窗口被反复撑大/缩小("画面逐渐自动变化大小")。Canvas 的
    # 尺寸由用户拖窗决定, 画面在其中等比缩放居中, 不会反馈改变窗口布局。
    video = Gs.video_canvas(root, 640, 480)
    video.pack(padx=8, pady=6, fill="both", expand=True)
    ui["video"] = video

    # ---- 状态栏(深色面板底) ----
    foot = ttk.Frame(root, style="Bar.TFrame"); foot.pack(fill="x")
    fb = ttk.Frame(foot, style="Bar.TFrame"); fb.pack(fill="x", padx=8, pady=6)
    badge = Gs.badge(fb, " 连接中 ")
    badge.pack(side="left")
    info = ttk.Label(fb, text="-", anchor="w")
    info.pack(side="left", fill="x", expand=True, padx=10)
    tip = ttk.Label(fb, text="Ctrl+Q 退出", foreground=Gs.FG_HINT)
    tip.pack(side="right")
    ui["badge"], ui["info"] = badge, info

    # ---- 事件绑定 ----
    def _build_switch_cmd():
        """生成当前传输模式下的切换命令: 直连发 v[code][U/N], 中继只发 v[code]。"""
        if core._relay_on[0]:
            return core._resync_bytes()
        return core.mode_cmd(core.G["res_code"])

    def on_mode(_ev=None):
        core.G["mode"] = mode_cb.get()
        core.G["res_code"] = code_of(res_cb.get())   # 记住当前选择, 供重连复现
        core._clear_rxbuffers()
        if core.G["mode"] == "TCP":
            core.prev_q.queue.clear()          # 切 TCP 清掉 UDP 残留画面
        cmd = _build_switch_cmd()
        if core.send_cmd(cmd):
            print(f"模式切换 -> {core.G['mode']}: {cmd!r}", flush=True)
    mode_cb.bind("<<ComboboxSelected>>", on_mode)

    def on_res(_ev=None):
        core.G["res_code"] = code_of(res_cb.get())   # 记住当前选择, 供重连复现
        # 固件全量重建(deinit+init)需 0.5~1s, 期间帧流中断; 设 3s 宽限期避免停流误判未连
        core.G["res_grace_until"] = time.time() + 3.0
        cmd = _build_switch_cmd()
        if core.send_cmd(cmd):
            print(f"分辨率 -> {res_cb.get()}: {cmd!r}", flush=True)
    res_cb.bind("<<ComboboxSelected>>", on_res)

    # 拖动中不发，松手才发一次（固件按单字节二进制值解析画质）
    def on_q_release(_ev=None):
        v = max(8, min(40, int(q_var.get())))
        q_lbl.config(text=str(v))
        core.G["quality"] = v                        # 记住手动画质, 供重连复现
        if core.send_cmd(bytes([v])):
            print(f"quality -> {v}", flush=True)
    q_scale.bind("<ButtonRelease-1>", on_q_release)

    # AUTOQ 与手动互斥: 勾选=自动接管(禁用滑杆), 取消=手动接管。
    # 固件 'a' 是 toggle: 借助 G["auto_q"] 状态跟踪, 仅在真正需要翻转时才发一次,
    # 保证本地复选框与固件 g_auto_quality 保持同步, 不会因重发/toggle 漂移。
    def on_auto_q(want_on):
        if want_on != core.G["auto_q"] and core.send_cmd(b"a"):
            core.G["auto_q"] = want_on
            print(f"AUTOQ -> {'ON' if want_on else 'OFF'}", flush=True)
        _sync_auto_ui()

    root.bind("<Control-q>", lambda e: close())
    root.bind("<Control-s>", lambda e: snapshot())
    root.protocol("WM_DELETE_WINDOW", close)


def take_photo():
    if time.time() < core.G["photo_until"]:
        return
    code = code_of(ui["photo_cb"].get())
    if not core.send_cmd(f"H{code}\nP".encode()):
        return
    core.G["photo_until"] = time.time() + 15.0     # UXGA 拍照+传输留足时间
    core.G["photo_pending"] = True                 # 下一个完整 TCP 帧单元路由给照片
    btn = ui["photo_btn"]
    btn.state(["disabled"]); btn.config(text=f"拍照中({code})…")
    ui["info"].config(text=f"拍照 {ui['photo_cb'].get()} 中，回传走可靠通道…")


def finish_photo(msg_ok):
    core.G["photo_pending"] = False
    btn = ui["photo_btn"]
    btn.state(["!disabled"]); btn.config(text="拍照")
    if not msg_ok:
        ui["info"].config(text="拍照超时/失败，可重试")


def save_jpeg_bytes(raw, prefix, w, h):
    os.makedirs(core.PHOTO_DIR, exist_ok=True)
    fn = os.path.join(core.PHOTO_DIR, "%s_%s_%dx%d.jpg"
                      % (prefix, time.strftime("%Y%m%d_%H%M%S"), w, h))
    with open(fn, "wb") as f:                 # 板子给的已是 JPEG, 直写不重编码
        f.write(raw)
    return fn


_photo = [None]      # PhotoImage 单例缓存
_canvas_item = [None]    # Canvas 图像 item 句柄(复用, 不随帧重建)


def show_on_video(img):
    """等比缩放铺进视频区居中显示, 不改变视频区/窗口尺寸。

    修复: 原实现每次上屏都 config Label 的 width/height = 画面缩放尺寸,
    Label 的 requested size 随图像变化 -> 窗口被反复撑大/缩小(画面逐渐
    自动变化大小)。改用 Canvas: 尺寸由用户拖窗决定, 画面等比缩放居中,
    不反馈改变窗口布局。
    高帧率优化(与 live_capture 的 render 对齐):
      1) 缩放显式指定重采样: 放大 2x 以上用 NEAREST(最省), 其余 BILINEAR。
      2) 复用 PhotoImage + canvas item: 同尺寸 paste 0.60ms/帧 vs 新建 1.10ms,
         且不产生 Tk 本机对象 GC 压力。
    注意: PIL 的 ImageTk.PhotoImage 只有 paste(), 没有 tkinter.PhotoImage 的 put()。
    """
    watchdog.feed(img)
    vw = max(ui["video"].winfo_width(), 320)
    vh = max(ui["video"].winfo_height(), 240)
    scale = min(vw / max(img.width, 1), vh / max(img.height, 1))
    nw = max(1, int(round(img.width * scale)))
    nh = max(1, int(round(img.height * scale)))
    if (nw, nh) == (img.width, img.height):
        disp = img
    elif nw >= img.width * 2:
        disp = img.resize((nw, nh), Image.NEAREST)
    else:
        disp = img.resize((nw, nh), Image.BILINEAR)
    ph = _photo[0]
    if ph is None or ph.width() != nw or ph.height() != nh:
        ph = ImageTk.PhotoImage(disp)         # 尺寸变化才重建(Tk 本机资源分配)
        _photo[0] = ph
    else:
        ph.paste(disp)                        # 同尺寸: 只换像素, 不新建对象
    cv = ui["video"]
    if _canvas_item[0] is None:
        _canvas_item[0] = cv.create_image(vw // 2, vh // 2, image=ph)
    else:
        cv.coords(_canvas_item[0], vw // 2, vh // 2)   # 用户拖窗后画面保持居中
        cv.itemconfig(_canvas_item[0], image=ph)
    cv.image = ph                             # 防 GC 回收


def handle_photo(raw, w, h):
    core.G["photo_until"] = 0.0
    fn = save_jpeg_bytes(raw, "photo", w, h)
    print("已保存照片:", fn, flush=True)
    try:
        img = Image.open(io.BytesIO(raw)).convert("RGB")
    except Exception as e:
        # 存盘已成功; 仅解码失败不让 Tk 回调链崩掉
        finish_photo(True)
        ui["info"].config(text=f"已保存但预览解码失败: {e}")
        return
    show_on_video(img)
    finish_photo(True)
    ui["info"].config(text=f"已保存照片 {fn}")


def snapshot():
    last = ui.get("last_frame")
    if not last:
        return
    fmt, raw, w, h = last
    os.makedirs(core.PHOTO_DIR, exist_ok=True)
    if fmt == 1:
        fn = save_jpeg_bytes(raw, "snap", w, h)
    else:
        img = Image.frombytes("RGB", (w, h), core.convert_rgb(raw, w, h))
        fn = os.path.join(core.PHOTO_DIR, "snap_%s_%dx%d.png" % (time.strftime("%Y%m%d_%H%M%S"), w, h))
        img.save(fn)
    ui["info"].config(text=f"已存图 {fn}")
    print("已存图:", fn, flush=True)


def close():
    core.G["stop"].set()
    try:
        ui["root"].destroy()
    except Exception:
        pass


def decode_worker(stop_ev):
    """后台解码线程: 从 prev_q 取最新帧解码为 PIL Image, 放入 display_q。
    将 JPEG 解码(VGA 下 15-30ms)移出 tkinter 主线程, 消除 UI 响应卡顿。

    关键: 取帧必须【非阻塞】。此前内层用 prev_q.get(timeout=0.1), 队列一空
    就要阻塞满 100ms 才交付本轮解码, 解码吞吐被锁死在 ≈1/0.1 = 10fps,
    无论链路多少帧上屏都上不去。改为 nowait 取空 + 短 sleep 让出 GIL。"""
    while not stop_ev.is_set():
        latest = None
        while True:
            try:
                latest = core.prev_q.get_nowait()    # 取到最后一个即最新帧
            except queue.Empty:
                break
        if latest is None:
            time.sleep(0.001)                   # 无帧: 让出 GIL, 不空转抢 Tk
            continue
        fmt, raw, w, h = latest
        ui["last_frame"] = latest          # 供 snapshot() 使用(GIL 下原子赋值)
        try:
            if fmt == 1:
                img = Image.open(io.BytesIO(raw)).convert("RGB")
            else:
                img = Image.frombytes("RGB", (w, h), core.convert_rgb(raw, w, h))
            # display_q 丢旧保新: 解码慢时只保留最新一帧
            try:
                while not display_q.empty():
                    display_q.get_nowait()
            except queue.Empty:
                pass
            display_q.put(img)
            # 解码完主动让出 GIL。PIL 解码/ImageTk 都持有 CPython GIL, 解码线程若一直
            # 占锁, Tk 主线程的 render 轮不到时间片 -> 上屏掉帧。1ms 让出不明显降低
            # 解码吞吐(理论仍 >500fps)。
            time.sleep(0.001)
        except Exception as e:
            print(f"解码异常: {e}", flush=True)


def render_loop(root):
    stop_ev = core.G["stop"]

    # 1) 拍照帧优先处理(不可丢)
    try:
        while True:
            raw, w, h = core.photo_q.get_nowait()
            handle_photo(raw, w, h)
    except queue.Empty:
        pass

    # 2) 拍照互斥超时释放
    if core.G["photo_until"] and time.time() > core.G["photo_until"]:
        core.G["photo_until"] = 0.0
        finish_photo(False)

    # 3) 预览: 从后台解码线程取最新已解码帧(解码已移出主线程, 避免 UI 卡顿)
    latest_img = None
    try:
        while True:
            latest_img = display_q.get_nowait()
    except queue.Empty:
        pass
    if latest_img:
        show_on_video(latest_img)
        core.up_samples.append(time.time())

    # 4) 状态栏节流刷新(4Hz)
    now = time.time()
    if now - ui.get("_st", 0) >= 0.25:
        ui["_st"] = now
        n, byt = core.window_stats()
        fps = n / core.STAT_WINDOW
        kbps = (byt / 1024) / core.STAT_WINDOW
        while core.up_samples and core.up_samples[0] < now - core.STAT_WINDOW:
            core.up_samples.popleft()
        fps_up = len(core.up_samples) / core.STAT_WINDOW
        mode = core.G["mode"]
        res = ui["res_cb"].get()
        connected = core.G["tcp"] is not None or core.G["relay"] is not None
        alive = connected and (now - core.G["last_good"] < core.RECONNECT_AFTER)
        badge = ui["badge"]
        if alive:
            badge.config(text=" ● 实时 ", bg=Gs.OK)
        elif connected:
            badge.config(text=" ◐ 等帧 ", bg=Gs.WARN)
        else:
            badge.config(text=" ○ 未连 ", bg=Gs.DANGER)
        note = ""
        if now < core.G["photo_until"]:
            note = " | 拍照中…"
        ui["info"].config(text=f"{mode} {res}  链路 {fps:5.1f} | 上屏 {fps_up:5.1f} fps  "
                               f"{kbps:4.0f} KB/s  帧计 {core.G['frames']} 丢帧 {core.G['drops']}{note}")

    if not stop_ev.is_set():
        # 自适应节奏: 刚画过帧(说明还有后续帧)则紧跟 2ms 贴近出流;
        # 空闲时退回 30ms 低频扫描, 减少与解码线程的 GIL 争用。
        root.after(2 if latest_img else 30, render_loop, root)
