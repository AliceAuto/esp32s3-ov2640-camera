# -*- coding: utf-8 -*-
"""
OV2640 实时预览 + 动态分辨率切换 + 拍照。
- 顶部下拉可随时切换【预览分辨率】(QQVGA/QVGA/VGA) 和【拍照分辨率】(VGA/SVGA/XGA/UXGA)。
- 界面实时显示预览（big-endian RGB565 解析，平滑放大显示）。
- 点"拍照": 固件切到所选拍照分辨率抓一张高清帧发回，自动保存 PNG 并回到预览。
- 数据经串口 2Mbps 传输。预览分辨率越高，帧率越低。

用法: python live_capture.py COM7 [baud]
快捷键: Ctrl+Q 退出
"""
import sys, time, threading, queue, os, re
import serial
from PIL import Image, ImageTk
import tkinter as tk

port = sys.argv[1] if len(sys.argv) > 1 else "COM7"
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 2000000

BEGIN = b"FRAME_BEGIN\n"
END = b"FRAME_END\n"
# 长时间没有任何字节/完整帧就强制重连串口（覆盖 MCU 死机复位的场景）
RECONNECT_AFTER = 5.0
# code -> (名称, w, h)
RES = {
    "q": ("QQVGA", 160, 120),
    "t": ("QVGA", 320, 240),
    "v": ("VGA", 640, 480),
    "s": ("SVGA", 800, 600),
    "x": ("XGA", 1024, 768),
    "u": ("UXGA", 1600, 1200),
}
PREVIEW_CHOICES = ["QQVGA", "QVGA", "VGA"]      # 预览可选
PHOTO_CHOICES = ["VGA", "SVGA", "XGA", "UXGA"]  # 拍照可选
out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "photos")

# 预览当前尺寸(线程共享, 用锁保护)；photo_cfg 记录当前选中的拍照分辨率
preview = {"code": "q", "w": 160, "h": 120}
photo_cfg = {"code": "u"}
photo_req = {"active": False, "w": 0, "h": 0, "rgb": None, "done": False}
q = queue.Queue()
frame_count = [0]

ser = None
wlock = threading.Lock()
state_lock = threading.Lock()
stop = threading.Event()


def convert_rgb(frame):
    n = len(frame) // 2
    rgb = bytearray(n * 3)
    j = 0
    for o in range(0, len(frame), 2):
        v = (frame[o] << 8) | frame[o + 1]   # big-endian
        r = (v >> 11) & 0x1F; g = (v >> 5) & 0x3F; b = v & 0x1F
        rgb[j] = (r * 255 + 15) // 31
        rgb[j + 1] = (g * 255 + 31) // 63
        rgb[j + 2] = (b * 255 + 15) // 31
        j += 3
    return bytes(rgb)


_FRAME_RE = re.compile(rb'FRAME_BEGIN\nSIZE (\d+) (\d+)\n(.*?)FRAME_END\n', re.S)


def open_serial():
    for _ in range(300):
        if stop.is_set():
            return None
        try:
            s = serial.Serial(port, baud, timeout=0.05)
            s.dtr = False
            s.rts = False
            s.reset_input_buffer()
            return s
        except Exception as e:
            time.sleep(0.5)
    return None


def resync_board():
    """MCU 复位后会恢复默认分辨率，这里在读到首帧后把界面当前选择重新下发对齐。"""
    with state_lock:
        pcode = preview.get("code", "q")
        hcode = photo_cfg.get("code", "u")
    try:
        with wlock:
            if ser:
                ser.write(("V" + pcode + "H" + hcode).encode())
                ser.flush()
        print(f"重同步下发 V{pcode} H{hcode}", flush=True)
    except Exception:
        pass


def reader():
    global ser
    ser = open_serial()
    if ser is None:
        stop.set()
        return
    print("串口已打开", flush=True)
    buf = bytearray()
    last_frame_time = 0.0
    while not stop.is_set():
        try:
            data = ser.read(16384)
        except Exception as e:
            print("串口异常, 尝试重连:", e, flush=True)
            try:
                ser.close()
            except Exception:
                pass
            ser = None
            ser = open_serial()
            if ser is None:
                stop.set()
                return
            print("串口已重连", flush=True)
            buf = bytearray()
            last_frame_time = 0.0
            continue
        if not data:
            # 心跳：MCU 死机复位会导致若干秒无数据，超时则重开串口强制重同步
            if last_frame_time and (time.time() - last_frame_time) > RECONNECT_AFTER:
                print("长时间无帧, 强制重连串口", flush=True)
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None
                ser = open_serial()
                if ser is None:
                    stop.set()
                    return
                buf = bytearray()
                last_frame_time = 0.0
            time.sleep(0.002)
            continue
        buf += data
        s = bytes(buf)
        if _FRAME_RE.search(s) is None:
            if len(buf) > 8 << 20:
                del buf[: (4 << 20)]
            continue
        resync_pending = last_frame_time == 0.0
        pos = 0
        while True:
            m = _FRAME_RE.search(s, pos)
            if not m:
                break
            w, h = int(m.group(1)), int(m.group(2))
            body = m.group(3)
            fb_len = w * h * 2
            end = m.end()
            with state_lock:
                pw, ph = preview["w"], preview["h"]
                want_photo = photo_req.get("active")
                pw_, ph_ = photo_req["w"], photo_req["h"]
            if len(body) >= fb_len:
                frame = body[:fb_len]
                if (w, h) == (pw, ph):
                    q.put(convert_rgb(bytes(frame)))
                    frame_count[0] += 1
                elif want_photo and (w, h) == (pw_, ph_):
                    with state_lock:
                        photo_req["rgb"] = convert_rgb(bytes(frame))
                        photo_req["done"] = True
                        photo_req["active"] = False
                    print(f"收到照片帧 {w}x{h}", flush=True)
                last_frame_time = time.time()
            pos = end
        buf = bytearray(s[pos:])
        # 重连(或首次)拿到首帧后，把界面选择的分辨率重新下发给板子，避免错位
        if resync_pending:
            resync_board()


# ---- 命令：切换预览/拍照分辨率 ----
def set_preview(name):
    code = [c for c, r in RES.items() if r[0] == name]
    if not code:
        return
    code = code[0]
    with state_lock:
        preview["code"] = code
        preview["w"], preview["h"] = RES[code][1], RES[code][2]
    try:
        with wlock:
            ser.reset_input_buffer()
            ser.write(("V" + code).encode())
            ser.flush()
        print(f"预览分辨率已切换: {name}", flush=True)
    except Exception as e:
        print("切换预览失败:", e, flush=True)


def set_photo(name):
    code = [c for c, r in RES.items() if r[0] == name]
    if not code:
        return
    code = code[0]
    with state_lock:
        photo_cfg["code"] = code
    try:
        with wlock:
            ser.write(("H" + code).encode())
            ser.flush()
        print(f"拍照分辨率已设为: {name} (下次拍照生效)", flush=True)
    except Exception as e:
        print("设置拍照分辨率失败:", e, flush=True)


def do_photo():
    with state_lock:
        photo_req.update(active=True, w=preview_w_for_photo(), h=preview_h_for_photo(),
                         rgb=None, done=False)
    try:
        with wlock:
            ser.reset_input_buffer()
            ser.write(b'P')
            ser.flush()
        print("已发送拍照指令, 等待高清帧...", flush=True)
        root.after(200, poll_photo)
    except Exception as e:
        print("拍照异常:", e, flush=True)


def preview_w_for_photo():
    # 拍照尺寸 = 当前选中的"拍照分辨率"
    return cur_photo_size()[0]


def preview_h_for_photo():
    return cur_photo_size()[1]


def cur_photo_size():
    name = photo_var.get()
    code = [c for c, r in RES.items() if r[0] == name][0]
    return RES[code][1], RES[code][2]


def poll_photo(_=None):
    with state_lock:
        done = photo_req.get("done")
        rgb = photo_req.get("rgb")
        w, h = photo_req["w"], photo_req["h"]
    if done and rgb:
        os.makedirs(out_dir, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        path = os.path.join(out_dir, f"photo_{ts}_{w}x{h}.png")
        Image.frombytes("RGB", (w, h), rgb).save(path)
        print(f"已保存照片 {w}x{h} -> {path}", flush=True)
        status_lbl.config(text=f"已保存 {w}x{h} -> {os.path.basename(path)}")
        with state_lock:
            photo_req["rgb"] = None
        return
    root.after(200, poll_photo)


def render():
    try:
        data = q.get_nowait()
    except queue.Empty:
        data = None
    if data:
        with state_lock:
            w, h = preview["w"], preview["h"]
        img = Image.frombytes("RGB", (w, h), data)
        scale = max(1, 640 // max(w, 1))
        big = img.resize((w * scale, h * scale), Image.NEAREST)
        photo_image = ImageTk.PhotoImage(big)
        label.config(image=photo_image)
        label.image = photo_image
        fps_lbl.config(text=f"帧数 {frame_count[0]} | 预览 {w}x{h}")
    root.after(30, render)


root = tk.Tk()
root.title(f"OV2640 实时预览+动态拍照 [{port}@{baud}]")

top = tk.Frame(root); top.pack(fill="x", padx=6, pady=6)
tk.Label(top, text="预览:").pack(side="left")
preview_var = tk.StringVar(value="QQVGA")
tk.OptionMenu(top, preview_var, *PREVIEW_CHOICES, command=set_preview).pack(side="left", padx=3)
tk.Label(top, text="拍照:").pack(side="left", padx=(12, 0))
photo_var = tk.StringVar(value="UXGA")
tk.OptionMenu(top, photo_var, *PHOTO_CHOICES, command=set_photo).pack(side="left", padx=3)
btn = tk.Button(top, text="拍 照", command=do_photo,
                bg="#2d8", activebackground="#1b6", font=("", 12, "bold"))
btn.pack(side="right")

status_lbl = tk.Label(root, text="连接中...", anchor="w", padx=6, fg="#666")
status_lbl.pack(fill="x")
label = tk.Label(root, bg="black")
label.pack()
fps_lbl = tk.Label(root, text="", anchor="w", padx=6)
fps_lbl.pack(fill="x")

root.bind("<Control-q>", lambda e: (stop.set(), root.destroy()))
th = threading.Thread(target=reader, daemon=True)
th.start()
print("reader 线程已启动, 等待串口...", flush=True)
root.after(30, render)
root.mainloop()
stop.set()
if ser:
    try:
        ser.close()
    except Exception:
        pass
print("GUI 已退出", flush=True)