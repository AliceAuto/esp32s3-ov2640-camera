# -*- coding: utf-8 -*-
"""
OV2640 实时预览 + 点击拍照（最高画质 UXGA）。
- 串口持续接收 QQVGA(160x120) RGB565 帧实时预览（big-endian 解析）。
- 点"拍照": 向固件发 'P', 固件切 UXGA 抓高清帧发回后自动回预览。
- UXGA 帧按帧头 SIZE 精确识别并保存为 PNG。

用法: python live_capture.py COM7 [baud]
"""
import sys, time, threading, queue, os, re
import serial
from PIL import Image, ImageTk
import tkinter as tk

port = sys.argv[1] if len(sys.argv) > 1 else "COM7"
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 2000000

BEGIN = b"FRAME_BEGIN\n"
END = b"FRAME_END\n"
SIZE = b"SIZE "
PRE_W, PRE_H = 160, 120
out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "photos")

q = queue.Queue()          # 预览帧(已转RGB)
photo_result = {}          # {w,h,rgb,done}
frame_count = [0]

ser = None
wlock = threading.Lock()
photo_lock = threading.Lock()
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
    return rgb


_FRAME_RE = re.compile(rb'FRAME_BEGIN\nSIZE (\d+) (\d+)\n(.*?)FRAME_END\n', re.S)


def reader():
    global ser
    try:
        ser = serial.Serial(port, baud, timeout=0.05)
        ser.reset_input_buffer()
        print("串口已打开", flush=True)
    except Exception as e:
        print("串口打开失败:", e, flush=True)
        stop.set()
        return
    buf = bytearray()
    while not stop.is_set():
        try:
            data = ser.read(16384)
        except Exception as e:
            print("读取异常:", e, flush=True)
            stop.set()
            return
        if not data:
            time.sleep(0.002)
            continue
        buf += data
        s = bytes(buf)
        m = _FRAME_RE.search(s)
        if not m:
            # 无完整帧: 若 buf 过大, 只保留尾部(防内存膨胀)
            if len(buf) > 1 << 20:
                del buf[: (1 << 19)]
            continue
        # 处理所有匹配帧
        pos = 0
        while True:
            m = _FRAME_RE.search(s, pos)
            if not m:
                break
            w, h = int(m.group(1)), int(m.group(2))
            body = m.group(3)
            fb_len = w * h * 2
            frame = body if len(body) == fb_len else body[:fb_len]
            end = m.end()
            if (w, h) == (PRE_W, PRE_H) and len(body) == fb_len:
                q.put(convert_rgb(bytes(frame)))
                frame_count[0] += 1
            elif len(body) == fb_len:
                with photo_lock:
                    photo_result.update(w=w, h=h, rgb=convert_rgb(bytes(frame)))
                    photo_result['done'] = True
                    print(f"收到高清帧 {w}x{h}", flush=True)
            pos = end
        # 丢弃已处理数据, 保留尾部可能不完整的帧头与帧尾
        # 用 BEGIN 最后一次出现位置对齐
        buf = bytearray(s[pos:])


def do_photo():
    with photo_lock:
        if photo_result.get('done'):
            photo_result.clear()
        else:
            photo_result['done'] = False
    try:
        with wlock:
            ser.reset_input_buffer()
            ser.write(b'P')
            ser.flush()
        print("已发送拍照指令, 等待高清帧...")
        for _ in range(300):          # 最多等30s
            with photo_lock:
                if photo_result.get('done'):
                    break
            time.sleep(0.1)
        with photo_lock:
            if not photo_result.get('done'):
                print("拍照超时")
                return
            w = photo_result['w']; h = photo_result['h']; rgb = photo_result['rgb']
            photo_result.clear()
        os.makedirs(out_dir, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        path = os.path.join(out_dir, f"photo_{ts}_{w}x{h}.png")
        Image.frombytes("RGB", (w, h), rgb).save(path)
        print(f"已保存高清照片 {w}x{h} -> {path}")
    except Exception as e:
        print("拍照异常:", e)


def render():
    try:
        data = q.get_nowait()
    except queue.Empty:
        data = None
    if data:
        img = Image.frombytes("RGB", (PRE_W, PRE_H), data)
        big = img.resize((PRE_W * 4, PRE_H * 4), Image.NEAREST)
        photo = ImageTk.PhotoImage(big)
        label.config(image=photo)
        label.image = photo
        fps_lbl.config(text=f"帧数 {frame_count[0]}")
        if frame_count[0] == 1:
            print("首个预览帧已渲染")
    root.after(30, render)


def startup_error():
    import traceback
    traceback.print_exc()
    root.destroy()


root = tk.Tk()
root.title(f"OV2640 实时预览+拍照 [{port}@{baud}]")
top = tk.Frame(root); top.pack(fill="x", padx=6, pady=6)
tk.Label(top, text="QQVGA 实时预览（点拍照获取 UXGA 最高画质）").pack(side="left")
btn = tk.Button(top, text="拍 照 (UXGA)", command=do_photo,
                bg="#2d8", activebackground="#1b6", font=("", 12, "bold"))
btn.pack(side="right")
label = tk.Label(root, bg="black")
label.pack()
fps_lbl = tk.Label(root, text="连接中...", anchor="w", padx=6)
fps_lbl.pack(fill="x")
root.bind("<Control-q>", lambda e: (stop.set(), root.destroy()))
try:
    th = threading.Thread(target=reader, daemon=True)
    th.start()
except Exception:
    root.after(0, startup_error)
print("reader 线程已启动, 等待串口...")
root.after(30, render)
root.mainloop()
stop.set()
if ser:
    try: ser.close()
    except Exception: pass
print("GUI 已退出")