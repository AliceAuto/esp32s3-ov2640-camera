# -*- coding: utf-8 -*-
"""
OV2640 实时串流窗口：串口持续读取 RGB565 (QQVGA 160x120) 帧并实时显示。
用真实串口波特率；若 CH343 不支持 2M 可降为 921600。
用法: python live_stream.py COM7 [baud]
"""
import sys, time, threading, queue
import serial
from PIL import Image, ImageTk
import tkinter as tk

W, H = 160, 120
FRAME_BYTES = W * H * 2
BEGIN = b"FRAME_BEGIN\n"
END = b"FRAME_END\n"

port = sys.argv[1] if len(sys.argv) > 1 else "COM3"
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 2000000

q = queue.Queue(maxsize=2)
stop = threading.Event()
fps_state = [0, 0.0]  # fps, last_time

def reader():
    ser = serial.Serial(port, baud, timeout=0.2)
    ser.reset_input_buffer()
    buf = bytearray()
    in_frame = False
    while not stop.is_set():
        data = ser.read(4096)
        if not data:
            continue
        buf += data
        if not in_frame:
            i = buf.find(BEGIN)
            if i < 0:
                if len(buf) > 8192:
                    del buf[: len(buf) - 8192]
                continue
            del buf[: i + len(BEGIN)]
            in_frame = True
        j = buf.find(END)
        if j < 0:
            if len(buf) > FRAME_BYTES + 4096:
                buf.clear(); in_frame = False
            continue
        frame = bytes(buf[:j])
        del buf[: j + len(END)]
        in_frame = False
        if len(frame) != FRAME_BYTES:
            continue
        rgb = bytearray(FRAME_BYTES // 2 * 3)
        k = 0
        for o in range(0, FRAME_BYTES, 2):
            v = frame[o] | (frame[o + 1] << 8)
            r = (v >> 11) & 0x1F; g = (v >> 5) & 0x3F; b = v & 0x1F
            rgb[k] = (r * 255 + 15) // 31
            rgb[k + 1] = (g * 255 + 31) // 63
            rgb[k + 2] = (b * 255 + 15) // 31
            k += 3
        now = time.time()
        if now - fps_state[1] >= 1.0:
            fps_state[0] = int(1.0 / (now - fps_state[1])) if fps_state[1] else 0
            fps_state[1] = now
        if q.full():
            try: q.get_nowait()
            except Exception: pass
        q.put(bytes(rgb))
    ser.close()

def render():
    try:
        data = q.get_nowait()
    except queue.Empty:
        data = None
    if data:
        img = Image.frombytes("RGB", (W, H), data)
        big = img.resize((W * 4, H * 4), Image.NEAREST)
        photo = ImageTk.PhotoImage(big)
        label.config(image=photo, text=f"{fps_state[0]} fps")
        label.image = photo
    root.after(20, render)

sys.path.insert(0, ".")
root = tk.Tk()
root.title(f"OV2640 实时串流 [{port}@{baud}]")
label = tk.Label(root, text="等待画面...", bg="black", fg="white")
label.pack()
root.bind("<Control-q>", lambda e: (stop.set(), root.destroy()))
threading.Thread(target=reader, daemon=True).start()
root.after(30, render)
root.mainloop()
stop.set()