# -*- coding: utf-8 -*-
"""
OV2640 串口拍照轮询预览上位机（适配当前板上 JPEG 固件）。
板子 loop 预览不落串口（被半开 TCP 吞帧），但 'P' 拍照命令能通过串口回退发出完整 JPEG 帧，
故采用"轮询拍照"方式取帧显示。分辨率表与切帧复用 lib/protocol、lib/frame_parser。

用法: python preview_serial.py [com] [baud] [photo_res]
  com       串口, 默认 COM7
  baud      波特率, 默认 2000000
  photo_res 拍照分辨率码 q/t/v/s/x/u (默认 t=QVGA 小帧高帧率)

流程: 打开串口(释放 EN) -> 设拍照分辨率 -> 循环: 发P -> 收 FRAME_BEGIN..FRAME_END -> 解码JPEG显示
快捷键: Ctrl+Q 退出
"""
import io
import os
import queue
import sys
import threading
import time

# 使 tools/ 下可 import host/lib 公共库(与主程序同一份帧解析实现)
_HOST = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "host")
sys.path.insert(0, _HOST)

import serial
import tkinter as tk
from PIL import Image, ImageTk

from lib import frame_parser
from lib import protocol

port = sys.argv[1] if len(sys.argv) > 1 else "COM7"
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 2000000
res = sys.argv[3] if len(sys.argv) > 3 else "t"     # 拍照分辨率, q/t/v/s/x/u

# code -> (名称, 宽, 高)，从协议层分辨率表派生
RES = {code: (name, w, h) for name, (code, w, h) in protocol.RES_INFO.items()}

out_dir = os.path.join(_HOST, "photos")   # 与主程序共用 host/photos
os.makedirs(out_dir, exist_ok=True)

ser = serial.Serial(port, baud, timeout=0.2)
ser.dtr = False; ser.rts = False     # 释放 EN, 避免板子被按住复位
ser.reset_input_buffer()
ser.write(b"H" + res.encode()); ser.flush()   # 设拍照分辨率

q = queue.Queue(maxsize=4)
stop = threading.Event()
frame_no = [0]
fps_state = {"t": time.time(), "n": 0}
cur_w, cur_h = RES[res][1], RES[res][2]


def reader():
    buf = bytearray()
    last_cmd = time.time()
    while not stop.is_set():
        # 空闲补发: 超过2s没有新数据则再触发一次拍照
        if time.time() - last_cmd > 2.0 and not buf:
            try:
                ser.write(b"P"); ser.flush(); last_cmd = time.time()
            except Exception:
                pass
        data = ser.read(65536)
        if not data:
            continue
        buf += data
        frames, consumed = frame_parser.split_frames(buf)
        del buf[:consumed]
        for w_, h_, fmt, body in frames:
            if fmt == "JPG" and len(body) > 100 and body[:2] == b"\xff\xd8":
                try:
                    img = Image.open(io.BytesIO(bytes(body))).convert("RGB")
                    q.put((img, w_, h_))
                    frame_no[0] += 1
                    fps_state["n"] += 1
                    print(f"帧 {frame_no[0]} {w_}x{h_} len={len(body)}", flush=True)
                except Exception:
                    pass
            # 每收完一帧立刻触发下一张拍照
            try:
                ser.write(b"P"); ser.flush(); last_cmd = time.time()
            except Exception:
                pass


def save_photo(img, w, h):
    ts = time.strftime("%Y%m%d_%H%M%S")
    path = os.path.join(out_dir, f"photo_{ts}_{w}x{h}.png")
    img.save(path)
    status_lbl.config(text=f"已保存 {w}x{h} -> {os.path.basename(path)}")


def do_save():
    try:
        img, w, h = q.get_nowait()
        save_photo(img, w, h)
    except queue.Empty:
        pass
    root.after(300, do_save)


# 首帧触发
ser.write(b"P"); ser.flush()

root = tk.Tk()
root.title(f"OV2640 串口轮询预览 [{port}@{baud}] 拍照={RES[res][0]}")

top = tk.Frame(root); top.pack(fill="x", padx=6, pady=6)
status_lbl = tk.Label(root, text="连接中...", anchor="w", padx=6, fg="#666")
status_lbl.pack(fill="x")
label = tk.Label(root, bg="black"); label.pack()
fps_lbl = tk.Label(root, text="", anchor="w", padx=6); fps_lbl.pack(fill="x")
tk.Button(top, text="保存当前帧", command=do_save, bg="#2d8", font=("", 11, "bold")).pack(side="right")

root.bind("<Control-q>", lambda e: (stop.set(), root.destroy()))
th = threading.Thread(target=reader, daemon=True); th.start()


def render():
    try:
        img, w, h = q.get_nowait()
    except queue.Empty:
        img = None
    if img:
        scale = max(1, 640 // max(w, 1))
        big = img.resize((w * scale, h * scale), Image.NEAREST)
        photo_image = ImageTk.PhotoImage(big)
        label.config(image=photo_image); label.image = photo_image
        now = time.time()
        fps = fps_state["n"] / max(now - fps_state["t"], 1e-6) if now != fps_state["t"] else 0
        fps_lbl.config(text=f"帧 {frame_no[0]} | {w}x{h} | ~{fps:.1f}fps")
        status_lbl.config(text=f"接收中... {w}x{h} JPEG")
    root.after(30, render)


root.after(30, render)
root.mainloop()
stop.set()
ser.close()
