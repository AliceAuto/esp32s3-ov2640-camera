# -*- coding: utf-8 -*-
"""
OV2640 串口拍照轮询预览上位机（适配当前板上 JPEG 固件）。
板子 loop 预览不落串口（被半开 TCP 吞帧），但 'P' 拍照命令能通过串口回退发出完整 JPEG 帧，
故采用"轮询拍照"方式取帧显示。

用法: python preview_serial.py [com] [baud] [photo_res]
  com       串口, 默认 COM7
  baud      波特率, 默认 2000000
  photo_res 拍照分辨率码 q/t/v/s/x/u (默认 t=QVGA 小帧高帧率)

流程: 打开串口(释放 EN) -> 设拍照分辨率 -> 循环: 发P -> 收 FRAME_BEGIN..FRAME_END -> 解码JPEG显示
快捷键: Ctrl+Q 退出
"""
import sys, time, threading, queue, os, io
import serial
from PIL import Image, ImageTk
import tkinter as tk

port   = sys.argv[1] if len(sys.argv) > 1 else "COM7"
baud   = int(sys.argv[2]) if len(sys.argv) > 2 else 2000000
res    = sys.argv[3] if len(sys.argv) > 3 else "t"     # 拍照分辨率, q/t/v/s/x/u

RES = {"q": ("QQVGA",160,120),"t":("QVGA",320,240),"v":("VGA",640,480),
       "s":("SVGA",800,600),"x":("XGA",1024,768),"u":("UXGA",1600,1200)}
BEGIN = b"FRAME_BEGIN\n"
END   = b"FRAME_END\n"
HDR_RE_STR = b"SIZE "

out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "photos")
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
        while True:
            i = buf.find(BEGIN)
            if i < 0:
                if len(buf) > 1 << 20:
                    del buf[: len(buf) - 4096]
                break
            del buf[: i + len(BEGIN)]
            nl = buf.find(b"\n")
            if nl < 0:
                break
            size_line = bytes(buf[:nl])
            del buf[: nl + 1]
            m = None
            if size_line.startswith(HDR_RE_STR):
                parts = size_line.split()
                if len(parts) >= 4:
                    try:
                        m = (int(parts[1]), int(parts[2]), parts[3].decode('ascii','ignore'))
                    except Exception:
                        m = None
            j = buf.find(END)
            if j < 0:
                break
            body = bytes(buf[:j])
            del buf[: j + len(END)]
            if m and m[2] == "JPG" and len(body) > 100 and body[:2] == b"\xff\xd8":
                try:
                    img = Image.open(io.BytesIO(body)).convert("RGB")
                    q.put((img, m[0], m[1]))
                    frame_no[0] += 1
                    fps_state["n"] += 1
                    print(f"帧 {frame_no[0]} {m[0]}x{m[1]} len={len(body)}", flush=True)
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
tk.Button(top, text="保存当前帧", command=do_save, bg="#2d8", font=("",11,"bold")).pack(side="right")

root.bind("<Control-q>", lambda e: (stop.set(), root.destroy()))
th = threading.Thread(target=reader, daemon=True); th.start()

def render():
    try:
        img, w, h = q.get_nowait()
    except queue.Empty:
        img = None
    if img:
        scale = max(1, 640 // max(w, 1))
        big = img.resize((w*scale, h*scale), Image.NEAREST)
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
