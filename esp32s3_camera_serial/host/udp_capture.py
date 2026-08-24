# -*- coding: utf-8 -*-
"""
OV2640 UDP 预览帧率测试（对比 TCP 性能）。

- 用 TCP 连接板子发送命令(开启 UDP 预览 'U')，同时保留拍照/命令通道。
- 预览帧改走 UDP(高帧率, 丢帧不重传) 接收并显示，实时统计帧率。
- 拍照仍走 TCP(可靠) —— 暂未在本脚本实现，仅测预览帧率。

用法: python udp_capture.py --wifi [host] [port]   (host 默认 192.168.4.1, port 默认 3333)
快捷键: Ctrl+Q 退出
"""
import sys, time, threading, queue, io
import socket
from PIL import Image, ImageTk
import tkinter as tk

_args = [x for x in sys.argv[1:] if x != "--wifi"]
host = _args[0] if _args else "192.168.4.1"
port = int(_args[1]) if len(_args) > 1 else 3333
# 第3个参数为分辨率 code(q/t/v/s/x/u), 拼成两字节命令 'v<code>' 再 'U' 开 UDP; 空则仅开 UDP
res_code = _args[2] if len(_args) > 2 else ""
res_cmd = (("v" + res_code) if res_code else "") + "U"

# UDP 帧头布局(与固件 send_frame_udp 一致)
HDR = 12
MAGIC = b"OVU"
CHUNK = 1400 - HDR

q = queue.Queue()
frame_count = [0]
stop = threading.Event()
start_t = [time.time()]

# 长时间没有任何完整帧就触发一次自恢复(板子重启/失锁会停发, 需重发 'U' 开启 UDP 模式)
RECONNECT_AFTER = 4.0

# UDP 帧重组状态
ctx = {"seq": None, "cnt": 0, "w": 0, "h": 0, "fmt": 0, "parts": None, "got": 0}
# 终端 fps 统计
fps_state = {"t": time.time(), "n": 0, "w": 0, "h": 0, "fmt": 0}


def convert_rgb(frame, w, h):
    rgb = bytearray(w * h * 3)
    j = 0
    for o in range(0, len(frame), 2):
        v = (frame[o] << 8) | frame[o + 1]      # big-endian RGB565
        r = (v >> 11) & 0x1F; g = (v >> 5) & 0x3F; b = v & 0x1F
        rgb[j] = (r * 255 + 15) // 31
        rgb[j + 1] = (g * 255 + 31) // 63
        rgb[j + 2] = (b * 255 + 15) // 31
        j += 3
    return bytes(rgb)


def udp_feed(pkt):
    """吃一个数据报, 重组整帧; 整帧到齐返回 True。"""
    global ctx
    if len(pkt) < HDR or pkt[:3] != MAGIC:
        return False
    seq = (pkt[4] << 8) | pkt[5]
    cnt = pkt[6] + 1
    idx = pkt[7]
    w = (pkt[8] << 8) | pkt[9]
    h = (pkt[10] << 8) | pkt[11]
    fmt = pkt[3]                                # 0=RGB565 1=JPEG
    payload = pkt[HDR:]
    if ctx["seq"] != seq:                        # 新帧
        ctx = {"seq": seq, "cnt": cnt, "w": w, "h": h, "fmt": fmt,
               "parts": [None] * cnt, "got": 0}
    if idx >= ctx["cnt"]:
        return False
    if ctx["parts"][idx] is None:
        ctx["parts"][idx] = payload
        ctx["got"] += 1
    if ctx["got"] == ctx["cnt"]:                 # 整帧到齐
        frame = b"".join(ctx["parts"])
        fw, fh, ffmt = ctx["w"], ctx["h"], ctx["fmt"]
        ctx = {"seq": None, "cnt": 0, "w": 0, "h": 0, "fmt": 0,
               "parts": None, "got": 0}
        # JPEG: 长度不定, 校验头部 FF D8 与尾 FF D9; RGB565: 长度需等于 w*h*2
        ok = False
        if ffmt == 1:
            ok = len(frame) > 100 and frame[:2] == b"\xff\xd8" and frame[-2:] == b"\xff\xd9"
        else:
            ok = len(frame) == fw * fh * 2 and fw > 0 and fh > 0
        if ok:
            q.put((ffmt, frame, fw, fh))
            frame_count[0] += 1
            fps_state["w"], fps_state["h"], fps_state["fmt"] = fw, fh, ffmt
        return True
    return False


def open_tcp_cmd():
    """建连板子 TCP 并下发命令(切分辨率+开 UDP)；失败自动重试。"""
    for _ in range(120):
        if stop.is_set():
            return None
        try:
            c = socket.create_connection((host, port), timeout=4)
            c.settimeout(0.05)
            c.sendall(res_cmd.encode())
            return c
        except Exception:
            time.sleep(1.0)
    return None


def reader():
    # UDP:预览通道(无状态, 断帧靠心跳自恢复)
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp.bind(("0.0.0.0", port))
    udp.settimeout(0.05)
    print(f"UDP 已监听 :{port}", flush=True)
    tcp = open_tcp_cmd()
    if tcp is None:
        stop.set()
        udp.close()
        return
    print("已发 'U' 开启 UDP 预览, TCP 通道就绪", flush=True)
    last_good = time.time()
    while not stop.is_set():
        # 心跳自恢复：板子重启/失锁会停发, 超时后重发 'U' 让它回到 UDP 模式
        if time.time() - last_good > RECONNECT_AFTER:
            print("UDP 长时间无帧, 重新下发 'U' 恢复", flush=True)
            try:
                tcp.close()
            except Exception:
                pass
            tcp = open_tcp_cmd()
            if tcp is None:
                break
            print("恢复重连成功, 等待帧回来", flush=True)
            last_good = time.time()
            ctx.clear(); ctx.update(seq=None, cnt=0, w=0, h=0, fmt=0, parts=None, got=0)
        try:
            pkt, _addr = udp.recvfrom(65535)
        except socket.timeout:
            time.sleep(0.002)
            continue
        except OSError:
            break
        if udp_feed(pkt):
            last_good = time.time()
        # 每 3 秒终端打印一次帧率
        now = time.time()
        if now - fps_state["t"] >= 3.0:
            fps = (frame_count[0] - fps_state["n"]) / (now - fps_state["t"])
            name = "JPEG" if fps_state["fmt"] == 1 else "RGB565"
            print(f"fps={fps:6.1f}  {fps_state['w']}x{fps_state['h']} {name}  累计帧 {frame_count[0]}", flush=True)
            fps_state["t"], fps_state["n"] = now, frame_count[0]
    udp.close()
    try:
        tcp.close()
    except Exception:
        pass


def render():
    try:
        fmt, raw, w, h = q.get_nowait()
        if raw:
            if fmt == 1:                      # JPEG -> 直接解码
                img = Image.open(io.BytesIO(raw))
                img = img.convert("RGB")
            else:                             # RGB565 big-endian
                img = Image.frombytes("RGB", (w, h), convert_rgb(raw, w, h))
            scale = max(1, 640 // max(w, 1))
            big = img.resize((w * scale, h * scale), Image.NEAREST)
            photo = ImageTk.PhotoImage(big)
            label.config(image=photo)
            label.image = photo
            elapsed = time.time() - start_t[0]
            fps = frame_count[0] / elapsed if elapsed > 0 else 0
            fps_lbl.config(text=f"UDP {'JPEG' if fmt == 1 else 'RGB565'} 帧率 {fps:.1f} fps | 帧数 {frame_count[0]} | {w}x{h}")
    except queue.Empty:
        pass
    except Exception:
        import traceback; traceback.print_exc()
    root.after(15, render)


root = tk.Tk()
root.title(f"OV2640 UDP 预览帧率测试 [{host}:{port}]")
label = tk.Label(root, bg="black")
label.pack()
fps_lbl = tk.Label(root, text="连接中...", anchor="w", padx=6)
fps_lbl.pack(fill="x")
root.bind("<Control-q>", lambda e: (stop.set(), root.destroy()))
th = threading.Thread(target=reader, daemon=True)
th.start()
root.after(15, render)
root.mainloop()
stop.set()
print("退出", flush=True)