# -*- coding: utf-8 -*-
"""
预览帧率命令行测试（UDP/TCP 双模式 + UDP 丢包统计）。

用法:
  python fps_test.py [res] [quality] [duration] [host] [port] [mode]
    res      分辨率码 q/t/v/s/x/u   (默认 v=VGA)
    quality  JPEG质量 0-63, 省略则不动
    duration 测试秒数               (默认 10)
    host     默认 192.168.10.118
    port     默认 3333
    mode     udp(默认) | tcp

行为:
  - TCP 连板子发命令(q<quality> + v<res> [+ U])
  - udp: 收 UDP 分片重组, 统计整帧帧率/帧长 + 丢片率
  - tcp: 不启 UDP, 收 TCP 帧流(可靠)统计帧率 -> 与 udp 对比可区分"丢包"与"传感器上限"

帧协议复用 lib/protocol(OVU 头常量/帧校验) 与 lib/frame_parser(FRAME 切帧)。
UDP 重组保留本地实现: 测试工具需要精确的丢片/残帧统计, 与主程序的 UdpAssembler 各司其职。
"""
import os
import socket
import sys
import time

# 使 tools/ 下可 import host/lib 公共库(与主程序同一份帧协议实现)
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "host"))

from lib import frame_parser
from lib import protocol

res = sys.argv[1] if len(sys.argv) > 1 else "v"
quality = int(sys.argv[2]) if len(sys.argv) > 2 and sys.argv[2].isdigit() else -1
duration = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0
host = sys.argv[4] if len(sys.argv) > 4 else "192.168.10.118"
port = int(sys.argv[5]) if len(sys.argv) > 5 else 3333
mode = sys.argv[6] if len(sys.argv) > 6 else "udp"

HDR = protocol.HDR
MAGIC = protocol.MAGIC
CHUNK = protocol.CHUNK

# 命令: q<quality> + v<res> + U/N (N 必须显式发: 板子 udp_mode 是全局态,
# 若上次测试开过 U, TCP 模式不发 N 的话板子仍在 UDP 模式, TCP 连接收不到流)
cmd = b""
if quality >= 0:
    cmd += b"q" + bytes([quality])
cmd += ("v" + res).encode() + (b"U" if mode == "udp" else b"N")
print(f"模式={mode} 命令: {cmd!r}  (res={res} quality={quality if quality >= 0 else '-'})")


def open_tcp():
    for _ in range(60):
        try:
            c = socket.create_connection((host, port), timeout=4)
            c.settimeout(0.05)
            c.sendall(cmd)
            return c
        except Exception:
            time.sleep(1.0)
    return None


tcp = open_tcp()
if not tcp:
    print("TCP 连接失败"); sys.exit(1)

t0 = time.time()
last_print = t0
frames = 0
total_len = 0
last_frames = 0
fw = fh = ffmt = 0

if mode == "udp":
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        udp.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)  # 4MB 接收缓冲, 减少高速分片丢包
    except Exception:
        pass
    udp.bind(("0.0.0.0", port))
    udp.settimeout(0.05)
    ctx = {"seq": None, "cnt": 0, "got": 0, "parts": None}
    lost_parts = 0   # 累计丢片数(帧不完整被放弃时累加)
    dropped_frames = 0  # 未收齐被放弃的帧数

    while time.time() - t0 < duration:
        try:
            pkt, _ = udp.recvfrom(4096)
        except socket.timeout:
            now = time.time()
            # 当前帧 3s 没收齐 -> 判定为残帧, 放弃并累计丢片
            if ctx["seq"] is not None and now - ctx.get("t", 0) > 3.0:
                lost_parts += ctx["cnt"] - ctx["got"]
                dropped_frames += 1
                ctx = {"seq": None, "cnt": 0, "got": 0, "parts": None}
            if frames == last_frames and now - t0 > 4 and now - last_print > 1:
                print("!! 4s 无帧, 尝试重连", flush=True)
                try:
                    tcp.close()
                except Exception:
                    pass
                tcp = open_tcp()
                last_print = now
            continue
        if len(pkt) < HDR or pkt[:3] != MAGIC:
            continue
        seq = (pkt[4] << 8) | pkt[5]
        cnt = pkt[6] + 1
        idx = pkt[7]
        w = (pkt[8] << 8) | pkt[9]
        h = (pkt[10] << 8) | pkt[11]
        fmt = pkt[3]
        payload = pkt[HDR:]
        if ctx["seq"] != seq:
            if ctx["seq"] is not None and ctx["got"] < ctx["cnt"]:
                lost_parts += ctx["cnt"] - ctx["got"]
                dropped_frames += 1
            ctx = {"seq": seq, "cnt": cnt, "got": 0, "parts": [None] * cnt, "t": time.time()}
        if idx >= ctx["cnt"]:
            continue
        if ctx["parts"][idx] is None:
            ctx["parts"][idx] = payload
            ctx["got"] += 1
        if ctx["got"] == ctx["cnt"]:
            frame = b"".join(ctx["parts"])
            fw, fh, ffmt = w, h, fmt
            ctx = {"seq": None, "cnt": 0, "got": 0, "parts": None}
            if protocol.frame_ok(ffmt, frame, fw, fh):
                frames += 1
                total_len += len(frame)
        now = time.time()
        if now - last_print >= 1.0:
            fps = (frames - last_frames) / (now - last_print)
            print(f"fps={fps:6.1f}  {fw}x{fh} fmt={ffmt} 帧={frames} "
                  f"丢片={lost_parts} 残帧={dropped_frames}", flush=True)
            last_print = now
            last_frames = frames
    udp.close()
else:
    # ---- TCP 模式: 可靠帧流 ----
    tcp.settimeout(0.2)
    buf = bytearray()
    while time.time() - t0 < duration:
        try:
            d = tcp.recv(65536)
        except socket.timeout:
            continue
        if not d:
            print("!! TCP 断开", flush=True)
            break
        buf += d
        out, consumed = frame_parser.split_frames(buf)
        del buf[:consumed]
        for fw_, fh_, fmt_, body in out:
            fw, fh, ffmt = fw_, fh_, (1 if fmt_ == "JPG" else 0)
            frames += 1
            total_len += len(body)
        now = time.time()
        if now - last_print >= 1.0:
            fps = (frames - last_frames) / (now - last_print)
            print(f"fps={fps:6.1f}  {fw}x{fh} fmt={ffmt} 帧={frames}", flush=True)
            last_print = now
            last_frames = frames

dt = time.time() - t0
print(f"\n===== 结果 {mode} res={res} quality={quality} =====")
print(f"总帧={frames}  耗时={dt:.1f}s  平均fps={frames/dt:.1f}")
if frames:
    print(f"平均帧长={total_len/frames/1024:.1f}KB  平均帧率吞吐={(total_len/dt)/1024:.0f}KB/s")
if mode == "udp":
    exp = frames * (total_len / frames / CHUNK if frames else 0)
    print(f"丢片={lost_parts}  残帧={dropped_frames}")
try:
    tcp.close()
except Exception:
    pass
