# -*- coding: utf-8 -*-
"""
预览帧率命令行测试（UDP/TCP 双模式 + UDP 丢包统计）。

用法:
  python udp_fps.py [res] [quality] [duration] [host] [port] [mode]
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
"""
import sys, time, socket, re

res = sys.argv[1] if len(sys.argv) > 1 else "v"
quality = int(sys.argv[2]) if len(sys.argv) > 2 and sys.argv[2].isdigit() else -1
duration = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0
host = sys.argv[4] if len(sys.argv) > 4 else "192.168.10.118"
port = int(sys.argv[5]) if len(sys.argv) > 5 else 3333
mode = sys.argv[6] if len(sys.argv) > 6 else "udp"

HDR = 12
MAGIC = b"OVU"
CHUNK = 1400 - HDR
FB_END = b"FRAME_END\n"
_FRAME = re.compile(rb'FRAME_BEGIN\nSIZE (\d+) (\d+) (\S+)\n(.*?)FRAME_END\n', re.S)

# 命令: q<quality> + v<res> (+ U 仅 udp 模式)
cmd = b""
if quality >= 0:
    cmd += b"q" + bytes([quality])
cmd += ("v" + res).encode() + (b"U" if mode == "udp" else b"")
print(f"模式={mode} 命令: {cmd!r}  (res={res} quality={quality if quality>=0 else '-'})")


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
                try: tcp.close()
                except Exception: pass
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
            ok = False
            if ffmt == 1:
                ok = len(frame) > 100 and frame[:2] == b"\xff\xd8" and frame[-2:] == b"\xff\xd9"
            else:
                ok = len(frame) == fw * fh * 2 and fw > 0 and fh > 0
            if ok:
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
    buf = b""
    while time.time() - t0 < duration:
        try:
            d = tcp.recv(65536)
        except socket.timeout:
            continue
        if not d:
            print("!! TCP 断开", flush=True)
            break
        buf += d
        m = _FRAME.search(buf)
        if m:
            fw, fh, fmt = int(m.group(1)), int(m.group(2)), m.group(3)
            ffmt = 1 if fmt == b"JPG" else 0
            frames += 1
            total_len += len(m.group(4))
            buf = buf[m.end():]
            # 校验 JPEG 完整性
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
