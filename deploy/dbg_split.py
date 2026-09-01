# -*- coding: utf-8 -*-
"""临时诊断: 用与 live_capture 相同的 RelayIo 解包 + split_frames 收流,
打印每 2s 的 切帧数/valid 判定/残留长度, 复现 valid=0 是否属于解析层。"""
import sys, socket, time, json
sys.path.insert(0, r"d:\Projects\ov2640\esp32s3_camera_serial\host")
from lib.frame_parser import split_frames, BEGIN, END

HOST, PORT, DEV = "8.148.79.85", 8001, "cam-1DCD23D8"

def sjson(s, o):
    b = json.dumps(o).encode()
    s.sendall(len(b).to_bytes(4, "big") + b)

def exact(s, n):
    d = b""
    while len(d) < n:
        r = s.recv(n - len(d))
        if not r:
            raise OSError("EOF")
        d += r
    return d

def recvmsg(s):
    return exact(s, int.from_bytes(exact(s, 4), "big"))

s = socket.create_connection((HOST, PORT), timeout=8)
sjson(s, {"t": "watch", "id": DEV})
r = json.loads(recvmsg(s))
assert r.get("t") == "watch_ok", r
json.loads(recvmsg(s))  # p2p_peer
s.settimeout(0.05)
print("watch ok, 收流中...", flush=True)

buf = bytearray()
t0 = time.time(); win = t0; parsed = 0; valid = 0; last_len = 0
while time.time() - t0 < 8:
    try:
        msg = recvmsg(s)
    except socket.timeout:
        # 模拟 reader 无数据分支
        if time.time() - win >= 2.0:
            print(f"win: parsed={parsed} valid={valid} buflen={len(buf)} lastLen={last_len}", flush=True)
            parsed = 0; valid = 0; win = time.time()
        continue
    except OSError:
        break
    if msg[:1] == b"{":
        continue
    last_len = len(msg)
    buf += msg
    frames, consumed = split_frames(bytes(buf))
    for w, h, fmt, body in frames:
        if fmt == "JPG":
            v = len(body) > 100 and body[:2] == b"\xff\xd8" and body[-2:] == b"\xff\xd9"
        else:
            v = True
        valid += 1 if v else 0
        parsed += 1
    if time.time() - win >= 2.0:
        print(f"win: parsed={parsed} valid={valid} buflen={len(buf)} lastLen={last_len}", flush=True)
        parsed = 0; valid = 0; win = time.time()
    del buf[:consumed]
s.close()
print("done")