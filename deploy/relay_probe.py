#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""跨网观看探针: 以观众身份连中继, watch 真实摄像头, 连收几帧校验视频能流下来。"""
import socket, json, sys

HOST = sys.argv[1] if len(sys.argv) > 1 else "8.148.79.85"
PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 8001
DEV = sys.argv[3] if len(sys.argv) > 3 else "cam-1DCD23D8"
FRAMES = int(sys.argv[4]) if len(sys.argv) > 4 else 5

def sendmsg(s, b):
    s.sendall(len(b).to_bytes(4, "big") + b)
def sendjson(s, o):
    sendmsg(s, json.dumps(o).encode())
def recvmsg(s, t=10):
    s.settimeout(t)
    def exact(n):
        d = b""
        while len(d) < n:
            r = s.recv(n - len(d))
            if not r:
                raise OSError("EOF")
            d += r
        return d
    return exact(int.from_bytes(exact(4), "big"))

s = socket.create_connection((HOST, PORT))
sendjson(s, {"t": "watch", "id": DEV})
# 建链信令: watch_ok + p2p_peer (按序到达, 都是 JSON)
r = json.loads(recvmsg(s)); 
if r.get("t") != "watch_ok":
    print(f"[FAIL] watch: {r}"); sys.exit(1)
json.loads(recvmsg(s))  # p2p_peer
print(f"[OK] watch {DEV} 建立, 等待真实视频帧...")

got = 0
for i in range(FRAMES):
    m = recvmsg(s)
    if m.startswith(b"FRAME_BEGIN\n") and m.endswith(b"FRAME_END\n"):
        head = m.split(b"\n", 2)[1]                   # "SIZE W H JPG"
        if head.startswith(b"SIZE") and b"JPG" in head:
            got += 1
            print(f"  帧{i+1}: {head.decode()} len={len(m)}B")
        else:
            print(f"  帧{i+1}: 帧头异常 {head[:40]!r}")
    else:
        print(f"  帧{i+1}: 非预期数据 {m[:40]!r}")

s.close()
print(f"{'RESULT: PASS' if got else 'RESULT: FAIL'} 收到 {got}/{FRAMES} 帧")
sys.exit(0 if got else 1)