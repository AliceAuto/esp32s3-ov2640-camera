#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""分辨率切换延迟测量: 连接中继 watch 摄像头, 依次下发分辨率切换命令,
统计每次切换后 3s 内看到的帧到达最大间隔 —— 该间隔近似"切换导致的黑屏/冻结时长"。
快路径(寄存器 set_framesize)期望 ~几十ms; 慢路径(deinit+init)期望 ~数百ms~1s。
"""
import socket, json, time, sys

HOST = sys.argv[1] if len(sys.argv) > 1 else "relay.example.com"
PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 8001
DEV  = sys.argv[3] if len(sys.argv) > 3 else "cam-1DCD23D8"

def exact(s, n):
    d = b""
    while len(d) < n:
        r = s.recv(n - len(d))
        if not r: raise OSError("EOF")
        d += r
    return d

def recvmsg(s):
    return exact(s, int.from_bytes(exact(s, 4), "big"))

def sendjson(s, o):
    s.sendall(len(json.dumps(o)).to_bytes(4, "big") + json.dumps(o).encode())

def sendcmd(s, cmd):
    c = cmd.encode() + b"\x00"
    s.sendall(len(c).to_bytes(4, "big") + c)

s = socket.create_connection((HOST, PORT))
sendjson(s, {"t": "watch", "id": DEV})
r = json.loads(recvmsg(s))
assert r.get("t") == "watch_ok", r
recvmsg(s)                       # p2p_peer
s.settimeout(0.5)
time.sleep(1.0)                  # 预热

def switch(cmd):
    sendcmd(s, cmd)
    gaps, head, t = [], b"", time.time()
    end = t + 3.0
    while time.time() < end:
        try:
            m = recvmsg(s)
        except socket.timeout:
            continue
        if m.startswith(b"FRAME_BEGIN\n"):
            now = time.time()
            gaps.append(now - t_prev if 't_prev' in locals() else 0.0)
            t_prev = now
            head = m.split(b"\n", 2)[1]
    gaps_sorted = sorted(gaps)
    max_gap = gaps_sorted[-1] * 1000.0
    avg_gap = (sum(gaps) / len(gaps)) * 1000.0 if gaps else 0
    print(f"switch {cmd!r:4}: max_gap={max_gap:6.0f}ms avg={avg_gap:5.0f}ms frames={len(gaps)} head={head.strip().decode(errors='replace')}")

for cmd in ["Vt", "Vq", "Vt", "Vq", "Vt", "Vq"]:
    switch(cmd)

s.close()
print("DONE")