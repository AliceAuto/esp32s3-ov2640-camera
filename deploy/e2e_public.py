#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""公网中继全链路 e2e: 真实服务器上 摄像头reg + 观众list/watch + 双向字节中继。"""
import socket, json, sys

HOST = sys.argv[1] if len(sys.argv) > 1 else "8.148.79.85"
PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 8001

def sendmsg(s, b):
    s.sendall(len(b).to_bytes(4, "big") + b)
def sendjson(s, o):
    sendmsg(s, json.dumps(o).encode())
def recvmsg(s, t=8):
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

def drain_json(s, k=2):
    """排掉建链后服务端主动下发的 p2p_peer 等 JSON 信令(它们不是被中继的二进制帧)。"""
    for _ in range(k):
        m = recvmsg(s)
        if m[:1] == b"{":
            continue
        return m

# 摄像头
cam = socket.create_connection((HOST, PORT)); cam.settimeout(8)
sendjson(cam, {"t": "reg", "id": "cam-e2e", "name": "E2E测试摄像头"})
r = json.loads(recvmsg(cam)); assert r["t"] == "reg_ok", r
print("[OK] 摄像头注册", r)

# 观众
v = socket.create_connection((HOST, PORT)); v.settimeout(8)
sendjson(v, {"t": "list"})
devs = json.loads(recvmsg(v))["devices"]
assert any(d["id"] == "cam-e2e" for d in devs), devs
print("[OK] 观众列表可见 cam-e2e")
sendjson(v, {"t": "watch", "id": "cam-e2e"})
assert json.loads(recvmsg(v))["t"] == "watch_ok"
assert json.loads(recvmsg(cam))["t"] == "viewer_attached"
print("[OK] 观看建立")

# 双向中继 (先排掉双方缓存的 p2p_peer 信令)
sendmsg(cam, b"FRAME_BEGIN fake jpeg FRAME_END")
got = drain_json(v)
if got != b"FRAME_BEGIN fake jpeg FRAME_END":
    raise SystemExit(f"上行中继失败: {got!r}")
print("[OK] 摄像头->观众 上行中继")
sendmsg(v, b"Vt")
got = drain_json(cam)
if got != b"Vt":
    raise SystemExit(f"下行中继失败: {got!r}")
print("[OK] 观众->摄像头 下行中继")

cam.close(); v.close()
print("PUBLIC E2E ALL PASS")