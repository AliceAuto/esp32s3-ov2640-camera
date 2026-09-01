# -*- coding: utf-8 -*-
"""临时诊断: 独立验证服务器 TURN 转发链路(不依赖固件)。

流程:
  A 端(扮演 viewer): STUN 上报 cam-1DCD23D8 -> 服务器记录 viewer:cam-1DCD23D8
  B 端(扮演 cam):    STUN 上报 cam-TEST   -> 服务器记录 cam:cam-TEST
  B 端: 发 C2V 信封(cam_id=TEST) 带一帧 OVU 数据
  A 端: 应收到服务器转发的 payload
"""
import socket, json, time, sys

HOST = "8.148.79.85"
UDP = 8002
CAM_ID = "cam-TEST"   # A/B 使用同一测试 id, 模拟真实 viewer 与 cam 用同一 cam_id

def stun(sock, cam_id, role):
    sock.sendto(json.dumps({"t": "stun", "id": cam_id, "role": role}).encode(), (HOST, UDP))

def wait_pkt(sock, secs, label, needle=None):
    sock.settimeout(secs)
    t0 = time.time()
    while time.time() - t0 < secs:
        try:
            data, addr = sock.recvfrom(4096)
        except socket.timeout:
            continue
        if needle is not None and needle not in data:
            print(f"  [{label}] 忽略非目标包 {len(data)}B: {data[:40]!r}", flush=True)
            continue
        print(f"  [{label}] 收到 {len(data)}B from {addr}: {data[:60]!r}", flush=True)
        return data
    print(f"  [{label}] 超时无数据", flush=True)
    return None

# A 端: viewer 身份
a = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
a.bind(("0.0.0.0", 0))
stun(a, CAM_ID, "viewer")
print("A(viewer) STUN 上报 cam-1DCD23D8", flush=True)
time.sleep(1.0)

# B 端: cam 身份
b = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
b.bind(("0.0.0.0", 0))
stun(b, "cam-TEST", "cam")
print("B(cam) STUN 上报 cam-TEST", flush=True)
time.sleep(1.0)

# B 发 C2V 信封 -> cam_id=TEST -> 服务器应转向 viewer:TEST(即 A)
cid = b"cam-TEST"
env = b"C2V" + len(cid).to_bytes(4, "big") + cid
payload = b"OVU" + b"\x01\x02\x03\x04\x05\x06\x07\x08\x09\x0a" + b"HELLO-TURN-TEST"
b.sendto(env + payload, (HOST, UDP))
print(f"B 发 C2V(cam-TEST) {len(payload)}B payload", flush=True)

# A 应收到转发(跳过 stun_ok, 专门等 payload)
r = wait_pkt(a, 6.0, "A 等 C2V 转发", needle=payload)
ok = r is not None and payload in r
print(f"\nRESULT: 服务器 TURN 转发 {'成功' if ok else '失败'}", flush=True)
a.close(); b.close()
sys.exit(0 if ok else 1)
