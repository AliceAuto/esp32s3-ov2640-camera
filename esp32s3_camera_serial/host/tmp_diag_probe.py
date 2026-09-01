# -*- coding: utf-8 -*-
"""读固件 DIAG 心跳: 连 TCP, 观察 cap(采帧)/sent(发送)/uerr/skp, 判断采帧与发送链路。"""
import socket, sys, time

CAM = sys.argv[1] if len(sys.argv) > 1 else "192.168.10.118"
PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 3333

tcp = socket.create_connection((CAM, PORT), timeout=4)
tcp.settimeout(2.0)
tcp.sendall(b"vtU")   # v 预览分辨率(t), U 开 UDP 模式
print(f"TCP 已连 {CAM}:{PORT}, 已发 'vtU', 监听 DIAG 心跳 10s...")

buf = b""
t0 = time.time()
diags = []
while time.time() - t0 < 10.0:
    try:
        d = tcp.recv(65536)
    except socket.timeout:
        continue
    except OSError:
        break
    if not d:
        break
    buf += d
    # 逐行提取 DIAG
    while b"\n" in buf:
        line, buf = buf.split(b"\n", 1)
        if line.startswith(b"DIAG:"):
            diags.append(line.decode("utf-8", "replace"))

print(f"\n=== DIAG 心跳 ({len(diags)} 条) ===")
for s in diags:
    print(s)
if not diags:
    print("(未收到任何 DIAG 心跳)")
print("--- 原始接收字节数:", len(buf), "---")
tcp.close()
