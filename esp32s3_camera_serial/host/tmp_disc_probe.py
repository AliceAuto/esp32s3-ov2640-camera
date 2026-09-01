# -*- coding: utf-8 -*-
"""一次性：UDP 广播发现，抓相机固件局域网广播(固件 disc_task DISCOVERY_PORT 9999)。"""
import socket, time

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
try:
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
except OSError:
    pass
s.bind(("0.0.0.0", 9999))
s.settimeout(0.5)
# 也监听 8999 备用
s2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
try:
    s2.bind(("0.0.0.0", 3333))
    s2.settimeout(0.5)
except OSError:
    s2.close(); s2 = None

socks = [s] + ([s2] if s2 else [])
t0 = time.time()
hits = []
while time.time() - t0 < 12.0:
    for x in socks:
        try:
            pkt, addr = x.recvfrom(4096)
        except socket.timeout:
            continue
        hits.append((addr, pkt))
        print(f"来自 {addr}: {pkt!r}", flush=True)
    time.sleep(0.05)
print(f"\n=== 收到 {len(hits)} 条 ===")
s.close()
if s2: s2.close()