# -*- coding: utf-8 -*-
"""快速扫描网段内开 3333 端口的设备(定位摄像头当前 IP)。"""
import socket, sys, concurrent.futures

def probe(ip, port=3333, timeout=0.6):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(timeout)
    try:
        s.connect((ip, port))
        return ip
    except Exception:
        return None
    finally:
        s.close()

# 默认扫 192.168.10.x 和 192.168.4.x
nets = ["192.168.10", "192.168.4", "192.168.31", "192.168.1"]
found = []
for net in nets:
    with concurrent.futures.ThreadPoolExecutor(max_workers=100) as ex:
        futs = {ex.submit(probe, f"{net}.{i}"): f"{net}.{i}" for i in range(1, 255)}
        for f in concurrent.futures.as_completed(futs):
            r = f.result()
            if r:
                found.append(r)
                print("FOUND:", r)
if not found:
    print("未发现 3333 端口开放的主机")