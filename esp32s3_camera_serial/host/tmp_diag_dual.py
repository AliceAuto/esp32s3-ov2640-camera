# -*- coding: utf-8 -*-
"""TCP+UDP 双通道诊断: 连设备, 同时抓 TCP 与 UDP 上的 DIAG/帧。"""
import socket, select, time

CAM = "192.168.10.118"
PORT = 3333

t = socket.create_connection((CAM, PORT), timeout=3)
t.settimeout(0)
udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
udp.bind(("0.0.0.0", PORT))
udp.setblocking(0)
t.sendall(b"vtU")

t0 = time.time()
seen = set()
tbytes = ubytes = 0
while time.time() - t0 < 10:
    r, _, _ = select.select([t, udp], [], [], 0.5)
    for s in r:
        if s is t:
            try:
                d = s.recv(65536)
                if d:
                    tbytes += len(d)
                    txt = d.decode("utf-8", "replace")
                    for ln in txt.split("\n"):
                        if ln.startswith("DIAG:"):
                            print("[%.1f] TCP-DIAG: %s" % (time.time() - t0, ln), flush=True)
                    if tbytes < 4000 and not txt.startswith("DIAG:"):
                        print("[%.1f] TCP raw(len=%d): %r" % (time.time() - t0, len(d), d[:100]), flush=True)
            except Exception:
                pass
        else:
            try:
                d, a = s.recvfrom(65536)
            except Exception:
                continue
            if a[0] != CAM:
                continue
            ubytes += len(d)
            if d in seen:
                continue
            seen.add(d)
            txt = d.decode("utf-8", "replace")
            if txt.startswith("DIAG:"):
                print("[%.1f] UDP-DIAG from %s: %s" % (time.time() - t0, a[0], txt), flush=True)
            else:
                print("[%.1f] UDP-%s raw len=%d head=%r" % (time.time() - t0, a[0], len(d), d[:30]), flush=True)
print("done: TCP=%dB UDP=%dB" % (tbytes, ubytes))
t.close()
udp.close()