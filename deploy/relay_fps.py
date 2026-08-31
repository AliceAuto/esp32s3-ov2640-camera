#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""定时帧率计数器: 以观众身份连中继 watch 摄像头, 统计 N 秒内收到的完整视频帧数与吞吐。"""
import socket, json, sys, time

HOST  = sys.argv[1] if len(sys.argv) > 1 else "8.148.79.85"
PORT  = int(sys.argv[2]) if len(sys.argv) > 2 else 8001
DEV   = sys.argv[3] if len(sys.argv) > 3 else "cam-1DCD23D8"
DUR   = float(sys.argv[4]) if len(sys.argv) > 4 else 12.0
CMD   = sys.argv[5] if len(sys.argv) > 5 else ""   # 如 "Vv" 切 VGA / "Vq" 切 QVGA

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

def sendjson(s, o):
    s.sendall(len(json.dumps(o)).to_bytes(4, "big") + json.dumps(o).encode())

s = socket.create_connection((HOST, PORT))
sendjson(s, {"t": "watch", "id": DEV})
r = json.loads(recvmsg(s))
if r.get("t") != "watch_ok":
    print(f"[FAIL] watch: {r}"); sys.exit(1)
json.loads(recvmsg(s))  # p2p_peer
if CMD:
    cmd = CMD.encode() + b"\x00"
    s.sendall(len(cmd).to_bytes(4, "big") + cmd)   # 长度前缀封装
    time.sleep(3.0)   # 等分辨率切换+sensor收敛
    print(f"[Cmd] 已下发 {CMD!r}")
print(f"[OK] watch {DEV} 建立, 计时 {DUR}s 统计帧数...")

frames = 0
bytes_ = 0
resol = None
s.settimeout(1.0)
t0 = time.time()
end = t0 + DUR
while time.time() < end:
    try:
        m = recvmsg(s)
    except socket.timeout:
        continue
    except OSError:
        break
    if m.startswith(b"FRAME_BEGIN\n") and m.endswith(b"FRAME_END\n"):
        head = m.split(b"\n", 2)[1]
        if head.startswith(b"SIZE"):
            frames += 1
            bytes_ += len(m)
            if resol is None:
                resol = head.decode()
dt = time.time() - t0
s.close()
print(f"RESULT frames={frames} dt={dt:.1f}s fps={frames/dt:.2f} "
      f"throughput={bytes_/1024/dt:.0f}KB/s avg={bytes_//max(frames,1)}B"
      + (f" resol={resol}" if resol else ""))
print("PASS" if frames > 0 else "FAIL")