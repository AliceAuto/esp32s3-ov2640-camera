# -*- coding: utf-8 -*-
"""诊断: 中继模式下运行时分辨率切换 Vu(UXGA) -> Vt(QVGA) 是否生效。"""
import socket, struct, json, sys, time

HOST, PORT = "8.148.79.85", 8001
CAM = "cam-1DCD23D8"

def send_json(s, o):
    b = json.dumps(o).encode()
    s.sendall(len(b).to_bytes(4, "big") + b)

def recv_exact(s, n):
    data = b""
    while len(data) < n:
        d = s.recv(n - len(data))
        if not d:
            return b""
        data += d
    return data

def recv_msg(s):
    h = recv_exact(s, 4)
    if len(h) < 4:
        return None
    n = struct.unpack(">I", h)[0]
    return recv_exact(s, n)

def cmd(s, c):
    s.sendall(len(c).to_bytes(4, "big") + c)
    print(f"  发送命令 {c!r}", flush=True)

def collect(s, secs, label):
    from collections import Counter
    cnt = Counter()
    t0 = time.time()
    while time.time() - t0 < secs:
        try:
            m = recv_msg(s)
        except socket.timeout:
            continue
        if not m:
            break
        if m[:1] == b"{":
            continue
        if m.startswith(b"FRAME_BEGIN"):
            parts = m.split(b"\n")[1].split()
            if len(parts) >= 4:
                cnt[(int(parts[1]), int(parts[2]))] += 1
    print(f"  [{label}] 帧尺寸统计: {dict(cnt)}  (总帧数={sum(cnt.values())})", flush=True)
    return cnt

def main():
    s = socket.create_connection((HOST, PORT), timeout=8)
    s.settimeout(8)
    send_json(s, {"t": "list"})
    recv_msg(s)
    send_json(s, {"t": "watch", "id": CAM})
    while True:
        m = recv_msg(s)
        if m is None:
            print("连接关闭", flush=True)
            return 1
        if m[:1] == b"{":
            continue
        if m.startswith(b"FRAME_BEGIN"):
            break
    s.settimeout(0.5)
    collect(s, 4, "初始")
    cmd(s, b"Vu")
    time.sleep(2)
    collect(s, 4, "Vu 后(期望 UXGA 1600x1200)")
    cmd(s, b"Vt")
    time.sleep(2)
    collect(s, 5, "Vt 后(期望 QVGA 320x240)")
    s.close()
    return 0

if __name__ == "__main__":
    sys.exit(main())
