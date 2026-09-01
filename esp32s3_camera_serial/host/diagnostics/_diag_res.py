# -*- coding: utf-8 -*-
"""诊断: 中继模式下 viewer 下发分辨率命令, 观察摄像头是否切到 QVGA。
连接 -> watch -> 先收帧统计尺寸 -> 下发 Vt(预览QVGA) -> 再统计尺寸。"""
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

    sizes = {}
    def collect(secs):
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
                first = m.split(b"\n")[1]  # SIZE w h FMT
                parts = first.split()
                if len(parts) >= 4:
                    wh = (int(parts[1]), int(parts[2]))
                    sizes[wh] = sizes.get(wh, 0) + 1
                    print(f"  帧 {wh[0]}x{wh[1]} len={len(m)}B", flush=True)

    print("== 下发前 5s 采集 ==", flush=True)
    collect(5)
    print("== 下发 Vt (预览 QVGA) ==", flush=True)
    s.settimeout(8)
    cmd = b"Vt"
    s.sendall(len(cmd).to_bytes(4, "big") + cmd)
    time.sleep(1)
    s.settimeout(0.5)
    print("== 下发后 6s 采集 ==", flush=True)
    sizes.clear()
    collect(6)
    print("尺寸统计:", dict(sizes), flush=True)
    s.close()
    return 0

if __name__ == "__main__":
    sys.exit(main())
