# -*- coding: utf-8 -*-
"""最小存活检查: 只连接->watch->收 6s 帧, 不下发任何命令。"""
import socket, struct, json, sys, time

def main():
    s = socket.create_connection(("8.148.79.85", 8001), timeout=8)
    s.settimeout(8)
    b = json.dumps({"t": "list"}).encode()
    s.sendall(len(b).to_bytes(4, "big") + b)
    h = s.recv(4)
    n = struct.unpack(">I", h)[0]
    data = b""
    while len(data) < n:
        d = s.recv(n - len(data))
        if not d:
            break
        data += d
    devs = json.loads(data).get("devices", [])
    cam = next((d for d in devs if d["id"] == "cam-1DCD23D8"), None)
    if not cam:
        print("摄像头不在线", flush=True)
        return 1
    print("摄像头在线:", cam.get("since"), flush=True)
    b = json.dumps({"t": "watch", "id": "cam-1DCD23D8"}).encode()
    s.sendall(len(b).to_bytes(4, "big") + b)
    s.settimeout(0.5)
    t0 = time.time()
    frames = 0
    sizes = {}
    while time.time() - t0 < 6:
        try:
            h = s.recv(4)
        except socket.timeout:
            continue
        if len(h) < 4:
            break
        n = struct.unpack(">I", h)[0]
        d = b""
        while len(d) < n:
            x = s.recv(n - len(d))
            if not x:
                break
            d += x
        if d[:1] == b"{":
            continue
        if d.startswith(b"FRAME_BEGIN"):
            frames += 1
            parts = d.split(b"\n")[1].split()
            if len(parts) >= 4:
                sizes[(int(parts[1]), int(parts[2]))] = sizes.get((int(parts[1]), int(parts[2])), 0) + 1
    print(f"6s 内收到 {frames} 帧, 尺寸 {dict(sizes)}", flush=True)
    s.close()
    return 0

if __name__ == "__main__":
    sys.exit(main())
