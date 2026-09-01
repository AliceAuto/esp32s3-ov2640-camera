# -*- coding: utf-8 -*-
"""诊断: 以观众身份注册 UDP 端点(STUN), watch 真实摄像头,
等待其 TURN(C2V) 转发的 OVU 帧 —— 验证 摄像头->服务器->观众 的 TURN 链路。"""
import socket, struct, json, sys, time, select

HOST, PORT = "8.148.79.85", 8001
UDP = PORT + 1
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
    return recv_exact(s, struct.unpack(">I", h)[0])

def main():
    # 1) UDP 候选: 开 3 个, 全部 STUN 上报
    socks = []
    for _ in range(3):
        u = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        u.bind(("0.0.0.0", 0))
        u.setblocking(False)
        socks.append(u)
    stun = json.dumps({"t": "stun", "id": CAM, "role": "viewer"}).encode()
    for u in socks:
        u.sendto(stun, (HOST, UDP))
        print(f"STUN 上报端点: {u.getsockname()}", flush=True)
    time.sleep(0.5)

    # 2) TCP: list -> watch
    s = socket.create_connection((HOST, PORT), timeout=8)
    s.settimeout(8)
    send_json(s, {"t": "list"})
    recv_msg(s)
    send_json(s, {"t": "watch", "id": CAM})
    # 等待 watch_ok 与 p2p_peer
    t0 = time.time()
    got_peer = False
    while time.time() - t0 < 8:
        m = recv_msg(s)
        if m is None:
            print("TCP 连接关闭", flush=True)
            return 1
        if m[:1] == b"{":
            txt = m.decode("utf-8", "replace")
            print("TCP 信令:", txt[:150], flush=True)
            if '"t": "watch_ok"' in txt or '"t":"watch_ok"' in txt or "watch_ok" in txt:
                break
        else:
            print("收到非信令数据", len(m), flush=True)
            break
    s.settimeout(0.2)

    # 3) 在 UDP 上等待 TURN 转发的 OVU 帧
    print("== 等待 TURN 帧 (12s) ==", flush=True)
    t0 = time.time()
    frames = 0
    ovu_pkts = 0
    while time.time() - t0 < 12:
        r, _, _ = select.select(socks + [s], [], [], 0.5)
        for u in r:
            if u is s:
                try:
                    m = s.recv(4096)
                except Exception:
                    continue
                if m and m[:1] == b"{":
                    print("TCP 信令:", m[:120], flush=True)
                continue
            try:
                pkt, addr = u.recvfrom(2048)
            except Exception:
                continue
            if pkt[:3] == b"P2P":
                print(f"  UDP 打洞包 from {addr}", flush=True)
                continue
            if pkt[:3] == b"OVU":
                ovu_pkts += 1
                if ovu_pkts <= 3:
                    print(f"  OVU 分片 from {addr} len={len(pkt)} fmt={pkt[3]} "
                          f"seq={(pkt[4]<<8)|pkt[5]} w={(pkt[8]<<8)|pkt[9]} h={(pkt[10]<<8)|pkt[11]}",
                          flush=True)
                continue
            print(f"  其他 UDP from {addr} len={len(pkt)}: {pkt[:30]!r}", flush=True)
        if ovu_pkts and time.time() - t0 > 12:
            break
    print(f"== 共收到 OVU 分片 {ovu_pkts} 个 ==", flush=True)
    for u in socks:
        u.close()
    s.close()
    return 0

if __name__ == "__main__":
    sys.exit(main())
