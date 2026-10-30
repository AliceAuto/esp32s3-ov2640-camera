# -*- coding: utf-8 -*-
"""组合诊断: 同一进程发起 TCP watch + 读取相机串口日志。
用于观察相机收到 watch/viewer_udp/p2p_peer 后走哪条发送路径。
用法: python relay_watch_cam.py <serial> <tcp_host> <tcp_port> <cam_id> [seconds]
"""
import sys, time, threading, socket, json, re, serial

SER = sys.argv[1] if len(sys.argv) > 1 else "COM10"
HOST = sys.argv[2] if len(sys.argv) > 2 else "relay.example.com"
PORT = int(sys.argv[3]) if len(sys.argv) > 3 else 8001
DEV = sys.argv[4] if len(sys.argv) > 4 else "cam-1DCD23D8"
SECS = int(sys.argv[5]) if len(sys.argv) > 5 else 15

def sendmsg(s, b):
    s.sendall(len(b).to_bytes(4, "big") + b)
def sendjson(s, o):
    sendmsg(s, json.dumps(o).encode())
def recvmsg(s, t):
    s.settimeout(t)
    def exact(n):
        d = b""
        while len(d) < n:
            r = s.recv(n - len(d))
            if not r: raise OSError("EOF")
            d += r
        return d
    return exact(int.from_bytes(exact(4), "big"))

# 相机日志监控线程
relay_events = []
stop = threading.Event()
KW = re.compile(r'\[(P2P|中继)\]|viewer|超时|直连|TURN|打洞|切|收到|进入|发送|cand|端点', re.I)
DROP = re.compile(r'endPacket|could not send|Preferences|nvs|WiFiUdp')

def cam_monitor():
    try:
        s = serial.Serial(SER, 2000000, timeout=0.2)
    except Exception as e:
        relay_events.append(f"[SER] 串口打开失败 {e}")
        return
    while not stop.is_set():
        d = s.read(4096)
        if not d: time.sleep(0.05); continue
        try: txt = d.decode("utf-8", "ignore")
        except: continue
        for line in txt.splitlines():
            if DROP.search(line): continue
            if KW.search(line): relay_events.append(line)
    s.close()

threading.Thread(target=cam_monitor, daemon=True).start()
time.sleep(0.5)

s = socket.create_connection((HOST, PORT))
sendjson(s, {"t": "watch", "id": DEV})
frame_pkts = 0
start = time.time()
try:
    while time.time() - start < SECS:
        try:
            m = recvmsg(s, 3)
        except (OSError, socket.timeout):
            print(f"[TCP] 3s 无消息, 累计帧包={frame_pkts}")
            if frame_pkts: break
            continue
        if m.startswith(b"FRAME_BEGIN\n"):
            frame_pkts += 1
            head = m.split(b"\n", 2)[1]
            print(f"[TCP] 帧#{frame_pkts}: {head.decode().strip()} len={len(m)}B")
        else:
            print(f"[TCP] 信令: {m[:120]!r}")
finally:
    s.close()
    stop.set()

print("\n===== 相机端关键日志 =====")
for line in relay_events[-40:]:
    print(line, flush=True)
print(f"\n结果: TCP 收到 {frame_pkts} 帧")