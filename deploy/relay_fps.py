#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""定时帧率计数器: 以观众身份连中继 watch 摄像头, 统计 N 秒内收到的完整视频帧数与吞吐。"""
import socket, json, sys, time

HOST  = sys.argv[1] if len(sys.argv) > 1 else "relay.example.com"
PORT  = int(sys.argv[2]) if len(sys.argv) > 2 else 8001
DEV   = sys.argv[3] if len(sys.argv) > 3 else "cam-1DCD23D8"
DUR   = float(sys.argv[4]) if len(sys.argv) > 4 else 12.0
CMD   = sys.argv[5] if len(sys.argv) > 5 else ""   # 如 "Vv" 切 VGA / "Vq" 切 QVGA
TOKEN = sys.argv[6] if len(sys.argv) > 6 else ""   # 服务器启用鉴权时的口令

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
watch = {"t": "watch", "id": DEV}
if TOKEN:
    watch["token"] = TOKEN
sendjson(s, watch)
r = json.loads(recvmsg(s))
if r.get("t") != "watch_ok":
    print(f"[FAIL] watch: {r}")
    if r.get("msg") == "unauthorized":
        print("提示: 服务器启用了 token 鉴权, 用法 python relay_fps.py HOST PORT DEV DUR CMD TOKEN")
    sys.exit(1)
json.loads(recvmsg(s))  # p2p_peer
if CMD:
    # 可靠 JSON 命令通道: 下发切分辨率并等待摄像头 cmd_ack 回执(结果可判空/可判定)。
    sendjson(s, {"t": "cmd", "res": CMD, "token": TOKEN})
    s.settimeout(6.0)
    deadline = time.time() + 6.0
    acked = None
    while time.time() < deadline:
        try:
            m = recvmsg(s)
        except socket.timeout:
            break
        except OSError:
            break
        try:
            j = json.loads(m.decode("utf-8"))
        except Exception:
            continue
        if j.get("t") == "cmd_ack" and j.get("res") == CMD:
            acked = j
            break
    if acked is not None:
        print(f"[Cmd] ACK ok={acked.get('ok')} fs={acked.get('fs')}"
              + (" (已执行)" if acked.get("ok") else " (被拒/失败)"))
    else:
        print("[Cmd] 未收到 ACK(超时), 命令结果未知")
    s.settimeout(None)
    time.sleep(2.0)   # 等分辨率切换+sensor收敛
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