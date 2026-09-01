# -*- coding: utf-8 -*-
"""诊断: 模拟 viewer 走 list->watch 握手, 观察每一步返回。"""
import socket, struct, json, sys, time

HOST, PORT = "8.148.79.85", 8001

def send_json(s, o):
    b = json.dumps(o).encode()
    s.sendall(len(b).to_bytes(4, "big") + b)

def recv_msg(s, label):
    h = s.recv(4)
    if len(h) < 4:
        print(f"[{label}] 连接被关闭", flush=True)
        return None
    n = struct.unpack(">I", h)[0]
    data = b""
    while len(data) < n:
        d = s.recv(n - len(data))
        if not d:
            break
        data += d
    print(f"[{label}] 收到 {len(data)}B: {data[:200]!r}", flush=True)
    return data

def main():
    s = socket.create_connection((HOST, PORT), timeout=8)
    s.settimeout(8)
    send_json(s, {"t": "list"})
    r = recv_msg(s, "list")
    if not r:
        return 1
    devs = json.loads(r).get("devices", [])
    target = next((d for d in devs if d["id"] == "cam-1DCD23D8"), None)
    if not target:
        print("cam-1DCD23D8 不在线", flush=True)
        return 1
    send_json(s, {"t": "watch", "id": "cam-1DCD23D8"})
    recv_msg(s, "watch")
    # 之后 3s 内看是否有 viewer_attached/p2p_peer 或任何数据
    s.settimeout(1.0)
    t0 = time.time()
    while time.time() - t0 < 4:
        try:
            recv_msg(s, "后续")
        except socket.timeout:
            continue
        except Exception as e:
            print("读取异常:", repr(e), flush=True)
            break
    print("握手诊断完成", flush=True)
    s.close()
    return 0

if __name__ == "__main__":
    sys.exit(main())
