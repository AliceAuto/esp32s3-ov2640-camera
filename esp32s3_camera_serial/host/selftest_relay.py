# -*- coding: utf-8 -*-
"""
relay_server.py 的本地自检测试(不需公网)。
模拟 1 台摄像头 + 1 名终端, 验证 reg/list/watch/双向转发全链路。
用法: python host/selftest_relay.py
"""
import sys, os, socket, threading, time, json, importlib.util
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_relay_py = os.path.join(ROOT, "relay_server.py")
_spec = importlib.util.spec_from_file_location("relay_server", _relay_py)
relay_server = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(relay_server)

HOST, PORT = "127.0.0.1", 0   # PORT=0 -> 随机端口, 避免冲突


def recv_msg(sock, timeout=3):
    sock.settimeout(timeout)
    head = b""
    while len(head) < 4:
        part = sock.recv(4 - len(head))
        if not part:            # 连接被服务端关闭(EOF)
            return None
        head += part
    ln = int.from_bytes(head, "big")
    payload = b""
    while len(payload) < ln:
        part = sock.recv(ln - len(payload))
        if not part:
            return None
        payload += part
    return payload


def send_payload(sock, payload: bytes):
    sock.sendall(len(payload).to_bytes(4, "big") + payload)


def send_json(sock, obj):
    send_payload(sock, json.dumps(obj).encode())


def main():
    srv = relay_server.Server(PORT)

    # 监听一个随机端口
    ln = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    ln.bind((HOST, 0))
    ln.listen(8)
    port = ln.getsockname()[1]
    threading.Thread(target=srv.accept_loop, args=(ln,), daemon=True).start()

    passed = []

    # ---- 模拟摄像头 ----
    cam = socket.create_connection((HOST, port))
    send_json(cam, {"t": "reg", "id": "cam-1", "name": u"客厅摄像头"})
    r = json.loads(recv_msg(cam).decode())
    assert r["t"] == "reg_ok", r
    passed.append("摄像头 注册 reg_ok")

    # ---- 模拟终端: 列表 ----
    v = socket.create_connection((HOST, port))
    send_json(v, {"t": "list"})
    devs = json.loads(recv_msg(v).decode())
    assert any(d.get("id") == "cam-1" for d in devs["devices"]), devs
    passed.append("终端 列表含 cam-1")

    # ---- 模拟终端: 观看 ----
    send_json(v, {"t": "watch", "id": "cam-1"})
    assert json.loads(recv_msg(v).decode())["t"] == "watch_ok"
    _att = json.loads(recv_msg(cam).decode())
    assert _att["t"] == "viewer_attached", _att
    passed.append("观看建立 viewer_attached/watch_ok")

    # ---- P2P 撮合: 双方应收到 p2p_peer(对端公网端点) ----
    _p2p_cam = json.loads(recv_msg(cam).decode())
    assert _p2p_cam["t"] == "p2p_peer" and _p2p_cam.get("ip") and _p2p_cam.get("port"), _p2p_cam
    _p2p_v = json.loads(recv_msg(v).decode())
    assert _p2p_v["t"] == "p2p_peer" and _p2p_v.get("ip") and _p2p_v.get("port"), _p2p_v
    passed.append(f"P2P 撮合 p2p_peer cam->{_p2p_cam['ip']}:{_p2p_cam['port']} viewer->{_p2p_v['ip']}:{_p2p_v['port']}")

    # ---- 摄像头数据上行 -> 终端收到 ----
    send_payload(cam, b"\xff\xd8FAKEFRAME\xff\xd9")
    got = recv_msg(v)
    assert got == b"\xff\xd8FAKEFRAME\xff\xd9", got
    passed.append("摄像头上行数据被中继到终端")

    # ---- 终端命令下行 -> 摄像头收到 ----
    send_payload(v, b"Vt")
    got = recv_msg(cam)
    assert got == b"Vt", got
    passed.append("终端下行命令被中继到摄像头")

    # ---- 摄像头离线 -> 终端被断开 ----
    cam.close()
    time.sleep(0.6)
    got = None
    try:
        got = recv_msg(v, 1)
    except socket.timeout:
        pass
    if got is not None:
        raise AssertionError(f"cam 离线后终端应被断开/无数据, got={got!r}")
    passed.append("摄像头离线后终端被正确回收/断开")

    print("ALL PASS:")
    for p in passed:
        print("  [OK]", p)


if __name__ == "__main__":
    main()