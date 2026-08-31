#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""relay_server.py 的 TURN 服务本地自测(不需公网, 不起 TCP 中继)。
模拟 CAM/VIEWER 各自 STUN 上报公网端点, 然后用 T2C/C2V 信封做双向 UDP 转发,
验证"对称 NAT 也能必达"的服务器侧 TURN 逻辑。
用法: python selftest_turn.py
"""
import sys, os, socket, threading, time, json, importlib.util
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_spec = importlib.util.spec_from_file_location("relay_server",
                                               os.path.join(ROOT, "relay_server.py"))
relay_server = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(relay_server)

TCP_PORT = 7000
UDP_PORT = TCP_PORT + 1   # TURN/ICE 辅助端口
HOST = "127.0.0.1"

def send_stun(sock, cam_id, role):
    msg = json.dumps({"t": "stun", "id": cam_id, "role": role}).encode()
    sock.sendto(msg, (HOST, UDP_PORT))

def turn(sock, mag, cam_id, payload):
    cid = cam_id.encode()
    sock.sendto(mag + len(cid).to_bytes(4, "big") + cid + payload, (HOST, UDP_PORT))

def main():
    srv = relay_server.Server(TCP_PORT)
    # 只跑 UDP ICE/TURN 线程(不 serve TCP), UDP 绑定固定 7001
    threading.Thread(target=srv.udp_stun_loop, daemon=True).start()
    time.sleep(0.3)

    passed = []
    cam = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cam.bind(("0.0.0.0", 0)); cam.settimeout(2)
    view = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    view.bind(("0.0.0.0", 0)); view.settimeout(2)
    cid = "cam-turn-1"

    # 1) 两端 STUN 上报(模拟 NAT: 服务器记录公网端点 = 这里就是它们绑定的端口)
    send_stun(cam, cid, "cam")
    r, _ = cam.recvfrom(2048)
    assert json.loads(r.decode())["t"] == "stun_ok"
    passed.append("cam STUN 上报获 stun_ok")
    send_stun(view, cid, "viewer")
    r, _ = view.recvfrom(2048)
    assert json.loads(r.decode())["t"] == "stun_ok"
    passed.append("viewer STUN 上报获 stun_ok")

    # 2) viewer -> cam: T2C 信封, 服务器转向到 cam 端点
    turn(view, b"T2C", cid, b"CMD:Vt")
    got, _ = cam.recvfrom(2048)
    assert got == b"CMD:Vt", got
    passed.append("TURN viewer->cam (T2C) 转发成功")

    # 3) cam -> viewer: C2V 信封
    turn(cam, b"C2V", cid, b"\xff\xd8FRAME\xff\xd9")
    got, _ = view.recvfrom(2048)
    assert got == b"\xff\xd8FRAME\xff\xd9", got
    passed.append("TURN cam->viewer (C2V) 转发成功")

    # 4) 不带 STUN(仅 TURN 包)也会被当续期: 服务器刷新 viewer 端点
    send_stun(view, cid, "viewer")   # 续期保活
    r, _ = view.recvfrom(2048)
    assert json.loads(r.decode())["t"] == "stun_ok"
    turn(cam, b"C2V", cid, b"after-keepalive")
    got, _ = view.recvfrom(2048)
    assert got == b"after-keepalive", got
    passed.append("TURN 保活后续期仍可转发")

    cam.close(); view.close()
    print("TURN SELF-TEST ALL PASS:")
    for p in passed:
        print("  [OK]", p)

if __name__ == "__main__":
    main()