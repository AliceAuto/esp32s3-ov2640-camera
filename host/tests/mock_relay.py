# -*- coding: utf-8 -*-
"""模拟中继服务端，用于验证 --relay 模式下 GUI 状态徽章和切换命令格式。
协议与 relay_server.py 的观众侧一致:
  每条消息 = 4B 大端长度 + payload
  list -> 返回设备列表
  watch -> 返回 watch_ok + p2p_peer
  持续推送模拟帧(FRAME_BEGIN\nSIZE 320 240 JPG\n...FRAME_END\n)
  收到字节命令(如 Vt / Vq) 直接打印到 stdout
"""
import json
import socket
import struct
import time
import threading

HOST = "127.0.0.1"
PORT = 8001
DEV_ID = "cam-test"


def send(sock, payload):
    sock.sendall(struct.pack(">I", len(payload)) + payload)


def relay_client(sock, addr):
    print(f"[{addr}] connected")
    try:
        while True:
            head = recv_exact(sock, 4)
            n = struct.unpack(">I", head)[0]
            msg = recv_exact(sock, n)
            try:
                j = json.loads(msg.decode("utf-8"))
            except Exception:
                # 字节命令
                print(f"[{addr}] raw cmd: {msg!r}")
                continue
            t = j.get("t")
            print(f"[{addr}] json: {j}")
            if t == "list":
                send(sock, json.dumps({"t": "list", "devices": [{"id": DEV_ID, "name": "test"}]}).encode())
            elif t == "watch":
                send(sock, json.dumps({"t": "watch_ok"}).encode())
                send(sock, json.dumps({"t": "p2p_peer"}).encode())
                # 持续推流
                threading.Thread(target=push_frames, args=(sock,), daemon=True).start()
    except Exception as e:
        print(f"[{addr}] closed: {e}")


def recv_exact(sock, n):
    data = b""
    while len(data) < n:
        r = sock.recv(n - len(data))
        if not r:
            raise OSError("EOF")
        data += r
    return data


def push_frames(sock):
    body = bytes([0xFF, 0xD8]) + bytes(7000) + bytes([0xFF, 0xD9])
    try:
        while True:
            payload = b"FRAME_BEGIN\nSIZE 320 240 JPG\n" + body + b"FRAME_END\n"
            send(sock, payload)
            time.sleep(0.05)
    except Exception as e:
        print(f"[push] stopped: {e}")


def main():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT))
    s.listen(5)
    print(f"mock relay listening on {HOST}:{PORT}")
    while True:
        conn, addr = s.accept()
        threading.Thread(target=relay_client, args=(conn, addr), daemon=True).start()


if __name__ == "__main__":
    main()
