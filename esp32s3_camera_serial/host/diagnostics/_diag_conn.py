# -*- coding: utf-8 -*-
"""诊断: 中继服务器可达性 + 在线设备列表。"""
import socket, struct, json, sys

def main():
    try:
        s = socket.create_connection(("8.148.79.85", 8001), timeout=6)
        print("TCP 8001 连接成功", flush=True)
    except Exception as e:
        print("TCP 8001 连接失败:", repr(e), flush=True)
        return 1
    try:
        msg = b'{"t":"list"}'
        s.sendall(len(msg).to_bytes(4, "big") + msg)
        h = s.recv(4)
        print("收到长度头:", h, flush=True)
        if len(h) < 4:
            print("服务器未返回长度头(连接被关闭)", flush=True)
            return 1
        n = struct.unpack(">I", h)[0]
        data = b""
        while len(data) < n:
            d = s.recv(n - len(data))
            if not d:
                break
            data += d
        print("在线设备:", data.decode("utf-8", "replace"), flush=True)
        return 0
    except Exception as e:
        print("协议异常:", repr(e), flush=True)
        return 1

if __name__ == "__main__":
    sys.exit(main())
