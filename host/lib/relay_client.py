# -*- coding: utf-8 -*-
"""中继客户端(对应 server/relay_server.py 的观众端)。

服务端协议: 每条消息 = 4B 大端长度 + payload; payload 为 JSON 信令或摄像头帧。
    list  -> 返回设备列表
    watch -> 返回 watch_ok 后持续推送帧消息

本模块只做消息流解包 + 信令握手, 帧内容由调用方交给 frame_parser 解析。
"""
import json
import socket
import threading
import time


def split_relay_spec(spec, default_port=8001):
    """拆中继服务端 "host:port" -> (host, port)。"""
    host, _, p = spec.rpartition(":")
    return (host or "127.0.0.1", int(p or default_port))


class RelayIo:
    """把中继服务端的"4B大端长度+payload"消息流解包成浏览器可读的数据流。

    观众端: 连中继 -> list -> watch(建立观看) -> 持续读取摄像头帧消息。
    read(n) 返回与 serial 同名的字节流(已滤掉 JSON 信令), 供 frame_parser 解析;
    write(b) 把命令打包成一条中继消息下发。"""

    def __init__(self, host, port, dev_id):
        self.s = socket.create_connection((host, port), timeout=8)
        self._rbuf = b""           # 原始 socket 字节(含长度前缀)
        self.buf = b""             # 解包后待消费的 payload 字节(供 read)
        self._buf_lock = threading.Lock()
        self.cam_id = None
        devs = self._handshake(dev_id)
        self.cam_id = devs["id"]
        # 握手完成后切换到短超时: 停流时 recv 最多阻塞 0.5s 即返回 None,
        # reader 循环的 RECONNECT_AFTER 检查能及时触发, 而非被 8s 默认超时卡死。
        self.s.settimeout(0.5)
        self.s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        print(f"经中继观看: {devs['id']} (via {host}:{port})", flush=True)

    def _send_json(self, o):
        b = json.dumps(o).encode()
        self.s.sendall(len(b).to_bytes(4, "big") + b)

    def _recv_exact(self, n):
        data = b""
        while len(data) < n:
            d = self.s.recv(n - len(data))
            if not d:
                raise OSError("中继连接关闭")
            data += d
        return data

    def _handshake(self, dev_id):
        self._send_json({"t": "list"})
        devs = json.loads(self._recv_exact(int.from_bytes(self._recv_exact(4), "big")))
        devs = devs.get("devices", [])
        if not devs:
            raise RuntimeError("中继上无在线摄像头")
        if dev_id:
            target = next((d for d in devs if d["id"] == dev_id or d["name"] == dev_id), None)
            if not target:
                raise RuntimeError(f"中继无设备 {dev_id}; 可用: {[d['id'] for d in devs]}")
        else:
            target = devs[0]
            print(f"中继在线: {[(d['id'], d.get('name')) for d in devs]}", flush=True)
            print(f"未指定 --id, 自动选择第一个: {target['id']}", flush=True)
        self._send_json({"t": "watch", "id": target["id"]})
        r = json.loads(self._recv_exact(int.from_bytes(self._recv_exact(4), "big")))
        if r.get("t") != "watch_ok":
            msg = r.get("msg", "")
            if msg == "unauthorized":
                raise RuntimeError("鉴权失败: 服务器要求 token, 请用 --token <口令> 指定")
            raise RuntimeError("观看请求失败: " + msg)
        return target

    def _read_one(self):
        """读一条完整消息; 无数据(超时)返回 None; 断连返回 b''。"""
        while len(self._rbuf) < 4:
            try:
                d = self.s.recv(4 - len(self._rbuf))
            except socket.timeout:
                return None
            except OSError:
                return b""
            if not d:
                return b""
            self._rbuf += d
        n = int.from_bytes(self._rbuf[:4], "big")
        self._rbuf = self._rbuf[4:]
        while len(self._rbuf) < n:
            try:
                d = self.s.recv(n - len(self._rbuf))
            except socket.timeout:
                return None
            except OSError:
                return b""
            if not d:
                return b""
            self._rbuf += d
        out, self._rbuf = self._rbuf[:n], self._rbuf[n:]
        return out

    def read(self, n):
        """返回至多 n 字节已解包数据(跳过服务端 JSON 信令); 无数据返回 b''。"""
        while True:
            with self._buf_lock:
                have = len(self.buf)
            if have:
                break
            msg = self._read_one()
            if msg is None or msg == b"":
                break
            if msg[:1] == b"{":
                continue          # 服务端信令, 跳过
            with self._buf_lock:
                self.buf += msg
                if self.buf:
                    break
        with self._buf_lock:
            out, self.buf = self.buf[:n], self.buf[n:]
        return out

    def write(self, b):
        if isinstance(b, str):
            b = b.encode()
        if not b:
            return 0
        self.s.sendall(len(b).to_bytes(4, "big") + b)
        return len(b)

    def flush(self):
        pass

    def close(self):
        try:
            self.s.close()
        except Exception:
            pass


def open_relay(spec, dev_id, stop_ev):
    """连接中继服务端并完成 watch 握手, 返回流式 RelayIo。失败逐次重试。"""
    host, port = split_relay_spec(spec)
    for _ in range(60):
        if stop_ev.is_set():
            return None
        try:
            return RelayIo(host, port, dev_id)
        except Exception as e:
            print("中继连接失败:", e, flush=True)
            time.sleep(1.0)
    return None
