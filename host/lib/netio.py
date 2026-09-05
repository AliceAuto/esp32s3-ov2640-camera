# -*- coding: utf-8 -*-
"""局域网网络 IO: 设备发现 + 命令/流通道连接。"""
import socket
import time

DISCOVERY_PORT = 9999


def discover_cameras(timeout=4.0):
    """监听 UDP 9999 广播，收集 {ip: 设备名}。摄像头入网后每 3s 广播一次。"""
    found = {}
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    except Exception:
        pass
    s.bind(("0.0.0.0", DISCOVERY_PORT))
    s.settimeout(0.5)
    end = time.time() + timeout
    while time.time() < end:
        try:
            data, _ = s.recvfrom(512)
        except socket.timeout:
            continue
        except Exception:
            break
        txt = data.decode(errors="ignore")
        if txt.startswith("OV2640|"):
            p = txt.split("|")
            if len(p) >= 3:
                found[p[2]] = p[1]
    s.close()
    return found


def open_cmd_channel(host, port, buf, *, retries=600, stop_ev=None,
                     timeout=4.0, tag="TCP", log=print):
    """连接摄像头 TCP 命令/流通道并下发初始命令。

    失败每秒重试; stop_ev 置位或重试耗尽返回 None。返回的 socket 已设为
    0.05s 超时(非阻塞轮询), tag 只用于日志前缀。"""
    for _ in range(retries):
        if stop_ev is not None and stop_ev.is_set():
            return None
        try:
            c = socket.create_connection((host, port), timeout=timeout)
            c.settimeout(0.05)
            c.sendall(buf)
            if log:
                log(f"{tag} 已连 {host}:{port}, 已复现 {buf!r}")
            return c
        except OSError:
            time.sleep(1.0)
    return None
