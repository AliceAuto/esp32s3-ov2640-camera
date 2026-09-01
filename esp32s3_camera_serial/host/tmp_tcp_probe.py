# -*- coding: utf-8 -*-
"""TCP 模式探针: 连固件, 发 'vtN'(关UDP走TCP), 读 TCP 流统计 FRAME_BEGIN。
结论: 收到帧 -> 采帧正常, 问题在 UDP 通道; 0 帧 -> 采帧/发送链路上游挂了。"""
import socket, sys, time

CAM = sys.argv[1] if len(sys.argv) > 1 else "192.168.10.118"
PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 3333

tcp = socket.create_connection((CAM, PORT), timeout=4)
tcp.settimeout(2.0)
tcp.sendall(b"vtN")          # v=预览分辨率(t), N=关UDP走TCP帧流
print(f"TCP 已连 {CAM}:{PORT}, 已发 'vtN'")

BEGIN = b"FRAME_BEGIN\n"
buf = b""
t0 = time.time()
n_frames = 0
total = 0
while time.time() - t0 < 8.0:
    try:
        d = tcp.recv(65536)
    except socket.timeout:
        continue
    except OSError:
        break
    if not d:
        break
    buf += d
    total += len(d)
    n_frames += buf.count(BEGIN)
    # 只保留尾部(避免无限增长)
    if len(buf) > 65536:
        buf = buf[-4096:]

print(f"\n=== TCP 模式结果 ===")
print(f"收到字节   : {total}")
print(f"FRAME_BEGIN : {n_frames}")
if n_frames > 0:
    print("=> 采帧正常, 问题在 UDP 通道(send_frame_udp/UDP目标/防火墙)。")
else:
    print("=> 8s 内无 TCP 帧, 采帧或发送链路有问题(cam_task/fb_get/sender)。")
tcp.close()
