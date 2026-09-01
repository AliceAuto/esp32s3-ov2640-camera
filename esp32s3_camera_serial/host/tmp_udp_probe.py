# -*- coding: utf-8 -*-
"""一次性 UDP 链路探针(烧录后复测)：确认固件是否在发 UDP 流。"""
import socket, sys, time

CAM = sys.argv[1] if len(sys.argv) > 1 else "192.168.10.118"
PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 3333

udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4*1024*1024)
udp.bind(("0.0.0.0", PORT))
udp.settimeout(0.2)
print(f"UDP 已监听 :{PORT}")

try:
    tcp = socket.create_connection((CAM, PORT), timeout=4)
    tcp.settimeout(0)
    tcp.sendall(b"vtU")
    print(f"TCP 已连 {CAM}:{PORT}, 已发 'vtU'")
except Exception as e:
    print(f"TCP 连接失败: {e}"); sys.exit(1)

t0 = time.time(); n_ovu=0; n_other=0; total=0
while time.time()-t0 < 10.0:
    try:
        pkt,_ = udp.recvfrom(65535)
    except socket.timeout:
        continue
    except OSError:
        break
    total += len(pkt)
    if pkt.startswith(b"OVU"):
        n_ovu += 1
    else:
        n_other += 1

print(f"\n=== 烧录后复测 ===")
print(f"OVU 数据报: {n_ovu}   其他: {n_other}   总字节: {total}")
print("=> 固件在发 UDP 流，链路 OK" if n_ovu>0 else "=> 仍无 OVU 包")
udp.close()
try: tcp.close()
except Exception: pass