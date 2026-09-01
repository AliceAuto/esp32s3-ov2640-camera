# -*- coding: utf-8 -*-
"""一次性：追踪 TCP 连接稳定性，检测固件是否在重启循环(连接周期性断开)。"""
import socket, time, sys

CAM = sys.argv[1] if len(sys.argv) > 1 else "192.168.10.118"
PORT = 3333
DUR = 30

time.sleep(2)   # 等固件稳定
conn_events = []
t0 = time.time()
while time.time() - t0 < DUR:
    try:
        s = socket.create_connection((CAM, PORT), timeout=3)
        t_conn = time.time() - t0
        s.settimeout(0.1)
        got = b""
        t_first = None
        while time.time() - t0 < DUR:
            try:
                d = s.recv(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            if d:
                if t_first is None: t_first = time.time() - t0
                got += d
                if b"DIAG:" in got:
                    pass
        s.close()
        t_break = time.time() - t0
        conn_events.append((t_conn, t_break, len(got), b"DIAG:" in got))
        print(f"连接: {t_conn:.1f}s 建立 -> {t_break:.1f}s 断开, 收到{len(got)}B, 含DIAG={b'DIAG:' in got}", flush=True)
    except Exception as e:
        # 连接失败, 等待固件回来
        print(f"连接失败: {e}, t={time.time()-t0:.1f}s", flush=True)
        time.sleep(1)

print(f"\n=== {len(conn_events)} 次连接事件 ===")
if len(conn_events) > 1:
    print("→ 固件疑似在重启循环(连接被周期性重置)")
else:
    print("→ 连接稳定(单次), 非重启循环")