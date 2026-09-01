# -*- coding: utf-8 -*-
"""诊断: 串口直接下发 Vu 切换分辨率, 抓取固件日志看命令路径与切换结果。"""
import serial, time, sys, io

port = "COM10"
s = serial.Serial(port, 2000000, timeout=0.05)
s.dtr = False
s.rts = False
s.reset_input_buffer()
t0 = time.time()

def log_until(secs, tag):
    out = []
    end = time.time() + secs
    while time.time() < end:
        d = s.read(4096)
        if d:
            out.append(d.decode("utf-8", "replace"))
        else:
            time.sleep(0.005)
    txt = "".join(out)
    print(f"--- [{tag}] ---", flush=True)
    print(txt[-3000:], flush=True)
    return txt

log_until(16, "启动+注册(16s)")
print(">>> 下发 Vu", flush=True)
s.write(b"Vu")
s.flush()
log_until(5, "Vu 后(5s)")
print(">>> 下发 Vt", flush=True)
s.write(b"Vt")
s.flush()
log_until(5, "Vt 后(5s)")
s.close()
