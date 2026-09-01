# -*- coding: utf-8 -*-
"""临时诊断: 监听摄像头固件串口(COM10, 2M) 40s, 打印固件日志(重点 [P2P]/[中继]/TURN)。"""
import serial, time, sys

port = sys.argv[1] if len(sys.argv) > 1 else "COM10"
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 40
s = serial.Serial(port, 2000000, timeout=0.05)
s.dtr = False
s.rts = False
s.reset_input_buffer()
t0 = time.time()
out = []
while time.time() - t0 < secs:
    d = s.read(4096)
    if d:
        out.append(d.decode("utf-8", "replace"))
    else:
        time.sleep(0.01)
print("".join(out)[-8000:])
