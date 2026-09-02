# -*- coding: utf-8 -*-
"""读板子串口启动/状态日志(2M baud)。用法: python _serial_dump.py [port] [sec]"""
import serial, time, sys

port = sys.argv[1] if len(sys.argv) > 1 else "COM10"
sec  = float(sys.argv[2]) if len(sys.argv) > 2 else 8
s = serial.Serial(port, 2000000, timeout=0.3)
s.dtr = False; s.rts = False
s.reset_input_buffer()
t0 = time.time()
while time.time() - t0 < sec:
    d = s.read(65536)
    if d:
        txt = d.decode("utf-8", "replace")
        print(txt, end="", flush=True)
s.close()
print("\n[dump-done]", flush=True)