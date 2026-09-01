# -*- coding: utf-8 -*-
"""临时诊断: 向板子串口发 'r' 触发 reinit, 抓取启动/CLKRC 日志。"""
import sys, time, serial

port = sys.argv[1] if len(sys.argv) > 1 else "COM7"
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 2000000
cmd = (sys.argv[3] if len(sys.argv) > 3 else "r").encode()
s = serial.Serial(port, baud, timeout=1)
time.sleep(0.5)
s.write(cmd)
t0 = time.time()
while time.time() - t0 < 6:
    d = s.read(s.in_waiting or 1)
    if d:
        print(d.decode("utf-8", errors="replace"), end="", flush=True)
s.close()
