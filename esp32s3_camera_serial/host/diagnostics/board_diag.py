# -*- coding: utf-8 -*-
"""板内诊断: 发'd'给固件, 读回 DIAG 统计行。
用法: python board_diag.py COM7 [baud]
"""
import sys, serial, time

port = sys.argv[1] if len(sys.argv) > 1 else "COM7"
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 2000000
ser = serial.Serial(port, baud, timeout=0.5)
time.sleep(0.3)
# 清空残留
ser.reset_input_buffer()
# 发多个'd'确保命中
for _ in range(4):
    ser.write(b'd')
    time.sleep(0.3)
# 收集到DIAG行
deadline = time.time() + 6
buf = b""
while time.time() < deadline and b"DIAG" not in buf[-200:]:
    chunk = ser.read(1024)
    if chunk:
        buf += chunk
# 打印含 DIAG 的行
for line in buf.split(b"\n"):
    if b"DIAG" in line:
        print(line.decode('utf-8', 'replace'))
ser.close()