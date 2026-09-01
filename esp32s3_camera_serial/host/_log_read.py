# -*- coding: utf-8 -*-
"""一次性: 读固件 COM10 日志, 发 U 触发, 查看是否采帧/收命令。"""
import serial, time, sys

port = sys.argv[1] if len(sys.argv) > 1 else "COM10"
BAUD = 2000000
try:
    ser = serial.Serial(port, BAUD, timeout=0.3)
    ser.reset_input_buffer()
except Exception as e:
    print("open fail:", e); sys.exit(1)

try:
    ser.write(b'U'); ser.flush()
except Exception:
    pass

buf = bytearray()
t0 = time.time()
try:
    while time.time() - t0 < 6.0:
        d = ser.read(65536)
        if d:
            buf += d
except Exception as e:
    print("read err:", e)

text = buf.decode("utf-8", errors="replace")
print("captured %d bytes" % len(buf))
print("--- tail 2000 chars ---")
print(text[-2000:])
ser.close()
