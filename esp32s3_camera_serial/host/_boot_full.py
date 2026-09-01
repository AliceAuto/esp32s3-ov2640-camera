# -*- coding: utf-8 -*-
"""一次性: 复位板子, 抓完整启动日志(含开头), 确认摄像头/WiFi/帧池。"""
import serial, time

port = "COM10"
BAUD = 2000000
ser = serial.Serial(port, BAUD, timeout=0.2)
ser.setDTR(False); ser.setRTS(True); time.sleep(0.15)
ser.setDTR(True);  ser.setRTS(False); time.sleep(0.05)
ser.setDTR(False); ser.setRTS(False)
ser.reset_input_buffer()

buf = bytearray()
t0 = time.time()
while time.time() - t0 < 6.0:
    d = ser.read(65536)
    if d:
        buf += d
text = buf.decode("utf-8", errors="replace")
print("captured %d bytes" % len(buf))
print("=== FULL (first 4000) ===")
print(text[:4000])
ser.close()
