# -*- coding: utf-8 -*-
"""一次性: 复位板子并读启动日志(2M 波特率), 确认固件版本/采帧/IP。"""
import serial, time

port = "COM10"
BAUD = 2000000
ser = serial.Serial(port, BAUD, timeout=0.3)
# 触发复位 (ESP32 烧录口 EN 由 DTR 控制, 一般 DTR/RTS 复位)
ser.setDTR(False); ser.setRTS(True); time.sleep(0.1)
ser.setDTR(True);  ser.setRTS(False); time.sleep(0.05)
ser.setDTR(False); ser.setRTS(False)
ser.reset_input_buffer()

buf = bytearray()
t0 = time.time()
while time.time() - t0 < 8.0:
    d = ser.read(65536)
    if d:
        buf += d
text = buf.decode("utf-8", errors="replace")
print("captured %d bytes" % len(buf))
print("--- tail 2500 chars ---")
print(text[-2500:])
ser.close()
