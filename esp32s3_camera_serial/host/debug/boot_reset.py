# -*- coding: utf-8 -*-
"""临时诊断：用 DTR/RTS 时序触发 ESP32 复位，读取启动日志。"""
import serial, time, sys, os

port = sys.argv[1] if len(sys.argv) > 1 else "COM7"
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 2000000

ser = serial.Serial(port, baud, timeout=0.2)

# ESP32 自动复位时序（esptool 风格）：EN 拉低复位 -> 释放
ser.setDTR(False); ser.setRTS(True); time.sleep(0.1)   # EN=1, IO0=0
ser.setDTR(True);  ser.setRTS(False); time.sleep(0.1)  # EN=0 (复位), IO0=1
ser.setDTR(False); ser.setRTS(False); time.sleep(0.1)  # EN=1 (释放), 正常运行
ser.reset_input_buffer()

buf = bytearray()
t0 = time.time()
while time.time() - t0 < 6:
    d = ser.read(65536)
    if d:
        buf += d
p = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dump_boot.txt")
with open(p, "wb") as f:
    f.write(buf)
print("boot log %d bytes -> %s" % (len(buf), p))
ser.close()
