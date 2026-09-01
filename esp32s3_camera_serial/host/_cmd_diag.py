# -*- coding: utf-8 -*-
"""组合诊断: 读运行日志 -> 串口发 U -> 看固件是否打印 'UDP 预览模式开启'。"""
import serial, time

port = "COM10"
BAUD = 2000000
ser = serial.Serial(port, BAUD, timeout=0.3)
ser.reset_input_buffer()

def drain(sec):
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < sec:
        d = ser.read(65536)
        if d:
            buf += d
    return buf.decode("utf-8", errors="replace")

print("=== 基线 3s(看 loop alive / WiFi IP)===")
print(drain(3.0))

print("=== 发 'U' 触发 UDP 模式 ===")
ser.write(b'U'); ser.flush()
time.sleep(0.3)
print(drain(3.0))

print("=== 发 'N' 切回 TCP ===")
ser.write(b'N'); ser.flush()
time.sleep(0.3)
print(drain(2.0))
ser.close()
