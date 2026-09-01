# -*- coding: utf-8 -*-
"""临时诊断：串口统计 FRAME_BEGIN/END 与总字节，判断板子是否回退串口发帧。"""
import serial, time, sys

port = sys.argv[1] if len(sys.argv) > 1 else "COM7"
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 2000000
dur = float(sys.argv[3]) if len(sys.argv) > 3 else 8.0

ser = serial.Serial(port, baud, timeout=0.2)
ser.dtr = False   # 释放 EN，让 ESP32 真正运行（避免被按住复位）
ser.rts = False
ser.reset_input_buffer()
t0 = time.time()
buf = bytearray()
begin = end = 0
while time.time() - t0 < dur:
    d = ser.read(65536)
    if d:
        buf += d
        begin += d.count(b"FRAME_BEGIN")
        end += d.count(b"FRAME_END")
print(f"总 {len(buf)} 字节 | FRAME_BEGIN x{begin} | FRAME_END x{end}")
# 打印尺寸头行（若有帧）
i = buf.find(b"FRAME_BEGIN\n")
if i >= 0:
    seg = buf[i:i+60]
    print("帧头附近:", seg)
else:
    print("无帧（仅调试日志）")
import os
p = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sniff_out.txt")
with open(p, "wb") as f:
    f.write(buf)
print("saved", len(buf), "->", p)
ser.close()
