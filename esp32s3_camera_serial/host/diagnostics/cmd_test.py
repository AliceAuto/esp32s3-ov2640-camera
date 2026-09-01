# -*- coding: utf-8 -*-
"""临时诊断：开 COM7 -> 发命令(N 关UDP / Vt 切预览) -> 读串口看是否有帧。"""
import serial, time, sys

port = sys.argv[1] if len(sys.argv) > 1 else "COM7"
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 2000000

ser = serial.Serial(port, baud, timeout=0.2)
ser.reset_input_buffer()

def dump(tag):
    t0 = time.time()
    n = 0
    frames = 0
    while time.time() - t0 < 5:
        d = ser.read(65536)
        if d:
            n += len(d)
            frames += d.count(b'FRAME_BEGIN')
    print(f"{tag}: {n} 字节, FRAME_BEGIN x{frames}", flush=True)

dump("baseline(无命令)")
ser.write(b'N'); ser.flush(); print("已发 N(关UDP)", flush=True); time.sleep(0.5)
dump("after N")
ser.write(b'Vt'); ser.flush(); print("已发 Vt(切QVGA预览)", flush=True)
dump("after Vt")
ser.write(b'R'); ser.flush(); print("已发 R(重连摄像头)", flush=True)
dump("after R")
ser.close()
