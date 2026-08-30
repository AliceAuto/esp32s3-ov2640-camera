# -*- coding: utf-8 -*-
"""临时诊断：N(关UDP) + Vq(切QQVGA) + 读串口输出，判断摄像头状态。"""
import serial, time, sys, os

port = sys.argv[1] if len(sys.argv) > 1 else "COM7"
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 2000000

ser = serial.Serial(port, baud, timeout=0.2)
ser.reset_input_buffer()

def read_text(dur):
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < dur:
        d = ser.read(65536)
        if d:
            buf += d
    return buf

read_text(1.0)
ser.write(b'N'); ser.flush()
ser.write(b'Vq'); ser.flush()
time.sleep(0.3)
out = read_text(6.0)
p = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dump_out.txt")
with open(p, "wb") as f:
    f.write(out)
print("saved %d bytes" % len(out))
ser.close()
