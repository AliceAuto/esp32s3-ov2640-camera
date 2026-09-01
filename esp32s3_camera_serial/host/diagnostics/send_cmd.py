# -*- coding: utf-8 -*-
"""临时诊断：开串口(释放EN) -> 发 P(拍照抓帧) -> 读串口看是否收到 JPEG 帧。"""
import serial, time, sys, os

port = sys.argv[1] if len(sys.argv) > 1 else "COM7"
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 2000000

ser = serial.Serial(port, baud, timeout=0.2)
ser.dtr = False
ser.rts = False
ser.reset_input_buffer()

def read_bytes(dur):
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < dur:
        d = ser.read(65536)
        if d:
            buf += d
    return buf

read_bytes(1.0)
cmd = sys.argv[3].encode() if len(sys.argv) > 3 else b'P'
ser.write(cmd); ser.flush()
print(f"已发 {cmd!r}", flush=True)
out = read_bytes(float(sys.argv[4]) if len(sys.argv) > 4 else 6.0)
begin = out.count(b"FRAME_BEGIN")
end = out.count(b"FRAME_END")
print(f"收到 {len(out)} 字节 | FRAME_BEGIN x{begin} | FRAME_END x{end}", flush=True)
p = os.path.join(os.path.dirname(os.path.abspath(__file__)), "photo_out.txt")
with open(p, "wb") as f:
    f.write(out)
print("saved", len(out), "->", p)
ser.close()
