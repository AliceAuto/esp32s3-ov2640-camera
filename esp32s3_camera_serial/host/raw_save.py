# -*- coding: utf-8 -*-
"""抓取一帧原始 RGB565 字节到文件。
用法: python raw_save.py COM7 2000000 out.bin [W] [H]
"""
import sys, serial, time

port = sys.argv[1] if len(sys.argv) > 1 else "COM7"
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 2000000
out = sys.argv[3] if len(sys.argv) > 3 else "raw.bin"
W = int(sys.argv[4]) if len(sys.argv) > 4 else 160
H = int(sys.argv[5]) if len(sys.argv) > 5 else 120
BEGIN = b"FRAME_BEGIN\n"
END = b"FRAME_END\n"

FRAME_BYTES = W * H * 2

ser = serial.Serial(port, baud, timeout=1)
ser.reset_input_buffer()
buf = bytearray()
in_frame = False
while True:
    d = ser.read(4096)
    if not d:
        time.sleep(0.02)
        continue
    buf += d
    if not in_frame:
        i = buf.find(BEGIN)
        if i < 0:
            if len(buf) > 8192:
                del buf[: len(buf) - 8192]
            continue
        del buf[: i + len(BEGIN)]
        in_frame = True
    j = buf.find(END)
    if j < 0:
        if len(buf) > FRAME_BYTES + 4096:
            buf.clear(); in_frame = False
        continue
    frame = bytes(buf[:j])
    del buf[: j + len(END)]
    in_frame = False
    if len(frame) == FRAME_BYTES:
        open(out, "wb").write(frame)
        print(f"已保存 {len(frame)} 字节 -> {out}")
        break
    else:
        print(f"帧长异常 {len(frame)}，重试")
ser.close()