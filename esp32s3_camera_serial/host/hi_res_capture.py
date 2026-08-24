# -*- coding: utf-8 -*-
"""高分辨率单帧拍照: 从串口接收一张 RGB565 帧转存 PNG。
用法: python hi_res_capture.py COM7 2000000 1600 1200 out.png [放大倍数]
"""
import sys, time, serial
from PIL import Image

port = sys.argv[1] if len(sys.argv) > 1 else "COM7"
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 2000000
W = int(sys.argv[3]) if len(sys.argv) > 3 else 1600
H = int(sys.argv[4]) if len(sys.argv) > 4 else 1200
out = sys.argv[5] if len(sys.argv) > 5 else "photo.png"
scale = int(sys.argv[6]) if len(sys.argv) > 6 else 1

BEGIN = b"FRAME_BEGIN\n"
END = b"FRAME_END\n"
FRAME_BYTES = W * H * 2

ser = serial.Serial(port, baud, timeout=2)
ser.reset_input_buffer()
print(f"拍照 {W}x{H} @{port}:{baud} -> {out} (帧长 {FRAME_BYTES} 字节, 约{FRAME_BYTES/(baud/10):.0f}s)")

buf = bytearray()
in_frame = False
t0 = time.time()
ok = False
try:
    while not ok:
        data = ser.read(16384)
        if not data:
            continue
        buf += data
        if not in_frame:
            i = buf.find(BEGIN)
            if i < 0:
                if len(buf) > 16384:
                    del buf[: len(buf) - 16384]
                continue
            del buf[: i + len(BEGIN)]
            in_frame = True
            t0 = time.time()
            print(f"帧头对齐, 接收数据中...")
        j = buf.find(END)
        if j < 0:
            if len(buf) > FRAME_BYTES + 8192:
                print(f"! 帧超长({len(buf)})，丢弃重试")
                buf.clear(); in_frame = False
            continue
        frame = bytes(buf[:j])
        in_frame = False
        if len(frame) != FRAME_BYTES:
            print(f"! 帧长异常 {len(frame)} != {FRAME_BYTES}, 重试")
            buf = bytearray(buf[j + len(END):])
            continue
        # RGB565 -> RGB888
        rgb = bytearray(FRAME_BYTES // 2 * 3)
        k = 0
        for o in range(0, FRAME_BYTES, 2):
            v = frame[o] | (frame[o + 1] << 8)
            rgb[k] = ((v >> 11) & 0x1F) * 255 // 31
            rgb[k + 1] = ((v >> 5) & 0x3F) * 255 // 63
            rgb[k + 2] = (v & 0x1F) * 255 // 31
            k += 3
        img = Image.frombytes("RGB", (W, H), bytes(rgb))
        if scale > 1:
            img = img.resize((W * scale, H * scale), Image.LANCZOS)
        img.save(out)
        print(f"已保存 {W}x{H} -> {out} (耗时 {time.time()-t0:.1f}s)")
        ok = True
finally:
    ser.close()