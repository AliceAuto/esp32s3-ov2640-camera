# -*- coding: utf-8 -*-
"""
连续采集固件串口输出的 RGB565 (QQVGA 160x120) 帧，转存为 PNG 图片。
每帧长 = 160*120*2 = 38400 字节。用 FRAME_BEGIN/FRAME_END 切分。
用法: python rgb_capture.py COM3 115200 outdir [maxframes]
"""
import sys, os, time, serial
from PIL import Image

W, H = 160, 120
FRAME_BYTES = W * H * 2
BEGIN = b"FRAME_BEGIN\n"
END = b"FRAME_END\n"

port = sys.argv[1] if len(sys.argv) > 1 else "COM3"
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
outdir = sys.argv[3] if len(sys.argv) > 3 else "rgb_frames"
maxf = int(sys.argv[4]) if len(sys.argv) > 4 else 30
os.makedirs(outdir, exist_ok=True)

ser = serial.Serial(port, baud, timeout=1)
print(f"采集 RGB565 {W}x{H} @{port}:{baud} -> {outdir}（{maxf}帧，Ctrl+C 退出）")

buf = bytearray()
in_frame = False
saved = 0
clean_frames = 0   # 连续成功计数，用于判断图像是否正常
t0 = time.time()
# 快速丢弃旧缓冲数据
ser.reset_input_buffer()

try:
    while saved < maxf:
        data = ser.read(2048)
        if not data:
            time.sleep(0.05)
            continue
        buf += data

        # 若不在帧中，找 BEGIN
        if not in_frame:
            i = buf.find(BEGIN)
            if i < 0:
                if len(buf) > 4096:
                    del buf[: len(buf) - 4096]
                continue
            del buf[: i + len(BEGIN)]
            in_frame = True

        # 在帧中，找 END
        j = buf.find(END)
        if j < 0:
            if len(buf) > FRAME_BYTES + 2048:
                # 数据异常超长，重置
                print("! 帧超长(%d)，放弃该帧" % len(buf))
                buf.clear()
                in_frame = False
            continue

        frame = bytes(buf[:j])
        del buf[: j + len(END)]
        in_frame = False

        if len(frame) != FRAME_BYTES:
            print("! 帧长异常 %d != %d，丢弃" % (len(frame), FRAME_BYTES))
            continue

        # RGB565 -> RGB888
        rgb = bytearray(FRAME_BYTES // 2 * 3)
        k = 0
        for o in range(0, FRAME_BYTES, 2):
            v = frame[o] | (frame[o + 1] << 8)
            r = (v >> 11) & 0x1F
            g = (v >> 5) & 0x3F
            b = v & 0x1F
            rgb[k] = (r * 255 + 15) // 31
            rgb[k + 1] = (g * 255 + 31) // 63
            rgb[k + 2] = (b * 255 + 15) // 31
            k += 3

        img = Image.frombytes("RGB", (W, H), bytes(rgb))
        # 判断是否接近纯色/非空（统计方差）
        import statistics
        px = list(img.getdata())
        # 采样判断
        vals = [px[i][0] + px[i][1] + px[i][2] for i in range(0, len(px), 97)]
        mean = sum(vals) / len(vals)
        var = sum((x - mean) ** 2 for x in vals) / len(vals)
        tag = "OK " if var > 40 else "FLAT"  # 方差小说明画面接近纯色
        if var > 40:
            clean_frames += 1

        path = os.path.join(outdir, "frame_%03d.png" % saved)
        img.save(path)
        print("[%d] %s mean=%d var=%d -> %s" % (saved, tag, mean, var, path))
        saved += 1

        # 生成一帧后读走后续空闲字节，避免累积
        ser.flush()

finally:
    ser.close()
    print("完成，共保存 %d 帧，其中图像正常 %d 帧" % (saved, clean_frames))