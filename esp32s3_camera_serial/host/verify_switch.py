# -*- coding: utf-8 -*-
"""
一次性验证脚本：验证新固件动态切分辨率 + UXGA 拍照是否卡死。
发 V<t> 把预览切到 QVGA(320x240)，收几帧预览；再发 P 拍 UXGA 收高清帧存 PNG；
最后发 V<q> 回到 QQVGA 确认不失锁。
用法: python verify_switch.py COM7
"""
import sys, time, re, os
import serial
from PIL import Image

port = sys.argv[1] if len(sys.argv) > 1 else "COM7"
baud = 2000000
out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "photos")
os.makedirs(out_dir, exist_ok=True)

RES = {
    "q": (160, 120), "t": (320, 240), "v": (640, 480),
    "s": (800, 600), "x": (1024, 768), "u": (1600, 1200),
}
def convert(frame):
    n = len(frame) // 2
    rgb = bytearray(n * 3)
    j = 0
    for o in range(0, len(frame), 2):
        v = (frame[o] << 8) | frame[o + 1]
        r = (v >> 11) & 0x1F; g = (v >> 5) & 0x3F; b = v & 0x1F
        rgb[j] = (r * 255 + 15) // 31
        rgb[j + 1] = (g * 255 + 31) // 63
        rgb[j + 2] = (b * 255 + 15) // 31
        j += 3
    return bytes(rgb)

_RE = re.compile(rb'FRAME_BEGIN\nSIZE (\d+) (\d+)\n(.*?)FRAME_END\n', re.S)

def main():
    ser = serial.Serial(port, baud, timeout=0.2)
    ser.dtr = False
    ser.rts = False
    ser.reset_input_buffer()
    time.sleep(0.5)
    buf = bytearray()

    def collect(cmd, nframes, timeout_s, want=None):
        nonlocal buf
        """发送 cmd 前不重置缓冲(连续流)。收集 nframes 个非预览尺寸的完整帧，或直到 timeout。"""
        if cmd:
            ser.write(cmd); ser.flush()
        end = time.time() + timeout_s
        got = []
        while time.time() < end and len(got) < nframes:
            data = ser.read(65536)
            if data:
                buf.extend(data)
            s = bytes(buf)
            m = _RE.search(s)
            if not m:
                if len(buf) > 4 << 20:
                    del buf[: (2 << 20)]
                continue
            pos = 0
            complete = False
            while True:
                m = _RE.search(s, pos)
                if not m:
                    break
                w, h = int(m.group(1)), int(m.group(2))
                body = m.group(3)
                fb = w * h * 2
                if len(body) >= fb:
                    frame = body[:fb]
                    if want is None or ((w, h) == want):
                        got.append((w, h, frame))
                        print(f"  收到帧 {w}x{h} len={fb}", flush=True)
                    complete = True
                    end_ = m.end()
                else:
                    complete = False
                    break
                pos = m.end()
            buf = bytearray(s[pos:])
        return got

    # 1) 切预览到 QVGA，收集预览帧
    print("== 1) 切预览到 QVGA(320x240) ==", flush=True)
    got = collect(b"Vt", 3, 8.0)
    print(f"QVGA 预览帧数: {len(got)}", flush=True)

    # 2) 拍照 UXGA
    print("== 2) 拍照 UXGA(1600x1200) ==", flush=True)
    got = collect(b"P", 1, 40.0, want=(1600, 1200))
    if got:
        w, h, frame = got[0]
        Image.frombytes("RGB", (w, h), convert(frame)).save(os.path.join(out_dir, "verify_uxga.png"))
        print(f"已存 photos/verify_uxga.png {w}x{h}", flush=True)
    else:
        print("UXGA 拍照超时/未收到（可能卡死）", flush=True)

    # 3) 切回 QQVGA 预览，确认不失锁
    print("== 3) 切回 QQVGA 预览 ==", flush=True)
    got = collect(b"Vq", 3, 8.0)
    print(f"QQVGA 预览帧数: {len(got)}", flush=True)

    ser.close()
    print("验证结束", flush=True)

if __name__ == "__main__":
    main()