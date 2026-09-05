# -*- coding: utf-8 -*-
"""
ESP32-S3 + OV2640 串口取图上位机
从串口读取 FRAME_BEGIN ... FRAME_END 之间的 JPEG 数据，保存到本地 jpg 文件。
切帧复用 lib/frame_parser（与预览/帧率测试同一份实现）。

用法：
    python capture_serial.py [串口] [波特率] [保存目录]
示例（Windows）：
    python capture_serial.py COM3 115200 shots
"""
import os
import sys
import time

# 使 tools/ 下可 import host/lib 公共库(与主程序同一份帧解析实现)
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "host"))

import serial

from lib import frame_parser


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM3"
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    outdir = sys.argv[3] if len(sys.argv) > 3 else "shots"
    os.makedirs(outdir, exist_ok=True)

    ser = serial.Serial(port, baud, timeout=1)
    print(f"已打开 {port}@{baud}，等待 FRAME_BEGIN ... (Ctrl+C 退出)")

    buf = bytearray()
    poll_t = time.time()

    try:
        while True:
            data = ser.read(4096)
            if data:
                buf += data
                frames, consumed = frame_parser.split_frames(buf)
                del buf[:consumed]
                for w, h, fmt, jpeg in frames:
                    if fmt == "JPG" and len(jpeg) > 100 and jpeg[:2] == b"\xff\xd8":
                        name = os.path.join(
                            outdir,
                            f"shot_{time.strftime('%Y%m%d_%H%M%S')}_{len(jpeg)}.jpg",
                        )
                        with open(name, "wb") as f:
                            f.write(jpeg)
                        print(f"已保存 {name}  ({len(jpeg)} 字节)")
                    else:
                        print(f"收到看似无效帧, len={len(jpeg)}，已忽略")
            else:
                # 一段时间无数据，做个保活提示
                if time.time() - poll_t > 5:
                    print("(等待数据中，请确认开发板已上电且串口正常)...")
                    poll_t = time.time()
    except KeyboardInterrupt:
        print("\n已退出")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
