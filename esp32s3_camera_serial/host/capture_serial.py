# -*- coding: utf-8 -*-
"""
ESP32-S3 + OV2640 串口取图上位机
从串口读取 FRAME_BEGIN ... FRAME_END 之间的 JPEG 数据，保存到本地 jpg 文件。

用法：
    python capture_serial.py [串口] [波特率] [保存目录]
示例（Windows）：
    python capture_serial.py COM3 115200 shots
"""

import sys
import os
import time
import serial

BEGIN = b"FRAME_BEGIN\n"
END   = b"FRAME_END\n"

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM3"
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    outdir = sys.argv[3] if len(sys.argv) > 3 else "shots"
    os.makedirs(outdir, exist_ok=True)

    ser = serial.Serial(port, baud, timeout=1)
    print(f"已打开 {port}@{baud}，等待 FRAME_BEGIN ... (Ctrl+C 退出)")

    buf = b""
    poll_t = time.time()

    try:
        while True:
            data = ser.read(4096)
            if data:
                buf += data

                # 处理可能的多帧：循环切出完整帧
                while True:
                    i_begin = buf.find(BEGIN)
                    if i_begin == -1:
                        # 丢弃累积的无效前缀，避免越搞越长（保留尾部可能不完整的 BEGIN）
                        if len(buf) > 4 * len(BEGIN):
                            keep = buf[-len(BEGIN):]
                            for k in range(1, len(BEGIN)):
                                if BEGIN[:k] == buf[-k:]:
                                    keep = buf[-k:]
                                    break
                            buf = keep
                        break
                    j_end = buf.find(END, i_begin + len(BEGIN))
                    if j_end == -1:
                        # 收到了 BEGIN 但还没到 END，等待更多数据
                        break

                    jpeg = buf[i_begin + len(BEGIN):j_end]
                    buf = buf[j_end + len(END):]

                    if jpeg and jpeg.startswith(b"\xff\xd8"):
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