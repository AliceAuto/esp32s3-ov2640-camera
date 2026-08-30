# -*- coding: utf-8 -*-
"""直接打印串口原始字节，观察固件输出。可指定运行秒数，到时自动退出释放串口。"""
import sys, time, serial

port = sys.argv[1] if len(sys.argv) > 1 else "COM3"
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
# 第三个参数为总运行秒数（默认 20s）；省略则永不退出，需手动 Ctrl+C
duration = float(sys.argv[3]) if len(sys.argv) > 3 else None
ser = serial.Serial(port, baud, timeout=1)
print(f"monitor {port}@{baud} (时长={'无限' if duration is None else '%gs' % duration}), 到时自动退出")
t0 = time.time()
t = t0
try:
    while True:
        if duration is not None and time.time() - t0 > duration:
            print("(到达指定时长，退出)", flush=True)
            break
        data = ser.read(2048)
        if data:
            t = time.time()
            txt = data.decode("utf-8", errors="replace")
            print(repr(txt), flush=True)
        elif time.time() - t > 15:
            print("(15s 无数据)", flush=True)
            t = time.time()
except KeyboardInterrupt:
    pass
finally:
    ser.close()