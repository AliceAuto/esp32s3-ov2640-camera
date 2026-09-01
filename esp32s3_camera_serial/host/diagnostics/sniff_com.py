# -*- coding: utf-8 -*-
"""临时诊断：打开 COM7 读 5 秒原始字节，统计字节流并打印可读文本。"""
import serial, time, sys

port = sys.argv[1] if len(sys.argv) > 1 else "COM7"
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 2000000
dur = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0

try:
    ser = serial.Serial(port, baud, timeout=0.2)
    ser.reset_input_buffer()
except Exception as e:
    print(f"打开失败: {e}")
    sys.exit(1)

print(f"已打开 {port} @ {baud}, 监听 {dur}s ...")
t0 = time.time()
total = 0
chunks = []
textbuf = bytearray()
try:
    while time.time() - t0 < dur:
        data = ser.read(4096)
        if data:
            total += len(data)
            chunks.append(data)
            textbuf += data
            if len(textbuf) > 4000:
                textbuf = textbuf[-4000:]
finally:
    ser.close()

print(f"共收到 {total} 字节")
if chunks:
    # 显示可读文本片段（ASCII）
    printable = []
    cur = bytearray()
    for b in textbuf:
        if 32 <= b < 127 or b in (10, 13):
            cur.append(b)
        else:
            if len(cur) >= 4:
                printable.append(cur.decode('ascii', 'replace'))
            cur = bytearray()
    if len(cur) >= 4:
        printable.append(cur.decode('ascii', 'replace'))
    print("--- 可读文本片段 ---")
    for s in printable[:20]:
        print(repr(s))
    # 首块 hex
    first = chunks[0][:64]
    print("--- 首块 hex ---")
    print(first.hex(' '))
