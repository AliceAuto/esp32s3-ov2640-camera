# -*- coding: utf-8 -*-
"""轮询 P 连续收帧测试（非GUI），验证能否连续稳定出帧。"""
import serial, time, sys
port = sys.argv[1] if len(sys.argv) > 1 else "COM7"
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 2000000
dur = float(sys.argv[3]) if len(sys.argv) > 3 else 6.0

ser = serial.Serial(port, baud, timeout=0.1)
ser.dtr = False; ser.rts = False
ser.reset_input_buffer()
ser.write(b"Ht"); ser.flush()      # 拍照分辨率 QVGA
time.sleep(0.3)
ser.write(b"P"); ser.flush()

BEGIN = b"FRAME_BEGIN\n"; END = b"FRAME_END\n"
buf = bytearray()
t0 = time.time(); frames = 0
last_cmd = time.time()
rx_total = 0
while time.time() - t0 < dur:
    if time.time() - last_cmd > 1.5 and not buf:
        ser.write(b"P"); ser.flush(); last_cmd = time.time()
        print(f"[{time.time()-t0:5.1f}s] 补发P (buf={len(buf)})", flush=True)
    d = ser.read(65536)
    if d:
        rx_total += len(d)
        buf += d
        if len(buf) < 60000:
            print(f"[{time.time()-t0:5.1f}s] 读到{len(d)}B 累计{rx_total}B buf={len(buf)} head={buf[:30]!r}", flush=True)
    # 提取所有完整帧
    while True:
        i = buf.find(BEGIN)
        if i < 0:
            if len(buf) > 1 << 20: del buf[:len(buf)-4096]
            break
        del buf[:i+len(BEGIN)]
        nl = buf.find(b"\n")
        if nl < 0: break
        size_line = bytes(buf[:nl]); del buf[:nl+1]
        m = None
        if size_line.startswith(b"SIZE "):
            p = size_line.split()
            if len(p) >= 4:
                m = (int(p[1]), int(p[2]), p[3].decode('ascii','ignore'))
        j = buf.find(END)
        if j < 0: break
        body = bytes(buf[:j]); del buf[:j+len(END)]
        frames += 1
        jpg_ok = body[:2] == b'\xff\xd8'
        print(f"[{time.time()-t0:5.1f}s] 帧{frames}  {m if m else '未知'}  len={len(body)}  jpg={'OK' if jpg_ok else 'BAD'}", flush=True)
        ser.write(b"P"); ser.flush(); last_cmd = time.time()
print(f"===== 总帧 {frames} =====")
ser.close()
