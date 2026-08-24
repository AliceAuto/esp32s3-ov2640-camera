# -*- coding: utf-8 -*-
"""验证 live_capture 的解析+convert 逻辑是否产出黑图。"""
import serial, time
BEGIN = b"FRAME_BEGIN\n"; END = b"FRAME_END\n"; SIZE = b"SIZE 160 120\n"
ser = serial.Serial("COM7", 2000000, timeout=1)
ser.reset_input_buffer()
buf = bytearray()
got = False
t0 = time.time()
while time.time() - t0 < 6 and not got:
    d = ser.read(4096)
    if not d: continue
    buf += d
    # 找帧头+SIZE
    i = buf.find(BEGIN + SIZE)
    if i < 0:
        if len(buf) > 65536: del buf[:len(buf)-32768]
        continue
    # 跳过帧头, 等 END
    head = i + len(BEGIN) + len(SIZE)
    j = buf.find(END, head)
    if j < 0:
        continue
    frame = bytes(buf[head:j])
    del buf[: j + len(END)]
    if len(frame) != 160*120*2:
        print("帧长异常", len(frame)); break
    # convert (big-endian, 与 live_capture 一致)
    rgb = bytearray(160*120*3)
    k = 0
    for o in range(0, len(frame), 2):
        v = (frame[o] << 8) | frame[o+1]
        r=(v>>11)&31; g=(v>>5)&63; b=v&31
        rgb[k]=(r*255+15)//31; rgb[k+1]=(g*255+31)//63; rgb[k+2]=(b*255+15)//31
        k+=3
    mx = max(rgb); mn = min(rgb); mean = sum(rgb)//len(rgb)
    print(f"帧长 {len(frame)}  max={mx} min={mn} mean={mean}")
    # 统计非零像素占比
    nz = sum(1 for x in rgb if x>20)/len(rgb)
    print(f"非黑像素占比 {nz*100:.1f}%")
    got = True
ser.close()
if not got:
    print("未捕获到完整帧")