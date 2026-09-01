# -*- coding: utf-8 -*-
"""实时流式固件日志: 边收边追加到文件, 便于随时查看(不只在结束时输出)。"""
import serial, time, sys, io

port = sys.argv[1] if len(sys.argv) > 1 else "COM10"
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 300
out = sys.argv[3] if len(sys.argv) > 3 else "_fw_stream.log"

f = io.open(out, "w", encoding="utf-8")
s = serial.Serial(port, 2000000, timeout=0.05)
s.dtr = False
s.rts = False
s.reset_input_buffer()
t0 = time.time()
while time.time() - t0 < secs:
    try:
        d = s.read(4096)
    except Exception:
        break
    if d:
        f.write(d.decode("utf-8", "replace"))
        f.flush()
    else:
        time.sleep(0.01)
f.close()
print("done")
