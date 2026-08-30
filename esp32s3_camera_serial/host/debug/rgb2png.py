# -*- coding: utf-8 -*-
"""RGB565 raw -> PNG。
用法: python rgb2png.py raw.bin out.png W H
"""
import sys
from PIL import Image

raw = open(sys.argv[1], 'rb').read()
out = sys.argv[2]
W = int(sys.argv[3]); H = int(sys.argv[4])
n = W * H
assert len(raw) >= n * 2, "数据不足"

img = Image.new('RGB', (W, H))
pxs = img.load()
for r in range(H):
    base = r * W * 2
    for c in range(W):
        o = base + c * 2
        v = raw[o] | (raw[o+1] << 8)
        r5 = (v >> 11) & 0x1F
        g6 = (v >> 5) & 0x3F
        b5 = v & 0x1F
        pxs[c, r] = ((r5*255+15)//31, (g6*255+31)//63, (b5*255+15)//31)
img.save(out)
print(f"saved {out} {W}x{H}")