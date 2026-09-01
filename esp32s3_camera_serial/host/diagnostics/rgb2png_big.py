# -*- coding: utf-8 -*-
"""转RGB565->PNG, 可切换字节序。用来验证花屏是否字节序错位所致。
用法: python rgb2png_big.py raw.bin out.png W H
读时按 big-endian(高字节在前)，即库输出大端而原脚本按小端误读。
"""
import sys, struct
from PIL import Image

raw = open(sys.argv[1], 'rb').read()
out = sys.argv[2]
W = int(sys.argv[3]); H = int(sys.argv[4])
img = Image.new('RGB', (W, H))
px = img.load()
for y in range(H):
    for x in range(W):
        o = (y*W+x)*2
        v = struct.unpack_from('>H', raw, o)[0]  # big-endian
        r = (v >> 11) & 0x1F; g = (v >> 5) & 0x3F; b = v & 0x1F
        px[x, y] = ((r*255+15)//31, (g*255+31)//63, (b*255+15)//31)
img.save(out)
print("saved", out)