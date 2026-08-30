# -*- coding: utf-8 -*-
"""打印第0,1,2,3行的开头像素与行号，检查行错位/卷绕。
花屏彩带常表现为: 每行像素被循环移位或重复previous行。
"""
import sys, struct
raw = open(sys.argv[1], 'rb').read()
W = int(sys.argv[2])
for r in range(0, min(4, int(sys.argv[3]) if len(sys.argv)>3 else 4)):
    o = r*W*2
    row = [struct.unpack_from('<H', raw, o+c*2)[0] for c in range(W)]
    first = " ".join(f"{v:04x}" for v in row[:16])
    last  = " ".join(f"{v:04x}" for v in row[-16:])
    print(f"行{r:02d} 首: {first}")
    print(f"    尾: {last}")