# -*- coding: utf-8 -*-
"""诊断 RGB565 raw 是否存在花屏彩带(位错乱)。
用法: python diag_color.py raw.bin W H
检测: 字节值分布、位翻转规律、行内高频突变。
"""
import sys, struct
from collections import Counter

raw = open(sys.argv[1], 'rb').read()
W = int(sys.argv[2]); H = int(sys.argv[3])
px = W * H
expected = px * 2
print(f"文件: {len(raw)} 字节, 应为 {expected} ({W}x{H}x2)")

# 取中间一段保证有效
n = min(px, len(raw)//2)
arr = raw[len(raw)//2 - n : len(raw)//2 + n]

# 1. 字节值出现计数
r5 = arr[::2]; g6b5 = arr[1::2]
cnt_lo = Counter(r5).most_common(3)
cnt_hi = Counter(g6b5).most_common(3)
print("低字节(低8位)高频值:", cnt_lo)
print("高字节(高8位)高频值:", cnt_hi)

# 2. 像素错乱：相邻像素 RGB 突变
bad = 0; total = 0
sample = n // 2  # px数
for i in range(0, sample - 1, 100):
    v1 = struct.unpack_from('<H', arr, i*2)[0]
    v2 = struct.unpack_from('<H', arr, (i+1)*2)[0]
    if abs(v1 - v2) > 2000:
        bad += 1
    total += 1
print(f"相邻像素大跳变比例: {bad}/{total}")

# 3. 检查是否为纯色/彩条: 每行主色
from collections import defaultdict
row_main = []
steps = 120
for r in range(0, sample, sample//steps):
    row = arr[r*2:(r+sample//steps)*2]
    c = Counter(row).most_common(1)
    row_main.append(c)
print("抽样行高频值:", row_main[:12])