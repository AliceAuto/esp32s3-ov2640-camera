# -*- coding: utf-8 -*-
"""分析花屏彩带的空间结构: 判断逐行错位/周期条带/行内规律。
用法: python diag_structure.py raw.bin W H
"""
import sys, struct
from collections import Counter

raw = open(sys.argv[1], 'rb').read()
W = int(sys.argv[2]); H = int(sys.argv[3])

def px(r, c):
    o = (r * W + c) * 2
    if o + 1 >= len(raw):
        return None
    return struct.unpack_from('<H', raw, o)[0]

# 1. 一段时间内 行内 是否存在周期条带: 取第10行的相邻列差值
print("== 行内结构(第10行, 前20像素) ==")
for c in range(20):
    v = px(10, c)
    print(f"({v:#06x} r{(v>>11)&31} g{(v>>5)&63} b{v&31})", end=" ")
print()

# 2. 整帧的 列间大跳变 统计, 判断是否有垂直条带
col_bad = Counter()
for r in range(0, H, 20):
    prev = px(r, 0)
    for c in range(1, W):
        v = px(r, c)
        if prev is not None and v is not None and abs(v - prev) > 1000:
            col_bad[c % 8] += 1
        prev = v
print("== 相邻列大跳变按列位置(c%8)分布 ==", dict(col_bad))

# 3. 行与行之间的错位: 比较第10行与第11行同位置
print("== 行间同列大跳变比例(高=行错位/图像噪声) ==")
bad = tot = 0
for r in range(10, min(H-1, 200), 1):
    for c in range(0, W, 3):
        v1 = px(r, c); v2 = px(r+1, c)
        if v1 is not None and v2 is not None:
            if abs(v1-v2) > 1000: bad += 1
            tot += 1
print(f"  {bad}/{tot} = {bad/tot*100:.1f}%")

# 4. 是否整帧几乎同一个色(单色花屏)
vals = Counter()
for r in range(0, H, 2):
    for c in range(0, W, 2):
        v = px(r, c)
        if v is not None: vals[v] += 1
print("== 抽样像素高频值 ==", vals.most_common(5))