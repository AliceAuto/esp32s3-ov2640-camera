# -*- coding: utf-8 -*-
"""按 DVP 数据位(线)分析错乱率: 判断是某根线恒出错 还是整体抖动。
RGB565 数据由高位线 D7..D0 组成。检查高位字节每个 bit 的翻转与卡死。
用法: python diag_gpio.py raw.bin W H
"""
import sys
from collections import Counter

raw = open(sys.argv[1], 'rb').read()
W = int(sys.argv[2]); H = int(sys.argv[3])
px = W * H
n = min(px, len(raw)//2)
byts = raw[:n*2]
print(f"共分析 {len(byts)} 字节")
for bit in range(8):
    ones = sum((b >> bit) & 1 for b in byts)
    print(f"BIT{bit}: {ones}/{len(byts)} = {ones/len(byts)*100:.1f}% 为1")