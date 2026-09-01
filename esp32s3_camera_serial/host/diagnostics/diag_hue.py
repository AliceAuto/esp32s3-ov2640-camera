# -*- coding: utf-8 -*-
"""诊断假色块: 整帧色相分布 + 行错位扫描。
互补色成对(紫粉<->绿, 橙<->蓝) = bit/字节错乱。
用法: python diag_hue.py raw.bin W H
"""
import sys, struct

raw = open(sys.argv[1], 'rb').read()
W = int(sys.argv[2]); H = int(sys.argv[3])

def to_rgb(v):
    return ((v>>11)&31, (v>>5)&63, v&31)

# 1. 色相分布: 判断是否充斥互补假色
# 简化: 统计 R>>G, G>>B, B>>R 占优的像素
n = 0
rG=bR=gB = 0  # 各通道占优计数
samples = []
step = max(1, W*H//8000)
for i in range(0, W*H, step):
    o = i*2
    if o+1 >= len(raw): break
    v = struct.unpack_from('<H', raw, o)[0]
    r,g,b = to_rgb(v)
    if r>g and r>b: rG+=1
    elif g>r and g>b: gB+=1
    elif b>r and b>g: bR+=1
    n+=1
tot = rG+gB+bR
print(f"通道占优分布(共{n}采样): R占优 {rG}, G占优 {gB}, B占优 {bR}")
print(f"占比: R{rG/tot*100:.0f}% G{gB/tot*100:.0f}% B{bR/tot*100:.0f}%")

# 2. 互补色对检测: 品红(高R高B低G)与绿(高G) 是否大量共存
mg = yl = grn = blu = 0
for i in range(0, W*H, step):
    o = i*2
    if o+1 >= len(raw): break
    v = struct.unpack_from('<H', raw, o)[0]
    r,g,b = to_rgb(v)
    if r>24 and b>24 and g<16: mg+=1     # 品红
    elif r>24 and g>24 and b<16: yl+=1   # 黄/橙
    elif g>40 and r<20 and b<20: grn+=1  # 纯绿
    elif b>24 and r<16 and g<16: blu+=1  # 纯蓝
print(f"品红{mg/n*100:.1f}% 黄/橙{yl/n*100:.1f}% 纯绿{grn/n*100:.1f}% 纯蓝{blu/n*100:.1f}%")

# 3. 行错位扫描: 尝试水平偏移 k 像素, 看能否让行内平滑
best = (0, None)
row = W//2  # 某一行
try:
    base_raw = raw[:W*2]  # 第0行像素(取整行)
    vals = [struct.unpack_from('<H', base_raw, c*2)[0] for c in range(W)]
    for k in range(0, 4):
        # 对比第k偏移处的相邻平滑度
        jumps = 0
        prev = vals[k]
        for c in range(k+1, W):
            v = vals[c]
            if abs(v-prev) > 800: jumps+=1
            prev = v
        print(f"  偏移{k}: 行内大跳变 {jumps}/{W}")
except Exception as e:
    print("scan err", e)