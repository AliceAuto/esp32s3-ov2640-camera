# -*- coding: utf-8 -*-
"""深度诊断花屏根因:
1) 位分布: 检测 DVP 数据线是否卡死(特定位恒0/恒1)
2) 行自相关+水平偏移: 检测行错位(每行相对上一行平移k像素则k处相关度最高)
3) 行间结构: 检测是否条纹来自行重复/行翻转
用法: python diag_stripes.py <raw.bin> [W] [H]
"""
import sys

data = open(sys.argv[1], "rb").read()
W = int(sys.argv[2]) if len(sys.argv) > 2 else 160
H = int(sys.argv[3]) if len(sys.argv) > 3 else 120

print("=" * 50)
print("[1] DVP 数据线位活性检测 (0%或100% = 该位卡死)")
for name, off in [("低字节(像素D0-D7线)", 0), ("高字节(像素D8-D15线)", 1)]:
    s = data[off::2]
    dead = []
    for bit in range(8):
        cnt = sum(1 for b in s if (b >> bit) & 1)
        pct = cnt / len(s) * 100
        mark = ""
        if pct < 1.0:
            mark = " <-- 恒0 卡死!"
        elif pct > 99.0:
            mark = " <-- 恒1 卡死!"
        if mark or pct < 5 or pct > 95:
            dead.append(f"bit{bit}: {pct:.0f}%{mark}")
    print(f"{name}: " + ("; ".join(dead) if dead else "全部位正常激活"))

print("=" * 50)
print("[2] 行相关性 + 水平偏移检测 (找错位条纹)")
# 灰度化(取G通道近似)
gray = [[0] * W for _ in range(H)]
for y in range(H):
    for x in range(W):
        o = (y * W + x) * 2
        v = data[o] | (data[o + 1] << 8)
        gray[y][x] = ((v >> 5) & 0x3F)  # G通道6bit

# 对每对相邻行, 找最佳水平偏移
from collections import Counter
best_shifts = Counter()
zero_shift_corr = []
for y in range(1, H):
    corrs = {}
    for k in range(-12, 13):
        c = 0
        n = 0
        for x in range(max(0, -k), min(W, W - k)):
            c += abs(gray[y][x] - gray[y - 1][x + k])
            n += 1
        corrs[k] = c / max(n, 1)
    best_k = min(corrs, key=corrs.get)
    best_shifts[best_k] += 1
    zero_shift_corr.append(corrs[0])

print(f"偏移=0 的平均行差(灰度G通道): {sum(zero_shift_corr)/len(zero_shift_corr):.1f}")
print(f"各行最佳水平偏移分布(前8): {best_shifts.most_common(8)}")
top = best_shifts.most_common(1)[0]
if top[0] != 0 and top[1] > H * 0.5:
    print(f">>> 大部分行在偏移 {top[0]} 像素处最相关: 典型行错位, PCLK采样/HREF时序问题")
else:
    print(">>> 行无固定水平偏移: 不是简单行错位")

print("=" * 50)
print("[3] 竖条纹周期检测 (列亮度方差)")
col_mean = [sum(gray[y][x] for y in range(H)) / H for x in range(W)]
cm = sum(col_mean) / W
var = sum((c - cm) ** 2 for c in col_mean) / W
print(f"列均值={cm:.1f} 列间方差={var:.1f} (大=>有竖条纹)")
# 检查偶/奇列差异(字节序错乱的典型特征)
even_cols = [col_mean[x] for x in range(0, W, 2)]
odd_cols = [col_mean[x] for x in range(1, W, 2)]
print(f"偶列均值={sum(even_cols)/len(even_cols):.1f} 奇列均值={sum(odd_cols)/len(odd_cols):.1f}")
if abs(sum(even_cols)/len(even_cols) - sum(odd_cols)/len(odd_cols)) > 3:
    print(">>> 奇偶列亮度差异明显: 可能字节序/像素错位问题")