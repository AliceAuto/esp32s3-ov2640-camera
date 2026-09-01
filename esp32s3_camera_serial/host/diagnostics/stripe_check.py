# -*- coding: utf-8 -*-
"""判断一张 PNG 是否是横向/竖向条纹或错位, 并检测奇偶行错位(色彩错乱特征)。
用法: python stripe_check.py <img.png>
"""
import sys
from PIL import Image
import statistics

img = Image.open(sys.argv[1]).convert("RGB")
W, H = img.size
px = img.load()

# 1) 行间相似度: 相邻两行逐通道差和
row_diffs = []
for y in range(1, H):
    s = 0
    for x in range(W):
        s += abs(px[x, y][0]-px[x, y-1][0]) + abs(px[x, y][1]-px[x, y-1][1]) + abs(px[x, y][2]-px[x, y-1][2])
    row_diffs.append(s)
avg_row = sum(row_diffs)/len(row_diffs)

# 2) 列间相似度
col_diffs = []
for x in range(1, W):
    s = 0
    for y in range(H):
        s += abs(px[x, y][0]-px[x-1, y][0]) + abs(px[x, y][1]-px[x-1, y][1]) + abs(px[x, y][2]-px[x-1, y][2])
    col_diffs.append(s)
avg_col = sum(col_diffs)/len(col_diffs)

# 3) 奇/偶行均值差异(检查是否为两帧混叠或行错位)
odd_mean = statistics.mean(px[x, y][0]+px[x, y][1]+px[x, y][2] * 100 for y in range(0, H, 2) for x in range(W))
# 修正:使用正确三项
odd_sum = even_sum = 0
odd_n = even_n = 0
for y in range(H):
    for x in range(W):
        v = px[x, y][0]+px[x, y][1]+px[x, y][2]
        if y % 2 == 0:
            even_sum += v; even_n += 1
        else:
            odd_sum += v; odd_n += 1

print(f"尺寸 {W}x{H}")
print(f"行间平均差 avg_row={avg_row:.0f}")
print(f"列间平均差 avg_col={avg_col:.0f}")
print(f"对比: 相邻像素差(前值) earlier frame ~=45左右")
print(f"偶数行均亮度={even_sum/even_n:.0f}  奇数行均亮度={odd_sum/odd_n:.0f}")
if odd_n and abs(even_sum/even_n - odd_sum/odd_n) > 100:
    print(">>> 奇偶行亮度差异大: 可能是两帧混叠或行间错位/撕裂")
else:
    print("奇偶行亮度接近, 无整体撕裂痕迹")

# 4) 行方向周期性条纹检测: 比较 y 和 y+2
pair = sum(abs(px[0,y][0]-px[0,y+2][0]) for y in range(0, H-2)) 
print(f"相隔一行(y vs y+2)第0列差: {0 if H<3 else pair}")