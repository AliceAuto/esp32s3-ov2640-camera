# -*- coding: utf-8 -*-
"""定量分析 RGB565 帧质量：看是否有条纹/偏色/噪声/移位等典型异常。"""
import sys
from PIL import Image, ImageStat
import statistics

img = Image.open(sys.argv[1]).convert("RGB")
W, H = img.size
px = img.load()

# 1. 颜色分布
r = statistics.mean(px[x, y][0] for x in range(W) for y in range(H))
g = statistics.mean(px[x, y][1] for x in range(W) for y in range(H))
b = statistics.mean(px[x, y][2] for x in range(W) for y in range(H))
print(f"尺寸 {W}x{H}  平均色 R={r:.0f} G={g:.0f} B={b:.0f}")

# 2. 每行/每列亮度差异（检测横/竖条纹）
col_mean = [statistics.mean(px[x, y][0] + px[x, y][1] + px[x, y][2] for y in range(H)) / 3 for x in range(W)]
row_mean = [statistics.mean(px[x, y][0] + px[x, y][1] + px[x, y][2] for x in range(W)) / 3 for y in range(H)]
print(f"列亮度范围 {min(col_mean):.0f}~{max(col_mean):.0f}, 行亮度范围 {min(row_mean):.0f}~{max(row_mean):.0f}")

# 3. 相邻像素突变（噪声/雪花）
diffs = []
for y in range(H):
    for x in range(1, W):
        d = abs(px[x, y][0] - px[x - 1, y][0]) + abs(px[x, y][1] - px[x - 1, y][1]) + abs(px[x, y][2] - px[x - 1, y][2])
        diffs.append(d)
diffs.sort()
avg = sum(diffs) / len(diffs)
p95 = diffs[int(len(diffs) * 0.95)]
print(f"相邻像素差 平均={avg:.1f} 95%分位={p95:.1f}")

# 4. 检查是否是单色/灰阶（RGB 失衡大=偏色）
print(f"RGB 失衡度 max-min={max(r,g,b)-min(r,g,b):.0f}")