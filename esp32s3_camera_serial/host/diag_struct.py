# -*- coding: utf-8 -*-
"""结构诊断: 判断图像是否是"花屏错位"而非真实自然场景。
自然图像: 相邻像素平滑(差小), 空间相关高, 行错位少。
花屏: 相邻像素差大, 高频噪声, 有周期性错位/汇块。
用法: python diag_struct.py raw.bin W H
"""
import sys, struct

raw = open(sys.argv[1], 'rb').read()
W = int(sys.argv[2]); H = int(sys.argv[3])
N = W*H
vals = [0]*N
for i in range(N):
    o = i*2
    if o+1 >= len(raw): break
    vals[i] = struct.unpack_from('<H', raw, o)[0]

# 1. 相邻像素差直方图
diffs_h = [abs(vals[r*W+c]-vals[r*W+c-1]) for r in range(H) for c in range(1,W)]
diffs_h.sort()
import statistics
mean_d = statistics.mean(diffs_h)
print(f"水平相邻像素差: mean={mean_d:.0f}  p50={diffs_h[len(diffs_h)//2]}  p95={diffs_h[int(len(diffs_h)*0.95)]}")
# 真实场景: mean_d 一般 <40; 花屏噪声: >80

# 2. 天然图 vs 随机噪声的"自相关"
# 计算第拾行像素的相邻相关
s = [0.0,0.0]
for probe_r in range(0, min(50,H)):
    row = vals[probe_r*W:(probe_r+1)*W]
    corr = 0
    for c in range(1,W):
        a=row[c]; b=row[c-1]
        corr += 1 if abs(a-b)<60 else 0
    s[0]+=corr/(W-1)
corr_ratio = s[0]/min(50,H)
print(f"相近像素比例(<60): {corr_ratio*100:.0f}%  (真实场景>60%, 花屏<40%)")

# 3. 行重复检测: 竖直方向整行重复(花屏常出现行错位/条纹)
rep = 0
for r in range(1, H):
    s_r = vals[r*W : (r+1)*W]
    p_r = vals[(r-1)*W : r*W]
    same = sum(1 for a,b in zip(s_r,p_r) if a==b)/W
    if same > 0.9: rep += 1
print(f"与下一行相同像素>90% 的行数: {rep}/{H}  (<5为正常)")

# 4. 像素值聚类: 真实场景对角度分布丰富; 花屏常集中在少数互补值
uniq = len(set(vals))
print(f"唯一像素值数量: {uniq} / {N}  (真实>50%, 花屏/重复<10%严重)")