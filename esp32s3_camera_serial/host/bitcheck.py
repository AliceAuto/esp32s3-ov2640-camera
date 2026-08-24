# -*- coding: utf-8 -*-
"""分析 RGB565 原始字节的位分布，排查是不是字节序/数据线问题。
用法: python bitcheck.py <raw_frame.bin>
"""
import sys

data = open(sys.argv[1], "rb").read()
print("帧长:", len(data))

# 统计偶数/奇数字节的位1比例
for name, off in [("even字节(b0)", 0), ("odd字节(b1)", 1)]:
    s = data[off::2]
    print(f"\n{name}: 共{len(s)}字节")
    for bit in range(8):
        cnt = sum(1 for b in s if (b >> bit) & 1)
        print(f"  bit{bit}: {cnt} (={cnt/len(s)*100:.0f}%)")

# 打印每像素值 v=小端
# 检查蓝色通道(低5位) vs 红色(高5位) vs 绿色(中6位)
sum_r = sum_g = sum_b = 0
n = len(data) // 2
nonblack = 0
for o in range(0, len(data) - 1, 2):
    v = data[o] | (data[o + 1] << 8)
    r = (v >> 11) & 0x1F
    g = (v >> 5) & 0x3F
    b = v & 0x1F
    if v > 0:
        nonblack += 1
    sum_r += r; sum_g += g; sum_b += b

print(f"\n像素数={n} 非黑(v>0)={nonblack}")
print(f"平均 R={sum_r/n:.1f} G={sum_g/n:.1f} B={sum_b/n:.1f} (5bit值, 0-31)")