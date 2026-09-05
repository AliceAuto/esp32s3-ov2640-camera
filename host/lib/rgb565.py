# -*- coding: utf-8 -*-
"""big-endian RGB565 -> RGB888 转换。

numpy 向量化优先(VGA 640x480 约 2ms); 无 numpy 时回退纯 Python 逐像素
(同一帧数百 ms, 会拖垮帧率, 建议 pip install numpy)。

舍入公式必须全局一致: (v*255+15)//31 / (v*255+31)//63 —— 两套客户端曾因
舍入差异导致同帧画面差 1 个色阶。
"""
try:
    import numpy as _np
except ImportError:
    _np = None


def convert_rgb(frame):
    """整帧 big-endian RGB565 bytes -> RGB888 bytes(长度 = len//2*3)。"""
    if _np is not None:
        a = _np.frombuffer(frame, dtype=">u2")     # 大端 uint16
        flat = _np.empty((a.size, 3), dtype=_np.uint8)
        flat[:, 0] = (((a >> 11) & 0x1F) * 255 + 15) // 31
        flat[:, 1] = (((a >> 5) & 0x3F) * 255 + 31) // 63
        flat[:, 2] = ((a & 0x1F) * 255 + 15) // 31
        return flat.tobytes()
    n = len(frame) // 2
    rgb = bytearray(n * 3)
    j = 0
    for o in range(0, len(frame), 2):
        v = (frame[o] << 8) | frame[o + 1]   # big-endian
        r = (v >> 11) & 0x1F
        g = (v >> 5) & 0x3F
        b = v & 0x1F
        rgb[j] = (r * 255 + 15) // 31
        rgb[j + 1] = (g * 255 + 31) // 63
        rgb[j + 2] = (b * 255 + 15) // 31
        j += 3
    return bytes(rgb)
