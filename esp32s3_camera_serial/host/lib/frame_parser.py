# -*- coding: utf-8 -*-
"""帧协议解析公共库。

固件 TCP/串口/中继流均以文本定界符切帧:
    FRAME_BEGIN\n
    SIZE <w> <h> <JPG|RGB>\n
    <payload>
    FRAME_END\n

用 find() 定位定界符 + 切片(一次线性扫描), 不用正则回溯 —— 高分辨率下
正则 .*? 的匹配开销是此前 live_capture 卡顿的来源之一。
"""
BEGIN = b"FRAME_BEGIN\n"
END = b"FRAME_END\n"


def split_frames(s):
    """从累积缓冲切出所有完整帧单元。

    返回 ([(w, h, fmt, body), ...], 已消费字节数)。
    未完整的一帧会整体保留在缓冲中(consumed 停在帧起点), 等待后续数据。
    """
    out = []
    pos = 0
    while True:
        i = s.find(BEGIN, pos)
        if i < 0:
            # 只保留可能被截断在缓冲尾部的半个 BEGIN
            return out, max(0, len(s) - len(BEGIN) + 1)
        nl = s.find(b"\n", i + len(BEGIN))
        if nl < 0:
            return out, i
        e = s.find(END, nl + 1)
        if e < 0:
            return out, i
        parts = s[i + len(BEGIN):nl].split()
        if len(parts) >= 3 and parts[0] == b"SIZE":
            try:
                out.append((int(parts[1]), int(parts[2]),
                            parts[3].decode("ascii", "ignore"), s[nl + 1:e]))
            except ValueError:
                pass
        pos = e + len(END)
    return out, pos
