# -*- coding: utf-8 -*-
"""OV2640 网络预览协议层。

UDP 分片协议(与固件 send_frame_udp 一致): 12 字节 OVU 头 + 载荷。
    byte0-2    MAGIC "OVU"
    byte3      fmt (1=JPEG, 0=RGB565)
    byte4-5    seq 帧序号(大端)
    byte6      cnt-1 总分片数-1
    byte7      idx 分片序号
    byte8-11   w,h 图像宽高(大端)
    byte12+    分片载荷

TCP/串口/中继文本流(FRAME_BEGIN/SIZE/FRAME_END)由 frame_parser 负责,
本模块只处理 OVU 数据报重组与帧校验。
"""
HDR = 12
MAGIC = b"OVU"
CHUNK = 1400 - HDR

# 分辨率表: 名称 -> (命令码, 宽, 高)
RES_INFO = {
    "QQVGA": ("q", 160, 120),
    "QVGA":  ("t", 320, 240),
    "VGA":   ("v", 640, 480),
    "SVGA":  ("s", 800, 600),
    "XGA":   ("x", 1024, 768),
    "UXGA":  ("u", 1600, 1200),
}


def res_label(name):
    code, w, h = RES_INFO[name]
    return f"{name} {w}x{h}"


RES_LABELS = [res_label(n) for n in RES_INFO]


def code_of(label):
    return RES_INFO[label.split()[0]][0]


def label_of_code(code):
    for n, (cd, w, h) in RES_INFO.items():
        if cd == code:
            return f"{n} {w}x{h}"
    return RES_LABELS[1]


def frame_ok(fmt, frame, w, h):
    """按格式校验一帧是否完整合法(JPEG 查 SOI/EOI, RGB565 按字节数)。"""
    if fmt == 1:
        return (len(frame) > 100 and frame[:2] == b"\xff\xd8"
                and frame[-2:] == b"\xff\xd9")
    return w > 0 and h > 0 and len(frame) == w * h * 2


class UdpAssembler:
    """OVU UDP 分片重组器: 按 seq 换帧, 到齐且校验通过返回完整帧。

    feed(pkt) 返回 None 表示帧未齐/非法; 返回 (fmt, frame, w, h, seq)
    表示成功重组出一帧(内部状态已复位, 可继续吃下一帧分片)。
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self._ctx = {"seq": None, "cnt": 0, "w": 0, "h": 0, "fmt": 0,
                     "parts": None, "got": 0}

    def feed(self, pkt):
        if len(pkt) < HDR or pkt[:3] != MAGIC:
            return None
        seq = (pkt[4] << 8) | pkt[5]
        cnt = pkt[6] + 1
        idx = pkt[7]
        w = (pkt[8] << 8) | pkt[9]
        h = (pkt[10] << 8) | pkt[11]
        fmt = pkt[3]
        ctx = self._ctx
        if ctx["seq"] != seq:
            ctx.update(seq=seq, cnt=cnt, w=w, h=h, fmt=fmt,
                       parts=[None] * cnt, got=0)
        if idx >= ctx["cnt"]:
            return None
        if ctx["parts"][idx] is None:
            ctx["parts"][idx] = pkt[HDR:]
            ctx["got"] += 1
        if ctx["got"] != ctx["cnt"]:
            return None
        frame = b"".join(ctx["parts"])
        fw, fh, ffmt = ctx["w"], ctx["h"], ctx["fmt"]
        self.reset()
        if not frame_ok(ffmt, frame, fw, fh):
            return None
        return ffmt, frame, fw, fh, seq
