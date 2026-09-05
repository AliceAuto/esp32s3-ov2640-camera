# -*- coding: utf-8 -*-
"""
OV2640 网络实时预览上位机 v2 —— 程序入口。

只负责: CLI 解析 -> 写入 lib.core 配置 -> 装配 GUI/收流/解码三线程。
具体逻辑按职责拆分在 host/lib/:
  core           运行时配置 + 共享状态 + 帧路由 + 命令 + 统计
  reader         收流编排(局域网 UDP/TCP / 跨网中继 / 自动回退)
  gui            Tkinter 界面 + 后台解码 + 渲染循环 + 拍照/存图
  protocol       OVU UDP 分片协议 + 分辨率表
  frame_parser   FRAME_BEGIN/SIZE/FRAME_END 文本流切帧(共享)
  rgb565         RGB565 -> RGB888(共享)
  relay_client   中继客户端(RelayIo)
  netio          局域网发现 / TCP 命令通道
  watchdog       色彩异常看门狗
  gui_style      深色主题(共享)

用法: python main.py --wifi [host] [port] [res_code]
      python main.py --relay 中继服务器:端口 [--id 设备ID]   # 纯跨网中继
      python main.py --lanrelay [--id 设备ID]              # 局域网优先, 失败自动回退中继
      python main.py --selftest          # 无 GUI 协议自检
快捷键: Ctrl+Q 退出, Ctrl+S 当前画面存图
"""
import os
import sys
import threading

try:
    import numpy as np
except ImportError:
    np = None

from lib import core
from lib import protocol as proto

# ---- 参数解析 (兼容旧用法: --wifi [host] [port] [res]) ----
_FLAGS = ("--wifi", "--relay", "--lanrelay", "--id", "--token")
def _fv(name):
    if name in sys.argv:
        i = sys.argv.index(name)
        if i + 1 < len(sys.argv):
            return sys.argv[i + 1]
    return None
SELFTEST = "--selftest" in sys.argv
# 传输模式: relay=强制中继; lanrelay=局域网优先, 失败回退中继; lan=仅局域网(UDP/TCP)
if "--relay" in sys.argv:
    TRANSPORT = "relay"
elif "--lanrelay" in sys.argv:
    TRANSPORT = "lanrelay"
else:
    TRANSPORT = "lan"
# 中继服务端 "host:port", 仅在 relay/lanrelay 用
_RELAY_SPEC = _fv("--relay") or "8.148.79.85:8001"
_ID = _fv("--id")
# 纯位置参数(去掉所有 --flag 及其取值)
_skip = _FLAGS + tuple(v for v in (_ID,) if v)
ARGV = [x for x in sys.argv[1:] if x not in _skip]
if TRANSPORT == "lan":
    host = ARGV[0] if ARGV else "192.168.4.1"
    port = int(ARGV[1]) if len(ARGV) > 1 else 3333
    res_code = (ARGV[2].strip() if len(ARGV) > 2 else "")
else:
    host = "192.168.4.1"     # 中继模式不使用局域网直连地址
    port = 3333
    res_code = ""

# ---- 把 CLI 解析结果写入共享配置(core), reader/gui 从 core 读取 ----
core.TRANSPORT = TRANSPORT
core.HOST = host
core.PORT = port
core.RELAY_SPEC = _RELAY_SPEC
core.DEV_ID = _ID
core.RES_CODE = res_code
core.G["res_code"] = res_code or "t"
core._relay_on[0] = (TRANSPORT == "relay")
core.PHOTO_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "photos")


# ================= 无 GUI 自检 =================
def run_selftest():
    import random
    random.seed(7)

    def jhdr(nchunks_minus1, idx, seq=1234, w=320, h=240, fmt=1):
        return (proto.MAGIC + bytes([fmt])
                + bytes([(seq >> 8) & 0xFF, seq & 0xFF])
                + bytes([nchunks_minus1, idx])
                + bytes([(w >> 8) & 0xFF, w & 0xFF, (h >> 8) & 0xFF, h & 0xFF]))

    # 1) JPEG 多分片重组
    body = b"\xff\xd8" + bytes(random.randrange(256) for _ in range(proto.CHUNK * 3 + 77)) + b"\xff\xd9"
    chunks = [body[i:i + proto.CHUNK] for i in range(0, len(body), proto.CHUNK)]
    assert any(core.udp_feed(jhdr(len(chunks) - 1, i) + ch) for i, ch in enumerate(chunks)), \
        "多分片 JPEG 应恰好重组出一帧"
    ffmt, fr, fw_, fh_ = core.prev_q.get_nowait()
    assert ffmt == 1 and fr == body and (fw_, fh_) == (320, 240), "重组内容不符"

    # 2) 起始标记损坏应拒收
    bad = b"\xff\xd9garbage" + b"\x01" * 200 + b"\xff\xd9"
    assert core.udp_feed(jhdr(0, 0, seq=555) + bad) is False, "坏头 JPEG 应回复 False"
    assert core.prev_q.qsize() == 0, "坏帧不得入队"

    # 3) RGB 尺寸校验: 声明 64x32 但数据不足 -> 拒收
    short = jhdr(0, 0, seq=777, w=64, h=32, fmt=0) + b"\x00" * 100
    assert core.udp_feed(short) is False, "RGB 尺寸不足应拒收"

    # 4) RGB 正确长度(4x4=32B 单片) -> 入队且内容一致
    rgb = bytes(random.randrange(256) for _ in range(32))
    assert core.udp_feed(jhdr(0, 0, seq=888, w=4, h=4, fmt=0) + rgb), "合法 RGB 应成帧"
    ffmt2, fr2, w2, h2 = core.prev_q.get_nowait()
    assert (ffmt2, fr2, w2, h2) == (0, rgb, 4, 4), "RGB 帧内容不符"

    # 5) numpy 与纯 Python 转换一致性
    if np is not None:
        raw = bytes(random.randrange(256) for _ in range(64 * 32 * 2))
        ref = bytearray()
        for o in range(0, len(raw), 2):
            v = (raw[o] << 8) | raw[o + 1]
            r = (v >> 11) & 0x1F; g = (v >> 5) & 0x3F; b = v & 0x1F
            ref.append((r * 255 + 15) // 31); ref.append((g * 255 + 31) // 63); ref.append((b * 255 + 15) // 31)
        fast = core.convert_rgb(raw, 64, 32)
        assert fast == bytes(ref), "numpy 与参考实现不一致"
    # 6) TCP 统一解复用: 单缓冲按帧单元解析, 预览/照片正确路由且任意切分不损坏
    def unit(w_, h_, payload):
        return b'FRAME_BEGIN\nSIZE %d %d JPG\n' % (w_, h_) + payload + b'FRAME_END\n'
    prev_body = b"\xff\xd8" + bytes(random.randrange(256) for _ in range(300)) + b"\xff\xd9"
    photo_body = b"\xff\xd8" + bytes(random.randrange(256) for _ in range(450)) + b"\xff\xd9"
    stream = unit(320, 240, prev_body) + unit(1600, 1200, photo_body)
    for piece in (stream[:77], stream[77:800], stream[800:1300], stream[1300:]):
        core._pump_bytes(piece)                   # 前1.5帧: pending=False -> 预览通道
    assert core.prev_q.qsize() == 2 and core.photo_q.qsize() == 0, "pending=False 应全走预览"
    f_, fr_, fw2_, fh2_ = core.prev_q.get_nowait()
    assert (fw2_, fh2_) == (320, 240) and fr_ == prev_body, "预览单元内容不符"
    while not core.prev_q.empty():            # 清掉第二预览单元, 保证下个用例初始态干净
        core.prev_q.get_nowait()
    del core.tcp_rx[:]
    core.G["photo_pending"] = True
    mixed = b'\xff\xd8junk' + unit(1600, 1200, photo_body)   # 前缀杂散文本应被跳过
    for piece in (mixed[:200], mixed[200:]):
        core._pump_bytes(piece)
    assert core.photo_q.qsize() == 1 and core.prev_q.qsize() == 0, "pending=True 应路由到照片"
    pb, pw, ph_ = core.photo_q.get_nowait()
    assert pw == 1600 and pb == photo_body, "照片单元内容不符"
    assert core.G["photo_pending"] is False, "收到照片后应自动清 pending"
    print("PASS: TCP 解复用路由与抗劈裂")

    print("SELFTEST OK")


# ================= 装配 =================
if SELFTEST:
    run_selftest()
    sys.exit(0)

import tkinter as tk
from lib import gui
from lib import reader


def main():
    root = tk.Tk()
    gui.build_ui(root)
    th = threading.Thread(target=reader.reader, args=(core.G["stop"],), daemon=True)
    th.start()
    th_dec = threading.Thread(target=gui.decode_worker, args=(core.G["stop"],), daemon=True)
    th_dec.start()
    root.after(200, gui.render_loop, root)
    root.mainloop()
    core.G["stop"].set()
    print("退出", flush=True)


if __name__ == "__main__":
    main()
