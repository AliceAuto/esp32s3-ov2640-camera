# -*- coding: utf-8 -*-
"""
OV2640 UDP 高帧率预览上位机 v2。

- UDP 收预览帧(丢帧不重传)，TCP 发命令/收拍照帧。
- 预览队列只保留最新一帧：解码跟不上时丢旧不滞后。
- 底部状态栏: 连接徽章 + 模式/分辨率/帧率/带宽；顶部工具栏: 传输模式、
  预览分辨率、JPEG画质滑杆、照片分辨率+拍照(防连点)、当前画面存图。

用法: python udp_capture.py --wifi [host] [port] [res_code]
      python udp_capture.py --selftest          # 无 GUI 协议自检
快捷键: Ctrl+Q 退出, Ctrl+S 当前画面存图
"""
import sys, os, io, re, time, threading, queue
import socket
from collections import deque

try:
    import numpy as np
except ImportError:
    np = None

# ---- 参数解析 (兼容旧用法: --wifi [host] [port] [res]) ----
_args = [x for x in sys.argv[1:] if x != "--wifi"]
SELFTEST = "--selftest" in sys.argv
host = _args[0] if _args else "192.168.4.1"
port = int(_args[1]) if len(_args) > 1 else 3333
res_code = (_args[2].strip() if len(_args) > 2 else "")

# ---- 协议常量(与固件 send_frame_udp 一致) ----
HDR = 12
MAGIC = b"OVU"
CHUNK = 1400 - HDR

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

PHOTO_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "photos")
RECONNECT_AFTER = 4.0        # 预览停流超此秒数 -> TCP 重连并重下发命令
STAT_WINDOW = 5.0            # 帧率统计窗口(秒)

# ---- 共享状态(主线程读、reader 线程写, 均为单赋值原子操作) ----
G = {
    "mode": "UDP",
    "tcp": None,
    "stop": threading.Event(),
    "last_good": 0.0,        # 最近一次完整预览帧到达时刻
    "photo_until": 0.0,      # 拍照互斥截止时刻
    "frames": 0,             # 完整帧总数(samples 是定长滚动窗, 不能当累计用)
    "drops": 0,              # 按帧序号缺口估算的链路丢帧数
    "photo_pending": False,  # 已发 P 待收照片: 下一个完整 TCP 帧单元路由给照片而非预览
}
samples = deque(maxlen=1024)         # (t, bytes) 完整帧滚动记录
seq_state = {"last": None}           # 最近一次完整帧的 seq(用于丢帧估算)
tcp_rx = bytearray()                 # TCP 唯一接收缓冲: 单点读取再分发给照片/预览解析
stat_t = [time.time()]
prev_q = queue.Queue(maxsize=2)      # 只保留最新一帧的预览队列
photo_q = queue.Queue()
udp_ctx = {"seq": None, "cnt": 0, "w": 0, "h": 0, "fmt": 0,
           "parts": None, "got": 0}
_TCP_FRAME = re.compile(rb'FRAME_BEGIN\nSIZE (\d+) (\d+) (\w+)\n(.*?)FRAME_END\n', re.S)


def _put_latest(item):
    """入预览队列；满则丢最旧，保证渲染端永远拿较新一帧。"""
    try:
        prev_q.put_nowait(item)
    except queue.Full:
        try:
            prev_q.get_nowait()
        except queue.Empty:
            pass
        try:
            prev_q.put_nowait(item)
        except queue.Full:
            pass


def convert_rgb(frame, w, h):
    """big-endian RGB565 -> RGB888。numpy 向量化优先，无则纯 Python。"""
    if np is not None:
        a = np.frombuffer(frame[:w * h * 2], dtype=np.uint16).byteswap()
        flat = np.empty((w * h, 3), dtype=np.uint8)
        flat[:, 0] = (((a >> 11) & 0x1F).astype(np.uint16) * 255 + 15) // 31
        flat[:, 1] = (((a >> 5) & 0x3F).astype(np.uint16) * 255 + 31) // 63
        flat[:, 2] = ((a & 0x1F).astype(np.uint16) * 255 + 15) // 31
        return flat.reshape(h, w, 3).tobytes()
    out = bytearray(w * h * 3)
    j = 0
    for o in range(0, len(frame), 2):
        v = (frame[o] << 8) | frame[o + 1]
        r = (v >> 11) & 0x1F; g = (v >> 5) & 0x3F; b = v & 0x1F
        out[j] = (r * 255 + 15) // 31
        out[j + 1] = (g * 255 + 31) // 63
        out[j + 2] = (b * 255 + 15) // 31
        j += 3
    return bytes(out)


def udp_feed(pkt):
    """吃一个数据报重组整帧；到齐且校验通过返回 True 并入预览队列。"""
    global udp_ctx
    if len(pkt) < HDR or pkt[:3] != MAGIC:
        return False
    seq = (pkt[4] << 8) | pkt[5]
    cnt = pkt[6] + 1
    idx = pkt[7]
    w = (pkt[8] << 8) | pkt[9]
    h = (pkt[10] << 8) | pkt[11]
    fmt = pkt[3]
    if udp_ctx["seq"] != seq:
        udp_ctx.update(seq=seq, cnt=cnt, w=w, h=h, fmt=fmt,
                       parts=[None] * cnt, got=0)
    if idx >= udp_ctx["cnt"]:
        return False
    if udp_ctx["parts"][idx] is None:
        udp_ctx["parts"][idx] = pkt[HDR:]
        udp_ctx["got"] += 1
    if udp_ctx["got"] != udp_ctx["cnt"]:
        return False
    frame = b"".join(udp_ctx["parts"])
    fw, fh, ffmt = udp_ctx["w"], udp_ctx["h"], udp_ctx["fmt"]
    udp_ctx = {"seq": None, "cnt": 0, "w": 0, "h": 0, "fmt": 0,
               "parts": None, "got": 0}
    if ffmt == 1:
        ok = len(frame) > 100 and frame[:2] == b"\xff\xd8" and frame[-2:] == b"\xff\xd9"
    else:
        ok = fw > 0 and fh > 0 and len(frame) == fw * fh * 2
    if not ok:
        return False
    samples.append((time.time(), len(frame)))
    G["last_good"] = time.time()
    G["frames"] += 1
    prev = seq_state["last"]
    if prev is not None:
        gap = (seq - prev - 1) & 0xFFFF     # 整帧丢失数量(含固件背压丢帧与网络丢片)
        if 0 < gap < 4096:
            G["drops"] += gap
    seq_state["last"] = seq
    _put_latest((ffmt, frame, fw, fh))
    return True


def _route_jpeg_frame(body, w, h):
    """按 pending 状态把一个完整 JPG 帧单元分发给照片(可靠消费, 不丢)或预览通道。"""
    samples.append((time.time(), len(body)))
    if G["photo_pending"]:
        G["photo_pending"] = False
        photo_q.put((bytes(body), w, h))
        return
    G["last_good"] = time.time()
    G["frames"] += 1
    _put_latest((1, bytes(body), w, h))


def _pump_bytes(data):
    """向唯一 TCP 缓冲喂字节并循环解出所有完整帧单元。

    此前照片与预览各自独立 recv 同一 socket, 一包数据被劈成两半拼出损坏 JPEG;
    统一缓冲后按帧单元完整解析再分发, 从根上消除拼接损坏。"""
    tcp_rx.extend(data)
    while True:
        i = tcp_rx.find(b"FRAME_BEGIN\n")
        if i < 0:
            del tcp_rx[:-12]
            return
        nl = tcp_rx.find(b"\n", i + 12)
        if nl < 0:
            if i > 0:
                del tcp_rx[:i]
            return
        m = re.match(rb"SIZE\s+(\d+)\s+(\d+)\s+(\w+)", bytes(tcp_rx[i + 12:nl]))
        e = tcp_rx.find(b"FRAME_END\n", nl + 1)
        if e < 0:
            if i > 0:
                del tcp_rx[:i]
            return
        body = bytes(tcp_rx[nl + 1:e])
        del tcp_rx[:e + len(b"FRAME_END\n")]
        if m and m.group(3) == b"JPG":
            w_, h_ = int(m.group(1)), int(m.group(2))
            if w_ > 0 and h_ > 0 and len(body) > 100 and body[:2] == b"\xff\xd8":
                _route_jpeg_frame(body, w_, h_)


def pump_tcp(tcp):
    """TCP 单点非阻塞读取入口。半开/对端关闭时置死连接触发重连。"""
    try:
        data = tcp.recv(65536)
    except (BlockingIOError, socket.timeout):
        return
    except OSError:
        _tcp_dead(tcp)
        return
    if not data:
        _tcp_dead(tcp)
        return
    _pump_bytes(data)


def _tcp_dead(sock):
    """TCP 对端半开/有序关闭: recv 返回空即连接已死, 置空触发 reader 重连。"""
    if G.get("tcp") is sock:
        G["tcp"] = None
        try:
            sock.close()
        except Exception:
            pass


def _clear_rxbuffers():
    """切模式/重连后清掉所有重组残留。"""
    global udp_ctx
    udp_ctx = {"seq": None, "cnt": 0, "w": 0, "h": 0, "fmt": 0,
               "parts": None, "got": 0}
    tcp_rx.clear()


def mode_cmd(code=None):
    s = ("v" + code) if code else ""
    return s + ("U" if G["mode"] == "UDP" else "N")


def send_cmd(data):
    c = G["tcp"]
    payload = data.encode() if isinstance(data, str) else data
    if c is None:
        print("发送失败: 无命令通道", flush=True)
        return False
    try:
        c.sendall(payload)
        return True
    except OSError as e:
        print("发送失败:", e, flush=True)
        return False


def open_tcp_cmd(stop_ev):
    for _ in range(600):
        if stop_ev.is_set():
            return None
        try:
            c = socket.create_connection((host, port), timeout=4)
            c.settimeout(0.05)
            c.sendall(mode_cmd(res_code).encode())
            print(f"TCP 已连 {host}:{port}, 已发 {mode_cmd(res_code)!r}", flush=True)
            return c
        except OSError:
            time.sleep(1.0)
    return None


def window_stats(win=STAT_WINDOW):
    """最近 win 秒内的 (帧数, 字节数)。"""
    t0 = time.time() - win
    n = byt = 0
    for t, b in reversed(samples):
        if t < t0:
            break
        n += 1; byt += b
    return n, byt


def _reader_body(stop_ev):
    """单次收流会话；异常交给外层 supervisor 捕获重启。"""
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        udp.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    except Exception:
        pass
    udp.bind(("0.0.0.0", port))
    udp.settimeout(0.05)
    print(f"UDP 已监听 :{port}", flush=True)

    tcp = open_tcp_cmd(stop_ev)
    if tcp is None:
        udp.close()
        return
    G["tcp"] = tcp
    tcp.settimeout(0)
    last_good = time.time()

    while not stop_ev.is_set():
        now = time.time()
        # 停流超时判定在拍照窗口内让路: P 命令期间预览必然暂停数秒,
        # 此时重连会把正在回传的照片流切断。
        # 注意不依赖 G["tcp"] 非空: 连接被对端硬拆时它已被置 None,
        # 以往这里会永远跳过重连且不打日志(静默假死)。
        if (now - max(last_good, G["last_good"]) > RECONNECT_AFTER
                and now >= G["photo_until"]):
            print(f"[{G['mode']}] 停流超时, 重连重下发...", flush=True)
            old = G["tcp"]
            G["tcp"] = None
            try:
                tcp.close()
            except Exception:
                pass
            if old is not None and old is not tcp:
                try:
                    old.close()
                except Exception:
                    pass
            _clear_rxbuffers()
            tcp = open_tcp_cmd(stop_ev)
            if tcp is None:
                break
            G["tcp"] = tcp
            tcp.settimeout(0)
            last_good = time.time()

        if G["mode"] == "UDP":
            got = completed = closed = False
            while True:
                try:
                    pkt, _addr = udp.recvfrom(65535)
                except socket.timeout:
                    break
                except OSError:
                    closed = True
                    break
                got = True
                if udp_feed(pkt):
                    completed = True
            if completed:
                last_good = time.time()
            if not got:
                time.sleep(0.002)
            if closed:
                break
            if G["tcp"]:
                pump_tcp(G["tcp"])
        else:
            n0 = len(samples)
            if G["tcp"]:
                pump_tcp(G["tcp"])
            if len(samples) != n0:
                last_good = time.time()

        if time.time() - stat_t[0] >= 3.0:
            n, byt = window_stats()
            print(f"{G['mode']:>3} fps={n / STAT_WINDOW:6.1f} kb={(byt / 1024) / STAT_WINDOW:5.0f}/s "
                  f"累计 {G['frames']} 丢帧 {G['drops']}", flush=True)
            stat_t[0] = time.time()

    try:
        udp.close()
    except Exception:
        pass
    G["tcp"] = None


def reader(stop_ev):
    """supervisor：收流线程任何未捕获异常都会被拉起重启，稳定性优先。

    已知妥协: 异常路径下旧 UDP socket 依赖进程退出/GC 回收, REUSEADDR 保证重绑不失败。"""
    while not stop_ev.is_set():
        try:
            _reader_body(stop_ev)
            return                       # body 正常返回(如连续连不上)时按原语义结束
        except Exception:
            import traceback
            traceback.print_exc()
            G["tcp"] = None
            print("reader 异常, 2s 后自动重启收流线程", flush=True)
            if stop_ev.wait(2.0):
                return


# ================= 无 GUI 自检 =================
def run_selftest():
    import random
    random.seed(7)

    def jhdr(nchunks_minus1, idx, seq=1234, w=320, h=240, fmt=1):
        return (MAGIC + bytes([fmt])
                + bytes([(seq >> 8) & 0xFF, seq & 0xFF])
                + bytes([nchunks_minus1, idx])
                + bytes([(w >> 8) & 0xFF, w & 0xFF, (h >> 8) & 0xFF, h & 0xFF]))

    # 1) JPEG 多分片重组
    body = b"\xff\xd8" + bytes(random.randrange(256) for _ in range(CHUNK * 3 + 77)) + b"\xff\xd9"
    chunks = [body[i:i + CHUNK] for i in range(0, len(body), CHUNK)]
    assert any(udp_feed(jhdr(len(chunks) - 1, i) + ch) for i, ch in enumerate(chunks)), \
        "多分片 JPEG 应恰好重组出一帧"
    ffmt, fr, fw_, fh_ = prev_q.get_nowait()
    assert ffmt == 1 and fr == body and (fw_, fh_) == (320, 240), "重组内容不符"

    # 2) 起始标记损坏应拒收
    bad = b"\xff\xd9garbage" + b"\x01" * 200 + b"\xff\xd9"
    assert udp_feed(jhdr(0, 0, seq=555) + bad) is False, "坏头 JPEG 应回复 False"
    assert prev_q.qsize() == 0, "坏帧不得入队"

    # 3) RGB 尺寸校验: 声明 64x32 但数据不足 -> 拒收
    short = jhdr(0, 0, seq=777, w=64, h=32, fmt=0) + b"\x00" * 100
    assert udp_feed(short) is False, "RGB 尺寸不足应拒收"

    # 4) RGB 正确长度(4x4=32B 单片) -> 入队且内容一致
    rgb = bytes(random.randrange(256) for _ in range(32))
    assert udp_feed(jhdr(0, 0, seq=888, w=4, h=4, fmt=0) + rgb), "合法 RGB 应成帧"
    ffmt2, fr2, w2, h2 = prev_q.get_nowait()
    assert (ffmt2, fr2, w2, h2) == (0, rgb, 4, 4), "RGB 帧内容不符"

    # 5) numpy 与纯 Python 转换一致性
    if np is not None:
        raw = bytes(random.randrange(256) for _ in range(64 * 32 * 2))
        ref = bytearray()
        for o in range(0, len(raw), 2):
            v = (raw[o] << 8) | raw[o + 1]
            r = (v >> 11) & 0x1F; g = (v >> 5) & 0x3F; b = v & 0x1F
            ref.append((r * 255 + 15) // 31); ref.append((g * 255 + 31) // 63); ref.append((b * 255 + 15) // 31)
        fast = convert_rgb(raw, 64, 32)
        assert fast == bytes(ref), "numpy 与参考实现不一致"
    # 6) TCP 统一解复用: 单缓冲按帧单元解析, 预览/照片正确路由且任意切分不损坏
    def unit(w_, h_, payload):
        return b'FRAME_BEGIN\nSIZE %d %d JPG\n' % (w_, h_) + payload + b'FRAME_END\n'
    prev_body = b"\xff\xd8" + bytes(random.randrange(256) for _ in range(300)) + b"\xff\xd9"
    photo_body = b"\xff\xd8" + bytes(random.randrange(256) for _ in range(450)) + b"\xff\xd9"
    stream = unit(320, 240, prev_body) + unit(1600, 1200, photo_body)
    for piece in (stream[:77], stream[77:800], stream[800:1300], stream[1300:]):
        _pump_bytes(piece)                    # 前1.5帧: pending=False -> 预览通道
    assert prev_q.qsize() == 2 and photo_q.qsize() == 0, "pending=False 应全走预览"
    f_, fr_, fw2_, fh2_ = prev_q.get_nowait()
    assert (fw2_, fh2_) == (320, 240) and fr_ == prev_body, "预览单元内容不符"
    while not prev_q.empty():             # 清掉第二预览单元, 保证下个用例初始态干净
        prev_q.get_nowait()
    del tcp_rx[:]
    G["photo_pending"] = True
    mixed = b'\xff\xd8junk' + unit(1600, 1200, photo_body)   # 前缀杂散文本应被跳过
    for piece in (mixed[:200], mixed[200:]):
        _pump_bytes(piece)
    assert photo_q.qsize() == 1 and prev_q.qsize() == 0, "pending=True 应路由到照片"
    pb, pw, ph_ = photo_q.get_nowait()
    assert pw == 1600 and pb == photo_body, "照片单元内容不符"
    assert G["photo_pending"] is False, "收到照片后应自动清 pending"
    print("PASS: TCP 解复用路由与抗劈裂")

    print("SELFTEST OK")


# ================= GUI =================
if SELFTEST:
    run_selftest()
    sys.exit(0)

import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk

ui = {}          # 控件句柄/渲染状态集中存放


def build_ui(root):
    ui["root"] = root
    root.title(f"OV2640 实时预览 [{host}:{port}]")

    # ---- 工具栏 ----
    bar = ttk.Frame(root); bar.pack(fill="x", padx=6, pady=(6, 0))

    ttk.Label(bar, text="传输:").pack(side="left")
    mode_cb = ttk.Combobox(bar, values=["UDP", "TCP"], state="readonly", width=6)
    mode_cb.set("UDP"); mode_cb.pack(side="left", padx=(4, 10))
    ui["mode_cb"] = mode_cb

    ttk.Label(bar, text="预览:").pack(side="left")
    # width 需容纳 "UXGA 1600x1200" 这类带尺寸标签, 否则选中框显示被截断
    res_cb = ttk.Combobox(bar, values=RES_LABELS, state="readonly", width=14)
    res_cb.set(res_label("QVGA")); res_cb.pack(side="left", padx=(4, 10))
    ui["res_cb"] = res_cb

    ttk.Label(bar, text="画质:").pack(side="left")
    q_var = tk.IntVar(value=10)
    q_scale = ttk.Scale(bar, from_=8, to=40, variable=q_var, length=110)
    q_scale.pack(side="left", padx=(4, 2))
    q_lbl = ttk.Label(bar, text=str(q_var.get()), width=3)
    q_lbl.pack(side="left", padx=(0, 10))
    ui["q_var"], ui["q_lbl"] = q_var, q_lbl

    ttk.Label(bar, text="拍照:").pack(side="left")
    photo_cb = ttk.Combobox(bar, values=RES_LABELS, state="readonly", width=14)
    photo_cb.set(res_label("UXGA")); photo_cb.pack(side="left", padx=(4, 4))
    ui["photo_cb"] = photo_cb

    photo_btn = ttk.Button(bar, text="拍照", command=lambda: take_photo())
    photo_btn.pack(side="left", padx=(0, 8))
    ui["photo_btn"] = photo_btn

    snap_btn = ttk.Button(bar, text="存图(Ctrl+S)", command=snapshot)
    snap_btn.pack(side="left")

    # ---- 视频区 ----
    video = tk.Label(root, bg="#101010", width=88, height=30)
    video.pack(padx=6, pady=4, fill="both", expand=True)
    ui["video"] = video

    # ---- 状态栏 ----
    foot = ttk.Frame(root); foot.pack(fill="x", padx=6, pady=(0, 6))
    badge = tk.Label(foot, text=" 连接中 ", fg="white", bg="#c60", padx=6)
    badge.pack(side="left")
    info = ttk.Label(foot, text="-", anchor="w")
    info.pack(side="left", fill="x", expand=True, padx=8)
    tip = ttk.Label(foot, text="Ctrl+Q 退出", foreground="#888")
    tip.pack(side="right")
    ui["badge"], ui["info"] = badge, info

    # ---- 事件绑定 ----
    def on_mode(_ev=None):
        G["mode"] = mode_cb.get()
        _clear_rxbuffers()
        if G["mode"] == "TCP":
            prev_q.queue.clear()          # 切 TCP 清掉 UDP 残留画面
        cmd = mode_cmd(code_of(res_cb.get()))
        if send_cmd(cmd):
            print(f"模式切换 -> {G['mode']}: {cmd!r}", flush=True)
    mode_cb.bind("<<ComboboxSelected>>", on_mode)

    def on_res(_ev=None):
        cmd = mode_cmd(code_of(res_cb.get()))
        if send_cmd(cmd):
            print(f"分辨率 -> {res_cb.get()}: {cmd!r}", flush=True)
    res_cb.bind("<<ComboboxSelected>>", on_res)

    # 拖动中不发，松手才发一次（固件按单字节二进制值解析画质）
    def on_q_release(_ev=None):
        v = max(8, min(40, int(q_var.get())))
        q_lbl.config(text=str(v))
        if send_cmd(bytes([v])):
            print(f"quality -> {v}", flush=True)
    q_scale.bind("<ButtonRelease-1>", on_q_release)

    root.bind("<Control-q>", lambda e: close())
    root.bind("<Control-s>", lambda e: snapshot())
    root.protocol("WM_DELETE_WINDOW", close)


def take_photo():
    if time.time() < G["photo_until"]:
        return
    code = code_of(ui["photo_cb"].get())
    if not send_cmd(f"H{code}\nP".encode()):
        return
    G["photo_until"] = time.time() + 15.0     # UXGA 拍照+传输留足时间
    G["photo_pending"] = True                 # 下一个完整 TCP 帧单元路由给照片
    btn = ui["photo_btn"]
    btn.state(["disabled"]); btn.config(text=f"拍照中({code})…")
    ui["info"].config(text=f"拍照 {ui['photo_cb'].get()} 中，回传走可靠通道…")


def finish_photo(msg_ok):
    G["photo_pending"] = False
    btn = ui["photo_btn"]
    btn.state(["!disabled"]); btn.config(text="拍照")
    if not msg_ok:
        ui["info"].config(text="拍照超时/失败，可重试")


def save_jpeg_bytes(raw, prefix, w, h):
    os.makedirs(PHOTO_DIR, exist_ok=True)
    fn = os.path.join(PHOTO_DIR, "%s_%s_%dx%d.jpg"
                      % (prefix, time.strftime("%Y%m%d_%H%M%S"), w, h))
    with open(fn, "wb") as f:                 # 板子给的已是 JPEG, 直写不重编码
        f.write(raw)
    return fn


# ---- 色彩异常看门狗 ----
# OV2640 的 DSP 寄存器状态可能被写坏(白平衡预设表与部分模块不兼容), 表现为整幅
# 蓝紫色偏(实测白桌 R185/G142/B254)且无法通过 w0 恢复, 只有固件 'r' 完整重建才痊愈。
# 监测中心区域均值: B 同时高出 R/G 达阈值并持续 ~3s -> 下发 'r' 自动重建。
CW = {"t": 0.0, "bad": 0, "cooldown": 0.0}


def watch_color(img):
    try:
        _watch_color_impl(img)
    except Exception as e:
        CW["bad"] = 0
        print("[watchdog] 内部异常已忽略:", e, flush=True)


def _watch_color_impl(img):
    now = time.time()
    if now < CW["t"] + 1.0 or now < CW["cooldown"]:
        return
    CW["t"] = now
    try:
        rgb = img.convert("RGB").crop((img.width // 4, img.height // 4,
                                       img.width * 3 // 4, img.height * 3 // 4))
        px = list(rgb.resize((24, 24)).getdata())
        r = sum(p[0] for p in px) / len(px)
        g = sum(p[1] for p in px) / len(px)
        b = sum(p[2] for p in px) / len(px)
    except Exception:
        return
    if b - r > 40 and b - g > 40:
        CW["bad"] += 1
    else:
        CW["bad"] = 0
        return
    if CW["bad"] >= 3:
        CW["bad"] = 0
        CW["cooldown"] = now + 12.0
        if not G.get("photo_pending") and send_cmd(b"r"):
            print("[watchdog] 检测到蓝紫色偏, 已下发传感器重建", flush=True)
            info = ui.get("info")
            if info is not None:
                info.config(text="检测到色彩异常(紫屏), 已自动重建摄像头")


def show_on_video(img):
    """等比缩放铺进视频区并刷新 Label。"""
    watch_color(img)
    vw = max(ui["video"].winfo_width(), 320)
    vh = max(ui["video"].winfo_height(), 240)
    img = img.copy()
    img.thumbnail((vw, vh))
    ph = ImageTk.PhotoImage(img)
    lb = ui["video"]
    lb.config(image=ph, width=img.width, height=img.height)
    lb.image = ph                             # 防 GC 回收


def handle_photo(raw, w, h):
    G["photo_until"] = 0.0
    fn = save_jpeg_bytes(raw, "photo", w, h)
    print("已保存照片:", fn, flush=True)
    try:
        img = Image.open(io.BytesIO(raw)).convert("RGB")
    except Exception as e:
        # 存盘已成功; 仅解码失败不让 Tk 回调链崩掉
        finish_photo(True)
        ui["info"].config(text=f"已保存但预览解码失败: {e}")
        return
    show_on_video(img)
    finish_photo(True)
    ui["info"].config(text=f"已保存照片 {fn}")


def snapshot():
    last = ui.get("last_frame")
    if not last:
        return
    fmt, raw, w, h = last
    os.makedirs(PHOTO_DIR, exist_ok=True)
    if fmt == 1:
        fn = save_jpeg_bytes(raw, "snap", w, h)
    else:
        img = Image.frombytes("RGB", (w, h), convert_rgb(raw, w, h))
        fn = os.path.join(PHOTO_DIR, "snap_%s_%dx%d.png" % (time.strftime("%Y%m%d_%H%M%S"), w, h))
        img.save(fn)
    ui["info"].config(text=f"已存图 {fn}")
    print("已存图:", fn, flush=True)


def close():
    G["stop"].set()
    try:
        ui["root"].destroy()
    except Exception:
        pass


display_q = queue.Queue(maxsize=2)


def decode_worker(stop_ev):
    """后台解码线程: 从 prev_q 取最新帧解码为 PIL Image, 放入 display_q。
    将 JPEG 解码(VGA 下 15-30ms)移出 tkinter 主线程, 消除 UI 响应卡顿。"""
    while not stop_ev.is_set():
        latest = None
        try:
            while True:
                latest = prev_q.get(timeout=0.1)
        except queue.Empty:
            pass
        if latest is None:
            continue
        fmt, raw, w, h = latest
        ui["last_frame"] = latest          # 供 snapshot() 使用(GIL 下原子赋值)
        try:
            if fmt == 1:
                img = Image.open(io.BytesIO(raw)).convert("RGB")
            else:
                img = Image.frombytes("RGB", (w, h), convert_rgb(raw, w, h))
            # display_q 丢旧保新: 解码慢时只保留最新一帧
            try:
                while not display_q.empty():
                    display_q.get_nowait()
            except queue.Empty:
                pass
            display_q.put(img)
        except Exception as e:
            print(f"解码异常: {e}", flush=True)


def render_loop(root):
    stop_ev = G["stop"]

    # 1) 拍照帧优先处理(不可丢)
    try:
        while True:
            raw, w, h = photo_q.get_nowait()
            handle_photo(raw, w, h)
    except queue.Empty:
        pass

    # 2) 拍照互斥超时释放
    if G["photo_until"] and time.time() > G["photo_until"]:
        G["photo_until"] = 0.0
        finish_photo(False)

    # 3) 预览: 从后台解码线程取最新已解码帧(解码已移出主线程, 避免 UI 卡顿)
    latest_img = None
    try:
        while True:
            latest_img = display_q.get_nowait()
    except queue.Empty:
        pass
    if latest_img:
        show_on_video(latest_img)

    # 4) 状态栏节流刷新(4Hz)
    now = time.time()
    if now - ui.get("_st", 0) >= 0.25:
        ui["_st"] = now
        n, byt = window_stats()
        fps = n / STAT_WINDOW
        kbps = (byt / 1024) / STAT_WINDOW
        mode = G["mode"]
        res = ui["res_cb"].get()
        connected = G["tcp"] is not None
        alive = connected and (now - G["last_good"] < RECONNECT_AFTER)
        badge = ui["badge"]
        if alive:
            badge.config(text=" ● 实时 ", bg="#0a0")
        elif connected:
            badge.config(text=" ◐ 等帧 ", bg="#c60")
        else:
            badge.config(text=" ○ 未连 ", bg="#a33")
        note = ""
        if now < G["photo_until"]:
            note = " | 拍照中…"
        ui["info"].config(text=f"{mode} {res}  {fps:5.1f} fps  {kbps:4.0f} KB/s  "
                               f"帧计 {G['frames']} 丢帧 {G['drops']}{note}")

    if not stop_ev.is_set():
        root.after(15, render_loop, root)


def main():
    root = tk.Tk()
    build_ui(root)
    th = threading.Thread(target=reader, args=(G["stop"],), daemon=True)
    th.start()
    th_dec = threading.Thread(target=decode_worker, args=(G["stop"],), daemon=True)
    th_dec.start()
    root.after(200, render_loop, root)
    root.mainloop()
    G["stop"].set()
    print("退出", flush=True)


if __name__ == "__main__":
    main()
