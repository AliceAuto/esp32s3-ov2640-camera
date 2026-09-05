# -*- coding: utf-8 -*-
"""核心运行层: 运行时配置 + 共享状态 + 帧路由 + 命令下发 + 统计。

main.py 解析 CLI 后把配置写入本模块(TRANSPORT/HOST/PORT/...), reader/gui 从这里读取,
避免循环导入。共享状态 G 与各队列是 reader 线程 / 解码线程 / GUI 主线程间通信的唯一通道。
"""
import queue
import socket
import threading
import time
from collections import deque

from lib import frame_parser
from lib import protocol as proto
from lib import rgb565 as _rgb565

# ---- 运行时配置(由 main.py 解析 CLI 后写入) ----
TRANSPORT = "lan"              # relay=强制中继; lanrelay=局域网优先回退; lan=仅局域网
HOST = "192.168.4.1"
PORT = 3333
RELAY_SPEC = "8.148.79.85:8001"
DEV_ID = None
RES_CODE = "t"                 # 当前预览分辨率码(GUI 下拉选择为准)
RECONNECT_AFTER = 4.0          # 预览停流超此秒数 -> TCP 重连并重下发命令
STAT_WINDOW = 5.0              # 帧率统计窗口(秒)
# 预览分辨率上限(命令码)。之前为防大帧断流把上限压到 VGA, 但用户需要切到最高画质,
# 现将上限放开到最高档 UXGA('u'), 所有分辨率码均放行, 不再被钳制。
# 代价: UXGA/XGA 大帧在跨网中继下会卡(1~2fps 甚至停流回退), 属带宽墙的物理限制。
PREVIEW_MAX_CODE = "u"         # 预览最大档位: u=UXGA(1600x1200), 即不限制
PHOTO_DIR = None               # 照片/存图目录(main.py 计算后写入 host/photos)

# ---- 共享状态(主线程读、reader 线程写, 均为单赋值原子操作) ----
G = {
    "mode": "UDP",
    "tcp": None,
    "relay": None,
    "stop": threading.Event(),
    "last_good": 0.0,          # 最近一次完整预览帧到达时刻
    "photo_until": 0.0,        # 拍照互斥截止时刻
    "res_grace_until": 0.0,    # 分辨率切换宽限期: 此期间不触发停流重连(固件全量重建需 0.5~1s)
    "switch_pending": False,   # 切换命令已下发, 等待切换后首帧(用于徽章显示"切换中")
    "switch_last_good": 0.0,   # 切换前的 last_good 快照: 新帧时间戳超过它即视为切换完成
    "frames": 0,               # 完整帧总数(samples 是定长滚动窗, 不能当累计用)
    "drops": 0,                # 按帧序号缺口估算的链路丢帧数
    "photo_pending": False,    # 已发 P 待收照片: 下一个完整 TCP 帧单元路由给照片而非预览
    "auto_q": True,            # AUTOQ 状态跟踪: 与固件 g_auto_quality 保持一致(默认 ON)
    # 权威当前状态: 所有首连/重连都要把"用户 GUI 选的分辨率与手动画质"复现到固件,
    # 杜绝重连时被命令行默认值覆盖、或 relay 重连不复现导致一直停在低画质。
    "res_code": RES_CODE,      # 当前预览分辨率码(GUI 下拉选择为准)
    "quality": 10,             # 手动画质 q(auto 关时随重连复现)
}
samples = deque(maxlen=1024)         # (t, bytes) 完整帧滚动记录
up_samples = deque(maxlen=512)       # 上屏时刻滚动记录(与 samples 同 5s 窗口口径)
seq_state = {"last": None}           # 最近一次完整帧的 seq(用于丢帧估算)
tcp_rx = bytearray()                 # TCP 唯一接收缓冲: 单点读取再分发给照片/预览解析
stat_t = [time.time()]
prev_q = queue.Queue(maxsize=2)      # 只保留最新一帧的预览队列
photo_q = queue.Queue()
udp_as = proto.UdpAssembler()        # OVU UDP 分片重组器
# 合并通道(relay/lanrelay)实际落地通道: True=走了中继兜底, False=局域网直连
_relay_on = [TRANSPORT == "relay"]


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
    """big-endian RGB565 -> RGB888(共享实现, 见 lib.rgb565)。"""
    return _rgb565.convert_rgb(frame[:w * h * 2])


def udp_feed(pkt):
    """吃一个数据报重组整帧；到齐且校验通过返回 True 并入预览队列。"""
    got = udp_as.feed(pkt)
    if got is None:
        return False
    ffmt, frame, fw, fh, seq = got
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
    统一缓冲后按帧单元完整解析再分发, 从根上消除拼接损坏。切帧复用
    lib.frame_parser(与串口/中继同源), 不再维护第二份正则实现。"""
    tcp_rx.extend(data)
    frames, consumed = frame_parser.split_frames(tcp_rx)
    del tcp_rx[:consumed]
    for w_, h_, fmt, body in frames:
        if fmt == "JPG" and w_ > 0 and h_ > 0 and len(body) > 100 and body[:2] == b"\xff\xd8":
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
    udp_as.reset()
    tcp_rx.clear()


def clamp_preview_code(code):
    """把任意分辨率码钳制到预览上限(PREVIEW_MAX_CODE)以内。

    v=VGA 及以上大帧在局域网直连易停流、跨网中继帧率崩, 预览不用; 拍照通道不走
    本函数, 仍可用 UXGA。返回合法的小分辨率码(默认回退 't'=QVGA)。"""
    if code not in ("q", "t", "v", "s", "x", "u"):
        return "t"
    # 'q'<'t'<'v'<'s'<'x'<'u'
    order = "qtvsxu"
    return code if order.index(code) <= order.index(PREVIEW_MAX_CODE) else PREVIEW_MAX_CODE


def mode_cmd(code=None):
    if code:
        code = clamp_preview_code(code)
    s = ("v" + code) if code else ""
    return s + ("U" if G["mode"] == "UDP" else "N")


def _qbyte():
    return max(8, min(63, G["quality"]))


def _resync_bytes():
    """把"当前 GUI 选择"编成固件命令(重连/首连复现用)。

    只含分辨率 + (auto 关时)手动画质。中继通道不发 UDP/TCP 模式后缀(避免改动
    udp_mode 影响中继推流); 局域网直连用 mode_cmd 走各自模式后缀。"""
    buf = bytearray(b"v" + clamp_preview_code(G["res_code"]).encode())
    if not G["auto_q"]:
        buf += b"q" + bytes([_qbyte()])
    return bytes(buf)


def send_cmd(data):
    payload = data.encode() if isinstance(data, str) else data
    if _relay_on[0]:
        c = G["relay"]
        if c is None:
            print("发送失败: 无中继命令通道", flush=True)
            return False
        try:
            c.write(payload)
            return True
        except OSError as e:
            print("发送失败:", e, flush=True)
            return False
    c = G["tcp"]
    if c is None:
        print("发送失败: 无命令通道", flush=True)
        return False
    try:
        c.sendall(payload)
        return True
    except OSError as e:
        print("发送失败:", e, flush=True)
        return False


def window_stats(win=STAT_WINDOW):
    """最近 win 秒内的 (帧数, 字节数)。"""
    t0 = time.time() - win
    n = byt = 0
    for t, b in reversed(samples):
        if t < t0:
            break
        n += 1; byt += b
    return n, byt


def _stat_report():
    """每 ~3s 打一次收流统计(清屏用, 不依赖 GUI)。"""
    n, byt = window_stats()
    fps = n / STAT_WINDOW
    print(f"[stats] {n}帧/{STAT_WINDOW:.0f}s  {fps:.1f}fps  {byt / 1024:.0f}KB/s  "
          f"帧计{G['frames']} 丢帧{G['drops']}", flush=True)
