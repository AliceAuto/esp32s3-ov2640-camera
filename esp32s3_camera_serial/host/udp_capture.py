# -*- coding: utf-8 -*-
"""
OV2640 UDP 高帧率预览上位机 v2。

- UDP 收预览帧(丢帧不重传)，TCP 发命令/收拍照帧。
- 预览队列只保留最新一帧：解码跟不上时丢旧不滞后。
- 底部状态栏: 连接徽章 + 模式/分辨率/帧率/带宽；顶部工具栏: 传输模式、
  预览分辨率、JPEG画质滑杆、照片分辨率+拍照(防连点)、当前画面存图。

用法: python udp_capture.py --wifi [host] [port] [res_code]
      python udp_capture.py --relay 中继服务器:端口 [--id 设备ID]   # 纯跨网中继
      python udp_capture.py --lanrelay [--id 设备ID]              # 局域网优先, 失败自动回退中继
      python udp_capture.py --selftest          # 无 GUI 协议自检
快捷键: Ctrl+Q 退出, Ctrl+S 当前画面存图
"""
import sys, os, io, re, time, threading, queue, json
import socket
from collections import deque

try:
    import numpy as np
except ImportError:
    np = None

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
def label_of_code(code):
    for n, (cd, w, h) in RES_INFO.items():
        if cd == code:
            return f"{n} {w}x{h}"
    return RES_LABELS[1]

PHOTO_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "photos")
RECONNECT_AFTER = 4.0        # 预览停流超此秒数 -> TCP 重连并重下发命令
STAT_WINDOW = 5.0            # 帧率统计窗口(秒)

# ---- 共享状态(主线程读、reader 线程写, 均为单赋值原子操作) ----
G = {
    "mode": "UDP",
    "tcp": None,
    "relay": None,
    "stop": threading.Event(),
    "last_good": 0.0,        # 最近一次完整预览帧到达时刻
    "photo_until": 0.0,      # 拍照互斥截止时刻
    "frames": 0,             # 完整帧总数(samples 是定长滚动窗, 不能当累计用)
    "drops": 0,              # 按帧序号缺口估算的链路丢帧数
    "photo_pending": False,  # 已发 P 待收照片: 下一个完整 TCP 帧单元路由给照片而非预览
    "auto_q": True,          # AUTOQ 状态跟踪: 与固件 g_auto_quality 保持一致(默认 ON)
    # 权威当前状态: 所有首连/重连都要把"用户 GUI 选的分辨率与手动画质"复现到固件,
    # 杜绝重连时被命令行默认值覆盖、或 relay 重连不复现导致一直停在低画质。
    "res_code": res_code or "t",   # 当前预览分辨率码(GUI 下拉选择为准)
    "quality": 10,                 # 手动画质 q(auto 关时随重连复现)
}
samples = deque(maxlen=1024)         # (t, bytes) 完整帧滚动记录
up_samples = deque(maxlen=512)       # 上屏时刻滚动记录(与 samples 同 5s 窗口口径)
seq_state = {"last": None}           # 最近一次完整帧的 seq(用于丢帧估算)
tcp_rx = bytearray()                 # TCP 唯一接收缓冲: 单点读取再分发给照片/预览解析
stat_t = [time.time()]
prev_q = queue.Queue(maxsize=2)      # 只保留最新一帧的预览队列
photo_q = queue.Queue()
udp_ctx = {"seq": None, "cnt": 0, "w": 0, "h": 0, "fmt": 0,
           "parts": None, "got": 0}
_TCP_FRAME = re.compile(rb'FRAME_BEGIN\nSIZE (\d+) (\d+) (\w+)\n(.*?)FRAME_END\n', re.S)
# 合并通道(relay/lanrelay)实际落地通道: True=走了中继兜底, False=局域网直连
_relay_on = [TRANSPORT == "relay"]
DISCOVERY_PORT = 9999


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


def _qbyte():
    return max(8, min(63, G["quality"]))


def _resync_bytes():
    """把"当前 GUI 选择"编成固件命令(重连/首连复现用)。

    只含分辨率 + (auto 关时)手动画质。中继通道不发 UDP/TCP 模式后缀(避免改动
    udp_mode 影响中继推流); 局域网直连用 mode_cmd 走各自模式后缀。"""
    buf = bytearray(b"v" + G["res_code"].encode())
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


def discover_cameras(timeout=4.0):
    """监听 UDP 9999 广播，收集 {ip: 设备名}。摄像头入网后每 3s 广播一次。"""
    found = {}
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    except Exception:
        pass
    s.bind(("0.0.0.0", DISCOVERY_PORT))
    s.settimeout(0.5)
    end = time.time() + timeout
    while time.time() < end:
        try:
            data, _ = s.recvfrom(512)
        except socket.timeout:
            continue
        except Exception:
            break
        txt = data.decode(errors="ignore")
        if txt.startswith("OV2640|"):
            p = txt.split("|")
            if len(p) >= 3:
                found[p[2]] = p[1]
    s.close()
    return found


class RelayIo:
    """把中继服务端的"4B大端长度+payload"消息流解包成浏览器可读的数据流。

    观众端: 连中继 -> list -> watch(建立观看) -> 持续读取摄像头帧消息。
    read(n) 返回与 serial 同名的字节流(已滤掉 JSON 信令), 供 _pump_bytes 解析;
    write(b) 把命令打包成一条中继消息下发。"""
    def __init__(self, host, port, dev_id):
        self.s = socket.create_connection((host, port), timeout=8)
        self._rbuf = b""           # 原始 socket 字节(含长度前缀)
        self.buf = b""             # 解包后待消费的 payload 字节(供 read)
        self._buf_lock = threading.Lock()
        self.cam_id = None
        devs = self._handshake(dev_id)
        self.cam_id = devs["id"]
        print(f"经中继观看: {devs['id']} (via {host}:{port})", flush=True)

    def _send_json(self, o):
        b = json.dumps(o).encode()
        self.s.sendall(len(b).to_bytes(4, "big") + b)

    def _recv_exact(self, n):
        data = b""
        while len(data) < n:
            d = self.s.recv(n - len(data))
            if not d:
                raise OSError("中继连接关闭")
            data += d
        return data

    def _handshake(self, dev_id):
        self._send_json({"t": "list"})
        devs = json.loads(self._recv_exact(int.from_bytes(self._recv_exact(4), "big")))
        devs = devs.get("devices", [])
        if not devs:
            raise RuntimeError("中继上无在线摄像头")
        if dev_id:
            target = next((d for d in devs if d["id"] == dev_id or d["name"] == dev_id), None)
            if not target:
                raise RuntimeError(f"中继无设备 {dev_id}; 可用: {[d['id'] for d in devs]}")
        else:
            target = devs[0]
            print(f"中继在线: {[(d['id'], d.get('name')) for d in devs]}", flush=True)
            print(f"未指定 --id, 自动选择第一个: {target['id']}", flush=True)
        self._send_json({"t": "watch", "id": target["id"]})
        r = json.loads(self._recv_exact(int.from_bytes(self._recv_exact(4), "big")))
        if r.get("t") != "watch_ok":
            msg = r.get("msg", "")
            if msg == "unauthorized":
                raise RuntimeError("鉴权失败: 服务器要求 token, 请用 --token <口令> 指定")
            raise RuntimeError("观看请求失败: " + msg)
        return target

    def _read_one(self):
        """读一条完整消息; 无数据(超时)返回 None; 断连返回 b''。"""
        while len(self._rbuf) < 4:
            try:
                d = self.s.recv(4 - len(self._rbuf))
            except socket.timeout:
                return None
            except OSError:
                return b""
            if not d:
                return b""
            self._rbuf += d
        n = int.from_bytes(self._rbuf[:4], "big")
        self._rbuf = self._rbuf[4:]
        while len(self._rbuf) < n:
            try:
                d = self.s.recv(n - len(self._rbuf))
            except socket.timeout:
                return None
            except OSError:
                return b""
            if not d:
                return b""
            self._rbuf += d
        out, self._rbuf = self._rbuf[:n], self._rbuf[n:]
        return out

    def read(self, n):
        """返回至多 n 字节已解包数据(跳过服务端 JSON 信令); 无数据返回 b''。"""
        while True:
            with self._buf_lock:
                have = len(self.buf)
            if have:
                break
            msg = self._read_one()
            if msg is None or msg == b"":
                break
            if msg[:1] == b"{":
                continue          # 服务端信令, 跳过
            with self._buf_lock:
                self.buf += msg
                if self.buf:
                    break
        with self._buf_lock:
            out, self.buf = self.buf[:n], self.buf[n:]
        return out

    def write(self, b):
        if isinstance(b, str):
            b = b.encode()
        if not b:
            return 0
        self.s.sendall(len(b).to_bytes(4, "big") + b)
        return len(b)

    def flush(self):
        pass

    def close(self):
        try:
            self.s.close()
        except Exception:
            pass


def _relay_spec(portn):
    """拆中继服务端 "host:port" -> (host, port)。"""
    host, _, p = _RELAY_SPEC.rpartition(":")
    return (host or "127.0.0.1", int(p or portn or 8001))


def open_relay(stop_ev):
    """连接中继服务端并完成 watch 握手, 返回流式 RelayIo。失败逐次重试。"""
    for _ in range(60):
        if stop_ev.is_set():
            return None
        try:
            host, p = _relay_spec(_RELAY_SPEC)
            return RelayIo(host, p, _ID)
        except Exception as e:
            print("中继连接失败:", e, flush=True)
            time.sleep(1.0)
    return None


def open_tcp_cmd(stop_ev):
    # 首连/重连时把"用户 GUI 当前选择"复现到固件: 分辨率按 G["res_code"](非命令行
    # 默认值), auto 关时再带上手动画质。固件分辨率是状态化的(上次会话/误触会残留,
    # 如 UXGA 1600x1200), 不发分辨率的纯 'U' 会导致固件继续发大帧、帧率掉到 2-3fps。
    buf = mode_cmd(G["res_code"]).encode()
    if not G["auto_q"]:
        buf += b"q" + bytes([_qbyte()])
    for _ in range(600):
        if stop_ev.is_set():
            return None
        try:
            c = socket.create_connection((host, port), timeout=4)
            c.settimeout(0.05)
            c.sendall(buf)
            print(f"TCP 已连 {host}:{port}, 已复现 {buf!r}", flush=True)
            return c
        except OSError:
            time.sleep(1.0)
    return None


def _open_lan_tcp(ip_, stop_ev):
    """局域网直连预览: 打开摄像头 TCP 3333 命令/流通道(与中继同为 FRAME_BEGIN 帧,
    _pump_bytes 解析)。打开后立即复现 GUI 当前状态: 'N' 让板子走 TCP 推流(而非被别
    的客户端设成 UDP 模式), 分辨率按 G["res_code"], auto 关时带手动画质。"""
    buf = ("V" + G["res_code"] + "N").encode()
    if not G["auto_q"]:
        buf += b"q" + bytes([_qbyte()])
    for _ in range(10):
        if stop_ev.is_set():
            return None
        try:
            c = socket.create_connection((ip_, 3333), timeout=4)
            c.settimeout(0.05)
            c.sendall(buf)
            print(f"局域网直连已连 {ip_}:3333, 已复现 {buf!r}", flush=True)
            return c
        except OSError:
            time.sleep(1.0)
    return None


def _lan_session(sock, stop_ev):
    """局域网直连 TCP 收流会话(FRAME_BEGIN 帧)。断流超时即返回, 由上层回退中继。"""
    G["tcp"] = sock
    sock.settimeout(0)
    _clear_rxbuffers()
    last_good = time.time()
    while not stop_ev.is_set():
        now = time.time()
        if (now - max(last_good, G["last_good"]) > RECONNECT_AFTER
                and now >= G["photo_until"]):
            print("[lan] 局域网直连停流超时, 结束本会话", flush=True)
            break
        pump_tcp(sock)
        if time.time() - stat_t[0] >= 3.0:
            _stat_report()
            stat_t[0] = time.time()
        time.sleep(0.002)
    try:
        sock.close()
    except Exception:
        pass
    G["tcp"] = None
    _relay_on[0] = True      # 局域网会话结束, 后续命令/重连走中继


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


def _reader_body(stop_ev):
    """单次收流会话；异常交给外层 supervisor 捕获重启。

    lanrelay(合并通道): 先局域网发现+直连(UDP/TCP), 不通自动回退中继兜底。
    relay: 直接走中继。"""
    if TRANSPORT in ("relay", "lanrelay"):
        # 中继通道: 统一走 RelayIo(收流 + 命令同一条连接), 不建 UDP / 局域网直连。
        if TRANSPORT == "lanrelay" and not _relay_on[0]:
            cam = discover_cameras()
            if cam:
                h = list(cam.keys())[0]
                print(f"发现 {len(cam)} 台: {cam}", flush=True)
                ip_ = h
                tok = _open_lan_tcp(ip_, stop_ev)
                if tok is not None:
                    _relay_on[0] = False      # 局域网直连成功, 不走中继
                    _lan_session(tok, stop_ev)   # 会话结束(断流)后回退中继兜底
                    print("局域网会话结束, 回退中继兜底", flush=True)
                else:
                    print("局域网直连失败, 自动回退中继兜底", flush=True)
            else:
                print("局域网未发现摄像头, 自动回退中继兜底", flush=True)
        _relay_on[0] = True
        _clear_rxbuffers()
        relay = open_relay(stop_ev)
        if relay is None:
            return
        G["relay"] = relay
        send_cmd(_resync_bytes())          # 首连即把 GUI 当前分辨率/手动画质复现到固件
        last_good = time.time()
        while not stop_ev.is_set():
            now = time.time()
            if now - max(last_good, G["last_good"]) > RECONNECT_AFTER and now >= G["photo_until"]:
                print("[relay] 停流超时, 重连中继...", flush=True)
                G["relay"] = None
                try:
                    relay.close()
                except Exception:
                    pass
                relay = open_relay(stop_ev)
                if relay is None:
                    break
                G["relay"] = relay
                _clear_rxbuffers()
                send_cmd(_resync_bytes())  # 重连后同样复现, 不会退回固件残留的低画质
                last_good = time.time()
            data = relay.read(65536)
            if data:
                _pump_bytes(data)
            if time.time() - stat_t[0] >= 3.0:
                _stat_report()
                stat_t[0] = time.time()
            time.sleep(0.002)
        try:
            relay.close()
        except Exception:
            pass
        G["relay"] = None
        return

    # ---- 局域网直连(UDP 预览 + TCP 命令) ----
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
            _stat_report()
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
from lib import gui_style as Gs

ui = {}          # 控件句柄/渲染状态集中存放


def build_ui(root):
    ui["root"] = root
    root.title(f"OV2640 实时预览 [{host}:{port}]")
    Gs.apply_theme(root)
    root.minsize(720, 540)

    # ---- 工具栏(深色面板底, 通栏) ----
    bar = ttk.Frame(root, style="Bar.TFrame"); bar.pack(fill="x")
    tb = ttk.Frame(bar, style="Bar.TFrame"); tb.pack(fill="x", padx=8, pady=6)

    Gs.section_label(tb, "传输:").pack(side="left")
    mode_cb = ttk.Combobox(tb, values=["UDP", "TCP"], state="readonly", width=6)
    mode_cb.set("UDP"); mode_cb.pack(side="left", padx=(4, 12))
    ui["mode_cb"] = mode_cb

    Gs.section_label(tb, "预览:").pack(side="left")
    # width 需容纳 "UXGA 1600x1200" 这类带尺寸标签, 否则选中框显示被截断
    res_cb = ttk.Combobox(tb, values=RES_LABELS, state="readonly", width=14)
    res_cb.set(label_of_code(G["res_code"])); res_cb.pack(side="left", padx=(4, 12))
    ui["res_cb"] = res_cb

    # AUTOQ 与手动互斥的 UI 同步(先定义, 画质栏初始化时立即调用)
    def _sync_auto_ui():
        ui["q_scale"].state(["disabled"] if G["auto_q"] else ["!disabled"])
        ui["q_lbl"].state(["disabled"] if G["auto_q"] else ["!disabled"])

    Gs.section_label(tb, "画质:").pack(side="left")
    # AUTOQ 复选框: 勾选=固件自适应接管(滑杆禁用); 取消=滑杆手动接管, 两者严格互斥
    auto_var = tk.BooleanVar(value=G["auto_q"])
    auto_cb = ttk.Checkbutton(tb, text="自动", variable=auto_var,
                              command=lambda: on_auto_q(auto_var.get()))
    auto_cb.pack(side="left", padx=(4, 0))
    ui["auto_var"], ui["auto_cb"] = auto_var, auto_cb
    q_var = tk.IntVar(value=G["quality"])
    q_scale = ttk.Scale(tb, from_=8, to=40, variable=q_var, length=110)
    q_scale.pack(side="left", padx=(4, 2))
    q_lbl = ttk.Label(tb, text=str(q_var.get()), width=3)
    q_lbl.pack(side="left", padx=(0, 12))
    ui["q_var"], ui["q_lbl"], ui["q_scale"] = q_var, q_lbl, q_scale
    _sync_auto_ui()

    Gs.section_label(tb, "拍照:").pack(side="left")
    photo_cb = ttk.Combobox(tb, values=RES_LABELS, state="readonly", width=14)
    photo_cb.set(res_label("UXGA")); photo_cb.pack(side="left", padx=(4, 4))
    ui["photo_cb"] = photo_cb

    photo_btn = ttk.Button(tb, text="拍照", style="Photo.TButton",
                           command=lambda: take_photo())
    photo_btn.pack(side="left", padx=(6, 4))
    ui["photo_btn"] = photo_btn

    snap_btn = ttk.Button(tb, text="存图(Ctrl+S)", command=snapshot)
    snap_btn.pack(side="left")

    # ---- 视频区(固定 Canvas: 画面居中适配, 不随帧变化撑动窗口) ----
    # 原用 Label 且每帧 config(width=画面宽, height=画面高): Label 的 requested
    # size 随图像变化 -> 窗口被反复撑大/缩小("画面逐渐自动变化大小")。Canvas 的
    # 尺寸由用户拖窗决定, 画面在其中等比缩放居中, 不会反馈改变窗口布局。
    video = Gs.video_canvas(root, 640, 480)
    video.pack(padx=8, pady=6, fill="both", expand=True)
    ui["video"] = video

    # ---- 状态栏(深色面板底) ----
    foot = ttk.Frame(root, style="Bar.TFrame"); foot.pack(fill="x")
    fb = ttk.Frame(foot, style="Bar.TFrame"); fb.pack(fill="x", padx=8, pady=6)
    badge = Gs.badge(fb, " 连接中 ")
    badge.pack(side="left")
    info = ttk.Label(fb, text="-", anchor="w")
    info.pack(side="left", fill="x", expand=True, padx=10)
    tip = ttk.Label(fb, text="Ctrl+Q 退出", foreground=Gs.FG_HINT)
    tip.pack(side="right")
    ui["badge"], ui["info"] = badge, info

    # ---- 事件绑定 ----
    def on_mode(_ev=None):
        G["mode"] = mode_cb.get()
        G["res_code"] = code_of(res_cb.get())   # 记住当前选择, 供重连复现
        _clear_rxbuffers()
        if G["mode"] == "TCP":
            prev_q.queue.clear()          # 切 TCP 清掉 UDP 残留画面
        cmd = mode_cmd(G["res_code"])
        if send_cmd(cmd):
            print(f"模式切换 -> {G['mode']}: {cmd!r}", flush=True)
    mode_cb.bind("<<ComboboxSelected>>", on_mode)

    def on_res(_ev=None):
        G["res_code"] = code_of(res_cb.get())   # 记住当前选择, 供重连复现
        cmd = mode_cmd(G["res_code"])
        if send_cmd(cmd):
            print(f"分辨率 -> {res_cb.get()}: {cmd!r}", flush=True)
    res_cb.bind("<<ComboboxSelected>>", on_res)

    # 拖动中不发，松手才发一次（固件按单字节二进制值解析画质）
    def on_q_release(_ev=None):
        v = max(8, min(40, int(q_var.get())))
        q_lbl.config(text=str(v))
        G["quality"] = v                        # 记住手动画质, 供重连复现
        if send_cmd(bytes([v])):
            print(f"quality -> {v}", flush=True)
    q_scale.bind("<ButtonRelease-1>", on_q_release)

    # AUTOQ 与手动互斥: 勾选=自动接管(禁用滑杆), 取消=手动接管。
    # 固件 'a' 是 toggle: 借助 G["auto_q"] 状态跟踪, 仅在真正需要翻转时才发一次,
    # 保证本地复选框与固件 g_auto_quality 保持同步, 不会因重发/toggle 漂移。
    def on_auto_q(want_on):
        if want_on != G["auto_q"] and send_cmd(b"a"):
            G["auto_q"] = want_on
            print(f"AUTOQ -> {'ON' if want_on else 'OFF'}", flush=True)
        _sync_auto_ui()

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
        # 直接取 RGB888 连续缓冲按通道切片求和(C 级): 原用 getdata() 逐像素构造
        # Python 元组, 每秒刷一次 DeprecationWarning 且慢一个量级(Pillow 14 将移除)。
        buf = rgb.resize((24, 24)).tobytes()
        n = len(buf) // 3
        r = sum(buf[0::3]) / n
        g = sum(buf[1::3]) / n
        b = sum(buf[2::3]) / n
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


_photo = [None]      # PhotoImage 单例缓存


_canvas_item = [None]    # Canvas 图像 item 句柄(复用, 不随帧重建)


def show_on_video(img):
    """等比缩放铺进视频区居中显示, 不改变视频区/窗口尺寸。

    修复: 原实现每次上屏都 config Label 的 width/height = 画面缩放尺寸,
    Label 的 requested size 随图像变化 -> 窗口被反复撑大/缩小(画面逐渐
    自动变化大小)。改用 Canvas: 尺寸由用户拖窗决定, 画面等比缩放居中,
    不反馈改变窗口布局。
    高帧率优化(与 live_capture 的 render 对齐):
      1) 缩放显式指定重采样: 放大 2x 以上用 NEAREST(最省), 其余 BILINEAR。
      2) 复用 PhotoImage + canvas item: 同尺寸 paste 0.60ms/帧 vs 新建 1.10ms,
         且不产生 Tk 本机对象 GC 压力。
    注意: PIL 的 ImageTk.PhotoImage 只有 paste(), 没有 tkinter.PhotoImage 的 put()。
    """
    watch_color(img)
    vw = max(ui["video"].winfo_width(), 320)
    vh = max(ui["video"].winfo_height(), 240)
    scale = min(vw / max(img.width, 1), vh / max(img.height, 1))
    nw = max(1, int(round(img.width * scale)))
    nh = max(1, int(round(img.height * scale)))
    if (nw, nh) == (img.width, img.height):
        disp = img
    elif nw >= img.width * 2:
        disp = img.resize((nw, nh), Image.NEAREST)
    else:
        disp = img.resize((nw, nh), Image.BILINEAR)
    ph = _photo[0]
    if ph is None or ph.width() != nw or ph.height() != nh:
        ph = ImageTk.PhotoImage(disp)         # 尺寸变化才重建(Tk 本机资源分配)
        _photo[0] = ph
    else:
        ph.paste(disp)                        # 同尺寸: 只换像素, 不新建对象
    cv = ui["video"]
    if _canvas_item[0] is None:
        _canvas_item[0] = cv.create_image(vw // 2, vh // 2, image=ph)
    else:
        cv.coords(_canvas_item[0], vw // 2, vh // 2)   # 用户拖窗后画面保持居中
        cv.itemconfig(_canvas_item[0], image=ph)
    cv.image = ph                             # 防 GC 回收


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
    将 JPEG 解码(VGA 下 15-30ms)移出 tkinter 主线程, 消除 UI 响应卡顿。

    关键: 取帧必须【非阻塞】。此前内层用 prev_q.get(timeout=0.1), 队列一空
    就要阻塞满 100ms 才交付本轮解码, 解码吞吐被锁死在 ≈1/0.1 = 10fps,
    无论链路多少帧上屏都上不去。改为 nowait 取空 + 短 sleep 让出 GIL。"""
    while not stop_ev.is_set():
        latest = None
        while True:
            try:
                latest = prev_q.get_nowait()    # 取到最后一个即最新帧
            except queue.Empty:
                break
        if latest is None:
            time.sleep(0.001)                   # 无帧: 让出 GIL, 不空转抢 Tk
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
            # 与 live_capture 对齐: 解码完主动让出 GIL。PIL 解码/ImageTk 都持有
            # CPython GIL, 解码线程若一直占锁, Tk 主线程的 render 轮不到时间片
            # -> 上屏掉帧。1ms 让出不明显降低解码吞吐(理论仍 >500fps)。
            time.sleep(0.001)
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
        up_samples.append(time.time())

    # 4) 状态栏节流刷新(4Hz)
    now = time.time()
    if now - ui.get("_st", 0) >= 0.25:
        ui["_st"] = now
        n, byt = window_stats()
        fps = n / STAT_WINDOW
        kbps = (byt / 1024) / STAT_WINDOW
        while up_samples and up_samples[0] < now - STAT_WINDOW:
            up_samples.popleft()
        fps_up = len(up_samples) / STAT_WINDOW
        mode = G["mode"]
        res = ui["res_cb"].get()
        connected = G["tcp"] is not None
        alive = connected and (now - G["last_good"] < RECONNECT_AFTER)
        badge = ui["badge"]
        if alive:
            badge.config(text=" ● 实时 ", bg=Gs.OK)
        elif connected:
            badge.config(text=" ◐ 等帧 ", bg=Gs.WARN)
        else:
            badge.config(text=" ○ 未连 ", bg=Gs.DANGER)
        note = ""
        if now < G["photo_until"]:
            note = " | 拍照中…"
        ui["info"].config(text=f"{mode} {res}  链路 {fps:5.1f} | 上屏 {fps_up:5.1f} fps  "
                               f"{kbps:4.0f} KB/s  帧计 {G['frames']} 丢帧 {G['drops']}{note}")

    if not stop_ev.is_set():
        # 自适应节奏: 刚画过帧(说明还有后续帧)则紧跟 2ms 贴近出流;
        # 空闲时退回 30ms 低频扫描, 减少与解码线程的 GIL 争用。
        root.after(2 if latest_img else 30, render_loop, root)


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
