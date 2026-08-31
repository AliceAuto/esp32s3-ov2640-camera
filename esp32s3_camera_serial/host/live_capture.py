# -*- coding: utf-8 -*-
"""
OV2640 实时预览 + 动态分辨率切换 + 拍照。
- 顶部下拉可随时切换【预览分辨率】(QQVGA/QVGA/VGA) 和【拍照分辨率】(VGA/SVGA/XGA/UXGA)。
- 界面实时显示预览（big-endian RGB565 解析，平滑放大显示）。
- 点"拍照": 固件切到所选拍照分辨率抓一张高清帧发回，自动保存 PNG 并回到预览。
- 数据经串口 2Mbps 传输。预览分辨率越高，帧率越低。

用法: python live_capture.py COM7 [baud]
快捷键: Ctrl+Q 退出
"""
import sys, time, threading, queue, os, re, socket, io, json
from select import select
import serial
from PIL import Image, ImageTk
import tkinter as tk
try:
    import numpy as _np
except ImportError:
    _np = None

# ---- 传输通道选择 ----
#   串口:   python live_capture.py COM7 [baud]
#   WiFi:   python live_capture.py --wifi [host] [port]   (host 默认 192.168.4.1)
#   自动:   python live_capture.py --auto                 (UDP广播发现后直连局域网摄像头)
#   中继:   python live_capture.py --relay 中继服务器:端口 [--id 设备ID] [--token 口令]
#           (服务器若启用了 token 鉴权, 必须带 --token)
_FLAGS = ("--wifi", "--auto", "--relay", "--id", "--p2p", "--token")
_P2P = "--p2p" in sys.argv   # 默认纯 TCP 中继(可靠); 加 --p2p 才启用打洞/TURN(跨网直连提速, 不稳定)


def _flag_value(name):
    """取 --name value 形式的可选参数值; 不存在返回 None。"""
    if name in sys.argv:
        i = sys.argv.index(name)
        if i + 1 < len(sys.argv):
            return sys.argv[i + 1]
    return None


_TOKEN = _flag_value("--token")   # 中继鉴权口令, 与服务器启动参数一致
if "--relay" in sys.argv:
    MODE = "relay"
elif "--auto" in sys.argv:
    MODE = "auto"
elif "--wifi" in sys.argv:
    MODE = "wifi"
else:
    MODE = "serial"

_auto_id = None
if "--id" in sys.argv:
    i = sys.argv.index("--id")
    if i + 1 < len(sys.argv):
        _auto_id = sys.argv[i + 1]

# 位置参数 = 既不是开关名、也不是开关取值的那些 argv(如中继地址 "host:port")
_flag_values = {_auto_id, _TOKEN}
_pos = [x for x in sys.argv[1:] if x not in _FLAGS and x not in _flag_values]

DISCOVERY_PORT = 9999
if MODE == "wifi":
    port = _pos[0] if _pos else "192.168.4.1"
    baud = int(_pos[1]) if len(_pos) > 1 else 3333
elif MODE == "relay":
    port = _pos[0] if _pos else "8.148.79.85:8001"   # "host:port" 中继服务端
    baud = 0
elif MODE == "auto":
    port = None
    baud = 3333
else:
    port = _pos[0] if _pos else "COM7"
    baud = int(_pos[1]) if len(_pos) > 1 else 2000000

BEGIN = b"FRAME_BEGIN\n"
END = b"FRAME_END\n"
# 长时间没有任何字节/完整帧就强制重连串口（覆盖 MCU 死机复位的场景）
RECONNECT_AFTER = 5.0
# code -> (名称, w, h)
RES = {
    "q": ("QQVGA", 160, 120),
    "t": ("QVGA", 320, 240),
    "v": ("VGA", 640, 480),
    "s": ("SVGA", 800, 600),
    "x": ("XGA", 1024, 768),
    "u": ("UXGA", 1600, 1200),
}
def res_label(code):
    """分辨率下拉显示名: 名字带像素尺寸, 如 'UXGA 1600x1200'。"""
    name, w, h = RES[code]
    return f"{name} {w}x{h}"

PREVIEW_CHOICES = [res_label(c) for c in "qtvsxu"]  # 预览可选(全档放开)
PHOTO_CHOICES = [res_label(c) for c in "vsxu"]      # 拍照可选
out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "photos")

# 预览当前尺寸(线程共享, 用锁保护)；photo_cfg 记录当前选中的拍照分辨率
preview = {"code": "t", "w": 320, "h": 240}
photo_cfg = {"code": "u"}
photo_req = {"active": False, "w": 0, "h": 0, "rgb": None, "done": False}
q = queue.Queue()
frame_count = [0]

ser = None
wlock = threading.Lock()
state_lock = threading.Lock()
stop = threading.Event()


def convert_rgb(frame):
    """big-endian RGB565 -> RGB888。

    numpy 向量化优先(VGA 640x480 约 2ms); 无 numpy 时回退纯 Python 逐像素
    (同一帧数百 ms, 会拖垮帧率, 建议 pip install numpy)。
    """
    if _np is not None:
        a = _np.frombuffer(frame, dtype=">u2")     # 大端 uint16
        flat = _np.empty((a.size, 3), dtype=_np.uint8)
        # 舍入公式必须与纯 Python 分支一致((v*255+15)//31), 否则结果差 1
        flat[:, 0] = (((a >> 11) & 0x1F) * 255 + 15) // 31
        flat[:, 1] = (((a >> 5) & 0x3F) * 255 + 31) // 63
        flat[:, 2] = ((a & 0x1F) * 255 + 15) // 31
        return flat.tobytes()
    n = len(frame) // 2
    rgb = bytearray(n * 3)
    j = 0
    for o in range(0, len(frame), 2):
        v = (frame[o] << 8) | frame[o + 1]   # big-endian
        r = (v >> 11) & 0x1F; g = (v >> 5) & 0x3F; b = v & 0x1F
        rgb[j] = (r * 255 + 15) // 31
        rgb[j + 1] = (g * 255 + 31) // 63
        rgb[j + 2] = (b * 255 + 15) // 31
        j += 3
    return bytes(rgb)


_BEGIN = b"FRAME_BEGIN\n"
_END = b"FRAME_END\n"


def _split_frames(s):
    """从累积缓冲切出所有完整帧单元。

    返回 ([(w, h, fmt, body), ...], 已消费字节数)。
    此前用 re.S + .*? 在累积缓冲上反复回溯匹配, 高分辨率下正则开销显著;
    改为 find() 定位定界符 + 切片(与 udp_capture._pump_bytes 一致), 一次线性扫描。
    """
    out = []
    pos = 0
    while True:
        i = s.find(_BEGIN, pos)
        if i < 0:
            # 只保留可能被截断在缓冲尾部的半个 _BEGIN
            return out, max(0, len(s) - len(_BEGIN) + 1)
        nl = s.find(b"\n", i + len(_BEGIN))
        if nl < 0:
            return out, i
        e = s.find(_END, nl + 1)
        if e < 0:
            return out, i
        parts = s[i + len(_BEGIN):nl].split()
        if len(parts) >= 3 and parts[0] == b"SIZE":
            try:
                out.append((int(parts[1]), int(parts[2]),
                            parts[3].decode("ascii", "ignore"), s[nl + 1:e]))
            except ValueError:
                pass
        pos = e + len(_END)
    return out, pos


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
    """把中继服务端的"4B大端长度+payload"消息流解包成与 serial 同名的字节流。
    观众端: 连中继 -> list -> watch(建立观看) -> 持续读取摄像头帧消息。
    write(): 下发命令(打包成一条中继消息)。"""
    def __init__(self, host, port, dev_id):
        self.s = socket.create_connection((host, port), timeout=8)
        # 注意: 此刻不要 setttimeout(0.05)。握手(list/watch_ok)需要阻塞等公网回包(>50ms)。
        # 等 _handshake 完成后, 由 start_streaming() 把超时降为 0.05 供流式非阻塞读取。
        self._relay_host = host
        self._relay_udp_port = port + 1   # 服务器 UDP STUN 端口 = TCP 中继端口 + 1
        self.buf = b""     # 解包后待消费的 payload 字节(供 read)
        self._rbuf = b""   # 原始 socket 字节(含长度前缀)
        self._buf_lock = threading.Lock()   # 保护 self.buf (中继线程 + P2P 线程共用)
        # P2P 直连状态(尽力而为, 失败回退中继)
        self.cam_id = None
        # ---- 可靠 P2P 状态 ----
        self._p2p_socks = []          # 本端候选 socket 列表(多端口, 交叉打洞用)
        self._p2p_cands = []          # 对端(摄像头)候选 "ip:port" 列表
        self._p2p_thread = None
        self._p2p_stop = threading.Event()
        self._p2p_active = False
        self._p2p_last_rx = 0.0
        self._p2p_start = 0.0
        self._p2p_retry_at = 0.0      # 降级后下次请求重新撮合的时刻
        self._p2p_last_done = None    # 最近一帧完成重组的 seq(丢帧统计基准)
        self._p2p_stat_t = 0.0        # 质量统计窗口起点
        self._p2p_ok = 0              # 窗口内完整帧数
        self._p2p_lost = 0            # 窗口内按 seq 缺口估算的丢帧数
        self._p2p_ctx = {"seq": None, "cnt": 0, "w": 0, "h": 0, "fmt": 0,
                         "parts": None, "got": 0}
        devs = self._handshake(dev_id)
        self.cam_id = devs["id"]
        print(f"经中继观看: {devs['id']} (via {host}:{port})", flush=True)

    def start_streaming(self):
        """握手完成调用: 切换为流式非阻塞读取(单次超时 50ms, 让 reader 轮询/回退 P2P)。"""
        self.s.settimeout(0.05)
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
        # 先清空可能残留的旧读缓冲, 再做 list/watch 握手
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
        # 默认不做打洞: 只走可靠的 TCP 中继。只有 --p2p 时才上报 UDP 候选/打洞。
        if _P2P:
            self._open_p2p_socks()
            self._send_stun(target["id"])
        watch = {"t": "watch", "id": target["id"]}
        if _TOKEN:
            watch["token"] = _TOKEN      # 服务器启用鉴权时必带
        self._send_json(watch)
        r = json.loads(self._recv_exact(int.from_bytes(self._recv_exact(4), "big")))
        if r.get("t") != "watch_ok":
            msg = r.get("msg", "")
            if msg == "unauthorized":
                raise RuntimeError("鉴权失败: 服务器要求 token, 请用 --token <口令> 指定")
            raise RuntimeError("观看请求失败: " + msg)
        return target

    def _open_p2p_socks(self):
        """创建 P2P 候选 UDP 套接字(多端口)。

        可靠 P2P 关键点: 单候选时对端回包源端口稍有偏差(端口受限/对称 NAT)
        即打不通; 多端口全部 STUN 上报后与对端候选交叉探测, 成倍提高命中率。
        """
        if self._p2p_socks:
            return
        for _ in range(3):
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.settimeout(0.05)
                s.bind(("0.0.0.0", 0))
                self._p2p_socks.append(s)
            except OSError as e:
                print("P2P UDP 初始化失败:", e, flush=True)
                break

    def _send_stun(self, cam_id):
        """全部候选套接字各向服务器 UDP 端口上报一次(每个端口一个反射候选)。"""
        if not self._p2p_socks:
            return
        try:
            stun = json.dumps({"t": "stun", "id": cam_id, "role": "viewer"}).encode()
            for s in self._p2p_socks:
                s.sendto(stun, (self._relay_host, self._relay_udp_port))
        except Exception:
            pass
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
        while True:
            with self._buf_lock:
                have = len(self.buf)
            if have >= n:
                break
            if self._p2p_active:
                # P2P/TURN 收流中: 优先消费 UDP 重组帧; 收流线程会在
                # 失活/高丢片时自行降级(_degrade_p2p), 这里只等它退出。
                if not self._p2p_socks:
                    continue
                time.sleep(0.002)
                continue
            if _P2P and self._p2p_retry_at and time.time() >= self._p2p_retry_at:
                # 降级后的周期恢复: 请求服务器重新撮合(重发双方最新候选再打洞)。
                # 网络状况会变(NAT 映射刷新/观众换网), 不应一次失败就永久放弃。
                self._p2p_retry_at = 0.0
                try:
                    self._send_json({"t": "p2p_refresh"})
                    print("请求重新撮合 P2P...", flush=True)
                except Exception:
                    pass
            msg = self._read_one()
            if msg is None:
                break
            if not msg:
                self.close()
                break
            if msg[:1] == b"{":
                self._handle_json(msg)
                continue   # 服务端信令, 跳过
            with self._buf_lock:
                self.buf += msg
        with self._buf_lock:
            out, self.buf = self.buf[:n], self.buf[n:]
        return out

    def _handle_json(self, msg):
        """处理中继下发的 JSON 信令; 关心 p2p_peer(打洞直连)与重撮合。"""
        try:
            o = json.loads(msg.decode("utf-8", "ignore"))
        except Exception:
            return
        if o.get("t") == "p2p_peer" and _P2P:
            # 可靠 P2P: 优先取 cands 全部候选("ip:port" 列表)交叉打洞;
            # 旧服务器只有单 ip/port, 回退兼容。
            cands = o.get("cands") or []
            if not cands and o.get("ip") and o.get("port"):
                cands = [f"{o['ip']}:{o['port']}"]
            if cands:
                self._start_p2p(cands)

    def _start_p2p(self, cands):
        """开启 P2P: 全部候选 socket x 对端全部候选交叉打洞 + select 收流重组。"""
        self._open_p2p_socks()
        if not self._p2p_socks:
            return
        if self._p2p_thread and self._p2p_thread.is_alive():
            self._p2p_cands = list(cands)     # 线程在跑, 仅更新候选表(重撮合场景)
            return
        self._p2p_stop.clear()
        self._p2p_cands = list(cands)
        self._p2p_start = time.time()
        self._p2p_last_rx = time.time()
        self._p2p_stat_t = self._p2p_start
        self._p2p_ok = self._p2p_lost = 0
        self._p2p_thread = threading.Thread(target=self._p2p_loop, daemon=True)
        self._p2p_thread.start()
        print(f"P2P 交叉打洞中: {len(self._p2p_socks)}x{len(cands)} 组合 -> {cands}", flush=True)

    def _degrade_p2p(self, reason):
        """P2P 降级: 停收流回退中继, 并安排 30s 后请求重新撮合(网络会恢复)。"""
        if not self._p2p_active and not self._p2p_socks:
            return
        print(f"P2P 降级({reason}), 回退中继, 30s 后重试", flush=True)
        self._p2p_active = False
        self._p2p_retry_at = time.time() + 30.0
        self._p2p_stop.set()
        for s in self._p2p_socks:
            try:
                s.close()
            except Exception:
                pass
        self._p2p_socks = []
        self._p2p_last_rx = 0.0

    def _p2p_loop(self):
        """后台线程: 交叉打洞/双向保活 + select 多路收流 + 质量探测自动降级。"""
        punch = b"P2P"                      # 与固件约定同 magic
        last_punch = 0.0
        last_stun = 0.0
        while not self._p2p_stop.is_set():
            now = time.time()
            peers = [self._cand_ep(c) for c in self._p2p_cands]
            # 打洞包: 前 2s 每 100ms 一发(覆盖全部本端x对端组合), 之后 500ms 保活。
            # 保活是双向的: 本端持续发维持本端 NAT 映射, 摄像头侧同理。
            if peers and now - last_punch >= (0.1 if now - self._p2p_start < 2.0 else 0.5):
                for s in self._p2p_socks:
                    for ep in peers:
                        try:
                            s.sendto(punch, ep)
                        except OSError:
                            pass
                last_punch = now
            # STUN 周期保活: 维持服务器上本端全部候选的反射端点。
            # 摄像头走 TURN 时帧由服务器转向这些端点; 端点过期则收不到。
            if now - last_stun >= 10.0:
                self._send_stun(self.cam_id)
                last_stun = now
            # 多路收流: 任一 socket 收到 OVU 都重组(TURN 冗余转发按 seq 去重)
            r, _, _ = select(self._p2p_socks, [], [], 0.05)
            got = False
            for s in r:
                try:
                    pkt, _ = s.recvfrom(2048)
                except OSError:
                    continue
                got = True
                if len(pkt) >= 3 and pkt[:3] == b"P2P":
                    continue                # 对端打洞/保活包: 活性由 got 体现
                if len(pkt) < 12 or pkt[:3] != b"OVU":
                    continue
                self._p2p_last_rx = time.time()
                self._feed_udp_frame(pkt)
            if got:
                self._p2p_last_rx = time.time()
            else:
                # 活性判定: 已建立直连后 3s 无任何对端包 -> 链路死亡
                if self._p2p_active and self._p2p_last_rx \
                        and now - self._p2p_last_rx > 3.0:
                    self._degrade_p2p("3s 无对端数据")
                    return
            # 质量探测: 每 5s 结算丢帧率, 持续高丢片(链路半死)也降级回中继
            if now - self._p2p_stat_t >= 5.0:
                total = self._p2p_ok + self._p2p_lost
                if total >= 20 and self._p2p_lost / total > 0.4:
                    self._degrade_p2p(
                        f"丢帧率 {self._p2p_lost * 100 // max(total, 1)}% 过高")
                    return
                self._p2p_stat_t = now
                self._p2p_ok = self._p2p_lost = 0
        self._p2p_active = False

    @staticmethod
    def _cand_ep(c):
        """'ip:port' -> (ip, port)；非法返回 None。"""
        try:
            ip, _, port = c.rpartition(":")
            return (ip, int(port))
        except Exception:
            return None

    def _feed_udp_frame(self, pkt):
        """把 12 字节头 + 分片重组为完整帧, 转成 FRAME 文本流追加到 self.buf。

        冗余去重: TURN 会对全部候选端点转发, 同一片可能从多个 socket 到达;
        已完成帧的副本直接丢弃, 重组内按 idx 去重。"""
        seq = (pkt[4] << 8) | pkt[5]
        cnt = pkt[6] + 1
        idx = pkt[7]
        w = (pkt[8] << 8) | pkt[9]
        h = (pkt[10] << 8) | pkt[11]
        fmt = pkt[3]
        if seq == self._p2p_last_done:      # 已完成帧的冗余副本
            return
        ctx = self._p2p_ctx
        if ctx["seq"] != seq:
            # 新帧: 以"最近完成帧"为基准统计缺口(TURN 冗余不会虚增丢帧)
            if self._p2p_last_done is not None:
                gap = (seq - self._p2p_last_done - 1) & 0xFFFF
                if 0 < gap < 4096:
                    self._p2p_lost += gap
            ctx = {"seq": seq, "cnt": cnt, "w": w, "h": h, "fmt": fmt,
                   "parts": [None] * cnt, "got": 0}
            self._p2p_ctx = ctx
        if idx >= ctx["cnt"] or ctx["parts"][idx] is not None:
            return
        ctx["parts"][idx] = pkt[12:]
        ctx["got"] += 1
        if ctx["got"] != ctx["cnt"]:
            return
        frame = b"".join(ctx["parts"])
        fw, fh, ffmt = ctx["w"], ctx["h"], ctx["fmt"]
        self._p2p_last_done = seq
        self._p2p_ok += 1
        self._p2p_ctx = {"seq": None, "cnt": 0, "w": 0, "h": 0, "fmt": 0,
                         "parts": None, "got": 0}
        if ffmt == 1:
            ok = len(frame) > 100 and frame[:2] == b"\xff\xd8" and frame[-2:] == b"\xff\xd9"
            fname = "JPG"
        else:
            ok = fw > 0 and fh > 0 and len(frame) == fw * fh * 2
            fname = "RGB"
        if not ok:
            return
        blob = b"FRAME_BEGIN\nSIZE %d %d %s\n" % (fw, fh, fname.encode()) + frame + b"FRAME_END\n"
        with self._buf_lock:
            self.buf += blob
        if not self._p2p_active:
            self._p2p_active = True
            print(f"P2P 直连已建立 ({fw}x{fh} {fname})", flush=True)

    def _stop_p2p(self):
        self._p2p_stop.set()
        for s in self._p2p_socks:
            try:
                s.close()
            except Exception:
                pass
        self._p2p_socks = []
        self._p2p_active = False
        self._p2p_last_rx = 0.0
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
        self._stop_p2p()
        try:
            self.s.close()
        except Exception:
            pass
    def reset_input_buffer(self):
        self.buf = b""
        self._rbuf = b""
        try:
            self.s.setblocking(False)
            while True:
                if not self.s.recv(65536):
                    break
        except Exception:
            pass
        finally:
            try:
                self.s.setblocking(True)
            except Exception:
                pass


def open_serial():
    """创建传输连接：串口用 serial.Serial，WiFi/UDP-发现 用 socket 封装成同名接口。
    返回 None 表示重试超时(上层 stop)。"""
    if MODE == "auto":
        print("正在局域网自动发现摄像头 (UDP:9999)...", flush=True)
        cam = discover_cameras()
        if not cam:
            print("未发现摄像头。确认已与摄像头同一 WiFi 并已入网。", flush=True)
            return None
        host = list(cam.keys())[0]
        print(f"发现 {len(cam)} 台: {cam}", flush=True)
        return _make_socket_io(host, 3333)
    if MODE == "relay":
        host, _, portn = port.rpartition(":")
        for _ in range(60):
            if stop.is_set():
                return None
            try:
                ro = RelayIo(host or "127.0.0.1", int(portn or 8000), _auto_id)
                ro.start_streaming()   # 握手完成, 切换为流式非阻塞
                return ro
            except Exception as e:
                print("中继连接失败:", e, flush=True)
                time.sleep(1.0)
        return None
    if MODE == "wifi":
        return _make_socket_io(port, baud)
    for _ in range(300):
        if stop.is_set():
            return None
        try:
            s = serial.Serial(port, baud, timeout=0.05)
            s.dtr = False
            s.rts = False
            s.reset_input_buffer()
            return s
        except Exception:
            time.sleep(0.5)
    return None


def _make_socket_io(host, portn):
    """TCP 直连摄像头(WiFi/UDP-发现共用)。返回 TcpIo 或 None。"""
    for _ in range(60):
        if stop.is_set():
            return None
        try:
            s = socket.create_connection((host, portn), timeout=4)
            s.settimeout(0.05)
            return TcpIo(s)
        except Exception:
            time.sleep(1.0)   # WiFi: 板子可能还在复位/开机, 慢速重试
    return None


class TcpIo:
    """把 socket 包装成与 serial.Serial 同名接口(read/write/flush/close/reset_input_buffer)。"""
    def __init__(self, s):
        self.s = s
    def read(self, n):
        try:
            return self.s.recv(n)
        except socket.timeout:
            return b""
    def write(self, b):
        if isinstance(b, str):
            b = b.encode()
        if not b:
            return 0
        try:
            self.s.sendall(b)
            return len(b)
        except OSError:
            raise
    def flush(self):
        pass
    def close(self):
        try:
            self.s.close()
        except Exception:
            pass
    def reset_input_buffer(self):
        """丢弃当前已收到但未处理的字节。"""
        try:
            self.s.setblocking(False)
            while True:
                if not self.s.recv(65536):
                    break
        except Exception:
            pass
        finally:
            try:
                self.s.setblocking(True)
            except Exception:
                pass


def resync_board():
    """MCU 复位后会恢复默认分辨率，这里在读到首帧后把界面当前选择重新下发对齐。"""
    with state_lock:
        pcode = preview.get("code", "q")
        hcode = photo_cfg.get("code", "u")
    try:
        with wlock:
            if ser:
                ser.write(("V" + pcode + "H" + hcode).encode())
                ser.flush()
        print(f"重同步下发 V{pcode} H{hcode}", flush=True)
    except Exception:
        pass


def reader():
    global ser
    ser = open_serial()
    if ser is None:
        stop.set()
        return
    print("串口已打开", flush=True)
    buf = bytearray()
    last_frame_time = 0.0
    while not stop.is_set():
        try:
            data = ser.read(16384)
        except Exception as e:
            print("串口异常, 尝试重连:", e, flush=True)
            try:
                ser.close()
            except Exception:
                pass
            ser = None
            ser = open_serial()
            if ser is None:
                stop.set()
                return
            print("串口已重连", flush=True)
            buf = bytearray()
            last_frame_time = 0.0
            continue
        if not data:
            # 心跳：MCU 死机复位会导致若干秒无数据，超时则重开串口强制重同步
            if last_frame_time and (time.time() - last_frame_time) > RECONNECT_AFTER:
                print("长时间无帧, 强制重连串口", flush=True)
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None
                ser = open_serial()
                if ser is None:
                    stop.set()
                    return
                buf = bytearray()
                last_frame_time = 0.0
            time.sleep(0.002)
            continue
        buf += data
        s = bytes(buf)
        frames, consumed = _split_frames(s)
        if not frames:
            if len(buf) > 8 << 20:
                del buf[: (4 << 20)]
            continue
        resync_pending = last_frame_time == 0.0
        for w, h, fmt, body in frames:
            fb_len = w * h * 2
            with state_lock:
                pw, ph = preview["w"], preview["h"]
                want_photo = photo_req.get("active")
                pw_, ph_ = photo_req["w"], photo_req["h"]
            ok = False
            frame_rgb = None
            if fmt == "JPG":
                # JPEG: 长度不定，直接整体解码（校验 JPEG 头尾）
                if len(body) > 100 and body[:2] == b"\xff\xd8" and body[-2:] == b"\xff\xd9":
                    ok = True
                    try:
                        frame_rgb = Image.open(io.BytesIO(body)).convert("RGB")
                    except Exception:
                        ok = False
            else:
                # RGB565 big-endian: 长度需等于 w*h*2
                ok = len(body) >= fb_len
                if ok:
                    frame_rgb = Image.frombytes("RGB", (w, h),
                                                convert_rgb(bytes(body[:fb_len])))
            if ok and frame_rgb:
                if (w, h) == (pw, ph):
                    q.put((frame_rgb, w, h))
                    frame_count[0] += 1
                elif want_photo and (w, h) == (pw_, ph_):
                    with state_lock:
                        photo_req["rgb"] = frame_rgb
                        photo_req["done"] = True
                        photo_req["active"] = False
                    print(f"收到照片帧 {w}x{h} {fmt}", flush=True)
                last_frame_time = time.time()
        buf = bytearray(s[consumed:])
        # 重连(或首次)拿到首帧后，把界面选择的分辨率重新下发给板子，避免错位
        if resync_pending:
            resync_board()


# ---- 命令：切换预览/拍照分辨率 ----
def set_preview(name):
    code = [c for c, r in RES.items() if name.split()[0] == r[0]]  # 'UXGA 1600x1200' -> 'UXGA'
    if not code:
        return
    code = code[0]
    with state_lock:
        preview["code"] = code
        preview["w"], preview["h"] = RES[code][1], RES[code][2]
    try:
        with wlock:
            ser.reset_input_buffer()
            ser.write(("V" + code).encode())
            ser.flush()
        print(f"预览分辨率已切换: {name}", flush=True)
    except Exception as e:
        print("切换预览失败:", e, flush=True)


def set_photo(name):
    code = [c for c, r in RES.items() if name.split()[0] == r[0]]
    if not code:
        return
    code = code[0]
    with state_lock:
        photo_cfg["code"] = code
    try:
        with wlock:
            ser.write(("H" + code).encode())
            ser.flush()
        print(f"拍照分辨率已设为: {name} (下次拍照生效)", flush=True)
    except Exception as e:
        print("设置拍照分辨率失败:", e, flush=True)


def do_photo():
    with state_lock:
        photo_req.update(active=True, w=preview_w_for_photo(), h=preview_h_for_photo(),
                         rgb=None, done=False)
    try:
        with wlock:
            ser.reset_input_buffer()
            ser.write(b'P')
            ser.flush()
        print("已发送拍照指令, 等待高清帧...", flush=True)
        root.after(200, poll_photo)
    except Exception as e:
        print("拍照异常:", e, flush=True)


def preview_w_for_photo():
    # 拍照尺寸 = 当前选中的"拍照分辨率"
    return cur_photo_size()[0]


def preview_h_for_photo():
    return cur_photo_size()[1]


def cur_photo_size():
    name = photo_var.get()
    code = [c for c, r in RES.items() if name.split()[0] == r[0]][0]
    return RES[code][1], RES[code][2]


def poll_photo(_=None):
    with state_lock:
        done = photo_req.get("done")
        rgb = photo_req.get("rgb")
        w, h = photo_req["w"], photo_req["h"]
    if done and rgb:
        os.makedirs(out_dir, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        path = os.path.join(out_dir, f"photo_{ts}_{w}x{h}.png")
        rgb.save(path)
        print(f"已保存照片 {w}x{h} -> {path}", flush=True)
        status_lbl.config(text=f"已保存 {w}x{h} -> {os.path.basename(path)}")
        with state_lock:
            photo_req["rgb"] = None
        return
    root.after(200, poll_photo)


def render():
    try:
        data = q.get_nowait()
    except queue.Empty:
        data = None
    if data:
        img, w, h = data
        # 固定显示框内"只缩小不放大": 大分辨率等比缩小, 小分辨率按原生像素居中。
        # 分辨率与画面大小无关, 切分辨率只影响清晰度, 绝不把画面放大。
        scale = min(1.0, DISP_W / max(w, 1), DISP_H / max(h, 1))
        nw, nh = max(1, int(round(w * scale))), max(1, int(round(h * scale)))
        disp = img.resize((nw, nh), Image.LANCZOS)
        photo_image = ImageTk.PhotoImage(disp)
        canvas.delete("all")
        canvas.create_image(DISP_W // 2, DISP_H // 2, image=photo_image)
        canvas.image = photo_image
        fps_lbl.config(text=f"帧数 {frame_count[0]} | 预览 {w}x{h} -> 显示 {nw}x{nh}")
    root.after(30, render)


root = tk.Tk()
root.title(f"OV2640 实时预览+动态拍照 [{port}@{baud}]")

top = tk.Frame(root); top.pack(fill="x", padx=6, pady=6)
tk.Label(top, text="预览:").pack(side="left")
preview_var = tk.StringVar(value=res_label("t"))
tk.OptionMenu(top, preview_var, *PREVIEW_CHOICES, command=set_preview).pack(side="left", padx=3)
tk.Label(top, text="拍照:").pack(side="left", padx=(12, 0))
photo_var = tk.StringVar(value=res_label("u"))
tk.OptionMenu(top, photo_var, *PHOTO_CHOICES, command=set_photo).pack(side="left", padx=3)
btn = tk.Button(top, text="拍 照", command=do_photo,
                bg="#2d8", activebackground="#1b6", font=("", 12, "bold"))
btn.pack(side="right")

status_lbl = tk.Label(root, text="连接中...", anchor="w", padx=6, fg="#666")
status_lbl.pack(fill="x")
DISP_W, DISP_H = 640, 480   # 固定显示框: 任何分辨率等比缩放后居中, 画面大小不变只更清晰
canvas = tk.Canvas(root, width=DISP_W, height=DISP_H, bg="black", highlightthickness=0)
canvas.pack()
fps_lbl = tk.Label(root, text="", anchor="w", padx=6)
fps_lbl.pack(fill="x")

root.bind("<Control-q>", lambda e: (stop.set(), root.destroy()))
th = threading.Thread(target=reader, daemon=True)
th.start()
print("reader 线程已启动, 等待串口...", flush=True)
root.after(30, render)
root.mainloop()
stop.set()
if ser:
    try:
        ser.close()
    except Exception:
        pass
print("GUI 已退出", flush=True)