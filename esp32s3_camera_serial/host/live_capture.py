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
import serial
from PIL import Image, ImageTk
import tkinter as tk

# ---- 传输通道选择 ----
#   串口:   python live_capture.py COM7 [baud]
#   WiFi:   python live_capture.py --wifi [host] [port]   (host 默认 192.168.4.1)
#   自动:   python live_capture.py --auto                 (UDP广播发现后直连局域网摄像头)
#   中继:   python live_capture.py --relay 中继服务器:端口 [--id 设备ID]
_FLAGS = ("--wifi", "--auto", "--relay", "--id")
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

_pos = [x for x in sys.argv[1:] if x not in _FLAGS and x != _auto_id]

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
PREVIEW_CHOICES = ["QQVGA", "QVGA", "VGA"]      # 预览可选
PHOTO_CHOICES = ["VGA", "SVGA", "XGA", "UXGA"]  # 拍照可选
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


_FRAME_RE = re.compile(rb'FRAME_BEGIN\nSIZE (\d+) (\d+) (\w+)\n(.*?)FRAME_END\n', re.S)


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
        self._p2p_sock = None
        self._p2p_thread = None
        self._p2p_stop = threading.Event()
        self._p2p_active = False
        self._p2p_last_rx = 0.0
        self._p2p_peer = None
        self._p2p_start = 0.0
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
        # 先建 P2P UDP 套接字并上报自身公网端点(STUN), 再 watch。
        # 这样服务器在 watch 时已拿到本端 UDP 端点, 撮合给对端。
        self._open_p2p_sock()
        self._send_stun(target["id"])
        self._send_json({"t": "watch", "id": target["id"]})
        r = json.loads(self._recv_exact(int.from_bytes(self._recv_exact(4), "big")))
        if r.get("t") != "watch_ok":
            raise RuntimeError("观看请求失败: " + r.get("msg", ""))
        return target

    def _open_p2p_sock(self):
        """创建 P2P UDP 套接字(绑定随机端口), 供 STUN 上报与后续打洞收流共用。"""
        if self._p2p_sock is not None:
            return
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.settimeout(0.05)
            s.bind(("0.0.0.0", 0))
            self._p2p_sock = s
        except OSError as e:
            print("P2P UDP 初始化失败:", e, flush=True)

    def _send_stun(self, cam_id):
        """用 P2P 套接字向服务器 UDP 端口上报自身公网端点(打洞辅助)。"""
        if self._p2p_sock is None:
            return
        try:
            stun = json.dumps({"t": "stun", "id": cam_id, "role": "viewer"}).encode()
            self._p2p_sock.sendto(stun, (self._relay_host, self._relay_udp_port))
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
                # P2P 直连已建立: 优先消费 UDP 重组帧; 长时间无 UDP 则回退中继
                if self._p2p_last_rx and (time.time() - self._p2p_last_rx) > 3.0:
                    print("P2P 链路超时, 回退中继", flush=True)
                    self._stop_p2p()
                    continue
                time.sleep(0.002)
                continue
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
        """处理中继下发的 JSON 信令; 目前只关心 p2p_peer(打洞直连)。"""
        try:
            o = json.loads(msg.decode("utf-8", "ignore"))
        except Exception:
            return
        if o.get("t") == "p2p_peer":
            ip = o.get("ip")
            port = o.get("port")
            if ip and port:
                self._start_p2p(ip, int(port))

    def _start_p2p(self, ip, port):
        """开启 P2P: 复用握手时建的 UDP 套接字, 向对端打洞 + 后台收流重组。"""
        if self._p2p_sock is None:
            self._open_p2p_sock()
        if self._p2p_sock is None:
            return
        self._p2p_stop.clear()
        self._p2p_peer = (ip, port)
        self._p2p_start = time.time()
        # 打洞: 持续向对端公网端点发包, 让双方 NAT 建立映射
        self._p2p_thread = threading.Thread(target=self._p2p_loop, daemon=True)
        self._p2p_thread.start()
        print(f"P2P 打洞中 -> {ip}:{port}", flush=True)

    def _p2p_loop(self):
        """后台线程: 向对端打洞 + 收 UDP 数据报重组为 FRAME 文本流喂给 self.buf。"""
        s = self._p2p_sock
        peer = self._p2p_peer
        punch = json.dumps({"t": "punch"}).encode()
        last_punch = 0.0
        last_stun = 0.0
        while not self._p2p_stop.is_set():
            now = time.time()
            # 打洞包: 前 2s 每 100ms 一发, 之后每 500ms 保活
            if now - last_punch >= (0.1 if now - self._p2p_start < 2.0 else 0.5):
                try:
                    s.sendto(punch, peer)
                except OSError:
                    pass
                last_punch = now
            # STUN 周期保活: 维持服务器上本端(viewer)的反射端点。
            # 摄像头打洞失败切 TURN 后, 帧由服务器转向本端点; 端点过期则收不到。
            if now - last_stun >= 10.0:
                self._send_stun(self.cam_id)
                last_stun = now
            try:
                pkt, _ = s.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            if len(pkt) < 12 or pkt[:3] != b"OVU":
                continue   # 非视频数据报(如对端打洞回包), 忽略
            self._p2p_last_rx = time.time()
            self._feed_udp_frame(pkt)
        self._p2p_active = False

    def _feed_udp_frame(self, pkt):
        """把 12 字节头 + 分片重组为完整帧, 转成 FRAME 文本流追加到 self.buf。"""
        seq = (pkt[4] << 8) | pkt[5]
        cnt = pkt[6] + 1
        idx = pkt[7]
        w = (pkt[8] << 8) | pkt[9]
        h = (pkt[10] << 8) | pkt[11]
        fmt = pkt[3]
        ctx = self._p2p_ctx
        if ctx["seq"] != seq:
            ctx.update(seq=seq, cnt=cnt, w=w, h=h, fmt=fmt,
                       parts=[None] * cnt, got=0)
        if idx >= ctx["cnt"] or ctx["parts"][idx] is not None:
            return
        ctx["parts"][idx] = pkt[12:]
        ctx["got"] += 1
        if ctx["got"] != ctx["cnt"]:
            return
        frame = b"".join(ctx["parts"])
        fw, fh, ffmt = ctx["w"], ctx["h"], ctx["fmt"]
        ctx.update(seq=None, cnt=0, w=0, h=0, fmt=0, parts=None, got=0)
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
        if self._p2p_sock is not None:
            try:
                self._p2p_sock.close()
            except Exception:
                pass
            self._p2p_sock = None
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
        if _FRAME_RE.search(s) is None:
            if len(buf) > 8 << 20:
                del buf[: (4 << 20)]
            continue
        resync_pending = last_frame_time == 0.0
        pos = 0
        while True:
            m = _FRAME_RE.search(s, pos)
            if not m:
                break
            w, h = int(m.group(1)), int(m.group(2))
            fmt = m.group(3).decode('ascii', 'ignore')      # JPG 或 RGB
            body = m.group(4)
            fb_len = w * h * 2
            end = m.end()
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
            pos = end
        buf = bytearray(s[pos:])
        # 重连(或首次)拿到首帧后，把界面选择的分辨率重新下发给板子，避免错位
        if resync_pending:
            resync_board()


# ---- 命令：切换预览/拍照分辨率 ----
def set_preview(name):
    code = [c for c, r in RES.items() if r[0] == name]
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
    code = [c for c, r in RES.items() if r[0] == name]
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
    code = [c for c, r in RES.items() if r[0] == name][0]
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
        scale = max(1, 640 // max(w, 1))
        big = img.resize((w * scale, h * scale), Image.NEAREST)
        photo_image = ImageTk.PhotoImage(big)
        label.config(image=photo_image)
        label.image = photo_image
        fps_lbl.config(text=f"帧数 {frame_count[0]} | 预览 {w}x{h}")
    root.after(30, render)


root = tk.Tk()
root.title(f"OV2640 实时预览+动态拍照 [{port}@{baud}]")

top = tk.Frame(root); top.pack(fill="x", padx=6, pady=6)
tk.Label(top, text="预览:").pack(side="left")
preview_var = tk.StringVar(value="QVGA")
tk.OptionMenu(top, preview_var, *PREVIEW_CHOICES, command=set_preview).pack(side="left", padx=3)
tk.Label(top, text="拍照:").pack(side="left", padx=(12, 0))
photo_var = tk.StringVar(value="UXGA")
tk.OptionMenu(top, photo_var, *PHOTO_CHOICES, command=set_photo).pack(side="left", padx=3)
btn = tk.Button(top, text="拍 照", command=do_photo,
                bg="#2d8", activebackground="#1b6", font=("", 12, "bold"))
btn.pack(side="right")

status_lbl = tk.Label(root, text="连接中...", anchor="w", padx=6, fg="#666")
status_lbl.pack(fill="x")
label = tk.Label(root, bg="black")
label.pack()
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