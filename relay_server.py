# -*- coding: utf-8 -*-
"""
OV2640 跨网信令 + 中继服务端。
================================
作用: 当摄像头和终端不在同一局域网时, 通过本公网服务端完成:
  1. 设备在线注册 / 心跳 / 列表查询   (信令)
  2. 终端与摄像头之间的双向字节转发   (中继)

架构: 
  摄像头(CAM)与终端(VIEWER)都以 TCP 连接到本服务端。
  - CAM 上线后发 reg 把自己注册为一个在线设备, 并保持此连接作为数据上行通道。
  - VIEWER 发 list 查在线设备, 发 watch <id> 请求观看某台摄像头。
  - SERVER 在 CAM 与 VIEWER 两条 TCP 连接之间建立双向转发: 
       摄像头帧上行(->viewer 收)  终端命令下行(->camera 收)。
  - 数据是纯字节透传, 服务端不解析视频帧, 因此与固件现有 FRAME/SIZE/END 协议解耦。

协议:                     长度前缀 + payload
  所有消息为: 4字节大端长度 L + L 字节 payload。
  payload 是 UTF-8 文本时: 必须以 JSON 格式出现(首字符 '{')。
  payload 是二进制时:     非 JSON(首字符非 '{'), 视为视频/命令字节流, 只中继不解析。

消息(JSON):
  CAM  -> SRV:  {"t":"reg", "id":"cam-xxx", "name":"客厅摄像头"}
  CAM  -> SRV:  {"t":"hb"}                                   (心跳)
  VIEWER-> SRV: {"t":"list"}
  SRV  -> VIEWER: {"t":"devices", "devices":[{"id","name","online","since"}]}
  VIEWER-> SRV: {"t":"watch", "id":"cam-xxx"}
  SRV  -> VIEWER: {"t":"watch_ok"}  或 {"t":"error","msg":"..."}
  SRV  -> CAM:   {"t":"viewer_attached"}      (有观众接入, CAM 开始推流)
  SRV  -> CAM:   {"t":"viewer_left"}
  SRV  -> CAM:   {"t":"p2p_peer","ip","port"} (对端观众公网端点, 供 UDP 打洞直连)
  SRV  -> VIEWER:{"t":"p2p_peer","ip","port"} (对端摄像头公网端点, 供 UDP 打洞直连)
  之后 CAM/VIEWER 之间发起的非 JSON 字节将双向透传(见上)。
  P2P 直连为"尽力而为": 打洞成功则视频改走 UDP 直连, 失败仍走本中继(TCP)。

用法:  python relay_server.py [port] [token]
默认端口 8000。需在云服务器防火墙/安全组放行 TCP 8000 (+UDP port+1)。
可选 token: 设置后 reg/watch 都必须携带相同 token, 防止任何人列举 id 即可观看
(此前的服务端完全无鉴权 —— 知道或猜到 cam id 就能看, 端口扫描器也能白嫖带宽)。
也可用环境变量 RELAY_TOKEN 传入。留空 = 不鉴权(兼容旧客户端/内网自用)。
"""
import sys, threading, time, json, socket, traceback, queue, os
from collections import OrderedDict

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8000
TOKEN = (sys.argv[2] if len(sys.argv) > 2 else "") or os.environ.get("RELAY_TOKEN", "")

# 每个连接的出站队列深度(单位: 条消息)。视频帧走 UDP 之外的 TCP 中继时,
# 一条消息就是一整帧(可达数十 KB)。队列化后慢观众只积压自己的队列,
# 不再把 sendall 的阻塞反压到摄像头的接收线程 -> 摄像头 TCP 窗口关窗 -> sender_task 停摆。
VIDEO_QUEUE_MAX = 8

# 可靠 P2P: 每端候选 socket 数上限。双方各开多个 UDP 端口并全部 STUN 上报,
# 打洞时交叉探测 N x M 个组合 —— 单候选对端口受限/对称 NAT 基本打不通。
P2P_MAX_CANDS = 4

# ---------------------------------------------------------------------------
# 连接对象: 持有 socket + 身份(device / viewer) + 转发对端
# ---------------------------------------------------------------------------
class Conn:
    def __init__(self, sock, addr, srv):
        self.sock = sock
        self.addr = addr
        self.srv = srv
        self.buf = b""
        self.device_id = None      # 若是摄像头: 注册 id
        self.is_camera = False
        self.peer = None           # 中继对端连接
        self.running = True
        self.pause_peer = False    # 转发暂停标记(供接收线程用)
        # 出站队列(由独立发送线程消费): 视频帧队列满时丢最旧保最新,
        # 使慢观众只影响自己, 不会把阻塞反压到摄像头
        self.out_q = queue.Queue(maxsize=VIDEO_QUEUE_MAX)
        self.out_alive = True
        self._out_thread = None
        self.name = None              # 显示名(来自 reg 消息)
        self.since = 0                # 注册时间戳
        self.bytes_relayed = 0        # 累计转发字节数(监控用)

    def _release(self):
        """真正关闭 socket —— 只在出站队列排空后调用。

        关闭时序很重要: 鉴权拒绝时若立即 shutdown, 待发的 unauthorized 提示
        会随 socket 一起消失, 对端只看到一个无原因的断连, 排查困难。
        """
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            self.sock.close()
        except OSError:
            pass

    def close(self):
        if not self.running:
            return
        self.running = False
        self.out_alive = False
        self.srv.detach(self)
        # 注意: 不能用 t.is_alive() 判断! Thread.is_alive() 在线程真正开始执行前
        # 返回 False(_started 事件由线程自身在 bootstrap 里设置)。鉴权拒绝这类
        # "accept 后立刻回复并关闭"的路径会稳定命中该窗口 -> 直接关 socket,
        # 待发的 unauthorized 提示随连接一起消失, 对端只看到无原因断连。
        # 改成"线程对象是否已创建": 出站线程任何退出路径都会自行 _release()。
        t = getattr(self, "_out_thread", None)
        if t is not None:
            try:
                # 哨兵排在已有消息之后, FIFO 保证先发完提示再关闭
                self.out_q.put_nowait(None)
            except Exception:
                self._release()
        else:
            self._release()          # 未启发送线程时直接关闭

    def start_out(self):
        self._out_thread = threading.Thread(target=self._out_loop, daemon=True)
        self._out_thread.start()

    def _out_loop(self):
        while True:
            item = self.out_q.get()
            if item is None:                  # 关闭哨兵(队列已排空)
                break
            try:
                self.sock.sendall(len(item).to_bytes(4, "big") + item)
                self.bytes_relayed += len(item) + 4
            except OSError:
                break
            if not self.out_alive and self.out_q.empty():
                break                         # 已请求关闭且队列空
        self._release()

    def send_payload(self, payload: bytes):
        """入队一条待发消息, 不阻塞调用方(摄像头接收线程)。

        JSON 信令: 体积小且必须可靠送达 -> 阻塞入队。
        视频帧(非 JSON): 体积大且实时性强 -> 队列满时丢最旧保最新。
        """
        if not self.running:
            return
        if payload[:1] == b"{":
            try:
                self.out_q.put(payload, timeout=5)
            except queue.Full:
                pass                              # 极小的信令, 满了说明对端基本已死
            return
        while True:
            try:
                self.out_q.put_nowait(payload)
                return
            except queue.Full:
                try:
                    self.out_q.get_nowait()       # 丢最旧一帧
                except queue.Empty:
                    pass

    def send_json(self, obj):
        self.send_payload(json.dumps(obj, ensure_ascii=False).encode("utf-8"))

class Server:
    def __init__(self, port, token=""):
        self.port = port
        self.token = token             # 共享口令; 空 = 不鉴权
        self.devices = OrderedDict()   # id -> Conn
        self.lock = threading.Lock()
        # P2P 打洞辅助: UDP STUN 端口(默认 port+1), 记录各端 UDP 公网端点。
        # 键: 摄像头用 "cam:<id>", 观众用 "viewer:<cam_id>"。
        self.udp_port = port + 1
        self.udp_endpoints = {}        # key -> (ip, port)
        self.udp_lock = threading.Lock()

    def detach(self, conn):
        """连接关闭时: 解除设备注册、断开中继对端。"""
        with self.lock:
            if conn.device_id and self.devices.get(conn.device_id) is conn:
                del self.devices[conn.device_id]
                self.log(f"设备离线: {conn.device_id}")
        # 通知/断开对端
        peer = conn.peer
        if peer is not None:
            if peer.is_camera and peer.running:
                peer.send_json({"t": "viewer_left"})
            peer.peer = None
            conn.peer = None
            # 对端若是观众, 直接断开; 是摄像头则保留等下一个观众
            if not peer.is_camera:
                peer.close()

    def log(self, msg):
        print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)

    # ---- 协议处理 ----
    def on_message(self, c: Conn, payload: bytes):
        if payload[:1] == b"{" and payload.rstrip()[:1] == b"{" and b"}" in payload:
            try:
                msg = json.loads(payload.decode("utf-8"))
            except Exception:
                return  # 非完整 JSON, 忽略
            self.handle_json(c, msg)
        else:
            # 二进制流: 中继给对端(观众命令上行, 摄像头数据下行)
            if c.peer is not None:
                c.peer.send_payload(payload)

    def handle_json(self, c: Conn, msg: dict):
        t = msg.get("t")
        # 观众下行命令 / 摄像头命令回执: 可靠 JSON 通道, 直接转发对端。
        # cmd: 观众 -> 摄像头(切分辨率/拍照分辨率); cmd_ack: 摄像头 -> 观众(执行回执)。
        if t in ("cmd", "cmd_ack"):
            if c.peer is not None:
                c.peer.send_json(msg)
            return
        # 鉴权: reg/watch 必须携带与服务器一致的 token。
        # 在处理任何业务前拒绝, 未授权的 reg 直接断连(防扫描器反复试探)。
        if self.token and t in ("reg", "watch") and msg.get("token") != self.token:
            self.log(f"拒绝未授权 {t} 来自 {c.addr}")
            c.send_json({"t": "error", "msg": "unauthorized"})
            if t == "reg":
                c.close()
            return
        if t == "reg":
            if c.is_camera:
                return
            c.device_id = msg.get("id") or f"cam-{c.addr[1]}"
            c.is_camera = True
            name = msg.get("name") or c.device_id
            c.name = name
            c.since = time.time()
            with self.lock:
                self.devices[c.device_id] = c
            self.log(f"设备注册: {c.device_id} name={name} from {c.addr}")
            c.send_json({"t": "reg_ok", "id": c.device_id})
        elif t == "hb":
            pass  # 连接存活本身就是心跳
        elif t == "list":
            if c.is_camera:
                return
            now = time.time()
            with self.lock:
                devs = [{"id": d.device_id,
                         "name": d.name or d.device_id,
                         "online": bool(d.running),
                         "since": int(d.since or now)} for d in self.devices.values()]
            c.send_json({"t": "devices", "devices": devs})
        elif t == "p2p_refresh":
            # 观众 P2P 失败回退中继后, 周期性请求重新撮合: 网络状况会变
            # (NAT 映射刷新/观众换网), 重新下发候选让双方再试一轮打洞。
            if c.is_camera or c.peer is None:
                return
            cam = c.peer
            self.match_p2p(cam.device_id, cam, c, why="refresh")
        elif t == "watch":
            if c.is_camera:
                return
            target_id = msg.get("id")
            with self.lock:
                cam = self.devices.get(target_id)
            if cam is None or not cam.running:
                c.send_json({"t": "error", "msg": "device offline"})
                return
            # 抢占式: 摄像头只服务一个观众
            old_peer = cam.peer
            if old_peer is not None and old_peer != c:
                with self.lock:
                    old_peer.send_json({"t": "error", "msg": "busy elsewhere"})
                old_peer.close()
            cam.peer = c
            c.peer = cam
            cam.send_json({"t": "viewer_attached"})
            c.send_json({"t": "watch_ok"})
            self.match_p2p(target_id, cam, c, why="watch")

    def match_p2p(self, target_id, cam, viewer, why=""):
        """P2P 撮合: 把双方 UDP 候选端点列表交叉下发给对方, 供打洞直连。

        可靠性设计:
        - cands 为 "ip:port" 字符串列表(本端全部候选 socket 的反射端点);
          同时保留 ip/port=主候选, 旧固件/旧观众仍可按单候选工作。
        - 观众没有任何 UDP 候选时不下发 p2p_peer: 此前用 TCP 公网端点兜底,
          但 TCP 端口与 UDP 映射无关, 打洞必失败还白耗 3s 打洞窗口。
        - 摄像头收到 viewer_udp 信令后视频立即走 TURN(不等打洞结果)。
        """
        with self.udp_lock:
            cam_cands = [f"{ip}:{port}" for ip, port in
                         (self.udp_endpoints.get(f"cam:{target_id}") or [])]
            view_cands = [f"{ip}:{port}" for ip, port in
                          (self.udp_endpoints.get(f"viewer:{target_id}") or [])]
        if not cam_cands and not view_cands:
            self.log(f"撮合跳过({why}): 双方均无 UDP 候选")
            return
        if view_cands:
            cam.send_json({"t": "p2p_peer", "ip": view_cands[0].split(":")[0],
                           "port": int(view_cands[0].rsplit(":", 1)[1]),
                           "cands": view_cands})
        if cam_cands:
            viewer.send_json({"t": "p2p_peer", "ip": cam_cands[0].split(":")[0],
                              "port": int(cam_cands[0].rsplit(":", 1)[1]),
                              "cands": cam_cands})
        # 观众有 UDP 候选 -> 摄像头视频直接走 TURN UDP, 不必先经过数秒 TCP 中继。
        # TCP 中继是队头阻塞的, 跨网帧率会被单帧 RTT 卡住。
        if view_cands:
            cam.send_json({"t": "viewer_udp"})
        self.log(f"撮合({why}) cam_cands={len(cam_cands)} view_cands={len(view_cands)}")

    def _recv_exact(self, c: Conn, n: int):
        """读满 n 字节; 返回空 bytes 表示连接已关闭/出错。"""
        data = b""
        while len(data) < n:
            try:
                chunk = c.sock.recv(n - len(data))
            except OSError:
                return b""
            if not chunk:
                return b""
            data += chunk
        return data

    def handle(self, c: Conn):
        c.sock.settimeout(45)  # 45s 无数据 = 死连接(NAT 半开), 强制断开清理
        try:
            while c.running:
                head = self._recv_exact(c, 4)
                if not head:
                    return
                ln = int.from_bytes(head, "big")
                if ln > 64 << 20:
                    return
                payload = self._recv_exact(c, ln)
                if not payload and ln:
                    return
                self.on_message(c, payload)
        finally:
            c.close()

    def accept_loop(self, listener):
        while True:
            sock, addr = listener.accept()
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            c = Conn(sock, addr, self)
            c.start_out()      # 出站发送线程(队列化, 慢对端不再反压转发链)
            threading.Thread(target=self.handle, args=(c,), daemon=True).start()

    def udp_stun_loop(self):
        """UDP 辅助端口(port+1): 三层 ICE 候选服务。
        1) STUN 反射:   {"t":"stun","id":"<cam_id>","role":"cam|viewer"} -> 记录公网端点并回显
                       (这是 server-reflexive 候选 + TURN 客户端到本机的恒等映射通道)。
        2) TURN 转发:   "T2C" 信封(观众->摄像头) / "C2V"(摄像头->观众)。
                       栈: magic(3) "T2C"/"C2V" + cam_id_len(4) + cam_id + payload。
                       服务器按 cam_id 把 payload 转向到对端真实公网 UDP 端点。
                       由于对端持续 STUN 上报保活其到本机的映射, 对称 NAT 也能送达。
        3) 保活续期:   任何 STUN/TURN 数据报都会刷新 udp_endpoints[目标]，保持映射。"""
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("0.0.0.0", self.udp_port))
        self.log(f"UDP ICE/TURN 监听 0.0.0.0:{self.udp_port} (放行 UDP {self.udp_port})")
        while True:
            try:
                data, addr = s.recvfrom(4096)
            except OSError:
                continue
            cam_id = ""
            if data[:1] == b"{":
                try:
                    msg = json.loads(data.decode("utf-8", "ignore"))
                except Exception:
                    continue
                cam_id = msg.get("id") or ""
                if msg.get("t") == "stun":
                    role = msg.get("role") or "cam"
                    key = f"cam:{cam_id}" if role == "cam" else f"viewer:{cam_id}"
                    with self.udp_lock:
                        # 可靠 P2P: 每端开多个候选 socket, 各自发一次 STUN。
                        # 端点累积成列表(去重, 滚动保留最新 P2P_MAX_CANDS 个),
                        # 打洞时交叉探测全部组合, 大幅提高受限/对称 NAT 的打通率。
                        lst = self.udp_endpoints.setdefault(key, [])
                        if (addr[0], addr[1]) not in lst:
                            lst.append((addr[0], addr[1]))
                        while len(lst) > P2P_MAX_CANDS:
                            lst.pop(0)
                    self.log(f"STUN {role} {cam_id}: {addr[0]}:{addr[1]} (cands={len(lst)})")
                    try:
                        s.sendto(json.dumps({"t": "stun_ok", "ip": addr[0],
                                             "port": addr[1]}).encode(), addr)
                    except OSError:
                        pass
                    continue
                else:
                    continue
            elif data[:3] in (b"T2C", b"C2V"):
                # 信封: magic(3) + cam_id_len(4) + cam_id + payload
                magic = data[:3]
                if len(data) < 7:
                    continue
                nl = int.from_bytes(data[3:7], "big")
                if nl <= 0 or 7 + nl > len(data):
                    continue
                cam_id = data[7:7 + nl].decode("utf-8", "ignore")
                payload = data[7 + nl:]
                target_key = f"viewer:{cam_id}" if magic == b"C2V" else f"cam:{cam_id}"
                with self.udp_lock:
                    targets = list(self.udp_endpoints.get(target_key) or [])
                if targets:
                    # 可靠 TURN: 向对端全部候选端点冗余转发。观众/摄像头多候选 socket
                    # 都在收, 重组按 (seq,idx) 去重, 冗余包无害; 任一映射仍有效即可达。
                    for t_ep in targets:
                        try:
                            s.sendto(payload, t_ep)
                        except OSError:
                            pass
                    self.log(f"TURN {magic.decode()} cam={cam_id} -> x{len(targets)} {len(payload)}B")
                else:
                    self.log(f"TURN 无对端端点 cam={cam_id} dir={magic.decode()}")
                continue
            # 忽略其他

    def status_loop(self):
        """简单 HTTP /status 端点: 返回在线设备数、活跃中继对数、UDP 端点数。"""
        import http.server
        srv_ref = self

        class Handler(http.server.BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path == "/status":
                    with srv_ref.lock:
                        n_devs = len(srv_ref.devices)
                        n_relays = sum(1 for d in srv_ref.devices.values() if d.peer is not None)
                        total_bytes = sum(d.bytes_relayed for d in srv_ref.devices.values())
                    with srv_ref.udp_lock:
                        n_eps = len(srv_ref.udp_endpoints)
                    body = json.dumps({"devices": n_devs, "active_relays": n_relays,
                                       "udp_endpoints": n_eps,
                                       "bytes_relayed": total_bytes}).encode()
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.end_headers()
                    self.wfile.write(body)
                else:
                    self.send_response(404)
                    self.end_headers()

            def log_message(self, *args):
                pass

        httpd = http.server.HTTPServer(("0.0.0.0", self.port + 2), Handler)
        self.log(f"监控端点: http://0.0.0.0:{self.port + 2}/status")
        httpd.serve_forever()

    def serve(self):
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind(("0.0.0.0", self.port))
        listener.listen(32)
        self.log(f"relay_server 监听 0.0.0.0:{self.port} (放行 TCP {self.port})")
        threading.Thread(target=self.udp_stun_loop, daemon=True).start()
        threading.Thread(target=self.status_loop, daemon=True).start()
        try:
            self.accept_loop(listener)
        except KeyboardInterrupt:
            self.log("exit")
        finally:
            listener.close()


if __name__ == "__main__":
    srv = Server(PORT, TOKEN)
    if TOKEN:
        srv.log("token 鉴权已启用 (reg/watch 需携带)")
    else:
        srv.log("警告: 未设置 token, 任何知道设备 id 的人都能观看")
    srv.serve()