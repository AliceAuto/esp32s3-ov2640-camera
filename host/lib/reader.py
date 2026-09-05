# -*- coding: utf-8 -*-
"""收流编排: 局域网 UDP/TCP 直连、跨网中继、局域网优先自动回退中继。

supervisor(reader) 捕获任何未捕获异常并拉起重启, 稳定性优先。
所有共享状态都经 lib.core 访问(G/队列/统计/命令), 本模块只负责连接与收流循环。
"""
import socket
import time

from lib import core
from lib import netio
from lib import relay_client


def open_tcp_cmd(stop_ev):
    # 首连/重连时把"用户 GUI 当前选择"复现到固件: 分辨率按 G["res_code"](非命令行
    # 默认值), auto 关时再带上手动画质。固件分辨率是状态化的(上次会话/误触会残留,
    # 如 UXGA 1600x1200), 不发分辨率的纯 'U' 会导致固件继续发大帧、帧率掉到 2-3fps。
    buf = core.mode_cmd(core.G["res_code"]).encode()
    if not core.G["auto_q"]:
        buf += b"q" + bytes([core._qbyte()])
    return netio.open_cmd_channel(core.HOST, core.PORT, buf,
                                  stop_ev=stop_ev, tag="TCP")


def _open_lan_tcp(ip_, stop_ev):
    """局域网直连预览: 打开摄像头 TCP 3333 命令/流通道(与中继同为 FRAME_BEGIN 帧,
    frame_parser 解析)。打开后立即复现 GUI 当前状态: 'N' 让板子走 TCP 推流(而非被别
    的客户端设成 UDP 模式), 分辨率按 G["res_code"], auto 关时带手动画质。"""
    buf = ("V" + core.G["res_code"] + "N").encode()
    if not core.G["auto_q"]:
        buf += b"q" + bytes([core._qbyte()])
    return netio.open_cmd_channel(ip_, 3333, buf, retries=10,
                                  stop_ev=stop_ev, tag="局域网直连")


def _lan_session(sock, stop_ev):
    """局域网直连 TCP 收流会话(FRAME_BEGIN 帧)。断流超时即返回, 由上层回退中继。"""
    core.G["tcp"] = sock
    sock.settimeout(0)
    core._clear_rxbuffers()
    last_good = time.time()
    while not stop_ev.is_set():
        now = time.time()
        if (now - max(last_good, core.G["last_good"]) > core.RECONNECT_AFTER
                and now >= core.G["photo_until"]
                and now >= core.G["res_grace_until"]):
            print("[lan] 局域网直连停流超时, 结束本会话", flush=True)
            break
        core.pump_tcp(sock)
        if time.time() - core.stat_t[0] >= 3.0:
            core._stat_report()
            core.stat_t[0] = time.time()
        time.sleep(0.002)
    try:
        sock.close()
    except Exception:
        pass
    core.G["tcp"] = None
    core._relay_on[0] = True      # 局域网会话结束, 后续命令/重连走中继


def _reader_body(stop_ev):
    """单次收流会话；异常交给外层 supervisor 捕获重启。

    lanrelay(合并通道): 先局域网发现+直连(UDP/TCP), 不通自动回退中继兜底。
    relay: 直接走中继。"""
    if core.TRANSPORT in ("relay", "lanrelay"):
        # 中继通道: 统一走 RelayIo(收流 + 命令同一条连接), 不建 UDP / 局域网直连。
        if core.TRANSPORT == "lanrelay" and not core._relay_on[0]:
            cam = netio.discover_cameras()
            if cam:
                h = list(cam.keys())[0]
                print(f"发现 {len(cam)} 台: {cam}", flush=True)
                ip_ = h
                tok = _open_lan_tcp(ip_, stop_ev)
                if tok is not None:
                    core._relay_on[0] = False      # 局域网直连成功, 不走中继
                    _lan_session(tok, stop_ev)     # 会话结束(断流)后回退中继兜底
                    print("局域网会话结束, 回退中继兜底", flush=True)
                else:
                    print("局域网直连失败, 自动回退中继兜底", flush=True)
            else:
                print("局域网未发现摄像头, 自动回退中继兜底", flush=True)
        core._relay_on[0] = True
        core._clear_rxbuffers()
        relay = relay_client.open_relay(core.RELAY_SPEC, core.DEV_ID, stop_ev)
        if relay is None:
            return
        core.G["relay"] = relay
        core.send_cmd(core._resync_bytes())    # 首连即把 GUI 当前分辨率/手动画质复现到固件
        last_good = time.time()
        last_hb = time.time()                  # 客户端心跳保活
        while not stop_ev.is_set():
            now = time.time()
            if (now - max(last_good, core.G["last_good"]) > core.RECONNECT_AFTER
                    and now >= core.G["photo_until"]
                    and now >= core.G["res_grace_until"]):
                print("[relay] 停流超时, 重连中继...", flush=True)
                core.G["relay"] = None
                try:
                    relay.close()
                except Exception:
                    pass
                relay = relay_client.open_relay(core.RELAY_SPEC, core.DEV_ID, stop_ev)
                if relay is None:
                    break
                core.G["relay"] = relay
                core._clear_rxbuffers()
                core.send_cmd(core._resync_bytes())  # 重连后同样复现, 不会退回固件残留的低画质
                last_good = time.time()
                last_hb = time.time()
            # 客户端心跳: 服务端 handle() 读侧有 45s 超时, 中间 NAT/防火墙可能更短。
            # 每 15s 发一条 {"t":"hb"} 保活, 防止单向空闲被判定为死连接。
            if now - last_hb >= 15.0:
                try:
                    relay.write(b'{"t":"hb"}')
                except OSError:
                    pass
                last_hb = now
            data = relay.read(65536)
            if data:
                core._pump_bytes(data)
            if time.time() - core.stat_t[0] >= 3.0:
                core._stat_report()
                core.stat_t[0] = time.time()
            time.sleep(0.002)
        try:
            relay.close()
        except Exception:
            pass
        core.G["relay"] = None
        return

    # ---- 局域网直连(UDP 预览 + TCP 命令) ----
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        udp.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    except Exception:
        pass
    udp.bind(("0.0.0.0", core.PORT))
    udp.settimeout(0.05)
    print(f"UDP 已监听 :{core.PORT}", flush=True)

    tcp = open_tcp_cmd(stop_ev)
    if tcp is None:
        udp.close()
        return
    core.G["tcp"] = tcp
    tcp.settimeout(0)
    last_good = time.time()

    while not stop_ev.is_set():
        now = time.time()
        if (now - max(last_good, core.G["last_good"]) > core.RECONNECT_AFTER
                and now >= core.G["photo_until"]
                and now >= core.G["res_grace_until"]):
            print(f"[{core.G['mode']}] 停流超时, 重连重下发...", flush=True)
            old = core.G["tcp"]
            core.G["tcp"] = None
            try:
                tcp.close()
            except Exception:
                pass
            if old is not None and old is not tcp:
                try:
                    old.close()
                except Exception:
                    pass
            core._clear_rxbuffers()
            tcp = open_tcp_cmd(stop_ev)
            if tcp is None:
                break
            core.G["tcp"] = tcp
            tcp.settimeout(0)
            last_good = time.time()

        if core.G["mode"] == "UDP":
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
                if core.udp_feed(pkt):
                    completed = True
            if completed:
                last_good = time.time()
            if not got:
                time.sleep(0.002)
            if closed:
                break
            if core.G["tcp"]:
                core.pump_tcp(core.G["tcp"])
        else:
            n0 = len(core.samples)
            if core.G["tcp"]:
                core.pump_tcp(core.G["tcp"])
            if len(core.samples) != n0:
                last_good = time.time()

        if time.time() - core.stat_t[0] >= 3.0:
            core._stat_report()
            core.stat_t[0] = time.time()

    try:
        udp.close()
    except Exception:
        pass
    core.G["tcp"] = None


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
            core.G["tcp"] = None
            print("reader 异常, 2s 后自动重启收流线程", flush=True)
            if stop_ev.wait(2.0):
                return
