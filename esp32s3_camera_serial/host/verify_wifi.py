# -*- coding: utf-8 -*-
"""
WiFi 端到端无头验证：预览帧率(QQVGA/QVGA) + 一键拍照(UXGA)。
先让本机连接到板子的热点 "OV2640Cam"（无密码），再运行：
    python verify_wifi.py [host] [port]    默认 192.168.4.1 3333
"""
import sys, time, socket, re, os, io
from PIL import Image

host = sys.argv[1] if len(sys.argv) > 1 else "192.168.4.1"
port = int(sys.argv[2]) if len(sys.argv) > 2 else 3333
FB_END = b"FRAME_END\n"
_FRAME = re.compile(rb'FRAME_BEGIN\nSIZE (\d+) (\d+) (\S+)\n(.*?)FRAME_END\n', re.S)
out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "photos")


def bigendian_rgb(frame):
    n = len(frame) // 2
    rgb = bytearray(n * 3)
    j = 0
    for o in range(0, len(frame), 2):
        v = (frame[o] << 8) | frame[o + 1]
        r = (v >> 11) & 0x1F; g = (v >> 5) & 0x3F; b = v & 0x1F
        rgb[j] = (r * 255 + 15) // 31
        rgb[j + 1] = (g * 255 + 31) // 63
        rgb[j + 2] = (b * 255 + 15) // 31
        j += 3
    return bytes(rgb)


def body_to_image(fmt, w, h, body):
    """按格式标记把帧体转成 RGB PIL 图。fmt='JPG' 走 JPEG 解码，否则 RGB565 big-endian。"""
    if fmt == b"JPG":
        return Image.open(io.BytesIO(body)).convert("RGB")
    return Image.frombytes("RGB", (w, h), bigendian_rgb(body))


def measure_fps(s, seconds=5.0):
    """统计一段时间内收到的完整帧数(以 FRAME_END 计数)。"""
    buf = b""
    cnt = 0
    t0 = time.time()
    while time.time() - t0 < seconds:
        try:
            d = s.recv(65536)
        except socket.timeout:
            continue
        if not d:
            break
        buf += d
        cnt += buf.count(FB_END)
        buf = b""  # 丢弃已统计部分(帧头帧尾计数足够)
    return cnt / seconds


def get_photo(s, timeout=40):
    """发 'P' 拍一张高清，收取并返回 (fmt, w, h, body)。JPEG 按实际 len 返回；RGB565 按 w*h*2。"""
    s.sendall(b"P")
    buf = b""
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            d = s.recv(65536)
        except socket.timeout:
            continue
        if not d:
            break
        buf += d
        m = _FRAME.search(buf)
        if not m:
            if len(buf) > 1 << 20:   # 防呆：长串无帧则丢一部分
                buf = buf[-65536:]
            continue
        w, h, fmt = int(m.group(1)), int(m.group(2)), m.group(3)
        body = m.group(4)
        if fmt == b"JPG":
            if body.endswith(b"\xff\xd9"):        # 完整的 JPEG 以 FFD9 结尾
                return w, h, fmt, body
            buf = buf[m.end():]
            continue
        # RGB565：需要 w*h*2 字节
        if len(body) >= w * h * 2:
            return w, h, fmt, body[: w * h * 2]
        buf = buf[m.end():]  # 未塞满则继续收(丢弃前一帧头, 简化)
    return None


def main():
    print(f"连接 WiFi 板 {host}:{port} ...", flush=True)
    s = socket.create_connection((host, port), timeout=5)
    s.settimeout(0.3)
    print("已连接", flush=True)

    # 初始 QQVGA 预览帧率
    print("测量 QQVGA 预览帧率(5s)...", flush=True)
    qqvga = measure_fps(s, 5.0)
    print(f"  QQVGA预览 ~{qqvga:.1f} fps", flush=True)

    # 切 QVGA 预览
    s.sendall(b"Vt")
    time.sleep(1)
    print("测量 QVGA 预览帧率(5s)...", flush=True)
    qvga = measure_fps(s, 5.0)
    print(f"  QVGA预览 ~{qvga:.1f} fps", flush=True)

    # 拍照 UXGA
    print("拍 UXGA ...", flush=True)
    res = get_photo(s)
    if res is None:
        print("拍照失败：未收到高清帧", flush=True)
    else:
        w, h, fmt, body = res
        os.makedirs(out_dir, exist_ok=True)
        img = body_to_image(fmt, w, h, body)
        path = os.path.join(out_dir, f"wifi_photo_{time.strftime('%H%M%S')}_{fmt.decode().lower()}_{w}x{h}.png")
        img.save(path)
        print(f"  已保存 {fmt.decode()} {w}x{h} ({len(body)}B) -> {path}", flush=True)

    s.close()
    print("完成", flush=True)


if __name__ == "__main__":
    main()