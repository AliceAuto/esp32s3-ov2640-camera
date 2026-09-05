# -*- coding: utf-8 -*-
"""色彩异常看门狗。

OV2640 的 DSP 寄存器状态可能被写坏(白平衡预设表与部分模块不兼容), 表现为整幅
蓝紫色偏(实测白桌 R185/G142/B254)且无法通过 w0 恢复, 只有固件 'r' 完整重建才痊愈。
监测中心区域均值: B 同时高出 R/G 达阈值并持续 ~3s -> 触发 on_violation 回调。
"""
import time


class ColorWatchdog:
    """持续采样画面中心区域, 检测到持续性蓝紫偏时触发回调。

    feed(img) 每帧调用即可; 内部按 interval 节流, 异常吞掉不抛出。
    """

    def __init__(self, on_violation, *, interval=1.0, bad_thresh=3,
                 b_over=40, cooldown=12.0):
        self.on_violation = on_violation
        self.interval = interval
        self.bad_thresh = bad_thresh
        self.b_over = b_over
        self.cooldown = cooldown
        self._t = 0.0
        self._bad = 0
        self._cooldown = 0.0

    def feed(self, img):
        try:
            self._feed(img)
        except Exception as e:
            self._bad = 0
            print("[watchdog] 内部异常已忽略:", e, flush=True)

    def _feed(self, img):
        now = time.time()
        if now < self._t + self.interval or now < self._cooldown:
            return
        self._t = now
        try:
            rgb = img.convert("RGB").crop((img.width // 4, img.height // 4,
                                           img.width * 3 // 4, img.height * 3 // 4))
            # 取中心区域 24x24 采样, 直接按通道切片求和(C 级), 不用 getdata() 逐像素
            buf = rgb.resize((24, 24)).tobytes()
            n = len(buf) // 3
            r = sum(buf[0::3]) / n
            g = sum(buf[1::3]) / n
            b = sum(buf[2::3]) / n
        except Exception:
            return
        if b - r > self.b_over and b - g > self.b_over:
            self._bad += 1
        else:
            self._bad = 0
            return
        if self._bad >= self.bad_thresh:
            self._bad = 0
            self._cooldown = now + self.cooldown
            self.on_violation()
