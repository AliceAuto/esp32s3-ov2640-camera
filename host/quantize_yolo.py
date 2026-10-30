# -*- coding: utf-8 -*-
"""YOLO11n ONNX INT8 量化 — 用真实监控画面校准。

用法: python quantize_yolo.py
"""
import os, sys, time, glob
import numpy as np
import onnx
import onnxruntime as ort
from onnxruntime.quantization import (quantize_static, quantize_dynamic,
    QuantType, QuantFormat, CalibrationDataReader, CalibrationMethod)
from PIL import Image

sys.path.insert(0, ".")
from lib.detector import Detector

FP32  = "models/yolo11n.onnx"
INT8S = "models/yolo11n-int8-static.onnx"

# ---- 模型信息 ----
print("=== 原始模型 (FP32) ===")
print(f"  大小: {os.path.getsize(FP32)/1048576:.1f} MB")
sess_tmp = ort.InferenceSession(FP32, providers=["CPUExecutionProvider"])
INP_NAME = sess_tmp.get_inputs()[0].name
print(f"  输入: {INP_NAME} {sess_tmp.get_inputs()[0].shape}")

# ---- 加载真实校准图像 ----
calib_files = sorted(glob.glob("calib_images/frame_*.jpg"))
print(f"\n校准图像: {len(calib_files)} 张真实监控画面")

class RealCalib(CalibrationDataReader):
    """用真实摄像头帧做校准 — letterbox 到 640x640, 归一化到 [0,1]。"""
    def __init__(self, files):
        self.files = files
        self.idx = 0
        self._data = []
        # 预处理所有校准图
        for f in files:
            img = Image.open(f).convert("RGB")
            w, h = img.size
            r = min(640 / w, 640 / h)
            rw, rh = round(w * r), round(h * r)
            img_rs = img.resize((rw, rh), Image.BILINEAR)
            canvas = Image.new("RGB", (640, 640), (114, 114, 114))
            canvas.paste(img_rs, ((640 - rw) // 2, (640 - rh) // 2))
            arr = np.asarray(canvas, dtype=np.float32) / 255.0
            arr = arr.transpose(2, 0, 1)[None]  # 1,3,640,640
            self._data.append(arr)
        print(f"  预处理完成: {len(self._data)} 样本")

    def get_next(self):
        if self.idx >= len(self._data):
            return None
        d = self._data[self.idx]
        self.idx += 1
        return {INP_NAME: d}

# ---- 静态 INT8 量化 (真实校准) ----
print("\n=== 静态 INT8 量化 (QDQ, per-channel, 真实校准) ===")
calib = RealCalib(calib_files)
t0 = time.time()
quantize_static(FP32, INT8S, calib,
    quant_format=QuantFormat.QDQ, per_channel=True,
    weight_type=QuantType.QInt8, activation_type=QuantType.QUInt8,
    calibrate_method=CalibrationMethod.MinMax)
dt_q = time.time() - t0
sz_f = os.path.getsize(FP32)  / 1048576
sz_i = os.path.getsize(INT8S) / 1048576
print(f"  量化耗时: {dt_q:.1f}s")
print(f"  大小: {sz_f:.1f} MB -> {sz_i:.1f} MB  (-{(1-sz_i/sz_f)*100:.0f}%)")

# ---- 测速 (100 次平均) ----
def bench(path, runs=100):
    s = ort.InferenceSession(path, providers=["CPUExecutionProvider"])
    n = s.get_inputs()[0].name
    x = np.zeros((1,3,640,640), dtype=np.float32)
    for _ in range(10): s.run(None, {n: x})       # 预热
    t0 = time.perf_counter()
    for _ in range(runs): s.run(None, {n: x})
    return (time.perf_counter() - t0) / runs * 1000

t_fp32 = bench(FP32)
t_int8 = bench(INT8S)
print(f"\n=== 测速 (100 次平均) ===")
print(f"  FP32: {t_fp32:.1f} ms/帧  ({1000/t_fp32:.0f} fps)")
print(f"  INT8: {t_int8:.1f} ms/帧  ({1000/t_int8:.0f} fps)  加速 {t_fp32/t_int8:.2f}x")

# ---- 精度对比 (真实监控画面) ----
print("\n=== 精度对比 (真实监控画面 detect_test.jpg) ===")
if os.path.exists("detect_test.jpg"):
    img = Image.open("detect_test.jpg").convert("RGB")

    d1 = Detector(conf=0.35); d1.load()
    r1 = d1.detect(img)

    d2 = Detector(conf=0.35)
    s2 = ort.InferenceSession(INT8S, providers=["CPUExecutionProvider"])
    d2.sess = s2; d2.inp_name = s2.get_inputs()[0].name; d2.loaded = True
    r2 = d2.detect(img)

    print(f"  FP32: {len(r1)} 目标")
    for r in r1:
        print(f"    {r['name']} {r['conf']:.3f}  {[int(v) for v in r['box']]}")
    print(f"  INT8: {len(r2)} 目标")
    for r in r2:
        print(f"    {r['name']} {r['conf']:.3f}  {[int(v) for v in r['box']]}")
    if r1 and r2:
        dc = r2[0]["conf"] - r1[0]["conf"]
        print(f"  置信度变化: {r1[0]['conf']:.3f} -> {r2[0]['conf']:.3f}  (Δ={dc:+.3f})")
    elif r1 and not r2:
        print("  ⚠ INT8 丢失目标! 校准仍不充分。")
else:
    print("  (detect_test.jpg 不存在, 跳过)")

# ---- 多帧精度对比 ----
print("\n=== 多帧精度对比 (校准集前 10 帧中含人的) ===")
fp_hits = 0; int8_hits = 0; tested = 0
for f in calib_files[:10]:
    img = Image.open(f).convert("RGB")
    r1 = d1.detect(img)
    r2 = d2.detect(img)
    tested += 1
    fp_p = sum(1 for r in r1 if r["name"] == "person")
    i8_p = sum(1 for r in r2 if r["name"] == "person")
    fp_hits += fp_p; int8_hits += i8_p
    if fp_p or i8_p:
        print(f"  {os.path.basename(f)}: FP32={fp_p}人 INT8={i8_p}人")
print(f"  合计 {tested} 帧: FP32={fp_hits}人  INT8={int8_hits}人")

print(f"\n量化模型: {INT8S}")
print("完成。")
