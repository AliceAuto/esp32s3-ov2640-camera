# -*- coding: utf-8 -*-
"""量化对比测试: FP32 vs INT8动态 vs INT8静态(降阈值)。"""
import os, sys, time
import numpy as np
import onnxruntime as ort
from onnxruntime.quantization import quantize_dynamic, QuantType
from PIL import Image

sys.path.insert(0, ".")
from lib.detector import Detector

FP32 = "models/yolo11n.onnx"
INT8_DYN = "models/yolo11n-int8-dynamic.onnx"
INT8_STA = "models/yolo11n-int8-static.onnx"

# --- 动态量化 (如果还没有) ---
if not os.path.exists(INT8_DYN):
    print("=== 动态 INT8 量化 (weight-only) ===")
    t0 = time.time()
    quantize_dynamic(FP32, INT8_DYN, weight_type=QuantType.QInt8)
    print("量化耗时: %.1fs" % (time.time() - t0))

sz_f = os.path.getsize(FP32) / 1048576
sz_d = os.path.getsize(INT8_DYN) / 1048576
print("大小: %.1f -> %.1f MB (-%.0f%%)" % (sz_f, sz_d, (1 - sz_d / sz_f) * 100))

# --- 测速 ---
def bench(path, runs=100):
    s = ort.InferenceSession(path, providers=["CPUExecutionProvider"])
    n = s.get_inputs()[0].name
    x = np.zeros((1, 3, 640, 640), dtype=np.float32)
    for _ in range(10):
        s.run(None, {n: x})
    t0 = time.perf_counter()
    for _ in range(runs):
        s.run(None, {n: x})
    return (time.perf_counter() - t0) / runs * 1000

t_fp32 = bench(FP32)
t_dyn = bench(INT8_DYN)
print("\n=== 测速 (100次) ===")
print("  FP32:      %.1f ms (%d fps)" % (t_fp32, 1000 / t_fp32))
print("  INT8 动态: %.1f ms (%d fps)  加速 %.2fx" % (t_dyn, 1000 / t_dyn, t_fp32 / t_dyn))
if os.path.exists(INT8_STA):
    t_sta = bench(INT8_STA)
    print("  INT8 静态: %.1f ms (%d fps)  加速 %.2fx" % (t_sta, 1000 / t_sta, t_fp32 / t_sta))

# --- 精度对比 (真实画面) ---
print("\n=== 精度对比 (detect_test.jpg) ===")
img = Image.open("detect_test.jpg").convert("RGB")

# FP32
d_fp32 = Detector(conf=0.35)
d_fp32.load()
r_fp32 = d_fp32.detect(img)
print("FP32 (conf=0.35): %d 目标" % len(r_fp32))
for r in r_fp32:
    print("  %s %.3f  %s" % (r["name"], r["conf"], [int(v) for v in r["box"]]))

# INT8 动态
d_dyn = Detector(conf=0.35)
s_dyn = ort.InferenceSession(INT8_DYN, providers=["CPUExecutionProvider"])
d_dyn.sess = s_dyn
d_dyn.inp_name = s_dyn.get_inputs()[0].name
d_dyn.loaded = True
r_dyn = d_dyn.detect(img)
print("INT8 动态 (conf=0.35): %d 目标" % len(r_dyn))
for r in r_dyn:
    print("  %s %.3f  %s" % (r["name"], r["conf"], [int(v) for v in r["box"]]))

# INT8 静态 + 多档阈值
if os.path.exists(INT8_STA):
    s_sta = ort.InferenceSession(INT8_STA, providers=["CPUExecutionProvider"])
    for c in [0.35, 0.10, 0.01, 0.001]:
        d_sta = Detector(conf=c)
        d_sta.sess = s_sta
        d_sta.inp_name = s_sta.get_inputs()[0].name
        d_sta.loaded = True
        r_sta = d_sta.detect(img)
        label = "conf=%.3f" % c
        if r_sta:
            print("INT8 静态 (%s): %d 目标" % (label, len(r_sta)))
            for r in r_sta[:3]:
                print("  %s %.4f  %s" % (r["name"], r["conf"], [int(v) for v in r["box"]]))
        else:
            print("INT8 静态 (%s): 0 目标" % label)

# --- 总结 ---
print("\n=== 总结 ===")
print("FP32:    %.1f ms, %d 目标" % (t_fp32, len(r_fp32)))
print("INT8动态: %.1f ms, %d 目标" % (t_dyn, len(r_dyn)))
if r_fp32 and r_dyn:
    dc = r_dyn[0]["conf"] - r_fp32[0]["conf"]
    print("动态量化置信度变化: %.3f -> %.3f (%+.3f)" % (r_fp32[0]["conf"], r_dyn[0]["conf"], dc))
