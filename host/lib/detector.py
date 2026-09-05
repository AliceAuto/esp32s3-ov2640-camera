# -*- coding: utf-8 -*-
"""本地 YOLO11 目标检测(ONNX Runtime)。

对串流帧做人/物检测, 返回框并在 PIL Image 上画框。依赖仅 onnxruntime + numpy + PIL。
模型: host/models/yolo11n.onnx (10.4MB, COCO 80 类, person 为 class 0)。
YOLO11 输出 [1, 84, 8400]: 前 4 行 = box(cx, cy, w, h 相对 640), 后 80 行 = 类别置信度。
"""
import os

import numpy as np
from PIL import Image, ImageDraw

import onnxruntime as ort

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(_THIS_DIR, "..", "models", "yolo11n.onnx")
MODEL_URL = "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.onnx"


def _ensure_model():
    """模型缺失时自动下载(约 10.4MB)。失败返回 False。"""
    if os.path.exists(MODEL_PATH):
        return True
    import urllib.request
    try:
        os.makedirs(os.path.dirname(MODEL_PATH), exist_ok=True)
        print("[detector] 模型缺失, 正在下载 yolo11n.onnx (~10MB)...", flush=True)
        urllib.request.urlretrieve(MODEL_URL, MODEL_PATH)
        return os.path.exists(MODEL_PATH)
    except Exception as e:  # noqa: BLE001
        print(f"[detector] 模型下载失败: {e}", flush=True)
        return False

# COCO 80 类名(顺序固定)
COCO = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag",
    "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
    "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon",
    "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot",
    "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant",
    "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote",
    "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush",
]

# 每类一个固定颜色(BGR 风格调色, 取鲜艳易辨的)
_COLORS = [
    (255, 56, 56), (255, 157, 151), (255, 112, 31), (255, 178, 29),
    (207, 210, 49), (72, 249, 10), (146, 204, 23), (61, 219, 134),
    (26, 147, 52), (0, 212, 187), (44, 153, 168), (0, 194, 255),
    (52, 69, 147), (100, 115, 255), (0, 24, 236), (132, 56, 255),
    (82, 0, 133), (203, 56, 255), (255, 149, 200), (255, 55, 199),
]


class Detector:
    """YOLO11 检测器。detect() 接收 PIL RGB Image, 返回检测结果列表。"""

    def __init__(self, imgsz=640, conf=0.35, iou=0.45):
        self.imgsz = imgsz
        self.conf = conf
        self.iou = iou
        self.sess = None
        self.inp_name = None
        self.loaded = False
        self.load_err = None

    def load(self):
        if self.loaded:
            return True
        try:
            if not _ensure_model():
                self.load_err = "模型文件缺失且下载失败: %s" % MODEL_PATH
                print("[detector] " + self.load_err, flush=True)
                return False
            self.sess = ort.InferenceSession(MODEL_PATH, providers=["CPUExecutionProvider"])
            self.inp_name = self.sess.get_inputs()[0].name
            self.loaded = True
            print(f"[detector] YOLO11 加载成功 conf={self.conf} imgsz={self.imgsz}", flush=True)
            return True
        except Exception as e:  # noqa: BLE001
            self.load_err = str(e)
            print(f"[detector] 加载失败: {e}", flush=True)
            return False

    # ---- 推理 + 后处理 ----
    def detect(self, img_pil):
        """对一张 PIL RGB Image 做检测。

        返回 list[dict]: {"cls", "name", "conf", "box"(x1,y1,x2,y2 相对原图像素)}。
        加载失败/无目标返回 []。
        """
        if not self.loaded and not self.load():
            return []
        w, h = img_pil.size

        # letterbox: 等比缩放 + 灰边填充到 imgsz x imgsz
        r = min(self.imgsz / w, self.imgsz / h)
        rw, rh = round(w * r), round(h * r)
        img_rs = img_pil.resize((rw, rh), Image.BILINEAR)
        canvas = Image.new("RGB", (self.imgsz, self.imgsz), (114, 114, 114))
        pad_x = (self.imgsz - rw) // 2
        pad_y = (self.imgsz - rh) // 2
        canvas.paste(img_rs, (pad_x, pad_y))

        arr = np.asarray(canvas, dtype=np.float32) / 255.0
        arr = arr.transpose(2, 0, 1)[None]  # 1,3,640,640
        out = self.sess.run(None, {self.inp_name: arr})[0]  # [1,84,8400]
        return self._postprocess(out[0], w, h, r, pad_x, pad_y)

    def _postprocess(self, preds, w, h, r, pad_x, pad_y):
        # preds: [84, 8400]
        boxes = preds[:4].T        # [8400,4] cx,cy,w,h
        scores = preds[4:].T       # [8400,80]
        cls = scores.argmax(1)
        conf = scores.max(1)
        mask = conf > self.conf
        boxes, cls, conf = boxes[mask], cls[mask], conf[mask]
        if boxes.shape[0] == 0:
            return []

        cx, cy, bw, bh = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
        x1 = cx - bw / 2.0
        y1 = cy - bh / 2.0
        x2 = cx + bw / 2.0
        y2 = cy + bh / 2.0
        # 去 letterbox 缩放 + 裁边偏移, 映射回原图像素坐标
        x1 = (x1 - pad_x) / r
        y1 = (y1 - pad_y) / r
        x2 = (x2 - pad_x) / r
        y2 = (y2 - pad_y) / r
        x1 = x1.clip(0, w); y1 = y1.clip(0, h)
        x2 = x2.clip(0, w); y2 = y2.clip(0, h)

        keep = self._nms(x1, y1, x2, y2, conf, self.iou)
        results = []
        for i in keep:
            c = int(cls[i])
            results.append({
                "cls": c,
                "name": COCO[c] if c < len(COCO) else str(c),
                "conf": float(conf[i]),
                "box": (float(x1[i]), float(y1[i]), float(x2[i]), float(y2[i])),
            })
        return results

    @staticmethod
    def _nms(x1, y1, x2, y2, scores, iou_thresh):
        areas = (x2 - x1) * (y2 - y1)
        order = scores.argsort()[::-1]
        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)
            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])
            inter = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
            union = areas[i] + areas[order[1:]] - inter
            iou = inter / np.maximum(union, 1e-9)
            order = order[1:][iou <= iou_thresh]
        return keep


def draw_boxes(img_pil, results, target_names=None, line_w=2):
    """在 PIL Image 上画检测框。target_names 为 None 时画全部; 否则只画指定类名。

    返回 (img_pil, 命中目标数)。"""
    if not results:
        return img_pil, 0
    draw = ImageDraw.Draw(img_pil)
    count = 0
    for r in results:
        name = r["name"]
        if target_names is not None and name not in target_names:
            continue
        count += 1
        x1, y1, x2, y2 = r["box"]
        color = _COLORS[r["cls"] % len(_COLORS)]
        draw.rectangle([x1, y1, x2, y2], outline=color, width=line_w)
        label = f"{name} {r['conf']:.2f}"
        ty = y1 - 14 if y1 >= 14 else y1 + 2
        # 先画底再画字, 保证可读性
        draw.rectangle([x1, ty, x1 + 8 * len(label) + 4, ty + 14], fill=color)
        draw.text((x1 + 2, ty + 1), label, fill=(255, 255, 255))
    return img_pil, count
