"""
USB 摄像头 + ONNX 模型推理测试 (独立脚本)
"""
import cv2
import numpy as np
import time
from openvino import Core
import os
import yaml

MODEL_PATH = "model/best.onnx"
CONF = 0.15
CAM_ID = 0
IMGSZ = 640

# ===== 加载模型 =====
print(f"加载: {MODEL_PATH}")
core = Core()
model = core.read_model(MODEL_PATH)
compiled = core.compile_model(model, "CPU", {
    "PERFORMANCE_HINT": "LATENCY",
    "NUM_STREAMS": "AUTO",
})
input_key = compiled.input(0)
output_key = compiled.output(0)
print(f"  输入: {input_key.partial_shape}")
print(f"  输出: {output_key.partial_shape}")

# 类别名
class_names = {0: "class0", 1: "class1"}
meta_path = "model/best1_openvino_model/metadata.yaml"
if os.path.exists(meta_path):
    with open(meta_path) as f:
        class_names = {int(k): v for k, v in yaml.safe_load(f)["names"].items()}
print(f"  类别: {class_names}")

# ===== 预处理 =====
def preprocess(frame):
    h, w = frame.shape[:2]
    r = min(IMGSZ / w, IMGSZ / h)
    new_w, new_h = int(w * r), int(h * r)
    dw = IMGSZ - new_w
    dh = IMGSZ - new_h
    dw2, dh2 = dw // 2, dh // 2

    img = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    img = cv2.copyMakeBorder(img, dh2, dh - dh2, dw2, dw - dw2,
                             cv2.BORDER_CONSTANT, value=(114, 114, 114))

    blob = cv2.dnn.blobFromImage(img, 1/255.0, (IMGSZ, IMGSZ), swapRB=False, crop=False)
    return blob, r, dw2, dh2

# ===== 后处理 (输出 [1,6,8400] 原始格式，需 NMS) =====
def postprocess(output, frame_shape, r, pad_l, pad_t):
    h, w = frame_shape[:2]
    # (1, 6, 8400) → (8400, 6): [cx, cy, bw, bh, cls0, cls1]
    data = output[0].T
    boxes_by_class = {}

    for row in data:
        cx, cy, bw, bh = float(row[0]), float(row[1]), float(row[2]), float(row[3])
        scores = row[4:]
        cls_id = int(np.argmax(scores))
        conf = float(scores[cls_id])
        if conf < CONF:
            continue

        # cxcywh → xyxy (letterbox 空间)
        x1 = cx - bw / 2
        y1 = cy - bh / 2
        x2 = cx + bw / 2
        y2 = cy + bh / 2

        # 映射回原图
        x1 = int((x1 - pad_l) / r)
        y1 = int((y1 - pad_t) / r)
        x2 = int((x2 - pad_l) / r)
        y2 = int((y2 - pad_t) / r)
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w, x2), min(h, y2)
        if x2 <= x1 or y2 <= y1:
            continue

        if cls_id not in boxes_by_class:
            boxes_by_class[cls_id] = ([], [])
        boxes_by_class[cls_id][0].append([x1, y1, x2 - x1, y2 - y1])
        boxes_by_class[cls_id][1].append(conf)

    # NMS 按类别
    dets = []
    for cls_id, (boxes, confs) in boxes_by_class.items():
        indices = cv2.dnn.NMSBoxes(boxes, confs, CONF, 0.7)
        if len(indices) == 0:
            continue
        for i in indices.flatten():
            bx, by, br, bb = boxes[i]
            dets.append({
                'bbox': (bx, by, bx + br, by + bb),
                'center': (bx + br // 2, by + bb // 2),
                'conf': confs[i],
                'cls': class_names.get(cls_id, str(cls_id)),
            })
    return dets

# ===== 主循环 =====
print("打开摄像头...")
cap = cv2.VideoCapture(CAM_ID)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
if not cap.isOpened():
    print(f"摄像头 {CAM_ID} 打不开")
    exit(1)

print("按 q 退出\n")
fps_t0 = time.time()
fps_cnt = 0
fps_val = 0

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    # 推理
    t0 = time.time()
    blob, r, pad_l, pad_t = preprocess(frame)
    result = compiled([blob])[output_key]
    detections = postprocess(result, frame.shape, r, pad_l, pad_t)
    dt = time.time() - t0

    # 画框
    for d in detections:
        x1, y1, x2, y2 = d['bbox']
        label = f"{d['cls']} {d['conf']:.0%}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, label, (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.circle(frame, d['center'], 4, (0, 0, 255), -1)
        print(f"  [{d['cls']}] conf={d['conf']:.2f} @({x1},{y1})-({x2},{y2})")

    # FPS
    fps_cnt += 1
    if fps_cnt >= 30:
        fps_val = fps_cnt / (time.time() - fps_t0)
        fps_t0 = time.time()
        fps_cnt = 0

    cv2.putText(frame, f"FPS:{fps_val:.0f} det:{dt*1000:.0f}ms",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.imshow("ONNX Test", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print("退出")
