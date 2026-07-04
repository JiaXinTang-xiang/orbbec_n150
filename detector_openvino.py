"""
OpenVINO 直接推理检测器 (绕过 ultralytics 开销)
"""
import cv2
import numpy as np
import time


class OpenVINODetector:
    def __init__(self, model_path="model/best1_openvino_model/",
                 conf=0.5, iou=0.7, imgsz=640):
        self.conf = conf
        self.iou = iou
        self.imgsz = imgsz
        self.class_names = {}

        from openvino import Core
        import os

        # 找到 .xml 文件
        if os.path.isdir(model_path):
            xml_files = [f for f in os.listdir(model_path) if f.endswith('.xml')]
            if not xml_files:
                raise FileNotFoundError(f"No .xml found in {model_path}")
            xml_path = os.path.join(model_path, xml_files[0])
        else:
            xml_path = model_path

        print(f"OpenVINO 加载: {xml_path}")
        core = Core()
        model = core.read_model(xml_path)
        self.compiled_model = core.compile_model(model, "CPU", {
            "PERFORMANCE_HINT": "LATENCY",
            "NUM_STREAMS": "AUTO",           # 多流并行推理，充分利用 CPU
        })

        # 获取输入输出
        self.input_key = self.compiled_model.input(0)
        self.output_key = self.compiled_model.output(0)
        self.input_shape = self.input_key.shape
        self._h_in, self._w_in = self.input_shape[2], self.input_shape[3]
        print(f"  输入: {self.input_shape}, 设备: CPU")

        # 从 metadata.yaml 读类别名
        meta_path = os.path.join(os.path.dirname(xml_path), "metadata.yaml")
        if os.path.exists(meta_path):
            import yaml
            with open(meta_path) as f:
                meta = yaml.safe_load(f)
            names = meta.get("names", {})
            self.class_names = {int(k): v for k, v in names.items()}

        print(f"  类别: {self.class_names}")

    def preprocess(self, frame):
        """预处理: letterbox + normalize (与 ultralytics 训练一致)"""
        h, w = frame.shape[:2]
        # Letterbox: 等比缩放 + 填充，保持纵横比
        r = min(self._w_in / w, self._h_in / h)
        new_w, new_h = int(w * r), int(h * r)
        dw = self._w_in - new_w
        dh = self._h_in - new_h
        dw2, dh2 = dw // 2, dh // 2

        img = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        img = cv2.copyMakeBorder(img, dh2, dh - dh2, dw2, dw - dw2,
                                 cv2.BORDER_CONSTANT, value=(114, 114, 114))

        # 保存 letterbox 参数供 postprocess 使用
        self._lb_ratio = r
        self._lb_pad_left = dw2
        self._lb_pad_top = dh2

        return cv2.dnn.blobFromImage(
            img, scalefactor=1/255.0, size=(self._w_in, self._h_in),
            swapRB=False, crop=False)

    def postprocess(self, output, frame_shape):
        """自动识别格式：
        [1, 300, 6] → 已含 NMS, [x1,y1,x2,y2,conf,cls]
        [1, 6, N]   → 原始输出, 需转置+sigmoid+NMS
        """
        h_frame, w_frame = frame_shape[:2]
        r = getattr(self, '_lb_ratio', 1.0)
        pad_l = getattr(self, '_lb_pad_left', 0)
        pad_t = getattr(self, '_lb_pad_top', 0)
        detections = []

        if output.shape[1] == 300:
            # 已处理格式，直接用
            for det in output[0]:
                x1, y1, x2, y2, conf, cls_id = det
                if conf < self.conf:
                    continue
                x1 = max(0, int((x1 - pad_l) / r))
                y1 = max(0, int((y1 - pad_t) / r))
                x2 = min(w_frame, int((x2 - pad_l) / r))
                y2 = min(h_frame, int((y2 - pad_t) / r))
                if x2 <= x1 or y2 <= y1:
                    continue
                cls_id = int(cls_id)
                detections.append({
                    'bbox': [x1, y1, x2, y2],
                    'center': ((x1 + x2) // 2, (y1 + y2) // 2),
                    'confidence': float(conf),
                    'class_id': cls_id,
                    'class_name': self.class_names.get(cls_id, str(cls_id)),
                })
        else:
            # 原始格式 [1, 6, N]
            data = output[0].T
            boxes_by_class = {}
            for row in data:
                cx, cy, w, h = float(row[0]), float(row[1]), float(row[2]), float(row[3])
                raw_scores = row[4:]
                class_scores = 1 / (1 + np.exp(-raw_scores))
                class_id = int(np.argmax(class_scores))
                conf = float(class_scores[class_id])
                if conf < self.conf:
                    continue
                x1 = (cx - w / 2 - pad_l) / r
                y1 = (cy - h / 2 - pad_t) / r
                x2 = (cx + w / 2 - pad_l) / r
                y2 = (cy + h / 2 - pad_t) / r
                x1, y1 = max(0, int(x1)), max(0, int(y1))
                x2, y2 = min(w_frame, int(x2)), min(h_frame, int(y2))
                if x2 <= x1 or y2 <= y1:
                    continue
                boxes_by_class.setdefault(class_id, ([], []))
                boxes_by_class[class_id][0].append([x1, y1, x2 - x1, y2 - y1])
                boxes_by_class[class_id][1].append(conf)
            for cls_id, (boxes, confs) in boxes_by_class.items():
                idx = cv2.dnn.NMSBoxes(boxes, confs, self.conf, self.iou)
                if len(idx) == 0:
                    continue
                for i in idx.flatten():
                    bx, by, br, bb = boxes[i]
                    detections.append({
                        'bbox': [bx, by, bx + br, by + bb],
                        'center': (bx + br // 2, by + bb // 2),
                        'confidence': confs[i],
                        'class_id': cls_id,
                        'class_name': self.class_names.get(cls_id, str(cls_id)),
                    })

        return detections

    def draw(self, frame, detections):
        """绘制检测框 (直接修改原图，避免拷贝)"""
        annotated = frame
        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            label = f"{det['class_name']} {det['confidence']:.0%}"
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(annotated, label, (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.circle(annotated, det['center'], 4, (0, 0, 255), -1)
        return annotated

    def infer(self, frame):
        """只推理，不画图 (异步模式用)"""
        input_data = self.preprocess(frame)
        result = self.compiled_model([input_data])[self.output_key]
        return self.postprocess(result, frame.shape)

    def detect(self, frame):
        """检测单帧 (推理 + 画图)
        Returns:
            detections: 同 YOLODetector 格式
            annotated_frame: 标注后的图像
        """
        detections = self.infer(frame)
        annotated = self.draw(frame, detections)
        return detections, annotated

class ONNXDetector:
    """ONNX 检测器 (ultralytics 导出，内置 NMS)"""

    def __init__(self, model_path="model/best.onnx", conf=0.15, iou=0.7):
        self.conf = conf
        self.iou = iou
        self.class_names = {}

        from openvino import Core
        import os

        print(f"ONNX 加载: {model_path}")
        core = Core()
        model = core.read_model(model_path)
        self.compiled_model = core.compile_model(model, "CPU", {
            "PERFORMANCE_HINT": "LATENCY",
            "NUM_STREAMS": "AUTO",
        })

        self.input_key = self.compiled_model.input(0)
        self.output_key = self.compiled_model.output(0)
        self._h_in, self._w_in = 640, 640
        print(f"  输入: [1,3,{self._h_in},{self._w_in}], 设备: CPU")

        # 从 best.pt 读类别名
        meta_path = os.path.join(os.path.dirname(model_path), "best1_openvino_model", "metadata.yaml")
        if not os.path.exists(meta_path):
            meta_path = os.path.join(os.path.dirname(model_path), "metadata.yaml")
        if os.path.exists(meta_path):
            import yaml
            with open(meta_path) as f:
                meta = yaml.safe_load(f)
            names = meta.get("names", {})
            self.class_names = {int(k): v for k, v in names.items()}
        if not self.class_names:
            self.class_names = {0: "class0", 1: "class1"}

        print(f"  类别: {self.class_names}")

    def preprocess(self, frame):
        """Letterbox 预处理"""
        h, w = frame.shape[:2]
        r = min(self._w_in / w, self._h_in / h)
        new_w, new_h = int(w * r), int(h * r)
        dw = self._w_in - new_w
        dh = self._h_in - new_h
        dw2, dh2 = dw // 2, dh // 2

        img = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        img = cv2.copyMakeBorder(img, dh2, dh - dh2, dw2, dw - dw2,
                                 cv2.BORDER_CONSTANT, value=(114, 114, 114))

        self._lb_ratio = r
        self._lb_pad_left = dw2
        self._lb_pad_top = dh2

        return cv2.dnn.blobFromImage(
            img, scalefactor=1/255.0, size=(self._w_in, self._h_in),
            swapRB=False, crop=False)

    def postprocess(self, output, frame_shape):
        """ONNX 输出 [1,6,8400] 原始格式 → 转置 + NMS"""
        h_frame, w_frame = frame_shape[:2]
        r = getattr(self, '_lb_ratio', 1.0)
        pad_l = getattr(self, '_lb_pad_left', 0)
        pad_t = getattr(self, '_lb_pad_top', 0)

        # (1, 6, N) → (N, 6): [cx, cy, w, h, cls0, cls1]
        data = output[0].T
        boxes_by_class = {}

        for row in data:
            cx, cy, bw, bh = float(row[0]), float(row[1]), float(row[2]), float(row[3])
            raw_scores = row[4:]
            # sigmoid: 原始 logits → 置信度
            scores = 1 / (1 + np.exp(-raw_scores))
            cls_id = int(np.argmax(scores))
            conf = float(scores[cls_id])
            if conf < self.conf:
                continue

            x1 = cx - bw / 2
            y1 = cy - bh / 2
            x2 = cx + bw / 2
            y2 = cy + bh / 2

            x1 = int((x1 - pad_l) / r)
            y1 = int((y1 - pad_t) / r)
            x2 = int((x2 - pad_l) / r)
            y2 = int((y2 - pad_t) / r)
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w_frame, x2), min(h_frame, y2)
            if x2 <= x1 or y2 <= y1:
                continue

            if cls_id not in boxes_by_class:
                boxes_by_class[cls_id] = ([], [])
            boxes_by_class[cls_id][0].append([x1, y1, x2 - x1, y2 - y1])
            boxes_by_class[cls_id][1].append(conf)

        detections = []
        for cls_id, (boxes, confs) in boxes_by_class.items():
            indices = cv2.dnn.NMSBoxes(boxes, confs, self.conf, self.iou)
            if len(indices) == 0:
                continue
            for i in indices.flatten():
                bx, by, br, bb = boxes[i]
                detections.append({
                    'bbox': [bx, by, bx + br, by + bb],
                    'center': (bx + br // 2, by + bb // 2),
                    'confidence': confs[i],
                    'class_id': cls_id,
                    'class_name': self.class_names.get(cls_id, str(cls_id)),
                })
        return detections

    def draw(self, frame, detections):
        annotated = frame
        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            label = f"{det['class_name']} {det['confidence']:.0%}"
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(annotated, label, (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.circle(annotated, det['center'], 4, (0, 0, 255), -1)
        return annotated

    def infer(self, frame):
        input_data = self.preprocess(frame)
        result = self.compiled_model([input_data])[self.output_key]
        return self.postprocess(result, frame.shape)

    def detect(self, frame):
        detections = self.infer(frame)
        annotated = self.draw(frame, detections)
        return detections, annotated

    def debug_output(self, frame):
        """调试：打印模型原始输出信息"""
        input_data = self.preprocess(frame)
        result = self.compiled_model([input_data])[self.output_key]
        print(f"=== 模型输出调试 ===")
        print(f"输出 shape: {result.shape}")
        print(f"输出 dtype: {result.dtype}")
        # Transpose if needed: [1, N, 6] or [1, 6, N]
        data = result[0]
        if data.shape[0] == 6:
            data = data.T  # [6, N] -> [N, 6]
        print(f"处理后 shape: {data.shape}")
        # 找到分数最高的前5个
        scores = data[:, 4:].max(axis=1)
        top5 = np.argsort(scores)[-5:][::-1]
        print(f"前5高置信度:")
        for idx in top5:
            row = data[idx]
            cx, cy, w, h = row[0], row[1], row[2], row[3]
            cls_scores = row[4:]
            cls_id = int(np.argmax(cls_scores))
            conf = float(cls_scores[cls_id])
            if conf < 0.001:
                continue
            x1 = cx - w/2
            y1 = cy - h/2
            x2 = cx + w/2
            y2 = cy + h/2
            print(f"  cls={cls_id} conf={conf:.4f} xywh=({cx:.4f},{cy:.4f},{w:.4f},{h:.4f}) → xyxy=({x1:.4f},{y1:.4f},{x2:.4f},{y2:.4f})")
        print(f"==================")
