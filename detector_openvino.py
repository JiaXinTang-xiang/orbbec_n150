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
        """预处理: resize + normalize + CHW + batch (使用 cv2.dnn 加速)"""
        return cv2.dnn.blobFromImage(
            frame, scalefactor=1/255.0, size=(self._w_in, self._h_in),
            swapRB=False, crop=False)

    def postprocess(self, output, frame_shape):
        """解析 ultralytics OpenVINO 输出为 detections 列表

        output shape: (1, 6, N) 其中 6 = [cx, cy, w, h, cls0, cls1, ...]
        坐标在模型输入空间 (self._w_in × self._h_in)
        """
        h_frame, w_frame = frame_shape[:2]
        detections = []

        # (1, 6, N) → (N, 6): 每行 [cx, cy, w, h, cls0, cls1]
        data = output[0].T
        num_classes = data.shape[1] - 4

        for row in data:
            cx, cy, w, h = float(row[0]), float(row[1]), float(row[2]), float(row[3])
            class_scores = row[4:]

            class_id = int(np.argmax(class_scores))
            conf = float(class_scores[class_id])

            if conf < self.conf:
                continue

            # cx,cy,w,h → x1,y1,x2,y2 (模型输入空间)
            x1 = cx - w / 2
            y1 = cy - h / 2
            x2 = cx + w / 2
            y2 = cy + h / 2

            # 缩放回原图坐标
            x1 = int(x1 * w_frame / self._w_in)
            y1 = int(y1 * h_frame / self._h_in)
            x2 = int(x2 * w_frame / self._w_in)
            y2 = int(y2 * h_frame / self._h_in)

            # clamp 到图像边界
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w_frame, x2), min(h_frame, y2)
            if x2 <= x1 or y2 <= y1:
                continue

            cx_img = (x1 + x2) // 2
            cy_img = (y1 + y2) // 2
            class_name = self.class_names.get(class_id, str(class_id))

            detections.append({
                'bbox': [x1, y1, x2, y2],
                'center': (cx_img, cy_img),
                'confidence': conf,
                'class_id': class_id,
                'class_name': class_name,
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
