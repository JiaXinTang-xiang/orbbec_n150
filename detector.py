"""
YOLO 目标检测模块
封装模型加载和推理逻辑
"""

from ultralytics import YOLO
import torch


class YOLODetector:
    def __init__(self, model_path="best.pt", conf=0.5, iou=0.7, imgsz=640):
        """初始化 YOLO 检测器

        参数:
            model_path: 模型文件路径
            conf: 置信度阈值
            iou: IoU 阈值
            imgsz: 推理图像尺寸
        """
        self.device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
        # OpenVINO 模型自动检测 (目录名或 .xml 文件)
        self.is_openvino = model_path.endswith('_openvino_model/') or model_path.endswith('.xml')
        self.model = YOLO(model_path, task='detect' if self.is_openvino else None)
        if not self.is_openvino:
            self.model = self.model.to(self.device)
        self.conf = conf
        self.iou = iou
        self.imgsz = imgsz

        print(f"YOLO 模型加载完成")
        print(f"  模型: {model_path}")
        print(f"  设备: {self.device}")
        print(f"  类别: {self.model.names}")

    def infer(self, frame):
        """只推理，不画图 (异步模式用)"""
        results = self.model(frame, conf=self.conf, iou=self.iou,
                            imgsz=self.imgsz, verbose=False)
        detections = []
        for r in results:
            for box in r.boxes:
                xyxy = box.xyxy[0].cpu().numpy().astype(int)
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                cls_name = r.names[cls_id]
                x1, y1, x2, y2 = xyxy
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                detections.append({
                    'bbox': [x1, y1, x2, y2],
                    'center': (cx, cy),
                    'confidence': conf,
                    'class_id': cls_id,
                    'class_name': cls_name,
                })
        return detections

    def detect(self, frame):
        """检测单帧 (推理 + 画图) — 同步模式用"""
        results = self.model(frame, conf=self.conf, iou=self.iou,
                            imgsz=self.imgsz, verbose=False)
        detections = []
        for r in results:
            for box in r.boxes:
                xyxy = box.xyxy[0].cpu().numpy().astype(int)
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                cls_name = r.names[cls_id]
                x1, y1, x2, y2 = xyxy
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                detections.append({
                    'bbox': [x1, y1, x2, y2],
                    'center': (cx, cy),
                    'confidence': conf,
                    'class_id': cls_id,
                    'class_name': cls_name,
                })
        annotated = results[0].plot()
        return detections, annotated
