#!/usr/bin/env python3
"""ROS2 NCNN object detection node using YOLOv8 NCNN model."""

import os
import sys
import time
from typing import List, Dict

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

# Allow using ncnn from a dedicated venv without breaking system Python.
VENV_SITE = os.path.expanduser('~/RoboDog_ws/.venv_ncnn/lib/python3.12/site-packages')
if VENV_SITE not in sys.path and os.path.isdir(VENV_SITE):
    sys.path.append(VENV_SITE)

import ncnn  # noqa: E402


COCO_CLASSES = [
    'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light',
    'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
    'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
    'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard',
    'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
    'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
    'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
    'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear',
    'hair drier', 'toothbrush'
]


class NcnnDetectionNode(Node):
    """Subscribe image topic, run NCNN YOLOv8 detection, publish annotated stream."""

    def __init__(self) -> None:
        super().__init__('ncnn_detection_node')

        self.declare_parameter('input_topic', '/robodog/camera/image_raw')
        self.declare_parameter('output_topic', '/robodog/camera/detections/image')
        self.declare_parameter('detections_topic', '/robodog/camera/detections/text')
        self.declare_parameter('show_window', False)
        self.declare_parameter('target_size', 416)
        self.declare_parameter('prob_threshold', 0.25)
        self.declare_parameter('nms_threshold', 0.45)
        self.declare_parameter('process_every_n', 1)
        self.declare_parameter('use_vulkan', False)
        self.declare_parameter('model_param_path', '/home/robodog/RoboDog_ws/src/robodog_vision/models/yolov8n.ncnn.param')
        self.declare_parameter('model_bin_path', '/home/robodog/RoboDog_ws/src/robodog_vision/models/yolov8n.ncnn.bin')

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.detections_topic = self.get_parameter('detections_topic').value
        self.show_window = bool(self.get_parameter('show_window').value)
        self.target_size = int(self.get_parameter('target_size').value)
        self.prob_threshold = float(self.get_parameter('prob_threshold').value)
        self.nms_threshold = float(self.get_parameter('nms_threshold').value)
        self.process_every_n = max(1, int(self.get_parameter('process_every_n').value))
        self.use_vulkan = bool(self.get_parameter('use_vulkan').value)
        self.model_param_path = self.get_parameter('model_param_path').value
        self.model_bin_path = self.get_parameter('model_bin_path').value

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(Image, self.input_topic, self._on_image, 10)
        self.image_pub = self.create_publisher(Image, self.output_topic, 10)
        self.det_pub = self.create_publisher(String, self.detections_topic, 10)

        self.net = ncnn.Net()
        self.net.opt.use_vulkan_compute = self.use_vulkan
        if self.net.load_param(self.model_param_path) != 0:
            raise RuntimeError(f'Failed to load model param: {self.model_param_path}')
        if self.net.load_model(self.model_bin_path) != 0:
            raise RuntimeError(f'Failed to load model bin: {self.model_bin_path}')

        self.frame_count = 0
        self.detect_count = 0
        self.last_dets: List[Dict] = []
        self.last_log_time = time.time()
        self.get_logger().info(
            'NCNN detector ready\n'
            f'  Input topic: {self.input_topic}\n'
            f'  Output topic: {self.output_topic}\n'
            f'  Model: {self.model_param_path}\n'
            f'  Target size: {self.target_size}\n'
            f'  Process every N frames: {self.process_every_n}\n'
            f'  Vulkan: {self.use_vulkan}'
        )

    @staticmethod
    def _sigmoid(x: np.ndarray) -> np.ndarray:
        return 1.0 / (1.0 + np.exp(-x))

    @staticmethod
    def _softmax(x: np.ndarray, axis: int = -1) -> np.ndarray:
        x_max = np.max(x, axis=axis, keepdims=True)
        e = np.exp(x - x_max)
        return e / np.sum(e, axis=axis, keepdims=True)

    @staticmethod
    def _iou(a: Dict, b: Dict) -> float:
        x1 = max(a['x0'], b['x0'])
        y1 = max(a['y0'], b['y0'])
        x2 = min(a['x1'], b['x1'])
        y2 = min(a['y1'], b['y1'])
        iw = max(0.0, x2 - x1)
        ih = max(0.0, y2 - y1)
        inter = iw * ih
        if inter <= 0.0:
            return 0.0
        area_a = max(0.0, a['x1'] - a['x0']) * max(0.0, a['y1'] - a['y0'])
        area_b = max(0.0, b['x1'] - b['x0']) * max(0.0, b['y1'] - b['y0'])
        union = area_a + area_b - inter
        return inter / union if union > 0.0 else 0.0

    def _nms_class_aware(self, dets: List[Dict]) -> List[Dict]:
        dets = sorted(dets, key=lambda d: d['score'], reverse=True)
        kept: List[Dict] = []
        for det in dets:
            keep = True
            for k in kept:
                if det['label'] != k['label']:
                    continue
                if self._iou(det, k) > self.nms_threshold:
                    keep = False
                    break
            if keep:
                kept.append(det)
        return kept

    def _preprocess(self, bgr: np.ndarray):
        img_h, img_w = bgr.shape[:2]
        w = img_w
        h = img_h
        scale = 1.0
        if w > h:
            scale = float(self.target_size) / float(w)
            w = self.target_size
            h = int(h * scale)
        else:
            scale = float(self.target_size) / float(h)
            h = self.target_size
            w = int(w * scale)

        resized = cv2.resize(bgr, (w, h), interpolation=cv2.INTER_LINEAR)

        max_stride = 32
        wpad = int(np.ceil(w / max_stride) * max_stride - w)
        hpad = int(np.ceil(h / max_stride) * max_stride - h)

        top = hpad // 2
        bottom = hpad - top
        left = wpad // 2
        right = wpad - left

        in_pad = cv2.copyMakeBorder(resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114, 114, 114))

        return in_pad, scale, left, top, img_w, img_h

    def _infer(self, bgr: np.ndarray) -> List[Dict]:
        in_pad, scale, left, top, img_w, img_h = self._preprocess(bgr)
        h_pad, w_pad = in_pad.shape[:2]

        mat_in = ncnn.Mat.from_pixels(in_pad, ncnn.Mat.PixelType.PIXEL_BGR2RGB, w_pad, h_pad)
        norm_vals = [1.0 / 255.0, 1.0 / 255.0, 1.0 / 255.0]
        mat_in.substract_mean_normalize([], norm_vals)

        ex = self.net.create_extractor()
        ex.input('in0', mat_in)
        ret, out = ex.extract('out0')
        if ret != 0:
            return []

        pred = np.array(out)

        strides = [8, 16, 32]
        reg_max_1 = 16
        proposals: List[Dict] = []

        row_offset = 0
        num_class = pred.shape[1] - reg_max_1 * 4
        dist_proj = np.arange(reg_max_1, dtype=np.float32)

        for stride in strides:
            num_grid_x = w_pad // stride
            num_grid_y = h_pad // stride
            num_grid = num_grid_x * num_grid_y
            pred_stride = pred[row_offset:row_offset + num_grid, :]
            row_offset += num_grid

            for gy in range(num_grid_y):
                for gx in range(num_grid_x):
                    idx = gy * num_grid_x + gx
                    p = pred_stride[idx]

                    cls_logits = p[reg_max_1 * 4:reg_max_1 * 4 + num_class]
                    label = int(np.argmax(cls_logits))
                    score = float(self._sigmoid(cls_logits[label]))
                    if score < self.prob_threshold:
                        continue

                    bbox_logits = p[:reg_max_1 * 4].reshape(4, reg_max_1)
                    bbox_prob = self._softmax(bbox_logits, axis=1)
                    ltrb = (bbox_prob * dist_proj).sum(axis=1) * float(stride)

                    cx = (gx + 0.5) * stride
                    cy = (gy + 0.5) * stride

                    x0 = cx - ltrb[0]
                    y0 = cy - ltrb[1]
                    x1 = cx + ltrb[2]
                    y1 = cy + ltrb[3]

                    x0 = (x0 - left) / scale
                    y0 = (y0 - top) / scale
                    x1 = (x1 - left) / scale
                    y1 = (y1 - top) / scale

                    x0 = float(np.clip(x0, 0, img_w - 1))
                    y0 = float(np.clip(y0, 0, img_h - 1))
                    x1 = float(np.clip(x1, 0, img_w - 1))
                    y1 = float(np.clip(y1, 0, img_h - 1))

                    if x1 <= x0 or y1 <= y0:
                        continue

                    proposals.append({'x0': x0, 'y0': y0, 'x1': x1, 'y1': y1, 'score': score, 'label': label})

        return self._nms_class_aware(proposals)

    def _draw(self, bgr: np.ndarray, dets: List[Dict]) -> np.ndarray:
        img = bgr.copy()
        for d in dets:
            p1 = (int(d['x0']), int(d['y0']))
            p2 = (int(d['x1']), int(d['y1']))
            cv2.rectangle(img, p1, p2, (20, 220, 20), 2)
            label_name = COCO_CLASSES[d['label']] if 0 <= d['label'] < len(COCO_CLASSES) else str(d['label'])
            txt = f"{label_name} {d['score']:.2f}"
            cv2.putText(img, txt, (p1[0], max(0, p1[1] - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (20, 220, 20), 2)
        return img

    def _on_image(self, msg: Image) -> None:
        try:
            rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

            self.frame_count += 1
            run_infer = (self.frame_count % self.process_every_n) == 0

            if run_infer:
                dets = self._infer(bgr)
                self.last_dets = dets
                self.detect_count += 1
            else:
                dets = self.last_dets

            annotated_bgr = self._draw(bgr, dets)
            annotated_rgb = cv2.cvtColor(annotated_bgr, cv2.COLOR_BGR2RGB)

            out_msg = self.bridge.cv2_to_imgmsg(annotated_rgb, encoding='rgb8')
            out_msg.header = msg.header
            self.image_pub.publish(out_msg)

            txt = String()
            txt.data = '; '.join([
                f"{COCO_CLASSES[d['label']] if 0 <= d['label'] < len(COCO_CLASSES) else d['label']}:{d['score']:.2f}"
                for d in dets[:10]
            ])
            self.det_pub.publish(txt)

            now = time.time()
            if now - self.last_log_time > 2.0:
                self.get_logger().info(
                    f'Detection running: input_frames={self.frame_count}, infer_frames={self.detect_count}, '
                    f'detections={len(dets)}'
                )
                self.last_log_time = now

            if self.show_window:
                cv2.imshow('RoboDog NCNN Detection', annotated_bgr)
                cv2.waitKey(1)

        except Exception as exc:
            self.get_logger().error(f'Detection callback error: {exc}')

    def destroy_node(self):
        if self.show_window:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NcnnDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if 'ExternalShutdownException' not in str(type(e)):
            raise
    finally:
        node.destroy_node()
        rclpy.shutdown()
