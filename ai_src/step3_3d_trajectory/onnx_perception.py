"""CPU inference for the SnapDeploy segmentation and pose ONNX exports."""
import ast

import cv2
import numpy as np
import onnxruntime as ort


INPUT_HEIGHT = 416
INPUT_WIDTH = 640


class OnnxPerception:
    def __init__(self, seg_path, pose_path, landmark_names):
        cv2.setNumThreads(1)
        self.landmark_names = landmark_names
        self.seg = self._open(seg_path, "segment")
        self.pose = self._open(pose_path, "pose")

    def _open(self, path, task):
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        options.enable_cpu_mem_arena = False
        options.enable_mem_pattern = False
        options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_BASIC
        model = ort.InferenceSession(str(path), sess_options=options, providers=["CPUExecutionProvider"])
        meta = model.get_modelmeta().custom_metadata_map
        inputs = model.get_inputs()
        outputs = model.get_outputs()
        if (meta.get("task") != task or len(inputs) != 1 or inputs[0].type != "tensor(float)"
                or inputs[0].shape != [1, 3, INPUT_HEIGHT, INPUT_WIDTH] or len(outputs) != (2 if task == "segment" else 1)):
            raise ValueError(f"{path.name}: cần model YOLO {task} ONNX FP32 416×640, NMS tắt.")
        # The output stores four box coordinates, class scores, then mask coefficients or landmarks.
        count = len(ast.literal_eval(meta["names"]))
        if task == "segment":
            prototype = outputs[1].shape
            if len(prototype) != 4 or not isinstance(prototype[1], int):
                raise ValueError(f"{path.name}: lớp prototype của model không hợp lệ.")
            extra = prototype[1]
        else:
            if tuple(ast.literal_eval(meta["kpt_shape"])) != (len(self.landmark_names), 3):
                raise ValueError(f"{path.name}: cần đúng {len(self.landmark_names)} mốc có độ tin cậy.")
            extra = len(self.landmark_names) * 3
        if len(outputs[0].shape) != 3 or outputs[0].shape[1] != 4 + count + extra:
            raise ValueError(f"{path.name}: cấu trúc đầu ra YOLO không hợp lệ.")
        return model, count

    @staticmethod
    def _candidates(prediction, class_count):
        rows = prediction[0].T
        scores = rows[:, 4:4 + class_count].max(axis=1)
        keep = scores >= 0.5
        return rows[keep], scores[keep]

    def _segment(self, image, height, width, top, left, scaled_height, scaled_width):
        model, class_count = self.seg
        boxes, prototypes = model.run(None, {model.get_inputs()[0].name: image})
        predictions, scores = self._candidates(boxes, class_count)
        if not len(predictions):
            raise ValueError("Khong nhan dien duoc vung quan; khong sinh quy dao.")
        # Convert center-based detection boxes to the top-left form required by OpenCV NMS.
        rects = predictions[:, :4].copy()
        rects[:, :2] -= rects[:, 2:] * 0.5
        classes = predictions[:, 4:4 + class_count].argmax(axis=1)
        indices = cv2.dnn.NMSBoxesBatched(rects.tolist(), scores.tolist(), classes.tolist(), 0.49999, 0.7)
        planes = prototypes[0]
        channels, proto_h, proto_w = planes.shape
        largest = None
        largest_area = 0.0
        for index in indices[:300]:
            logits = (predictions[index, 4 + class_count:] @ planes.reshape(channels, -1)).reshape(proto_h, proto_w)
            x, y, w, h = rects[index]
            x0 = int(np.clip(np.ceil(x * proto_w / INPUT_WIDTH), 0, proto_w))
            x1 = int(np.clip(np.ceil((x + w) * proto_w / INPUT_WIDTH), 0, proto_w))
            y0 = int(np.clip(np.ceil(y * proto_h / INPUT_HEIGHT), 0, proto_h))
            y1 = int(np.clip(np.ceil((y + h) * proto_h / INPUT_HEIGHT), 0, proto_h))
            logits[:y0] = 0
            logits[y1:] = 0
            logits[:, :x0] = 0
            logits[:, x1:] = 0
            logits = cv2.resize(logits, (INPUT_WIDTH, INPUT_HEIGHT))
            mask = (logits[top:top + scaled_height, left:left + scaled_width] > 0).astype(np.uint8)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                candidate = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(candidate)
                if area > largest_area:
                    largest, largest_area = candidate, area
        if largest is None:
            raise ValueError("Khong nhan dien duoc vung quan; khong sinh quy dao.")
        scale = np.array([width / scaled_width, height / scaled_height])
        contour = np.rint(largest * scale).astype(np.int32)
        mask = np.zeros((height, width), dtype=np.uint8)
        cv2.fillPoly(mask, [contour], 255)
        return mask

    def predict(self, bgr):
        height, width = bgr.shape[:2]
        ratio = min(INPUT_HEIGHT / height, INPUT_WIDTH / width)
        scaled_height, scaled_width = max(1, round(height * ratio)), max(1, round(width * ratio))
        top, left = (INPUT_HEIGHT - scaled_height) // 2, (INPUT_WIDTH - scaled_width) // 2
        resized = cv2.resize(bgr, (scaled_width, scaled_height))
        padded = cv2.copyMakeBorder(resized, top, INPUT_HEIGHT - top - scaled_height,
                                    left, INPUT_WIDTH - left - scaled_width, cv2.BORDER_CONSTANT, value=(114, 114, 114))
        image = cv2.dnn.blobFromImage(padded, 1 / 255.0, swapRB=True)
        mask = self._segment(image, height, width, top, left, scaled_height, scaled_width)

        model, class_count = self.pose
        output, = model.run(None, {model.get_inputs()[0].name: image})
        rows, scores = self._candidates(output, class_count)
        if not len(rows):
            raise ValueError("Khong du 12 moc tin cay; hay chup lai hoac kiem tra model.")
        keypoints = rows[scores.argmax(), 4 + class_count:].reshape(-1, 3)
        if np.any(keypoints[:, 2] < 0.25):
            raise ValueError("Khong du 12 moc tin cay; hay chup lai hoac kiem tra model.")
        result = {}
        for index, (name, (x, y, confidence)) in enumerate(zip(self.landmark_names, keypoints)):
            result[name] = {
                "id": index,
                "pixel_u": int(round(np.clip((x - left) / ratio, 0, width - 1))),
                "pixel_v": int(round(np.clip((y - top) / ratio, 0, height - 1))),
                "confidence": round(float(confidence), 4),
            }
        return mask, result
