import os
from ultralytics import YOLO

class TargetDetector:
    def __init__(self, model_name='yolo11n.pt'):
        self.model_path = os.path.abspath(model_name)
        
        if not os.path.exists(self.model_path):
            print(f"Model {self.model_path} not found locally. It will be downloaded once by Ultralytics.")
        else:
            print(f"Using local YOLO model: {self.model_path}")
            
        # Using imgsz=320 to halve inference time as per handoff recommendation
        self.model = YOLO(self.model_path)
        print("YOLO tracker initialized.")

    def detect_target(self, frame, target_class=0, conf_threshold=0.4):
        """
        Runs YOLO inference on the frame to detect target_class (0 is usually 'person').
        Returns center_x, center_y of the highest confidence box.
        """
        # verbose=False prevents it from overflowing standard output every frame
        results = self.model(frame, imgsz=320, conf=conf_threshold, verbose=False, classes=[target_class])
        
        for r in results:
            boxes = r.boxes
            if len(boxes) > 0:
                # Take highest confidence box (YOLO usually sorts by confidence)
                b = boxes[0]
                x1, y1, x2, y2 = map(int, b.xyxy[0])
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                return cx, cy
                
        return -1, -1
