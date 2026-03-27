"""
Targeting Logic - Target Selection and Prioritization
Works with Detection objects from detector.py
"""

from typing import List, Optional
import math


class TargetSelector:
    """Selects highest priority target from detections"""
    
    def __init__(
        self,
        strategy: str = "closest",
        min_confidence: float = 0.5
    ):
        """
        Initialize target selector
        
        Args:
            strategy: Selection strategy
                - "closest": Nearest to frame center
                - "confident": Highest confidence
                - "largest": Largest bounding box
                - "combined": Weighted combination (recommended)
            min_confidence: Minimum confidence threshold
        """
        self.strategy = strategy
        self.min_confidence = min_confidence
        self.frame_width = 1280
        self.frame_height = 720
        
        print(f"[TARGETING] Strategy: {strategy}, min_conf: {min_confidence}")
    
    def update_frame_size(self, width: int, height: int):
        """Update frame dimensions"""
        self.frame_width = width
        self.frame_height = height
    
    def select_target(self, detections: List) -> Optional:
        """
        Select highest priority target from detections
        
        Args:
            detections: List of Detection objects (from your teammate's detector)
            
        Returns:
            Selected Detection or None
        """
        if not detections:
            return None
        
        # Filter by confidence
        valid = [d for d in detections if d.confidence >= self.min_confidence]
        
        if not valid:
            return None
        
        # Apply strategy
        if self.strategy == "closest":
            return self._select_closest(valid)
        elif self.strategy == "confident":
            return max(valid, key=lambda d: d.confidence)
        elif self.strategy == "largest":
            return max(valid, key=lambda d: d.width * d.height)
        elif self.strategy == "combined":
            return self._select_combined(valid)
        else:
            return self._select_closest(valid)
    
    def _select_closest(self, detections: List):
        """Select target closest to frame center"""
        frame_center = (self.frame_width / 2, self.frame_height / 2)
        
        def distance(det):
            cx, cy = det.center
            return math.sqrt((cx - frame_center[0])**2 + (cy - frame_center[1])**2)
        
        return min(detections, key=distance)
    
    def _select_combined(self, detections: List):
        """Combined scoring: confidence × (1 - distance) × size"""
        frame_center = (self.frame_width / 2, self.frame_height / 2)
        max_dist = math.sqrt(self.frame_width**2 + self.frame_height**2) / 2
        max_area = self.frame_width * self.frame_height
        
        def score(det):
            cx, cy = det.center
            distance = math.sqrt((cx - frame_center[0])**2 + (cy - frame_center[1])**2)
            dist_factor = distance / max_dist
            size_factor = (det.width * det.height) / max_area
            
            return det.confidence * (1 - dist_factor * 0.3) * (0.5 + 0.5 * size_factor)
        
        return max(detections, key=score)