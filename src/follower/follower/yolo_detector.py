import cv2
import numpy as np
import logging

try:
    from ultralytics import YOLO
    ULTRALYTICS_AVAILABLE = True
except ImportError:
    ULTRALYTICS_AVAILABLE = False

class YoloDetector:
    def __init__(self, model_path="yolov8n.pt", fallback_to_mock=True):
        """
        Decoupled YOLOv8 Object Detector.
        
        Args:
            model_path (str): Path to the YOLOv8 weights file.
            fallback_to_mock (bool): If True, falls back to a custom OpenCV color-based obstacle detector 
                                     if ultralytics fails to load or import.
        """
        self.logger = logging.getLogger("YoloDetector")
        self.model = None
        self.fallback_mode = False
        
        if ULTRALYTICS_AVAILABLE:
            try:
                self.model = YOLO(model_path)
                self.logger.info(f"YOLOv8 model successfully loaded from {model_path}.")
            except Exception as e:
                self.logger.warning(f"Failed to load YOLO model: {e}.")
                if fallback_to_mock:
                    self.fallback_mode = True
                    self.logger.warning("Switching to Mock/CV2-based obstacle detection fallback.")
                else:
                    raise e
        else:
            self.logger.warning("Ultralytics library not available.")
            if fallback_to_mock:
                self.fallback_mode = True
                self.logger.warning("Switching to Mock/CV2-based obstacle detection fallback.")
            else:
                raise ImportError("Ultralytics is not installed.")

    def detect(self, bgr_image):
        """
        Run inference on a BGR image.
        
        Args:
            bgr_image (numpy.ndarray): The input BGR image.
            
        Returns:
            list of dicts: List of detections. Each detection dictionary contains:
                'bbox': [x_center, y_center, width, height]
                'class_name': str
                'confidence': float
                'proximity': float (virtual distance/proximity value [0.0 - 1.0], larger = closer)
        """
        # Defensive programming check: empty, invalid or None image
        if bgr_image is None or not isinstance(bgr_image, np.ndarray) or bgr_image.size == 0:
            return []

        # Check for completely black image
        if np.all(bgr_image == 0):
            return []

        if not self.fallback_mode and self.model is not None:
            try:
                results = self.model.predict(bgr_image, conf=0.25, verbose=False)
                detections = []
                h, w, _ = bgr_image.shape
                
                for result in results:
                    if result.boxes is None:
                        continue
                    for box in result.boxes:
                        # xywh format of bbox coordinates
                        xywh = box.xywh[0].cpu().numpy()
                        cls_id = int(box.cls[0].cpu().numpy())
                        class_name = self.model.names[cls_id]
                        conf = float(box.conf[0].cpu().numpy())
                        
                        # Proximity logic: box size and bottom-Y coordinate
                        # bottom_y is a good estimator of distance in ground plane projection
                        bottom_y = xywh[1] + (xywh[3] / 2.0)
                        area = xywh[2] * xywh[3]
                        norm_area = area / (w * h)
                        
                        # Virtual distance estimate (proximity scale: 0 = far, 1 = close)
                        # Combines box area and Y position on image plane
                        proximity = float(min(1.0, norm_area * 10.0 + (bottom_y / h) * 0.5))
                        
                        detections.append({
                            'bbox': [float(xywh[0]), float(xywh[1]), float(xywh[2]), float(xywh[3])],
                            'class_name': class_name,
                            'confidence': conf,
                            'proximity': proximity
                        })
                return detections
            except Exception as e:
                self.logger.error(f"Error during YOLO inference: {e}. Falling back to OpenCV detection.")
                # fall through to mock/CV2 detection
                pass

        return self._run_mock_cv2_detector(bgr_image)

    def _run_mock_cv2_detector(self, bgr_image):
        """
        Fallback detector using HSV color segmentation to detect obstacles 
        (e.g., the red box and traffic lights) in the simulation.
        """
        detections = []
        try:
            h, w, _ = bgr_image.shape
            hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
            
            # Red color ranges for the red box obstacle
            lower_red1 = np.array([0, 70, 50])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([170, 70, 50])
            upper_red2 = np.array([180, 255, 255])
            
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            red_mask = mask1 | mask2
            
            # Find red contours (Obstacle box or Red traffic light)
            contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 100:  # Threshold out small noise
                    x, y, box_w, box_h = cv2.boundingRect(cnt)
                    center_x = x + box_w / 2.0
                    center_y = y + box_h / 2.0
                    bottom_y = y + box_h
                    norm_area = area / (w * h)
                    proximity = float(min(1.0, norm_area * 10.0 + (bottom_y / h) * 0.5))
                    
                    # Heuristic: traffic lights are on the right side of the straight track,
                    # while the obstacle box is on the left.
                    if center_x > w * 0.55:
                        class_name = 'traffic_light_red'
                    else:
                        class_name = 'obstacle_box'
                    
                    detections.append({
                        'bbox': [float(center_x), float(center_y), float(box_w), float(box_h)],
                        'class_name': class_name,
                        'confidence': 0.9,
                        'proximity': proximity
                    })

            # Yellow range for traffic light elements
            lower_yellow = np.array([15, 100, 100])
            upper_yellow = np.array([35, 255, 255])
            yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
            contours_y, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours_y:
                area = cv2.contourArea(cnt)
                if area > 50:
                    x, y, box_w, box_h = cv2.boundingRect(cnt)
                    center_x = x + box_w / 2.0
                    center_y = y + box_h / 2.0
                    bottom_y = y + box_h
                    norm_area = area / (w * h)
                    proximity = float(min(1.0, norm_area * 10.0 + (bottom_y / h) * 0.5))
                    
                    detections.append({
                        'bbox': [float(center_x), float(center_y), float(box_w), float(box_h)],
                        'class_name': 'traffic_light_yellow',
                        'confidence': 0.85,
                        'proximity': proximity
                    })

            # Green range for traffic light elements
            lower_green = np.array([35, 70, 50])
            upper_green = np.array([85, 255, 255])
            green_mask = cv2.inRange(hsv, lower_green, upper_green)
            contours_g, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours_g:
                area = cv2.contourArea(cnt)
                if area > 50:
                    x, y, box_w, box_h = cv2.boundingRect(cnt)
                    center_x = x + box_w / 2.0
                    center_y = y + box_h / 2.0
                    bottom_y = y + box_h
                    norm_area = area / (w * h)
                    proximity = float(min(1.0, norm_area * 10.0 + (bottom_y / h) * 0.5))
                    
                    detections.append({
                        'bbox': [float(center_x), float(center_y), float(box_w), float(box_h)],
                        'class_name': 'traffic_light_green',
                        'confidence': 0.85,
                        'proximity': proximity
                    })

        except Exception as e:
            self.logger.error(f"Fallback detector failed: {e}")
            
        return detections
