import pytest
import numpy as np
from follower.yolo_detector import YoloDetector

def test_yolo_detector_init():
    """Test YoloDetector initialization and fallback mode."""
    # Since ultralytics is missing, it should fallback to mock mode safely
    detector = YoloDetector(model_path="dummy.pt", fallback_to_mock=True)
    assert detector.fallback_mode is True

def test_yolo_detector_empty_input():
    """Test defensive handling of invalid or empty images."""
    detector = YoloDetector(fallback_to_mock=True)
    
    # None image
    detections = detector.detect(None)
    assert detections == []

    # Empty array
    detections = detector.detect(np.array([]))
    assert detections == []

def test_yolo_detector_black_input():
    """Test that a completely black image returns no detections."""
    detector = YoloDetector(fallback_to_mock=True)
    black_img = np.zeros((480, 640, 3), dtype=np.uint8)
    
    detections = detector.detect(black_img)
    assert detections == []

def test_yolo_detector_fallback_obstacle_box():
    """Test fallback mode detection of a red obstacle box."""
    detector = YoloDetector(force_mock=True)
    
    # Create white image background
    img = np.ones((480, 640, 3), dtype=np.uint8) * 255
    
    # Draw a red box in the image (red color is BGR [0, 0, 255])
    # Set blue and green to 0, red to 255 in a box region
    img[200:300, 300:400, 0] = 0
    img[200:300, 300:400, 1] = 0
    img[200:300, 300:400, 2] = 255
    
    detections = detector.detect(img)
    
    assert len(detections) > 0
    obs = [d for d in detections if d['class_name'] == 'obstacle_box']
    assert len(obs) > 0
    
    box = obs[0]
    assert box['confidence'] == 0.9
    # Bounding box center should be close to center of drawn red box:
    # X center: 300 + 50 = 350
    # Y center: 200 + 50 = 250
    assert abs(box['bbox'][0] - 350) < 5
    assert abs(box['bbox'][1] - 250) < 5
    # Bounding box size: width 100, height 100
    assert abs(box['bbox'][2] - 100) < 5
    assert abs(box['bbox'][3] - 100) < 5
    # Proximity should be successfully calculated (positive value)
    assert box['proximity'] > 0.0
    assert box['proximity'] <= 1.0

def test_yolo_detector_fallback_traffic_light():
    """Test fallback mode detection of a yellow traffic light."""
    detector = YoloDetector(force_mock=True)
    
    # Create white image background
    img = np.ones((480, 640, 3), dtype=np.uint8) * 255
    
    # Draw a yellow box (yellow is BGR [0, 255, 255])
    # Set blue to 0, green and red to 255
    img[100:150, 200:250, 0] = 0
    img[100:150, 200:250, 1] = 255
    img[100:150, 200:250, 2] = 255
    
    detections = detector.detect(img)
    
    assert len(detections) > 0
    tl = [d for d in detections if d['class_name'] == 'traffic_light_yellow']
    assert len(tl) > 0
    
    box = tl[0]
    assert box['confidence'] == 0.85
    # Center X: 200 + 25 = 225, Center Y: 100 + 25 = 125
    assert abs(box['bbox'][0] - 225) < 5
    assert abs(box['bbox'][1] - 125) < 5
    assert box['proximity'] > 0.0
