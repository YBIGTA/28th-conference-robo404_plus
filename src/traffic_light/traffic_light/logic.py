import math
import re

import cv2
import numpy as np


STATE_UNKNOWN = "UNKNOWN"
STATE_RED = "RED"
STATE_GREEN = "GREEN"


class DetectionCandidate:
    def __init__(self, class_name, score, center_x, center_y, size_x, size_y):
        self.class_name = class_name
        self.score = score
        self.center_x = center_x
        self.center_y = center_y
        self.size_x = size_x
        self.size_y = size_y


class TrafficLightConfig:
    def __init__(
        self,
        traffic_light_class_names=("traffic light", "traffic_light"),
        min_detection_confidence=0.5,
        min_bbox_area_px=25,
        red_h_low_1=0,
        red_h_high_1=10,
        red_h_low_2=170,
        red_h_high_2=180,
        red_s_min=80,
        red_v_min=80,
        green_h_low=40,
        green_h_high=90,
        green_s_min=60,
        green_v_min=60,
        min_red_ratio=0.03,
        min_green_ratio=0.03,
        red_green_margin=1.2,
    ):
        self.traffic_light_class_names = traffic_light_class_names
        self.min_detection_confidence = min_detection_confidence
        self.min_bbox_area_px = min_bbox_area_px
        self.red_h_low_1 = red_h_low_1
        self.red_h_high_1 = red_h_high_1
        self.red_h_low_2 = red_h_low_2
        self.red_h_high_2 = red_h_high_2
        self.red_s_min = red_s_min
        self.red_v_min = red_v_min
        self.green_h_low = green_h_low
        self.green_h_high = green_h_high
        self.green_s_min = green_s_min
        self.green_v_min = green_v_min
        self.min_red_ratio = min_red_ratio
        self.min_green_ratio = min_green_ratio
        self.red_green_margin = red_green_margin


def normalize_class_name(class_name):
    normalized = class_name.strip().lower()
    normalized = normalized.replace("-", " ").replace("_", " ")
    normalized = re.sub(r"\s+", " ", normalized)
    return normalized


def analyze_traffic_light(detections, image_bgr, config=None):
    if image_bgr is None:
        return STATE_UNKNOWN

    if config is None:
        config = TrafficLightConfig()

    selection = select_traffic_light_detection(detections, image_bgr.shape, config)
    if selection is None:
        return STATE_UNKNOWN

    _, crop_rect = selection
    crop = crop_image(image_bgr, crop_rect)
    if crop.size == 0:
        return STATE_UNKNOWN

    return classify_crop_color(crop, config)


def select_traffic_light_detection(detections, image_shape, config):
    class_names = {normalize_class_name(name) for name in config.traffic_light_class_names}
    best_detection = None
    best_rect = None

    for detection in detections:
        if normalize_class_name(detection.class_name) not in class_names:
            continue

        if detection.score < config.min_detection_confidence:
            continue

        crop_rect = bbox_to_crop_rect(detection, image_shape)
        if crop_rect is None:
            continue

        x1, y1, x2, y2 = crop_rect
        if (x2 - x1) * (y2 - y1) < config.min_bbox_area_px:
            continue

        if best_detection is None or detection.score > best_detection.score:
            best_detection = detection
            best_rect = crop_rect

    if best_detection is None:
        return None

    return best_detection, best_rect


def bbox_to_crop_rect(detection, image_shape):
    image_height, image_width = image_shape[:2]

    if detection.size_x <= 0 or detection.size_y <= 0:
        return None

    x1 = math.floor(detection.center_x - detection.size_x / 2.0)
    y1 = math.floor(detection.center_y - detection.size_y / 2.0)
    x2 = math.ceil(detection.center_x + detection.size_x / 2.0)
    y2 = math.ceil(detection.center_y + detection.size_y / 2.0)

    x1 = max(0, min(image_width, x1))
    y1 = max(0, min(image_height, y1))
    x2 = max(0, min(image_width, x2))
    y2 = max(0, min(image_height, y2))

    if x2 <= x1 or y2 <= y1:
        return None

    return x1, y1, x2, y2


def crop_image(image_bgr, crop_rect):
    x1, y1, x2, y2 = crop_rect
    return image_bgr[y1:y2, x1:x2]


def classify_crop_color(crop_bgr, config=None):
    if config is None:
        config = TrafficLightConfig()

    if crop_bgr.size == 0:
        return STATE_UNKNOWN

    hsv = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2HSV)
    red_ratio, green_ratio = color_ratios(hsv, config)

    red_clear = (
        red_ratio >= config.min_red_ratio
        and red_ratio >= green_ratio * config.red_green_margin
    )
    green_clear = (
        green_ratio >= config.min_green_ratio
        and green_ratio >= red_ratio * config.red_green_margin
    )

    if red_clear and not green_clear:
        return STATE_RED
    if green_clear and not red_clear:
        return STATE_GREEN
    return STATE_UNKNOWN


def color_ratios(hsv, config):
    pixel_count = hsv.shape[0] * hsv.shape[1]
    if pixel_count == 0:
        return 0.0, 0.0

    red_mask_1 = cv2.inRange(
        hsv,
        np.array([config.red_h_low_1, config.red_s_min, config.red_v_min]),
        np.array([config.red_h_high_1, 255, 255]),
    )
    red_mask_2 = cv2.inRange(
        hsv,
        np.array([config.red_h_low_2, config.red_s_min, config.red_v_min]),
        np.array([config.red_h_high_2, 255, 255]),
    )
    red_mask = cv2.bitwise_or(red_mask_1, red_mask_2)

    green_mask = cv2.inRange(
        hsv,
        np.array([config.green_h_low, config.green_s_min, config.green_v_min]),
        np.array([config.green_h_high, 255, 255]),
    )

    red_ratio = float(np.count_nonzero(red_mask)) / float(pixel_count)
    green_ratio = float(np.count_nonzero(green_mask)) / float(pixel_count)
    return red_ratio, green_ratio
