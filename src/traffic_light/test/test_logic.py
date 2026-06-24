import numpy as np

from traffic_light.logic import (
    STATE_GREEN,
    STATE_RED,
    STATE_UNKNOWN,
    DetectionCandidate,
    TrafficLightConfig,
    analyze_traffic_light,
    normalize_class_name,
)


def make_detection(
    class_name="traffic light",
    score=0.9,
    center_x=50.0,
    center_y=50.0,
    size_x=20.0,
    size_y=20.0,
):
    return DetectionCandidate(
        class_name=class_name,
        score=score,
        center_x=center_x,
        center_y=center_y,
        size_x=size_x,
        size_y=size_y,
    )


def make_image(color=(0, 0, 0), height=100, width=100):
    image = np.zeros((height, width, 3), dtype=np.uint8)
    image[:, :] = color
    return image


def test_no_detections_returns_unknown():
    assert analyze_traffic_light([], make_image()) == STATE_UNKNOWN


def test_non_traffic_light_class_returns_unknown():
    detections = [make_detection(class_name="person")]

    assert analyze_traffic_light(detections, make_image((0, 0, 255))) == STATE_UNKNOWN


def test_low_score_detection_returns_unknown():
    detections = [make_detection(score=0.1)]

    assert analyze_traffic_light(detections, make_image((0, 0, 255))) == STATE_UNKNOWN


def test_small_bbox_returns_unknown():
    detections = [make_detection(size_x=2.0, size_y=2.0)]

    assert analyze_traffic_light(detections, make_image((0, 0, 255))) == STATE_UNKNOWN


def test_red_crop_returns_red():
    detections = [make_detection()]

    assert analyze_traffic_light(detections, make_image((0, 0, 255))) == STATE_RED


def test_green_crop_returns_green():
    detections = [make_detection()]

    assert analyze_traffic_light(detections, make_image((0, 255, 0))) == STATE_GREEN


def test_weak_color_returns_unknown():
    detections = [make_detection()]

    assert analyze_traffic_light(detections, make_image((0, 0, 0))) == STATE_UNKNOWN


def test_ambiguous_red_green_mix_returns_unknown():
    image = make_image((0, 0, 0))
    image[40:60, 40:50] = (0, 0, 255)
    image[40:60, 50:60] = (0, 255, 0)
    detections = [make_detection()]

    assert analyze_traffic_light(detections, image) == STATE_UNKNOWN


def test_highest_score_traffic_light_bbox_is_used():
    image = make_image((0, 0, 0))
    image[20:40, 20:40] = (0, 0, 255)
    image[60:80, 60:80] = (0, 255, 0)
    detections = [
        make_detection(score=0.6, center_x=30.0, center_y=30.0),
        make_detection(score=0.9, center_x=70.0, center_y=70.0),
    ]

    assert analyze_traffic_light(detections, image) == STATE_GREEN


def test_class_name_variants_are_normalized():
    assert normalize_class_name("Traffic-Light") == "traffic light"
    assert normalize_class_name(" traffic_light ") == "traffic light"

    detections = [make_detection(class_name="Traffic-Light")]

    assert analyze_traffic_light(detections, make_image((0, 0, 255))) == STATE_RED


def test_custom_thresholds_can_make_color_unknown():
    detections = [make_detection()]
    config = TrafficLightConfig(min_red_ratio=1.1)

    assert analyze_traffic_light(detections, make_image((0, 0, 255)), config) == STATE_UNKNOWN


def test_direct_red_classification_returns_red():
    detections = [make_detection(class_name="red", score=0.9)]
    # Even if image is black, direct classification overrides HSV checks
    assert analyze_traffic_light(detections, make_image((0, 0, 0))) == STATE_RED


def test_direct_green_classification_returns_green():
    detections = [make_detection(class_name="green", score=0.9)]
    # Even if image is black, direct classification overrides HSV checks
    assert analyze_traffic_light(detections, make_image((0, 0, 0))) == STATE_GREEN

