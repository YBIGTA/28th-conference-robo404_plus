from debug_monitor.logic import (
    DECISION_FOLLOW_LINE,
    DECISION_STOP_FOR_RED,
    PATH_LINE_VISIBLE,
    TRAFFIC_RED,
    Freshness,
    PipelineSnapshot,
    build_warnings,
    command_label,
    normalize_class_name,
    summarize_detections,
    twists_close,
)


class Vector:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z


class Twist:
    def __init__(self, linear_x=0.0, angular_z=0.0):
        self.linear = Vector(x=linear_x)
        self.angular = Vector(z=angular_z)


class BBox:
    def __init__(self, width, height):
        self.size = Vector(x=width, y=height)


class Detection:
    def __init__(self, class_name, score, width, height):
        self.class_name = class_name
        self.score = score
        self.bbox = BBox(width, height)


def fresh(name):
    return Freshness(name=name, timeout_sec=1.0, age_sec=0.1, stale=False, missing=False)


def freshness():
    return {
        "path_state": fresh("path_state"),
        "cmd_vel_line": fresh("cmd_vel_line"),
        "yolo_detections": fresh("yolo_detections"),
        "traffic_light_state": fresh("traffic_light_state"),
        "decision_state": fresh("decision_state"),
        "cmd_vel": fresh("cmd_vel"),
    }


def test_normalize_class_name_matches_yolo_variants():
    assert normalize_class_name("Traffic_Light") == "traffic light"
    assert normalize_class_name(" traffic-light  ") == "traffic light"


def test_summarize_detections_counts_traffic_light_candidates():
    detections = [
        Detection("person", 0.95, 10, 20),
        Detection("traffic_light", 0.70, 20, 30),
        Detection("Traffic Light", 0.90, 15, 25),
    ]

    summary = summarize_detections(detections, ["traffic light"])

    assert summary.detection_count == 3
    assert summary.traffic_light_count == 2
    assert summary.best_score == 0.90
    assert summary.best_area == 375
    assert summary.best_class_name == "Traffic Light"


def test_follow_line_command_passes_when_cmd_matches_line_cmd():
    cmd = Twist(linear_x=20.0, angular_z=-0.12)
    snapshot = PipelineSnapshot(
        path_state=PATH_LINE_VISIBLE,
        traffic_light_state="GREEN",
        decision_state=DECISION_FOLLOW_LINE,
        cmd_vel=cmd,
        cmd_vel_line=Twist(linear_x=20.0, angular_z=-0.1205),
        freshness=freshness(),
    )

    assert command_label(snapshot, 0.001, 0.001) == "PASS"
    assert twists_close(snapshot.cmd_vel, snapshot.cmd_vel_line, 0.001)


def test_follow_line_zero_command_is_still_pass_when_line_cmd_matches():
    snapshot = PipelineSnapshot(
        path_state=PATH_LINE_VISIBLE,
        traffic_light_state="GREEN",
        decision_state=DECISION_FOLLOW_LINE,
        cmd_vel=Twist(),
        cmd_vel_line=Twist(),
        freshness=freshness(),
    )

    assert command_label(snapshot, 0.001, 0.001) == "PASS"


def test_red_light_follow_line_produces_decision_and_pass_warnings():
    snapshot = PipelineSnapshot(
        path_state=PATH_LINE_VISIBLE,
        traffic_light_state=TRAFFIC_RED,
        decision_state=DECISION_FOLLOW_LINE,
        cmd_vel=Twist(linear_x=20.0, angular_z=0.2),
        cmd_vel_line=Twist(linear_x=10.0, angular_z=0.1),
        freshness=freshness(),
    )

    warnings = build_warnings(snapshot, 0.001, 0.001)

    assert "decision_mismatch traffic=RED decision=FOLLOW_LINE" in warnings
    assert any(warning.startswith("pass_mismatch") for warning in warnings)


def test_stop_decision_warns_when_cmd_vel_is_nonzero():
    snapshot = PipelineSnapshot(
        path_state=PATH_LINE_VISIBLE,
        traffic_light_state=TRAFFIC_RED,
        decision_state=DECISION_STOP_FOR_RED,
        cmd_vel=Twist(linear_x=0.1, angular_z=0.0),
        cmd_vel_line=Twist(linear_x=20.0, angular_z=0.0),
        freshness=freshness(),
    )

    warnings = build_warnings(snapshot, 0.001, 0.001)

    assert any(warning.startswith("cmd_mismatch") for warning in warnings)
