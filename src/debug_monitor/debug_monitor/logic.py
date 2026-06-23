import re
from dataclasses import dataclass, field
from typing import Optional


STALE_VALUE = "STALE"

PATH_LINE_VISIBLE = "LINE_VISIBLE"
PATH_LINE_LOST = "LINE_LOST"
PATH_FINAL_STOP = "FINAL_STOP"

TRAFFIC_UNKNOWN = "UNKNOWN"
TRAFFIC_RED = "RED"
TRAFFIC_GREEN = "GREEN"

DECISION_IDLE = "IDLE"
DECISION_FOLLOW_LINE = "FOLLOW_LINE"
DECISION_STOP_FOR_RED = "STOP_FOR_RED"
DECISION_WAIT_GREEN = "WAIT_GREEN"
DECISION_LINE_LOST = "LINE_LOST"
DECISION_FINAL_STOP = "FINAL_STOP"

STOP_DECISIONS = {
    DECISION_IDLE,
    DECISION_STOP_FOR_RED,
    DECISION_WAIT_GREEN,
    DECISION_LINE_LOST,
    DECISION_FINAL_STOP,
}


@dataclass(frozen=True)
class Freshness:
    name: str
    timeout_sec: float
    age_sec: Optional[float] = None
    stale: bool = True
    missing: bool = True


@dataclass(frozen=True)
class DetectionSummary:
    detection_count: int = 0
    traffic_light_count: int = 0
    best_score: Optional[float] = None
    best_area: Optional[float] = None
    best_class_name: str = ""


@dataclass(frozen=True)
class PipelineSnapshot:
    path_state: str = STALE_VALUE
    traffic_light_state: str = STALE_VALUE
    decision_state: str = STALE_VALUE
    cmd_vel: Optional[object] = None
    cmd_vel_line: Optional[object] = None
    detection_summary: DetectionSummary = field(default_factory=DetectionSummary)
    freshness: dict = field(default_factory=dict)


TIMEOUT_WARNING_NAMES = {
    "path_state": "follower_timeout",
    "cmd_vel_line": "follower_timeout",
    "yolo_detections": "yolo_timeout",
    "traffic_light_state": "traffic_timeout",
    "decision_state": "decision_timeout",
    "cmd_vel": "cmd_vel_timeout",
}

REASON_BY_TIMEOUT_TOPIC = {
    "path_state": "follower_timeout",
    "cmd_vel_line": "follower_timeout",
    "yolo_detections": "yolo_timeout",
    "traffic_light_state": "traffic_timeout",
    "decision_state": "decision_timeout",
    "cmd_vel": "cmd_vel_timeout",
}


def normalize_state(value):
    if value is None:
        return STALE_VALUE
    return str(value).strip().upper() or STALE_VALUE


def normalize_class_name(class_name):
    normalized = str(class_name).strip().lower()
    normalized = normalized.replace("-", " ").replace("_", " ")
    return re.sub(r"\s+", " ", normalized)


def make_freshness(name, last_time, timeout_sec, now):
    timeout_sec = float(timeout_sec)
    if last_time is None:
        return Freshness(name=name, timeout_sec=timeout_sec)

    age_sec = max(0.0, float(now) - float(last_time))
    return Freshness(
        name=name,
        timeout_sec=timeout_sec,
        age_sec=age_sec,
        stale=age_sec > timeout_sec,
        missing=False,
    )


def is_topic_fresh(freshness, name):
    status = freshness.get(name)
    return status is not None and not status.stale


def twist_components(twist):
    if twist is None:
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    return (
        float(twist.linear.x),
        float(twist.linear.y),
        float(twist.linear.z),
        float(twist.angular.x),
        float(twist.angular.y),
        float(twist.angular.z),
    )


def is_zero_twist(twist, epsilon):
    return all(abs(value) <= epsilon for value in twist_components(twist))


def twists_close(left, right, epsilon):
    return all(
        abs(left_value - right_value) <= epsilon
        for left_value, right_value in zip(twist_components(left), twist_components(right))
    )


def format_twist_short(twist):
    if twist is None:
        return "(STALE)"
    return "({:.2f},{:.2f})".format(float(twist.linear.x), float(twist.angular.z))


def detection_bbox_area(detection):
    try:
        width = max(0.0, float(detection.bbox.size.x))
        height = max(0.0, float(detection.bbox.size.y))
    except AttributeError:
        return 0.0
    return width * height


def summarize_detections(detections, traffic_light_class_names):
    if isinstance(traffic_light_class_names, str):
        traffic_light_class_names = [traffic_light_class_names]

    class_names = {
        normalize_class_name(class_name)
        for class_name in traffic_light_class_names
    }
    summary = DetectionSummary(detection_count=len(detections))

    traffic_light_count = 0
    best_detection = None
    best_area = None

    for detection in detections:
        if normalize_class_name(detection.class_name) not in class_names:
            continue

        traffic_light_count += 1
        score = float(detection.score)
        area = detection_bbox_area(detection)
        if best_detection is None or score > float(best_detection.score):
            best_detection = detection
            best_area = area

    if best_detection is None:
        return DetectionSummary(
            detection_count=summary.detection_count,
            traffic_light_count=traffic_light_count,
        )

    return DetectionSummary(
        detection_count=summary.detection_count,
        traffic_light_count=traffic_light_count,
        best_score=float(best_detection.score),
        best_area=best_area,
        best_class_name=str(best_detection.class_name),
    )


def command_label(snapshot, zero_twist_epsilon, cmd_compare_epsilon):
    if not is_topic_fresh(snapshot.freshness, "cmd_vel"):
        return STALE_VALUE

    if (
        snapshot.decision_state == DECISION_FOLLOW_LINE
        and is_topic_fresh(snapshot.freshness, "cmd_vel_line")
        and twists_close(snapshot.cmd_vel, snapshot.cmd_vel_line, cmd_compare_epsilon)
    ):
        return "PASS"

    if is_zero_twist(snapshot.cmd_vel, zero_twist_epsilon):
        return "ZERO"

    return "NONZERO"


def build_warnings(snapshot, zero_twist_epsilon, cmd_compare_epsilon):
    warnings = []
    warnings.extend(freshness_warnings(snapshot.freshness))

    path_fresh = is_topic_fresh(snapshot.freshness, "path_state")
    traffic_fresh = is_topic_fresh(snapshot.freshness, "traffic_light_state")
    decision_fresh = is_topic_fresh(snapshot.freshness, "decision_state")
    cmd_fresh = is_topic_fresh(snapshot.freshness, "cmd_vel")
    cmd_line_fresh = is_topic_fresh(snapshot.freshness, "cmd_vel_line")
    yolo_fresh = is_topic_fresh(snapshot.freshness, "yolo_detections")

    if (
        path_fresh
        and decision_fresh
        and snapshot.path_state in {PATH_LINE_LOST, PATH_FINAL_STOP}
        and snapshot.decision_state not in STOP_DECISIONS
    ):
        warnings.append(
            "decision_mismatch path={} decision={}".format(
                snapshot.path_state,
                snapshot.decision_state,
            )
        )

    if (
        traffic_fresh
        and decision_fresh
        and snapshot.traffic_light_state == TRAFFIC_RED
        and snapshot.decision_state not in STOP_DECISIONS
    ):
        warnings.append(
            "decision_mismatch traffic=RED decision={}".format(
                snapshot.decision_state,
            )
        )

    if (
        decision_fresh
        and cmd_fresh
        and snapshot.decision_state in STOP_DECISIONS
        and not is_zero_twist(snapshot.cmd_vel, zero_twist_epsilon)
    ):
        warnings.append(
            "cmd_mismatch decision={} cmd_vel nonzero cmd_vel={}".format(
                snapshot.decision_state,
                format_twist_short(snapshot.cmd_vel),
            )
        )

    if (
        decision_fresh
        and cmd_fresh
        and cmd_line_fresh
        and snapshot.decision_state == DECISION_FOLLOW_LINE
        and not twists_close(snapshot.cmd_vel, snapshot.cmd_vel_line, cmd_compare_epsilon)
    ):
        warnings.append(
            "pass_mismatch decision=FOLLOW_LINE cmd_vel differs from cmd_vel_line "
            "cmd_vel={} cmd_vel_line={}".format(
                format_twist_short(snapshot.cmd_vel),
                format_twist_short(snapshot.cmd_vel_line),
            )
        )

    if (
        yolo_fresh
        and traffic_fresh
        and snapshot.traffic_light_state == TRAFFIC_UNKNOWN
        and snapshot.detection_summary.detection_count > 0
        and snapshot.detection_summary.traffic_light_count == 0
    ):
        warnings.append(
            "no_traffic_light_bbox detections={} traffic_candidates=0 "
            "traffic=UNKNOWN".format(snapshot.detection_summary.detection_count)
        )

    return warnings


def freshness_warnings(freshness):
    warnings = []
    for name, status in freshness.items():
        if not status.stale:
            continue

        warning_name = TIMEOUT_WARNING_NAMES.get(name, "{}_timeout".format(name))
        if status.missing:
            warnings.append("{} {} missing".format(warning_name, name))
        else:
            warnings.append(
                "{} {} stale age={:.2f}s timeout={:.2f}s".format(
                    warning_name,
                    name,
                    status.age_sec,
                    status.timeout_sec,
                )
            )
    return warnings


def build_pipeline_state(snapshot, warnings, zero_twist_epsilon, cmd_compare_epsilon):
    severity = "WARN" if warnings else "OK"
    yolo_tokens = format_yolo_tokens(snapshot)
    cmd = command_label(snapshot, zero_twist_epsilon, cmd_compare_epsilon)
    return "{} path={} traffic={} decision={} {} cmd={} cmd_vel={} reason={}".format(
        severity,
        state_or_stale(snapshot, "path_state", snapshot.path_state),
        state_or_stale(
            snapshot,
            "traffic_light_state",
            snapshot.traffic_light_state,
        ),
        state_or_stale(snapshot, "decision_state", snapshot.decision_state),
        yolo_tokens,
        cmd,
        format_twist_short(snapshot.cmd_vel)
        if is_topic_fresh(snapshot.freshness, "cmd_vel")
        else "(STALE)",
        derive_reason(snapshot, warnings),
    )


def format_warning_line(warnings):
    if not warnings:
        return "OK no_warnings"
    return "WARN " + " | ".join(warnings)


def format_yolo_tokens(snapshot):
    if not is_topic_fresh(snapshot.freshness, "yolo_detections"):
        return "yolo=STALE tl_bbox=STALE tl_score=NA tl_area=NA"

    summary = snapshot.detection_summary
    score = "NA" if summary.best_score is None else "{:.2f}".format(summary.best_score)
    area = "NA" if summary.best_area is None else "{:.0f}".format(summary.best_area)
    return "yolo={} tl_bbox={} tl_score={} tl_area={}".format(
        summary.detection_count,
        summary.traffic_light_count,
        score,
        area,
    )


def state_or_stale(snapshot, topic_name, value):
    if not is_topic_fresh(snapshot.freshness, topic_name):
        return STALE_VALUE
    return value


def derive_reason(snapshot, warnings):
    for name, status in snapshot.freshness.items():
        if status.stale:
            return REASON_BY_TIMEOUT_TOPIC.get(name, "topic_timeout")

    if warnings:
        first_warning = warnings[0]
        if first_warning.startswith("no_traffic_light_bbox"):
            return "no_traffic_light_bbox"
        if first_warning.startswith("decision_mismatch"):
            return "decision_mismatch"
        if first_warning.startswith("cmd_mismatch"):
            return "cmd_mismatch"
        if first_warning.startswith("pass_mismatch"):
            return "pass_mismatch"
        return "warning"

    if snapshot.decision_state == DECISION_STOP_FOR_RED:
        return "red_light"
    if snapshot.decision_state == DECISION_WAIT_GREEN:
        return "wait_green"
    if snapshot.decision_state == DECISION_LINE_LOST:
        return "line_lost"
    if snapshot.decision_state == DECISION_FINAL_STOP:
        return "final_stop"
    if snapshot.decision_state == DECISION_IDLE:
        return "idle"
    if snapshot.decision_state == DECISION_FOLLOW_LINE:
        return "follow_line"
    return "unknown"
