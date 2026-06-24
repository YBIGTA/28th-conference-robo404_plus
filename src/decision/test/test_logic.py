from decision.logic import (
    DECISION_FINAL_STOP,
    DECISION_FOLLOW_LINE,
    DECISION_IDLE,
    DECISION_LINE_LOST,
    DECISION_STOP_FOR_RED,
    DECISION_WAIT_GREEN,
    PATH_FINAL_STOP,
    PATH_LINE_LOST,
    PATH_LINE_VISIBLE,
    TRAFFIC_GREEN,
    TRAFFIC_RED,
    TRAFFIC_UNKNOWN,
    DecisionInput,
    decide,
)


def make_input(**overrides):
    values = {
        "motion_enabled": True,
        "path_state": PATH_LINE_VISIBLE,
        "traffic_light_state": TRAFFIC_UNKNOWN,
        "red_latched": False,
        "cmd_vel_line_timed_out": False,
        "path_state_timed_out": False,
    }
    values.update(overrides)
    return DecisionInput(**values)


def test_disabled_state_publishes_idle_and_zero_command():
    result = decide(make_input(motion_enabled=False))

    assert result.state == DECISION_IDLE
    assert result.pass_cmd_vel_line is False


def test_final_stop_has_priority_over_follow_line():
    result = decide(make_input(path_state=PATH_FINAL_STOP))

    assert result.state == DECISION_FINAL_STOP
    assert result.pass_cmd_vel_line is False


def test_path_timeout_is_line_lost():
    result = decide(make_input(path_state_timed_out=True))

    assert result.state == DECISION_LINE_LOST
    assert result.pass_cmd_vel_line is False


def test_cmd_vel_line_timeout_is_line_lost():
    result = decide(make_input(cmd_vel_line_timed_out=True))

    assert result.state == DECISION_LINE_LOST
    assert result.pass_cmd_vel_line is False


def test_line_lost_publishes_zero_command():
    result = decide(make_input(path_state=PATH_LINE_LOST))

    assert result.state == DECISION_LINE_LOST
    assert result.pass_cmd_vel_line is False


def test_red_light_stops_and_latches_red():
    result = decide(make_input(traffic_light_state=TRAFFIC_RED))

    assert result.state == DECISION_STOP_FOR_RED
    assert result.pass_cmd_vel_line is False
    assert result.red_latched is True


def test_unknown_after_red_waits_for_green():
    result = decide(
        make_input(
            red_latched=True,
            traffic_light_state=TRAFFIC_UNKNOWN,
        )
    )

    assert result.state == DECISION_WAIT_GREEN
    assert result.pass_cmd_vel_line is False
    assert result.red_latched is True


def test_green_after_red_returns_to_follow_line():
    result = decide(
        make_input(
            red_latched=True,
            traffic_light_state=TRAFFIC_GREEN,
        )
    )

    assert result.state == DECISION_FOLLOW_LINE
    assert result.pass_cmd_vel_line is True
    assert result.red_latched is False


def test_unknown_before_red_follows_line():
    result = decide(make_input(traffic_light_state=TRAFFIC_UNKNOWN))

    assert result.state == DECISION_FOLLOW_LINE
    assert result.pass_cmd_vel_line is True


def test_unknown_path_state_is_treated_as_line_lost():
    result = decide(make_input(path_state="BROKEN"))

    assert result.state == DECISION_LINE_LOST
    assert result.pass_cmd_vel_line is False
