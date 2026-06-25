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

PATH_STATES = {
    PATH_LINE_VISIBLE,
    PATH_LINE_LOST,
    PATH_FINAL_STOP,
}

TRAFFIC_LIGHT_STATES = {
    TRAFFIC_UNKNOWN,
    TRAFFIC_RED,
    TRAFFIC_GREEN,
}


class DecisionInput:
    def __init__(
        self,
        motion_enabled,
        path_state,
        traffic_light_state,
        red_latched,
        cmd_vel_line_timed_out,
        path_state_timed_out,
    ):
        self.motion_enabled = motion_enabled
        self.path_state = path_state
        self.traffic_light_state = traffic_light_state
        self.red_latched = red_latched
        self.cmd_vel_line_timed_out = cmd_vel_line_timed_out
        self.path_state_timed_out = path_state_timed_out


class DecisionResult:
    def __init__(self, state, pass_cmd_vel_line, red_latched):
        self.state = state
        self.pass_cmd_vel_line = pass_cmd_vel_line
        self.red_latched = red_latched


def normalize_path_state(path_state):
    if path_state in PATH_STATES:
        return path_state
    return PATH_LINE_LOST


def normalize_traffic_light_state(traffic_light_state):
    if traffic_light_state in TRAFFIC_LIGHT_STATES:
        return traffic_light_state
    return TRAFFIC_UNKNOWN


def decide(inputs):
    path_state = normalize_path_state(inputs.path_state)
    traffic_light_state = normalize_traffic_light_state(inputs.traffic_light_state)
    red_latched = inputs.red_latched

    if traffic_light_state == TRAFFIC_RED:
        red_latched = True
    else:
        # GREEN or UNKNOWN both release the latch: as on a real road, when no
        # light is clearly visible (UNKNOWN) the car is allowed to proceed.
        # Only an actively-seen RED holds it.
        red_latched = False

    if not inputs.motion_enabled:
        return DecisionResult(DECISION_IDLE, False, red_latched)

    if path_state == PATH_FINAL_STOP:
        return DecisionResult(DECISION_FINAL_STOP, False, red_latched)

    if inputs.cmd_vel_line_timed_out or inputs.path_state_timed_out:
        return DecisionResult(DECISION_LINE_LOST, False, red_latched)

    if path_state == PATH_LINE_LOST:
        return DecisionResult(DECISION_LINE_LOST, True, red_latched)

    if traffic_light_state == TRAFFIC_RED:
        return DecisionResult(DECISION_STOP_FOR_RED, False, red_latched)

    if red_latched and traffic_light_state != TRAFFIC_GREEN:
        return DecisionResult(DECISION_WAIT_GREEN, False, red_latched)

    return DecisionResult(DECISION_FOLLOW_LINE, True, red_latched)
