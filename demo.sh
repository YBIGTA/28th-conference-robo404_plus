#!/usr/bin/env bash
# One-click line-following demo, PID or RL mode.
#
#   ./demo.sh rl      # RL (SAC) follower
#   ./demo.sh pid     # classic PID follower
#   ./demo.sh both    # run PID first, then RL (sequential, same track)
#   ./demo.sh         # defaults to rl
#
# Scenario (auto-demo): car waits at 1st red -> turns green -> drives ->
# passes 2nd green -> stops at 3rd red -> waits -> turns green -> continues.
# Record the Gazebo window yourself.
#
PROJECT=/home/jiucai/28th-conference-robo404_plus
MODE="${1:-rl}"
MAX_SPEED="${MAX_SPEED:-0.35}"   # RL: max_linear_speed ; PID: linear_speed

cd "$PROJECT"
source /opt/ros/foxy/setup.bash 2>/dev/null
source install/setup.bash 2>/dev/null
export DISPLAY="${DISPLAY:-:0}"

cleanup() {
    for p in "ros2 launch robo_description" gzserver gzclient \
             rl_follower follower_node sim_yolo traffic_light_node \
             decision_node demo_orchestrator; do
        pkill -9 -f "$p" 2>/dev/null
    done
    sleep 3
}

run_one() {
    local mode="$1"
    cleanup
    if [ "$mode" = "pid" ]; then
        echo "==> Launching PID demo ..."
        nohup ros2 launch robo_description sim_test.launch.py \
            auto_demo:=true rviz:=false linear_speed:="$MAX_SPEED" \
            > /tmp/demo_launch.log 2>&1 &
    else
        echo "==> Launching RL demo ..."
        nohup ros2 launch robo_description rl_sim_test.launch.py \
            auto_demo:=true rviz:=false max_linear_speed:="$MAX_SPEED" \
            > /tmp/demo_launch.log 2>&1 &
    fi
    local pid=$!
    echo "    launch PID $pid  (mode=$mode)"
    for i in $(seq 1 40); do
        grep -q "Successfully spawned entity" /tmp/demo_launch.log 2>/dev/null && break
        sleep 1
    done
    sleep 6
    echo "✅ ${mode^^} demo running. Auto-scenario in progress -- record the Gazebo window."
    echo "$pid"
}

case "$MODE" in
    pid|rl)
        run_one "$MODE" >/dev/null
        echo ""
        echo "Running. Stop with:  pkill -9 -f gzserver"
        ;;
    both)
        echo "### PID run first ###"
        run_one pid >/dev/null
        echo "   (let the PID car finish the course, then press ENTER for RL)"
        read -r _
        echo "### RL run ###"
        run_one rl >/dev/null
        echo ""
        echo "Both shown. Stop with:  pkill -9 -f gzserver"
        ;;
    *)
        echo "usage: ./demo.sh [pid|rl|both]"
        exit 1
        ;;
esac
