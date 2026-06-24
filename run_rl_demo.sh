#!/usr/bin/env bash
# One-click RL line-following demo with screen recording.
#
#   ./run_rl_demo.sh
#
# Launches the full sim pipeline (RL follower + YOLO + traffic light +
# decision), starts driving, and records the screen to an mp4. The car
# drives from the start, passes the two green lights, and stops at the
# 3rd red light.
#
# Output video:  ./rl_demo_<timestamp>.mp4
PROJECT=/home/jiucai/28th-conference-robo404_plus
REC_SECONDS=${REC_SECONDS:-40}        # how long to record
MAX_SPEED=${MAX_SPEED:-0.30}          # sim drive speed (real robot stays 0.05)
OUT="$PROJECT/rl_demo_$(date +%Y%m%d_%H%M%S).mp4"

cd "$PROJECT"
# NOTE: ROS setup scripts reference unset vars, so do NOT use `set -u` here.
source /opt/ros/foxy/setup.bash 2>/dev/null
source install/setup.bash 2>/dev/null
export DISPLAY=${DISPLAY:-:0}

echo "==> Cleaning up any running sim ..."
pkill -9 -f "ros2 launch robo_description" 2>/dev/null
pkill -9 -f gzserver 2>/dev/null; pkill -9 -f gzclient 2>/dev/null
pkill -9 -f rl_follower 2>/dev/null; pkill -9 -f sim_yolo 2>/dev/null
pkill -9 -f traffic_light_node 2>/dev/null; pkill -9 -f decision_node 2>/dev/null
pkill -9 -f demo_orchestrator 2>/dev/null
sleep 3

echo "==> Launching RL demo pipeline (green/green/red world) ..."
nohup ros2 launch robo_description rl_sim_test.launch.py \
    auto_demo:=false rviz:=false max_linear_speed:="$MAX_SPEED" \
    > /tmp/rl_demo_launch.log 2>&1 &
LAUNCH_PID=$!

echo "==> Waiting for the robot to spawn ..."
for i in $(seq 1 40); do
    grep -q "Successfully spawned entity" /tmp/rl_demo_launch.log 2>/dev/null && break
    sleep 1
done
sleep 5    # let nodes finish wiring up

echo "==> Starting the car ..."
ros2 service call /start_follower std_srvs/srv/Empty >/dev/null 2>&1
ros2 service call /start_driving  std_srvs/srv/Empty >/dev/null 2>&1

# Detect full screen size so we capture the whole display, not a 640x480 corner.
SCREEN_SIZE=$(xdpyinfo -display "$DISPLAY" 2>/dev/null | awk '/dimensions:/{print $2; exit}')
SCREEN_SIZE=${SCREEN_SIZE:-1920x1080}
echo "==> Recording ${REC_SECONDS}s of the screen (${SCREEN_SIZE}) -> $OUT"
ffmpeg -y -f x11grab -framerate 25 -video_size "$SCREEN_SIZE" -i "$DISPLAY" \
    -t "$REC_SECONDS" -c:v libx264 -preset ultrafast -pix_fmt yuv420p \
    -vf "scale=1280:-2" \
    "$OUT" 2>/tmp/rl_demo_ffmpeg.log
RC=$?

echo "==> Stopping the sim ..."
kill -9 "$LAUNCH_PID" 2>/dev/null
pkill -9 -f gzserver 2>/dev/null; pkill -9 -f gzclient 2>/dev/null
pkill -9 -f rl_follower 2>/dev/null; pkill -9 -f sim_yolo 2>/dev/null
pkill -9 -f traffic_light_node 2>/dev/null; pkill -9 -f decision_node 2>/dev/null

if [ "$RC" -eq 0 ] && [ -s "$OUT" ]; then
    echo ""
    echo "✅ Demo recorded: $OUT"
    ls -lh "$OUT"
else
    echo "⚠️  Recording failed. See /tmp/rl_demo_ffmpeg.log"
    tail -5 /tmp/rl_demo_ffmpeg.log
fi
