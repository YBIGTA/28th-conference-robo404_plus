#!/usr/bin/env bash
# One-click RL demo: clean launch + auto-start driving.
# Car drives from the start, passes 2 green lights, stops at the 3rd red.
# Record the Gazebo window yourself.
#
#   ./demo.sh
#
PROJECT=/home/jiucai/28th-conference-robo404_plus
MAX_SPEED=${MAX_SPEED:-0.30}

cd "$PROJECT"
source /opt/ros/foxy/setup.bash 2>/dev/null
source install/setup.bash 2>/dev/null
export DISPLAY=${DISPLAY:-:0}

echo "==> Cleaning up any old sim ..."
for p in "ros2 launch robo_description" gzserver gzclient rl_follower \
         sim_yolo traffic_light_node decision_node demo_orchestrator; do
    pkill -9 -f "$p" 2>/dev/null
done
sleep 3

echo "==> Launching demo pipeline (this opens the Gazebo window) ..."
nohup ros2 launch robo_description rl_sim_test.launch.py \
    auto_demo:=false rviz:=false max_linear_speed:="$MAX_SPEED" \
    > /tmp/rl_demo_launch.log 2>&1 &

echo "==> Waiting for the robot to spawn ..."
for i in $(seq 1 40); do
    grep -q "Successfully spawned entity" /tmp/rl_demo_launch.log 2>/dev/null && break
    sleep 1
done
sleep 6   # let all nodes finish wiring + first camera frames arrive

echo ""
echo "✅ Sim is up and the car is sitting at the START line (not moving yet)."
echo "   1. Switch to the Gazebo window and START YOUR RECORDING."
echo "   2. Come back here and press ENTER to make the car drive."
echo ""
read -r -p "Press ENTER to start the car... " _

echo "==> GO! Car driving -> passes 2 greens -> stops at 3rd red."
ros2 service call /start_follower std_srvs/srv/Empty >/dev/null 2>&1
ros2 service call /start_driving  std_srvs/srv/Empty >/dev/null 2>&1

echo ""
echo "   It reaches the red light in ~18s. Stop recording after it stops."
echo "   To shut down the sim later:  pkill -9 -f gzserver"
