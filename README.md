# decoupled-perception-simulation

Decoupled ROS 2 Foxy perception pipeline (OpenCV Line Tracker and YOLOv8 Detector) running in a Gazebo simulation environment.

---

## ⚡ One-Click Launch (Recommended)

You can launch the entire system (Gazebo simulation, Line Tracker node, YOLO Detector node, Safety Arbiter node, and the graphical rqt_image_view visualizer) with a single command:

```bash
cd ~/28th-conference-robo404_plus
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch follower full_system.launch.py
```

---

## 🚗 How to Make the Car Move (Safety Arbiter & WASD Teleop)

Neither perception node is allowed to publish directly to `/cmd_vel`. Instead, the **Safety Arbiter** acts as the central coordinator:
1. **OpenCV Line Tracker** publishes to `/cmd_vel_path`.
2. **YOLOv8 Obstacle Detector** publishes to `/yolo_detections`.
3. **Safety Arbiter** decides if the path is safe (no obstacles too close) and forwards the correct commands to `/cmd_vel` to move the robot:
   - **Manual WASD Override**: You can manually drive the robot using the WASD keyboard node. Teleop commands take priority. If you stop typing WASD for 1.0 second, the robot automatically resumes autonomous line tracking!
   - **Emergency Stop (Collision Prevention)**: In both manual and autonomous mode, the Safety Arbiter will automatically block motion and trigger an Emergency Stop if you try to drive into the red obstacle box.

### 🎮 Manual Keyboard Teleop (WASD)
In a new terminal window, run the WASD teleop node:
```bash
cd ~/28th-conference-robo404_plus
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 run follower wasd_teleop_node
```
Use keys `w`, `a`, `s`, `d` to speed up/slow down/steer, and `space` / `x` to stop.

---

## 🚀 Step-by-Step Manual Commands Guide

If you prefer to start each component manually in separate terminals (make sure to run the setup sourcing in every tab):

### Step 1: Launch the Gazebo Simulation
Starts the Gazebo simulation loaded with the track, traffic lights, obstacle box, and the differential drive robot.
```bash
# Terminal 1
cd ~/28th-conference-robo404_plus
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch follower simulation.launch.py
```

---

### Step 2: View the Robot Camera Feed (Camera Command)
Launches the ROS image viewer. In the GUI window, select `/camera/image_raw` from the dropdown menu in the top-left corner to see what the robot sees.
```bash
# Terminal 2
cd ~/28th-conference-robo404_plus
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 run rqt_image_view rqt_image_view
```

---

### Step 3: Run the OpenCV Line Tracker Node
Starts the line tracking perception wrapper.
```bash
# Terminal 3
cd ~/28th-conference-robo404_plus
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 run follower line_tracker_node
```

---

### Step 4: Run the YOLOv8 Obstacle Detector Node
Starts the obstacle detector wrapper (operates in CV2 HSV fallback mode if `ultralytics` is not installed on the system).
```bash
# Terminal 4
cd ~/28th-conference-robo404_plus
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 run follower yolo_detector_node
```

---

### Step 5: Monitor Intermediate Perception Topics (Verification)
Neither perception node commands the robot directly. You can echo the safety-decoupled topics to see the commands and detections intended for the Safety Arbiter:

```bash
# Echo Line steering commands
ros2 topic echo /cmd_vel_path

# Echo Track path visibility status ("visible" / "lost")
ros2 topic echo /path_state

# Echo Obstacle bounding boxes and virtual proximity metrics
ros2 topic echo /yolo_detections
```

---

## 🧪 Running Unit Tests (Offline/TDD)

You can run the offline pytest suite to verify the core OpenCV math and YOLO fallback logic without launching Gazebo:

```bash
cd ~/28th-conference-robo404_plus
PYTHONPATH=src/follower python3 -m pytest src/follower/test
```
