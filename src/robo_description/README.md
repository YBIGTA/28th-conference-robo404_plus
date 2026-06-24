# robo_description

URDF/Xacro description and **Gazebo Classic** simulation assets for the
**robo404+** 4WD acrylic smart-car (the yellow TT-motor chassis).

The 4-wheel, no-steering chassis is modelled as a **skid-steer / differential
drive** robot with a forward-facing camera, matching the `follower` line-follow
node (`cmd_vel` in, camera `Image` out).

## Layout

```
urdf/
  robo404.urdf.xacro    # main model (links, joints, dimensions)
  macros.xacro          # inertia helper macros
  robo404.gazebo.xacro  # Gazebo materials + diff-drive & camera plugins
launch/
  gazebo.launch.py      # spawn into Gazebo Classic + robot_state_publisher
  display.launch.py     # view in RViz2 with joint sliders (no physics)
config/robo404.rviz     # RViz config
worlds/empty.world      # ground plane + sun
```

## Approximate dimensions (standard yellow 4WD kit)

| part | value |
|------|-------|
| chassis plate | 0.22 m × 0.145 m |
| wheel | Ø 0.066 m, 0.026 m wide |
| track (L–R wheel sep) | 0.171 m |
| wheelbase | 0.150 m |

Tune the `xacro:property` values at the top of `robo404.urdf.xacro` to match
your real car.

## Build

```bash
cd <workspace>          # the dir containing src/
colcon build --packages-select robo_description
source install/setup.bash
```

## Run

View the model in RViz2:

```bash
ros2 launch robo_description display.launch.py
```

Simulate in Gazebo Classic:

```bash
ros2 launch robo_description gazebo.launch.py
```

Drive it with the keyboard:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## ROS interface (Gazebo)

| topic | type | direction |
|-------|------|-----------|
| `/cmd_vel` | `geometry_msgs/Twist` | command (in) |
| `/odom` | `nav_msgs/Odometry` | odometry (out) |
| `/camera/image_raw` | `sensor_msgs/Image` | camera (out) |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | camera (out) |

To feed the `follower` node, remap its image subscription to
`/camera/image_raw` (and confirm it publishes to `/cmd_vel`).
