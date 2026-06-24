# robo_description — Gazebo simulation of the robo404+ pipeline

Desktop (x86 / ARM, no Jetson/TensorRT required) Gazebo Classic simulation of
the full robo404+ software pipeline: a differential-drive car that line-follows
a track, detects traffic lights with the finetuned YOLO model, and stops/goes
according to the decision logic. Built for demos and presentation recordings.

## Quick start

```bash
# Build
cd ~/28th-conference-robo404_plus
colcon build --packages-select follower robo_description --symlink-install
source install/setup.bash

# Launch everything (Gazebo + RViz + nodes + auto-demo)
ros2 launch robo_description sim_test.launch.py
```

The launch auto-starts the car a few seconds in, and switches each red light to
green once the car stops in front of it, so it runs hands-off.

### Useful launch args
| arg | default | meaning |
|-----|---------|---------|
| `gui` | `true` | show the Gazebo client window |
| `rviz` | `true` | open RViz (3D car + both camera feeds) |
| `auto_demo` | `true` | auto start-driving + auto red→green light switching |
| `model_path` | bundled `.pt` | finetuned YOLO weights for the sim YOLO node |
| `x_pose`/`y_pose`/`yaw` | derived from `track_path` | spawn pose (auto-matches line start) |

To drive manually instead of the auto-demo: `auto_demo:=false`, then call
`/start_follower` and `/start_driving` (both `std_srvs/srv/Empty`).

## Architecture / data flow

```
 bottom camera ─/camera/image_raw──────────▶ follower_node ─┬─/cmd_vel_line─▶ decision_node ─/cmd_vel─▶ diff_drive (Gazebo)
                                                            └─/path_state──▶
 top camera ───/camera/rgb/image_raw─▶ sim_yolo_node ─/yolo/detections─▶ traffic_light_node ─/traffic_light_state─▶ decision_node
                                              └────────────/yolo/dbg_image (annotated, for RViz)
```

- **follower_node** (`follower` pkg) — line tracking from the bottom camera;
  publishes a candidate `/cmd_vel_line` and `/path_state`.
- **sim_yolo_node** (`scripts/sim_yolo_node.py`) — desktop replacement for the
  Jetson TensorRT path. Loads the finetuned ultralytics model
  (`models_yolo/best_traffic_nano_yolo.pt`, classes red/green/off/yellow),
  runs CPU inference on the top camera, publishes `yolo_msgs/DetectionArray` on
  `/yolo/detections` plus an annotated `/yolo/dbg_image`.
- **traffic_light_node** (`traffic_light` pkg) — classifies the light state.
  Class names `red`/`green` from the YOLO node map directly to the state, so
  `simulation_mode` (HSV shortcut) is **off** here — the real model drives it.
- **decision_node** (`decision` pkg) — FSM arbitrating `/cmd_vel`. Red latches a
  stop until green is seen; line-loss / timeouts stop the car.
- **demo_orchestrator** (`scripts/demo_orchestrator.py`) — hands-off demo:
  auto-calls the start services, then swaps each red light prop for a green one
  (delete_entity + spawn_entity) when the car stops in front of it, with a
  cooldown so one stop can't trigger two switches.

## Track

Defined once in `scripts/track_path.py` (single source of truth) and consumed by
both generators, so the painted line and the prop placement always agree:

- **`scripts/track_path.py`** — the centerline as a sequence of
  (length, turn-angle) primitives, integrated to points. Also exposes
  `start_pose()` (used by the launch spawn) and `light_world_poses()`.
- **`scripts/gen_track_texture.py`** — paints `track.png` (the ground texture).
- **`scripts/gen_world.py`** — writes `worlds/traffic_light_track.world` with the
  ground + 3 traffic-light props placed along the path.

Shape: a **stadium loop matching a 400 m athletics track**, at **real scale**
(~1.0 m straights, ~0.43 m bend radius → straight/radius ≈ 2.31). Ground is
4×4 m. Two straights + two 180° U-turns; the loop closes on itself.

To change the track, edit `track_path.py` then regenerate:
```bash
cd src/robo_description/scripts
python3 gen_world.py && python3 gen_track_texture.py
colcon build --packages-select robo_description --symlink-install
```

## Robot model

- `urdf/robo404.urdf.xacro` + `urdf/robo404.gazebo.xacro` — 4-wheel skid-steer
  differential drive, two cameras:
  - **bottom camera** → `/camera/image_raw` (line follower). 90° FOV, pitched
    ~30° down — wide/low so the line stays in frame through the tight U-turns.
  - **top camera** → `/camera/rgb/image_raw` (YOLO / traffic light).
- Diff-drive turns by differential wheel speed (can turn in place); there is no
  minimum turning radius. When the car "can't make a turn" in sim it is almost
  always the **line leaving the bottom camera's view**, not a turning limit —
  fix by widening the camera FOV / pitch or raising the follower steering gain.

## Sim tuning notes

The follower's real-robot constants are far too fast for the small sim car, so
`linear_speed` and `kp` are exposed as ROS params (defaults match the real
robot) and overridden in `sim_test.launch.py`:

| param | real default | sim value | why |
|-------|--------------|-----------|-----|
| `linear_speed` | 20.0 m/s | ~0.3 m/s | sim car is ~0.3 m; 20 m/s is unusable |
| `kp` | 0.015 | ~0.012 | steering gain for the line follower |

If the car loses the line on a turn: widen the bottom-camera FOV, increase its
downward pitch, raise `kp`, or slow `linear_speed` — in that order of impact.

## Known environment notes

- This machine is ARM64. ultralytics/torch hit a "cannot allocate memory in
  static TLS block" error unless torch's bundled `libgomp` is preloaded;
  `sim_yolo_node.py` handles this with a self-`LD_PRELOAD` re-exec at startup.
- Requires `gazebo_ros` (Gazebo Classic 11), `rviz2`, and the `follower`,
  `decision`, `traffic_light`, `yolo_msgs` packages built in the workspace.
