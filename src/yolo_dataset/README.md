# yolo_dataset

Auto-labelling **data pipeline** for the Robo404 YOLO detector.

This is the one piece the whole "drop VLA, retrain YOLO" plan actually needs.
It drives the car around the **Gazebo** sim with a classical autopilot and
records frames that are **fully auto-labelled from ground truth** — no manual
boxing — then exports them in **Ultralytics YOLO** format ready for training.

## Why sim auto-labelling

YOLO failed on traffic lights because of domain shift; the fix is retraining on
domain-matched data. Hand-labelling is the bottleneck. In Gazebo we already know
every object's 3D pose, so we can:

| label | source (free, exact) |
|-------|----------------------|
| 2D bounding box | project the object's 3D box through the camera (`projection.py`) |
| distance | object centre in the camera frame |
| traffic-light state | `red_light` / `green_light` as separate classes |
| expert action | the `/cmd_vel` the autopilot is commanding |

Train YOLO on the boxes; keep the rest (distance, state, action) to validate the
rule-based `decision` layer. Then fine-tune on a small **real** set for sim-to-real.

## Outputs

```
<output_dir>/
  images/000000.png ...
  labels/000000.txt ...   # Ultralytics: "<cls> <cx> <cy> <w> <h>" normalized
  frames.jsonl            # rich per-frame record (boxes + distance + state + action)
  dataset.yaml            # Ultralytics data config (class names)
```

`images/` + `labels/` + `dataset.yaml` train YOLO directly:
```bash
yolo detect train data=<output_dir>/dataset.yaml model=yolov8n.pt epochs=100 imgsz=640
```

## Run

```bash
colcon build --packages-select yolo_dataset && source install/setup.bash

# 1) sim + camera + diff-drive
ros2 launch robo_description gazebo.launch.py
# 2) a classical expert driving /cmd_vel (line follower)
ros2 run follower follower_node
# 3) the recorder
ros2 launch yolo_dataset record.launch.py output_dir:=runs/track1
```

The traffic-light state is read from `/traffic_light/state` (`std_msgs/String`,
`"red"`/`"green"`). Publish it from your light model/controller; without it,
lights are left unlabelled (so you never train on an unknown state).

## Config — `config/objects.yaml`

- `classes`: YOLO class list (red_light, green_light, obstacle).
- `camera_chain`: `base_footprint → camera_optical_link` (defaults match the
  `robo_description` URDF; change if you move the camera).
- `objects`: Gazebo model-name substring → `{type, half_extents}`. `traffic_light`
  type resolves to `red_light`/`green_light` from the state topic; `static` types
  use a fixed class.

Key parameters (override via launch args / `ros2 param`):
`output_dir`, `record_rate` (Hz cap on saved frames), `min_distance_m`,
`max_distance_m`, `robot_model_name`, and the topic names.

## Design notes / caveats

- **red/green as classes** (not `traffic_light` + runtime HSV): in sim the state
  is known, so labelling the right class is free and the detector outputs the
  state directly — no HSV gate at runtime.
- **Distance** here is exact (sim ground truth). The bbox-size heuristic discussed
  for the *real* car is a cruder monocular fallback; in sim you don't need it.
- The geometry (`projection.py`) is unit-tested (`test/`). The **ROS node has not
  been run against a live Gazebo from this machine** — validate `camera_chain`
  and the `/gazebo/model_states` model names on the ROS box (a wrong extrinsic
  shows up immediately as boxes that don't sit on the objects; dump a few frames
  and eyeball `bbox_xyxy_px`).

## Tests

```bash
cd src/yolo_dataset
PYTHONPATH=. python3 test/test_projection.py
PYTHONPATH=. python3 test/test_schema.py
```
