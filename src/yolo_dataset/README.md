# yolo_dataset

Auto-labelling **data pipeline** for the Robo404 YOLO detector.

Drives the car around **Gazebo** sim with a classical autopilot and records
frames that are **fully auto-labelled from ground truth** — no manual boxing —
then exports them in **Ultralytics YOLO** format ready for fine-tuning.

---

## Full Pipeline Overview

```
1. Build worlds    ──►  5 domain-randomised .world files
2. Collect data    ──►  images/ + labels/ per world (YOLO format)
3. Merge & split   ──►  train/val split BY WORLD (no data leakage)
4. Train (4090)    ──►  yolov8n fine-tune
5. Export          ──►  best.onnx for Jetson Nano deployment
```

---

## Prerequisites

```bash
# ROS2 Foxy + Gazebo Classic (already on the sim machine)
sudo apt install ros-foxy-gazebo-ros-pkgs ros-foxy-gazebo-msgs

# Build workspace
cd ~/28th-conference-robo404_plus
colcon build --packages-select robo_description follower yolo_dataset --symlink-install
source install/setup.bash
```

---

## Step 1 — Generate Diverse Worlds

We use **domain randomisation** to prevent overfitting to a single sim scene.
5 pre-built worlds are included with varied:

| World | Track layout | Ground | Lighting | Traffic light | Distractors |
|-------|-------------|--------|----------|---------------|-------------|
| `world_l_turn` | 90° right turn | White | Bright neutral | Custom, facing car | Cones, hydrant |
| `world_s_curve` | S-shaped curves | Grey | Dim overcast | Realistic (Gazebo DB) | Barrier, sign, postbox |
| `world_u_turn` | 180° hairpin | Grass | Warm afternoon | Custom, short pole | Trees, hydrant |
| `world_oval_loop` | Closed oval circuit | Wood | Cool indoor | Realistic (Gazebo DB) | Walls, bricks, cone |
| `world_zigzag` | W-shaped zigzag | White | Dusk orange | Custom, glow lights | Lamp, person, SUV |

To **regenerate** or **customise** the worlds:

```bash
python3 src/follower/scripts/build_worlds.py
```

The generator script (`build_worlds.py`) computes arc/curve segments
programmatically and orients every traffic light to face the approaching car.

World files live in `src/follower/worlds/`.

---

## Step 2 — Collect Data (one world at a time)

Each world requires 3 terminals. Repeat for every world.

```bash
# Terminal 1 — Gazebo + Robot
ros2 launch robo_description gazebo.launch.py \
    world:=$(ros2 pkg prefix follower --share)/worlds/world_l_turn.world

# Terminal 2 — Autopilot (line follower)
ros2 run follower line_tracker_node

# Terminal 3 — Recorder
ros2 launch yolo_dataset record.launch.py \
    output_dir:=dataset/world_l_turn \
    record_rate:=5.0
```

Let it run for **60 seconds** per world (~300 frames at 5 Hz), then Ctrl-C
the recorder and kill Gazebo. Repeat with the next world:

```bash
# Repeat for each world:
#   world_l_turn, world_s_curve, world_u_turn, world_oval_loop, world_zigzag
```

Or use the batch script to automate all 5:

```bash
bash src/yolo_dataset/scripts/collect_dataset.sh \
    src/follower/worlds \
    dataset \
    60
```

### Output per world

```
dataset/world_l_turn/
  images/000000.png ...           # 640x480 camera frames
  labels/000000.txt ...           # YOLO format: "<cls> <cx> <cy> <w> <h>"
  frames.jsonl                    # rich metadata (bbox, distance, action, state)
  dataset.yaml                    # Ultralytics config
```

### Important: Traffic Light State

The recorder reads `/traffic_light/state` (`std_msgs/String`: `"red"` or
`"green"`) to decide the class. **You must publish this topic** from a traffic
light controller node. Without it, lights are left unlabelled.

### Disk estimate

| Duration/world | 5 worlds total | Disk |
|----------------|---------------|------|
| 30 s | ~750 frames | ~50 MB |
| 60 s | ~1500 frames | ~105 MB |
| 120 s | ~3000 frames | ~205 MB |

---

## Step 3 — Merge & Split (by world, NOT by frame)

**Critical**: split train/val by world, not by random frame sampling.
Frames from the same trajectory are near-duplicates — random splitting causes
data leakage and inflated val accuracy.

```python
# merge_dataset.py — run after collection
import shutil, os, yaml, random
from pathlib import Path

SRC = Path("dataset")
OUT = Path("dataset_merged")
WORLDS = ["world_l_turn", "world_s_curve", "world_u_turn",
          "world_oval_loop", "world_zigzag"]

# Hold out 1 world for validation
random.seed(42)
val_world = random.choice(WORLDS)
train_worlds = [w for w in WORLDS if w != val_world]
print(f"Val world: {val_world}")
print(f"Train worlds: {train_worlds}")

for split, worlds in [("train", train_worlds), ("val", [val_world])]:
    img_dir = OUT / split / "images"
    lbl_dir = OUT / split / "labels"
    img_dir.mkdir(parents=True, exist_ok=True)
    lbl_dir.mkdir(parents=True, exist_ok=True)

    idx = 0
    for w in worlds:
        src_img = SRC / w / "images"
        src_lbl = SRC / w / "labels"
        if not src_img.exists():
            continue
        for img_file in sorted(src_img.glob("*.png")):
            lbl_file = src_lbl / img_file.with_suffix(".txt").name
            new_name = f"{idx:06d}"
            shutil.copy2(img_file, img_dir / f"{new_name}.png")
            if lbl_file.exists():
                shutil.copy2(lbl_file, lbl_dir / f"{new_name}.txt")
            idx += 1
    print(f"  {split}: {idx} frames from {len(worlds)} world(s)")

# Write dataset.yaml
cfg = {
    "path": str(OUT.resolve()),
    "train": "train/images",
    "val": "val/images",
    "names": {0: "red_light", 1: "green_light", 2: "obstacle"},
}
with open(OUT / "dataset.yaml", "w") as f:
    yaml.dump(cfg, f, default_flow_style=False)
print(f"Config: {OUT / 'dataset.yaml'}")
```

```bash
python3 merge_dataset.py
```

Result:
```
dataset_merged/
  dataset.yaml
  train/
    images/  labels/        # 4 worlds (~1200 frames)
  val/
    images/  labels/        # 1 held-out world (~300 frames)
```

---

## Step 4 — Train on 4090

On the 4090 machine:

```bash
pip install ultralytics
```

Transfer the `dataset_merged/` folder to the 4090, then:

```python
from ultralytics import YOLO

model = YOLO("yolov8n.pt")       # pretrained COCO weights
model.train(
    data="dataset_merged/dataset.yaml",
    epochs=50,
    imgsz=640,
    batch=16,
    device=0,                     # GPU 0 (the 4090)
    project="runs/traffic_light",
    name="v1",
)
```

Or via CLI:
```bash
yolo detect train \
    data=dataset_merged/dataset.yaml \
    model=yolov8n.pt \
    epochs=50 imgsz=640 batch=16 device=0 \
    project=runs/traffic_light name=v1
```

Training should take **~5-10 minutes** on a 4090 with ~1500 frames.

Check results in `runs/traffic_light/v1/`:
- `weights/best.pt` — best checkpoint
- `results.png` — loss & mAP curves
- `confusion_matrix.png` — per-class accuracy
- `val_batch*_pred.png` — prediction samples on val set

---

## Step 5 — Export to ONNX for Jetson Nano

```python
from ultralytics import YOLO

model = YOLO("runs/traffic_light/v1/weights/best.pt")
model.export(
    format="onnx",
    imgsz=640,
    simplify=True,       # onnx-simplifier for Nano compatibility
    opset=11,            # TensorRT-friendly opset
)
# Output: runs/traffic_light/v1/weights/best.onnx
```

Transfer `best.onnx` to the Jetson Nano and load it in the
`yolo_detector_node` inference pipeline.

---

## Why This Approach

### Overfitting risks (two kinds)

1. **Sim-to-real gap** — model learns Gazebo textures/lighting, fails on real
   car. **Fix**: domain randomisation (5 diverse worlds with varied ground,
   lighting, distractors) + a small real-data fine-tune later.

2. **Intra-dataset overfitting** — consecutive frames from one trajectory are
   near-duplicates. **Fix**: split train/val by world, not by frame. Sim
   metrics only tell half the story — always validate on real images.

### Why NOT parallel collection

N identical sims in parallel just produces more correlated data. Diversity
matters more than volume. Single sim, sequential world switching is
sufficient for a fine-tune on 3 classes.

---

## Config Reference

### `config/objects.yaml`

| Field | Description |
|-------|-------------|
| `classes` | YOLO class list: `[red_light, green_light, obstacle]` |
| `camera_chain` | `base_footprint → camera_optical_link` transforms (match URDF) |
| `objects` | Gazebo model-name substring → `{type, class, half_extents}` |

### Recorder parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `output_dir` | `robo404_dataset` | Output directory |
| `record_rate` | `5.0` | Max frames/sec saved |
| `min_distance_m` | `0.2` | Ignore objects closer than this |
| `max_distance_m` | `8.0` | Ignore objects farther than this |
| `model_states_topic` | `/model_states` | Gazebo ground-truth poses |
| `robot_model_name` | `robo404` | Robot name in Gazebo model list |

---

## Tests

```bash
cd src/yolo_dataset
PYTHONPATH=. python3 test/test_projection.py
PYTHONPATH=. python3 test/test_schema.py
```
