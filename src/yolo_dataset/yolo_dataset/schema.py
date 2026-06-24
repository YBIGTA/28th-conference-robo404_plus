"""
Dataset record schema + writer.

On-disk layout (one dataset = one output dir):

    <out>/
      images/000000.png ...
      labels/000000.txt ...      # Ultralytics YOLO: "<cls> <cx> <cy> <w> <h>" normalized
      frames.jsonl               # one rich record per frame (below)
      dataset.yaml               # Ultralytics data config (class names)

Rich per-frame record (frames.jsonl):

    {
      "frame_id": "000123",
      "stamp": 1234.56,
      "image": "images/000123.png",
      "width": 640, "height": 480,
      "detections": [
        {"class": "red_light", "class_id": 0,
         "bbox_xywh_norm": [cx,cy,w,h], "bbox_xyxy_px": [x1,y1,x2,y2],
         "distance_m": 3.2, "truncated": false}
      ],
      "light_state": "red",            # red | green | none
      "line_offset": null,             # optional, if a topic provides it
      "action": {"linear_x": 0.2, "angular_z": -0.1}   # expert /cmd_vel
    }

The YOLO labels/ + dataset.yaml train the detector directly; frames.jsonl keeps
everything else (distance, light_state, expert action) for validating the rules
and for any future imitation-learning use.
"""
import json
import os


class DatasetWriter:
    def __init__(self, out_dir, class_names):
        self.out = out_dir
        self.class_names = list(class_names)
        self.class_id = {c: i for i, c in enumerate(self.class_names)}
        self.images_dir = os.path.join(out_dir, "images")
        self.labels_dir = os.path.join(out_dir, "labels")
        os.makedirs(self.images_dir, exist_ok=True)
        os.makedirs(self.labels_dir, exist_ok=True)
        self._jsonl = open(os.path.join(out_dir, "frames.jsonl"), "a")
        self._count = 0
        self._write_dataset_yaml()

    def _write_dataset_yaml(self):
        path = os.path.join(self.out, "dataset.yaml")
        with open(path, "w") as f:
            f.write("# Ultralytics dataset config (auto-generated)\n")
            f.write("path: .\ntrain: images\nval: images\n\nnames:\n")
            for i, c in enumerate(self.class_names):
                f.write(f"  {i}: {c}\n")

    def image_path(self, frame_id):
        return os.path.join(self.images_dir, f"{frame_id}.png")

    def write(self, frame_id, width, height, detections, light_state, action, line_offset=None, stamp=None):
        """detections: list of dicts each with 'class' + projection.project_object output."""
        # YOLO label file
        lines = []
        rich = []
        for d in detections:
            cls = d["class"]
            cid = self.class_id.get(cls)
            if cid is None:
                continue
            cx, cy, w, h = d["bbox_xywh_norm"]
            lines.append(f"{cid} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}")
            rich.append({
                "class": cls, "class_id": cid,
                "bbox_xywh_norm": d["bbox_xywh_norm"], "bbox_xyxy_px": d["bbox_xyxy_px"],
                "distance_m": d["distance_m"], "truncated": d.get("truncated", False),
            })
        with open(os.path.join(self.labels_dir, f"{frame_id}.txt"), "w") as f:
            f.write("\n".join(lines))

        record = {
            "frame_id": frame_id, "stamp": stamp,
            "image": f"images/{frame_id}.png", "width": width, "height": height,
            "detections": rich, "light_state": light_state,
            "line_offset": line_offset, "action": action,
        }
        self._jsonl.write(json.dumps(record) + "\n")
        self._jsonl.flush()
        self._count += 1

    @property
    def count(self):
        return self._count

    def close(self):
        try:
            self._jsonl.close()
        except Exception:
            pass
