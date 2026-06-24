"""DatasetWriter output-format checks (no ROS)."""
import json
import os
import tempfile

from yolo_dataset.schema import DatasetWriter


def test_writer_emits_yolo_labels_and_jsonl():
    with tempfile.TemporaryDirectory() as d:
        w = DatasetWriter(d, ["red_light", "green_light", "obstacle"])
        dets = [
            {"class": "green_light", "bbox_xywh_norm": [0.5, 0.4, 0.05, 0.1],
             "bbox_xyxy_px": [300, 200, 340, 250], "distance_m": 3.1, "truncated": False},
            {"class": "obstacle", "bbox_xywh_norm": [0.2, 0.7, 0.2, 0.2],
             "bbox_xyxy_px": [60, 290, 200, 380], "distance_m": 1.2, "truncated": True},
            {"class": "not_a_class", "bbox_xywh_norm": [0, 0, 0, 0],   # unknown -> dropped
             "bbox_xyxy_px": [0, 0, 0, 0], "distance_m": 9, "truncated": False},
        ]
        w.write("000000", 640, 480, dets, "green",
                {"linear_x": 0.2, "angular_z": -0.1}, line_offset=12.0, stamp=1.0)
        w.close()

        # dataset.yaml
        with open(os.path.join(d, "dataset.yaml")) as f:
            y = f.read()
        assert "0: red_light" in y and "1: green_light" in y and "2: obstacle" in y

        # YOLO label: green_light -> class id 1, obstacle -> 2, unknown dropped
        with open(os.path.join(d, "labels", "000000.txt")) as f:
            rows = [r for r in f.read().splitlines() if r]
        assert len(rows) == 2
        assert rows[0].split()[0] == "1" and rows[1].split()[0] == "2"
        # values are normalized floats
        assert all(0.0 <= float(v) <= 1.0 for v in rows[0].split()[1:])

        # rich jsonl
        with open(os.path.join(d, "frames.jsonl")) as f:
            rec = json.loads(f.readline())
        assert rec["light_state"] == "green"
        assert rec["action"]["linear_x"] == 0.2
        assert rec["line_offset"] == 12.0
        assert len(rec["detections"]) == 2          # unknown class dropped here too
        assert rec["detections"][0]["distance_m"] == 3.1


if __name__ == "__main__":
    test_writer_emits_yolo_labels_and_jsonl()
    print("PASS test_writer_emits_yolo_labels_and_jsonl")
