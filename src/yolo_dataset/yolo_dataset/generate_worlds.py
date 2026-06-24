#!/usr/bin/env python3
"""
Generate randomised Gazebo Classic .world files for domain-randomised
dataset collection.  Each world varies lighting, ground texture, track
layout, traffic-light appearance/position, obstacle properties, and
distractor objects.

Usage:
    python3 -m yolo_dataset.generate_worlds \
        --config config/world_randomization.yaml \
        --output-dir /tmp/worlds \
        --num 20 --seed 42
"""
import argparse
import math
import os
import random
import textwrap
from pathlib import Path

import yaml


def _u(cfg, key):
    lo, hi = cfg[key]
    return random.uniform(lo, hi)


def _choice(cfg, key):
    return random.choice(cfg[key])


def _sdf_model_box(name, pose, size, material):
    sx, sy, sz = size
    px, py, pz, roll, pitch, yaw = pose
    return textwrap.dedent(f"""\
    <model name="{name}">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material><script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>{material}</name>
          </script></material>
        </visual>
      </link>
      <pose>{px} {py} {pz} {roll} {pitch} {yaw}</pose>
    </model>
    """)


def _sdf_model_cylinder(name, pose, radius, length, material):
    px, py, pz, roll, pitch, yaw = pose
    return textwrap.dedent(f"""\
    <model name="{name}">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry>
        </collision>
        <visual name="visual">
          <geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry>
          <material><script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>{material}</name>
          </script></material>
        </visual>
      </link>
      <pose>{px} {py} {pz} {roll} {pitch} {yaw}</pose>
    </model>
    """)


def _generate_track(cfg):
    tc = cfg["track"]
    seg1_len = _u(tc, "seg1_length")
    seg2_fwd = _u(tc, "seg2_forward")
    seg2_lat = _u(tc, "seg2_lateral")
    seg3_len = _u(tc, "seg3_length")
    line_w = _u(tc, "line_width")
    mat = _choice(tc, "materials")

    seg2_len = math.sqrt(seg2_fwd ** 2 + seg2_lat ** 2)
    seg2_yaw = math.atan2(seg2_lat, seg2_fwd)

    seg1_cx = seg1_len / 2
    seg2_cx = seg1_len + seg2_fwd / 2
    seg2_cy = seg2_lat / 2
    seg3_start_x = seg1_len + seg2_fwd
    seg3_cy = seg2_lat
    seg3_cx = seg3_start_x + seg3_len / 2

    sdf = ""
    sdf += _sdf_model_box("lane_segment_1",
                          (seg1_cx, 0, 0.0005, 0, 0, 0),
                          (seg1_len, line_w, 0.001), mat)
    sdf += _sdf_model_box("lane_segment_2",
                          (seg2_cx, seg2_cy, 0.0005, 0, 0, seg2_yaw),
                          (seg2_len, line_w, 0.001), mat)
    sdf += _sdf_model_box("lane_segment_3",
                          (seg3_cx, seg3_cy, 0.0005, 0, 0, 0),
                          (seg3_len, line_w, 0.001), mat)

    track_info = {
        "seg1_end_x": seg1_len,
        "seg2_lat": seg2_lat,
        "seg3_start_x": seg3_start_x,
        "seg3_cy": seg3_cy,
        "seg3_len": seg3_len,
    }
    return sdf, track_info


def _generate_traffic_light_custom(cfg, track):
    tc = cfg["traffic_light"]
    lat = _u(tc, "lateral_offset")
    pole_h = _u(tc, "pole_height")
    scale = _u(tc, "head_scale")
    disc_r = _u(tc, "disc_radius")

    x = track["seg1_end_x"]
    y = lat

    head_w = 0.2 * scale
    head_h = 0.7 * scale
    head_z = pole_h + head_h / 2

    red_z = pole_h + head_h * 0.85
    yellow_z = pole_h + head_h * 0.5
    green_z = pole_h + head_h * 0.15

    sdf = textwrap.dedent(f"""\
    <model name="traffic_light">
      <static>true</static>
      <link name="pole">
        <collision name="collision">
          <geometry><cylinder><radius>0.05</radius><length>{pole_h}</length></cylinder></geometry>
        </collision>
        <visual name="visual">
          <geometry><cylinder><radius>0.05</radius><length>{pole_h}</length></cylinder></geometry>
          <material><script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Grey</name>
          </script></material>
        </visual>
        <pose>0 0 {pole_h / 2:.4f} 0 0 0</pose>
      </link>
      <link name="head">
        <collision name="collision">
          <geometry><box><size>{head_w:.4f} {head_w:.4f} {head_h:.4f}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{head_w:.4f} {head_w:.4f} {head_h:.4f}</size></box></geometry>
          <material><script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Black</name>
          </script></material>
        </visual>
        <pose>0 0 {head_z:.4f} 0 0 0</pose>
      </link>
      <link name="red_light">
        <visual name="visual">
          <geometry><cylinder><radius>{disc_r:.4f}</radius><length>0.02</length></cylinder></geometry>
          <material><script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Red</name>
          </script></material>
        </visual>
        <pose>0 {head_w / 2 + 0.001:.4f} {red_z:.4f} 1.5708 0 0</pose>
      </link>
      <link name="yellow_light">
        <visual name="visual">
          <geometry><cylinder><radius>{disc_r:.4f}</radius><length>0.02</length></cylinder></geometry>
          <material><script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Yellow</name>
          </script></material>
        </visual>
        <pose>0 {head_w / 2 + 0.001:.4f} {yellow_z:.4f} 1.5708 0 0</pose>
      </link>
      <link name="green_light">
        <visual name="visual">
          <geometry><cylinder><radius>{disc_r:.4f}</radius><length>0.02</length></cylinder></geometry>
          <material><script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Green</name>
          </script></material>
        </visual>
        <pose>0 {head_w / 2 + 0.001:.4f} {green_z:.4f} 1.5708 0 0</pose>
      </link>
      <pose>{x} {y} 0.0 0 0 1.57079632679</pose>
    </model>
    """)
    return sdf


def _generate_traffic_light_realistic(cfg, track):
    tc = cfg["traffic_light"]
    lat = _u(tc, "lateral_offset")
    pole_h = _u(tc, "pole_height")

    x = track["seg1_end_x"]
    y = lat

    sdf = _sdf_model_cylinder("traffic_light_pole",
                              (x, y, pole_h / 2, 0, 0, 0),
                              0.03, pole_h, "Gazebo/Grey")
    sdf += textwrap.dedent(f"""\
    <include>
      <uri>model://stop_light</uri>
      <pose>{x} {y} {pole_h + 0.4} 0 0 1.57079632679</pose>
    </include>
    """)
    return sdf


def _generate_obstacle(cfg, track):
    oc = cfg["obstacle"]
    frac = _u(oc, "along_seg3")
    sx = _u(oc, "size_x")
    sy = _u(oc, "size_y")
    sz = _u(oc, "size_z")
    mat = _choice(oc, "materials")

    ox = track["seg3_start_x"] + frac * track["seg3_len"]
    oy = track["seg3_cy"]

    return _sdf_model_box("obstacle_box",
                          (ox, oy, sz / 2, 0, 0, 0),
                          (sx, sy, sz), mat)


def _generate_distractors(cfg, track):
    dc = cfg["distractors"]
    n = random.randint(*dc["count"])
    sdf = ""
    track_xs = [0, track["seg1_end_x"], track["seg3_start_x"],
                track["seg3_start_x"] + track["seg3_len"]]

    for i in range(n):
        x = random.uniform(min(track_xs), max(track_xs))
        side = random.choice([-1, 1])
        y = side * _u(dc, "distance")
        sz = _u(dc, "size")
        mat = _choice(dc, "materials")

        if random.random() < 0.5:
            sdf += _sdf_model_box(f"distractor_{i}",
                                  (x, y, sz / 2, 0, 0, random.uniform(0, math.pi)),
                                  (sz, sz, sz), mat)
        else:
            sdf += _sdf_model_cylinder(f"distractor_{i}",
                                       (x, y, sz / 2, 0, 0, 0),
                                       sz / 2, sz, mat)
    return sdf


def generate_world(cfg, world_id):
    sc = cfg["sun"]
    gc = cfg["ground"]

    dr = _u(sc, "diffuse_r")
    dg = _u(sc, "diffuse_g")
    db = _u(sc, "diffuse_b")
    spec = _u(sc, "specular")
    dx = _u(sc, "dir_x")
    dy = _u(sc, "dir_y")
    dz = _u(sc, "dir_z")

    ground_mat = _choice(gc, "materials")

    track_sdf, track_info = _generate_track(cfg)

    tc = cfg["traffic_light"]
    use_realistic = random.random() < tc.get("realistic_prob", 0.3)
    if use_realistic:
        light_sdf = _generate_traffic_light_realistic(cfg, track_info)
    else:
        light_sdf = _generate_traffic_light_custom(cfg, track_info)

    obstacle_sdf = _generate_obstacle(cfg, track_info)
    distractor_sdf = _generate_distractors(cfg, track_info)

    world = textwrap.dedent(f"""\
    <?xml version="1.0" ?>
    <sdf version="1.6">
      <world name="rand_world_{world_id:03d}">
        <light type="directional" name="sun">
          <cast_shadows>true</cast_shadows>
          <pose>0 0 10 0 0 0</pose>
          <diffuse>{dr:.3f} {dg:.3f} {db:.3f} 1</diffuse>
          <specular>{spec:.3f} {spec:.3f} {spec:.3f} 1</specular>
          <attenuation>
            <range>1000</range>
            <constant>0.9</constant>
            <linear>0.01</linear>
            <quadratic>0.001</quadratic>
          </attenuation>
          <direction>{dx:.3f} {dy:.3f} {dz:.3f}</direction>
        </light>

        <model name="ground_plane">
          <static>true</static>
          <link name="link">
            <collision name="collision">
              <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
              <surface><friction><ode><mu>100</mu><mu2>50</mu2></ode></friction></surface>
            </collision>
            <visual name="visual">
              <cast_shadows>false</cast_shadows>
              <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
              <material><script>
                <uri>file://media/materials/scripts/gazebo.material</uri>
                <name>{ground_mat}</name>
              </script></material>
            </visual>
          </link>
        </model>

    """)

    world += track_sdf
    world += light_sdf
    world += obstacle_sdf
    world += distractor_sdf

    world += textwrap.dedent("""\
      </world>
    </sdf>
    """)
    return world, track_info


def main():
    parser = argparse.ArgumentParser(description="Generate randomised Gazebo worlds")
    parser.add_argument("--config", required=True, help="Path to world_randomization.yaml")
    parser.add_argument("--output-dir", required=True, help="Directory for generated .world files")
    parser.add_argument("--num", type=int, default=None, help="Override num_worlds from config")
    parser.add_argument("--seed", type=int, default=None, help="Random seed for reproducibility")
    args = parser.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)

    n = args.num or cfg.get("num_worlds", 20)
    if args.seed is not None:
        random.seed(args.seed)

    out = Path(args.output_dir)
    out.mkdir(parents=True, exist_ok=True)

    manifest = []
    for i in range(n):
        world_sdf, track_info = generate_world(cfg, i)
        fname = f"rand_{i:03d}.world"
        (out / fname).write_text(world_sdf)
        manifest.append({"world": fname, "track_info": track_info})
        print(f"  [{i + 1}/{n}] {fname}")

    manifest_path = out / "manifest.yaml"
    with open(manifest_path, "w") as f:
        yaml.dump(manifest, f, default_flow_style=False)

    print(f"\nGenerated {n} worlds in {out}/")
    print(f"Manifest: {manifest_path}")


if __name__ == "__main__":
    main()
