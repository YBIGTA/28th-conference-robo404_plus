#!/usr/bin/env python3
"""
Generate 5 diverse Gazebo worlds with varied track layouts, lighting,
textures, and correctly-oriented traffic lights.

Track types: L-turn, S-curve, U-turn, oval loop, zigzag
"""
import math
import os
import textwrap

WORLDS_DIR = os.path.join(os.path.dirname(__file__), "..", "worlds")

# --------------- geometry helpers ---------------

def arc_segments(cx, cy, radius, start_angle, end_angle, n_segs, width=0.10):
    """Generate box segments approximating a circular arc on the ground."""
    segs = []
    for i in range(n_segs):
        a0 = start_angle + (end_angle - start_angle) * i / n_segs
        a1 = start_angle + (end_angle - start_angle) * (i + 1) / n_segs
        mid_a = (a0 + a1) / 2
        seg_len = abs(a1 - a0) * radius * 1.05  # slight overlap

        mx = cx + radius * math.cos(mid_a)
        my = cy + radius * math.sin(mid_a)
        yaw = mid_a + math.pi / 2  # tangent direction

        segs.append((mx, my, seg_len, width, yaw))
    return segs


def straight_segment(x0, y0, x1, y1, width=0.10):
    """Single box segment from (x0,y0) to (x1,y1)."""
    dx, dy = x1 - x0, y1 - y0
    length = math.sqrt(dx**2 + dy**2)
    yaw = math.atan2(dy, dx)
    mx, my = (x0 + x1) / 2, (y0 + y1) / 2
    return (mx, my, length, width, yaw)


def direction_at(x0, y0, x1, y1):
    """Unit direction vector from p0 to p1."""
    dx, dy = x1 - x0, y1 - y0
    d = math.sqrt(dx**2 + dy**2)
    return dx / d, dy / d


# --------------- SDF building blocks ---------------

def sdf_header(world_name, sun_diffuse, sun_specular, sun_dir, ground_material):
    return textwrap.dedent(f"""\
    <?xml version="1.0" ?>
    <sdf version="1.6">
      <world name="{world_name}">
        <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so"/>
        <light type="directional" name="sun">
          <cast_shadows>true</cast_shadows>
          <pose>0 0 10 0 0 0</pose>
          <diffuse>{sun_diffuse}</diffuse>
          <specular>{sun_specular}</specular>
          <attenuation>
            <range>1000</range><constant>0.9</constant>
            <linear>0.01</linear><quadratic>0.001</quadratic>
          </attenuation>
          <direction>{sun_dir}</direction>
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
                <name>{ground_material}</name>
              </script></material>
            </visual>
          </link>
        </model>
    """)


def sdf_footer():
    return "  </world>\n</sdf>\n"


def sdf_track_seg(name, mx, my, length, width, yaw, material):
    return textwrap.dedent(f"""\
        <model name="{name}">
          <static>true</static>
          <link name="link">
            <visual name="visual">
              <geometry><box><size>{length:.4f} {width:.4f} 0.001</size></box></geometry>
              <material><script>
                <uri>file://media/materials/scripts/gazebo.material</uri>
                <name>{material}</name>
              </script></material>
            </visual>
          </link>
          <pose>{mx:.4f} {my:.4f} 0.0005 0 0 {yaw:.4f}</pose>
        </model>
    """)


def sdf_custom_traffic_light(name, x, y, approach_dx, approach_dy,
                              pole_h=0.4, head_scale=1.0, disc_r=0.06):
    """Custom traffic light facing the approaching car."""
    # Light discs face +Y in model frame.
    # After yaw rotation, they face (-sin(yaw), cos(yaw)).
    # We want them to face -approach direction (toward the car).
    yaw = math.atan2(approach_dx, -approach_dy)

    hw = 0.20 * head_scale
    hh = 0.70 * head_scale
    head_z = pole_h + hh / 2
    red_z = pole_h + hh * 0.85
    yel_z = pole_h + hh * 0.50
    grn_z = pole_h + hh * 0.15
    face_offset = hw / 2 + 0.001

    return textwrap.dedent(f"""\
        <model name="{name}">
          <static>true</static>
          <link name="pole">
            <collision name="collision">
              <geometry><cylinder><radius>0.05</radius><length>{pole_h:.3f}</length></cylinder></geometry>
            </collision>
            <visual name="visual">
              <geometry><cylinder><radius>0.05</radius><length>{pole_h:.3f}</length></cylinder></geometry>
              <material><script>
                <uri>file://media/materials/scripts/gazebo.material</uri>
                <name>Gazebo/Grey</name>
              </script></material>
            </visual>
            <pose>0 0 {pole_h/2:.4f} 0 0 0</pose>
          </link>
          <link name="head">
            <collision name="collision">
              <geometry><box><size>{hw:.3f} {hw:.3f} {hh:.3f}</size></box></geometry>
            </collision>
            <visual name="visual">
              <geometry><box><size>{hw:.3f} {hw:.3f} {hh:.3f}</size></box></geometry>
              <material><script>
                <uri>file://media/materials/scripts/gazebo.material</uri>
                <name>Gazebo/Black</name>
              </script></material>
            </visual>
            <pose>0 0 {head_z:.4f} 0 0 0</pose>
          </link>
          <link name="red_light">
            <visual name="visual">
              <geometry><cylinder><radius>{disc_r:.3f}</radius><length>0.02</length></cylinder></geometry>
              <material><script>
                <uri>file://media/materials/scripts/gazebo.material</uri>
                <name>Gazebo/Red</name>
              </script></material>
            </visual>
            <pose>0 {face_offset:.4f} {red_z:.4f} 1.5708 0 0</pose>
          </link>
          <link name="yellow_light">
            <visual name="visual">
              <geometry><cylinder><radius>{disc_r:.3f}</radius><length>0.02</length></cylinder></geometry>
              <material><script>
                <uri>file://media/materials/scripts/gazebo.material</uri>
                <name>Gazebo/Yellow</name>
              </script></material>
            </visual>
            <pose>0 {face_offset:.4f} {yel_z:.4f} 1.5708 0 0</pose>
          </link>
          <link name="green_light">
            <visual name="visual">
              <geometry><cylinder><radius>{disc_r:.3f}</radius><length>0.02</length></cylinder></geometry>
              <material><script>
                <uri>file://media/materials/scripts/gazebo.material</uri>
                <name>Gazebo/Green</name>
              </script></material>
            </visual>
            <pose>0 {face_offset:.4f} {grn_z:.4f} 1.5708 0 0</pose>
          </link>
          <pose>{x:.4f} {y:.4f} 0.0 0 0 {yaw:.4f}</pose>
        </model>
    """)


def sdf_realistic_traffic_light(name, x, y, approach_dx, approach_dy, pole_h=0.8):
    """Realistic stop_light model from Gazebo DB, correctly oriented.
    The mesh lights face -Y in model frame by default."""
    # Lights face -Y → after yaw: (sin(yaw), -cos(yaw))
    # Want to face (-approach_dx, -approach_dy)
    yaw = math.atan2(-approach_dx, approach_dy)

    sdf = textwrap.dedent(f"""\
        <model name="{name}_pole">
          <static>true</static>
          <link name="link">
            <collision name="collision">
              <geometry><cylinder><radius>0.03</radius><length>{pole_h:.3f}</length></cylinder></geometry>
            </collision>
            <visual name="visual">
              <geometry><cylinder><radius>0.03</radius><length>{pole_h:.3f}</length></cylinder></geometry>
              <material><script>
                <uri>file://media/materials/scripts/gazebo.material</uri>
                <name>Gazebo/DarkGrey</name>
              </script></material>
            </visual>
          </link>
          <pose>{x:.4f} {y:.4f} {pole_h/2:.4f} 0 0 0</pose>
        </model>
        <include>
          <uri>model://stop_light</uri>
          <name>{name}</name>
          <pose>{x:.4f} {y:.4f} {pole_h + 0.4:.4f} 0 0 {yaw:.4f}</pose>
        </include>
    """)
    return sdf


def sdf_obstacle(name, x, y, sx, sy, sz, material):
    return textwrap.dedent(f"""\
        <model name="{name}">
          <static>true</static>
          <link name="link">
            <collision name="collision">
              <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
            </collision>
            <visual name="visual">
              <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
              <material><script>
                <uri>file://media/materials/scripts/gazebo.material</uri>
                <name>{material}</name>
              </script></material>
            </visual>
          </link>
          <pose>{x:.4f} {y:.4f} {sz/2:.4f} 0 0 0</pose>
        </model>
    """)


def sdf_include(uri, name, x, y, z=0, yaw=0):
    return textwrap.dedent(f"""\
        <include>
          <uri>{uri}</uri>
          <name>{name}</name>
          <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.3f}</pose>
        </include>
    """)


# --------------- World definitions ---------------

def build_world_l_turn():
    """World 1: L-turn (90° right turn). Bright, white ground."""
    segs = []
    line_w = 0.10
    line_mat = "Gazebo/Black"

    # Straight 6m along +X
    segs.append(straight_segment(0, 0, 6, 0, line_w))
    # 90° right turn (clockwise = negative angle change)
    # Center at (6, -1.5), radius 1.5, from 90° to 0°
    R = 1.5
    segs += arc_segments(6, -R, R, math.pi/2, 0, 10, line_w)
    # Straight 5m along -Y from (6+R, -R)
    turn_exit_x = 6 + R
    turn_exit_y = -R
    segs.append(straight_segment(turn_exit_x, turn_exit_y, turn_exit_x, turn_exit_y - 5, line_w))

    # Traffic light: placed beside the first straight, car approaches in +X
    tl_x, tl_y = 4.5, -0.8
    approach_dx, approach_dy = 1, 0

    # Obstacle on the exit straight
    obs_x, obs_y = turn_exit_x, turn_exit_y - 4.0

    sdf = sdf_header("l_turn", "0.95 0.95 0.90 1", "0.3 0.3 0.3 1",
                      "-0.3 0.2 -0.9", "Gazebo/White")
    for i, (mx, my, l, w, yaw) in enumerate(segs):
        sdf += sdf_track_seg(f"lane_{i}", mx, my, l, w, yaw, line_mat)
    sdf += sdf_custom_traffic_light("traffic_light", tl_x, tl_y,
                                     approach_dx, approach_dy, pole_h=0.45)
    sdf += sdf_obstacle("obstacle_box", obs_x, obs_y, 0.4, 0.4, 0.4, "Gazebo/Red")
    sdf += sdf_include("model://construction_cone", "cone_1", 2.0, 1.2)
    sdf += sdf_include("model://construction_cone", "cone_2", 8.5, -4.0)
    sdf += sdf_include("model://fire_hydrant", "hydrant_1", 1.0, -1.5)
    sdf += sdf_footer()
    return "world_l_turn.world", sdf


def build_world_s_curve():
    """World 2: S-curve. Overcast, grey ground."""
    segs = []
    line_w = 0.12
    line_mat = "Gazebo/Black"

    # Straight 3m entry
    segs.append(straight_segment(0, 0, 3, 0, line_w))

    # First curve: left (counter-clockwise), radius 2, ~90°
    R1 = 2.0
    c1x, c1y = 3, R1
    segs += arc_segments(c1x, c1y, R1, -math.pi/2, 0, 8, line_w)

    # Short connector straight going +Y
    conn_x = c1x + R1
    conn_y = c1y
    segs.append(straight_segment(conn_x, conn_y, conn_x, conn_y + 1.5, line_w))

    # Second curve: right (clockwise), radius 2, ~90°
    R2 = 2.0
    c2x, c2y = conn_x + R2, conn_y + 1.5
    segs += arc_segments(c2x, c2y, R2, math.pi, math.pi/2, 8, line_w)

    # Straight exit 4m along +X
    exit_x = c2x
    exit_y = c2y + R2
    segs.append(straight_segment(exit_x, exit_y, exit_x + 4, exit_y, line_w))

    # Traffic light on the exit straight, car approaches in +X
    tl_x, tl_y = exit_x + 2, exit_y - 0.8
    approach_dx, approach_dy = 1, 0

    # Obstacle further along exit
    obs_x, obs_y = exit_x + 3.5, exit_y

    sdf = sdf_header("s_curve", "0.55 0.55 0.62 1", "0.08 0.08 0.08 1",
                      "-0.1 -0.3 -0.95", "Gazebo/Grey")
    for i, (mx, my, l, w, yaw) in enumerate(segs):
        sdf += sdf_track_seg(f"lane_{i}", mx, my, l, w, yaw, line_mat)
    sdf += sdf_realistic_traffic_light("traffic_light", tl_x, tl_y,
                                        approach_dx, approach_dy, pole_h=0.7)
    sdf += sdf_obstacle("obstacle_box", obs_x, obs_y, 0.5, 0.3, 0.5, "Gazebo/Orange")
    sdf += sdf_include("model://jersey_barrier", "barrier_1", 1.0, -1.5)
    sdf += sdf_include("model://stop_sign", "sign_1", exit_x + 1, exit_y + 1.5)
    sdf += sdf_include("model://postbox", "postbox_1", 6, 1.0)
    sdf += sdf_footer()
    return "world_s_curve.world", sdf


def build_world_u_turn():
    """World 3: U-turn (180° hairpin). Park setting, grass ground."""
    segs = []
    line_w = 0.10
    line_mat = "Gazebo/FlatBlack"

    # Straight 5m along +X
    segs.append(straight_segment(0, 0, 5, 0, line_w))

    # 180° U-turn to the right (car ends up going -X, shifted by 2*R in -Y)
    R = 1.2
    cx, cy = 5, -R
    segs += arc_segments(cx, cy, R, math.pi/2, -math.pi/2, 14, line_w)

    # Straight 5m along -X
    segs.append(straight_segment(5, -2*R, 0, -2*R, line_w))

    # Traffic light on the return straight, car approaches in -X
    tl_x, tl_y = 2.0, -2*R + 0.8
    approach_dx, approach_dy = -1, 0

    # Obstacle on the outbound straight
    obs_x, obs_y = 3.5, 0

    sdf = sdf_header("u_turn", "0.90 0.82 0.65 1", "0.25 0.22 0.15 1",
                      "-0.7 0.4 -0.55", "Gazebo/Grass")
    for i, (mx, my, l, w, yaw) in enumerate(segs):
        sdf += sdf_track_seg(f"lane_{i}", mx, my, l, w, yaw, line_mat)
    sdf += sdf_custom_traffic_light("traffic_light", tl_x, tl_y,
                                     approach_dx, approach_dy, pole_h=0.35)
    sdf += sdf_obstacle("obstacle_box", obs_x, obs_y, 0.45, 0.45, 0.35, "Gazebo/Blue")
    sdf += sdf_include("model://oak_tree", "tree_1", -1, 2)
    sdf += sdf_include("model://pine_tree", "tree_2", 6.5, -4)
    sdf += sdf_include("model://oak_tree", "tree_3", -1, -4)
    sdf += sdf_include("model://fire_hydrant", "hydrant_1", 7, 1.5)
    sdf += sdf_footer()
    return "world_u_turn.world", sdf


def build_world_oval():
    """World 4: Oval closed loop. Urban, light ground."""
    segs = []
    line_w = 0.10
    line_mat = "Gazebo/FlatBlack"

    # Oval: two straights (along X) + two semicircles
    straight_len = 5.0
    R = 2.0
    # Bottom straight: (0,0) → (straight_len, 0) going +X
    segs.append(straight_segment(0, 0, straight_len, 0, line_w))
    # Right semicircle: center (straight_len, R), from -π/2 to π/2
    segs += arc_segments(straight_len, R, R, -math.pi/2, math.pi/2, 12, line_w)
    # Top straight: (straight_len, 2R) → (0, 2R) going -X
    segs.append(straight_segment(straight_len, 2*R, 0, 2*R, line_w))
    # Left semicircle: center (0, R), from π/2 to 3π/2
    segs += arc_segments(0, R, R, math.pi/2, 3*math.pi/2, 12, line_w)

    # Traffic light on the top straight, car approaches in -X
    tl_x, tl_y = 2.5, 2*R + 0.8
    approach_dx, approach_dy = -1, 0

    # Obstacle on bottom straight
    obs_x, obs_y = 3.0, 0

    sdf = sdf_header("oval_loop", "0.60 0.58 0.70 1", "0.10 0.10 0.12 1",
                      "0.2 -0.5 -0.8", "Gazebo/Wood")
    for i, (mx, my, l, w, yaw) in enumerate(segs):
        sdf += sdf_track_seg(f"lane_{i}", mx, my, l, w, yaw, line_mat)
    sdf += sdf_realistic_traffic_light("traffic_light", tl_x, tl_y,
                                        approach_dx, approach_dy, pole_h=0.6)
    sdf += sdf_obstacle("obstacle_box", obs_x, obs_y, 0.5, 0.5, 0.6, "Gazebo/Purple")
    sdf += sdf_include("model://grey_wall", "wall_1", -2.5, 2.0)
    sdf += sdf_include("model://brick_box_3x1x3", "brick_1", 7.5, 2.0)
    sdf += sdf_include("model://construction_cone", "cone_1", 1.0, -1.0)
    sdf += sdf_include("model://lamp_post", "lamp_1", -2, 5)
    sdf += sdf_footer()
    return "world_oval_loop.world", sdf


def build_world_zigzag():
    """World 5: Zigzag / W-shape. Dusk, warm tones."""
    segs = []
    line_w = 0.10
    line_mat = "Gazebo/Black"

    # W pattern: alternating diagonal segments
    pts = [
        (0, 0),
        (2, -1.5),   # down-right
        (4, 1.0),    # up-right
        (6, -1.0),   # down-right
        (8, 1.5),    # up-right
        (11, 1.5),   # straight exit
    ]
    for i in range(len(pts) - 1):
        segs.append(straight_segment(*pts[i], *pts[i+1], line_w))

    # Traffic light beside the exit straight, car approaches in +X
    tl_x, tl_y = 9.5, 0.5
    approach_dx, approach_dy = 1, 0

    # Obstacle on one of the middle segments
    obs_x, obs_y = 5.0, 0.0

    sdf = sdf_header("zigzag", "0.75 0.50 0.30 1", "0.15 0.10 0.05 1",
                      "-0.8 0.0 -0.4", "Gazebo/White")
    for i, (mx, my, l, w, yaw) in enumerate(segs):
        sdf += sdf_track_seg(f"lane_{i}", mx, my, l, w, yaw, line_mat)
    sdf += sdf_custom_traffic_light("traffic_light", tl_x, tl_y,
                                     approach_dx, approach_dy, pole_h=0.4)
    sdf += sdf_obstacle("obstacle_box", obs_x, obs_y, 0.35, 0.50, 0.45, "Gazebo/Yellow")
    sdf += sdf_include("model://lamp_post", "lamp_1", 1, 2.5)
    sdf += sdf_include("model://person_standing", "person_1", 7.5, -2.5, yaw=-0.3)
    sdf += sdf_include("model://pine_tree", "tree_1", -1, -2)
    sdf += sdf_include("model://suv", "car_1", 12, -1.5, yaw=1.2)
    sdf += sdf_footer()
    return "world_zigzag.world", sdf


def main():
    os.makedirs(WORLDS_DIR, exist_ok=True)
    builders = [
        build_world_l_turn,
        build_world_s_curve,
        build_world_u_turn,
        build_world_oval,
        build_world_zigzag,
    ]
    for build_fn in builders:
        fname, sdf = build_fn()
        path = os.path.join(WORLDS_DIR, fname)
        with open(path, "w") as f:
            f.write(sdf)
        print(f"  {fname}")
    print(f"\nDone! {len(builders)} worlds in {WORLDS_DIR}/")


if __name__ == "__main__":
    main()
