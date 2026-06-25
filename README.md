<div align="center">

# ROBO 404++ — Vision-based 4WD Autonomous Driving Mobility

**비전 센서 기반 4WD 자율주행 · OpenCV + YOLO 융합 차선 주행 · Jetson Nano 엣지 추론**

YBIGTA 28기 컨퍼런스 프로젝트 (robo404++)

📦 **하드웨어 레포**: [jiy0-0nv/robo404pp_hardware](https://github.com/jiy0-0nv/robo404pp_hardware)

![ROS2](https://img.shields.io/badge/ROS2-Humble-22314E?logo=ros&logoColor=white)
![YOLO](https://img.shields.io/badge/YOLO-Ultralytics-00FFFF?logo=yolo&logoColor=black)
![TensorRT](https://img.shields.io/badge/TensorRT-Jetson%20Nano-76B900?logo=nvidia&logoColor=white)
![OpenCV](https://img.shields.io/badge/OpenCV-Vision-5C3EE8?logo=opencv&logoColor=white)
![License](https://img.shields.io/badge/License-Mixed-blue.svg)

</div>

---

## 🚀 Quick Start

```bash
# 1. 워크스페이스에 클론
git clone https://github.com/YBIGTA/28th-conference-robo404_plus.git ros2_ws/src

# 2. 빌드
cd ros2_ws && colcon build && source install/setup.bash

# 3. 실행 (라인 팔로워 + YOLO)
ros2 run follower follower
ros2 launch yolo_bringup yolov8.launch.py
```

자세한 실행 순서는 [8. How to Run](#8-how-to-run) 참고.

---

## 📋 목차

1. [Project Overview](#1-project-overview)
2. [Problem Statement](#2-problem-statement)
3. [System Architecture](#3-system-architecture)
4. [Tech Stack](#4-tech-stack)
5. [Dataset](#5-dataset)
6. [Methodology](#6-methodology)
7. [Experiments & Results](#7-experiments--results)
8. [How to Run](#8-how-to-run)
9. [Directory Structure](#9-directory-structure)
10. [ROS 2 Interface](#10-ros-2-interface)
11. [Limitations & Future Work](#11-limitations--future-work)
12. [Team](#12-team)

---

## 1. Project Overview

ROBO 404++는 **비전 센서 기반 4WD 자율주행 모빌리티**입니다. 카메라 영상을 **OpenCV(차선 인식)** 와 **YOLOv8(신호등 인식)** 로 분석하고, 이를 **하나의 의사결정 노드(Decision Node)에서 융합**하여 안전하게 차선을 주행하는 로봇입니다.

- **입력**: 카메라 영상 (Pi Camera V2)
- **처리**: OpenCV 차선 추종(`follower node`) + YOLO 신호등 인식 → **Decision Node에서 융합**
- **출력**: 주행 명령(`/cmd_vel`) → `micro_ros_agent` → 펌웨어(Raspberry Pi Pico 2) → 4WD 구동

## 2. Problem Statement

> 한 대의 카메라로 차선을 안정적으로 추종하면서, OpenCV와 YOLO의 분석 결과를 하나의 의사결정 노드에서 융합해 안전하게 주행한다.

- **대상 환경**: 차선이 있는 주행 트랙
- **제약**: Jetson Nano의 제한된 연산 자원에서 실시간(FPS) 확보
- **핵심 과제**: 차선 인식(OpenCV) + 객체 인식(YOLO)의 융합 의사결정과 제어 안정성 양립

## 3. System Architecture

```text
              ┌──────────────────────┐
              │      YOLOv8 Node      │
         ┌───▶│  · 신호등 인식         │───┐ /yolo_events
         │    │  · Bounding Box 연산   │   │ (event message)
         │    └──────────────────────┘   ▼
   Camera│                          ┌──────────────────────┐
 (Pi Cam │                          │   Traffic Light Node  │  ← 신호 판단
   V2)   │                          │  (YOLO 결과로 STOP/    │
         │                          │   MOVE 이벤트 생성)    │
         │    ┌──────────────────────┐   │
         └───▶│  Line Tracking Node   │   │
              │  (= follower node)    │   │
              │  · 차선 인식           │   │
              │  · Edge / Vanishing   │   │
              │    point 검출         │   │
              └───────────┬──────────┘   │ /cmd_vel_line
                          │              │ (geometry message)
                          ▼              ▼
                        ┌──────────────────────┐
                        │     Decision Node     │
                        │ 1. 신호 이벤트 검사     │
                        │ 2. STOP → cmd_vel_line │
                        │         무시            │
                        │ 3. MOVE → cmd_vel_line │
                        │         전달            │
                        └───────────┬──────────┘
                                    │ /cmd_vel
                                    ▼
                          ┌───────────────────┐
                          │   micro_ros_agent  │
                          └─────────┬─────────┘
                                    ▼
                  Firmware (Raspberry Pi Pico 2) → 4WD 구동
```

**주요 컴포넌트**
- **YOLOv8 Node** — 카메라 영상에서 신호등을 인식하고 Bounding Box를 연산
- **Traffic Light Node** — YOLO 탐지 결과로부터 신호(STOP/MOVE)를 판단해 이벤트를 발행
- **Line Tracking Node (= follower node)** — 차선을 인식(Edge, Vanishing point)하여 주행 명령 후보(`/cmd_vel_line`)를 발행
- **Decision Node** — 신호 이벤트를 검사해 STOP이면 차선 주행 명령을 무시하고, MOVE이면 `/cmd_vel_line`을 `/cmd_vel`로 전달 (**융합 의사결정**)
- **micro_ros_agent → Firmware (Raspberry Pi Pico 2)** — `/cmd_vel`을 받아 4WD 모터를 구동

> **구현 현황**: 현재 레포에는 **Line Tracking(follower node)** 와 **YOLO Node** 가 구현되어 있습니다.
> YOLO 다음 단의 **Traffic Light Node(신호 판단)** 와 **Decision Node** 는 설계상 노드이며, 입력 카메라는 현재 **1대**로 동작합니다. (다이어그램의 상/하단 2-카메라 구성은 목표 설계)

### Decision Node — 주행 판단 로직

차량이 움직여야 하는지 최종적으로 결정하는 노드입니다. 매 사이클마다 입력을 정리(timeout 처리, `red_latched` 갱신)한 뒤 상태 머신으로 판단합니다.

```text
Decision Cycle
   │
   ▼
Input 정리 (timeout 처리 / red_latched 갱신)
   │
   ▼
주행 가능 상태인가? ──┬─ motion_enabled == false ──▶ IDLE        ─┐
                     ├─ path_state == FINAL_STOP ──▶ FINAL_STOP  ─┤
                     ├─ line timeout / LINE_LOST ──▶ LINE_LOST   ─┼─▶ zero Twist (정지)
                     │                                            │
                     └─ 주행 가능 ─▶ 신호등 상태? ──┬─ RED ──────▶ STOP_FOR_RED ─┤
                                                   ├─ RED→GREEN  ▶ WAIT_GREEN   ─┘
                                                   │  (전환 대기)
                                                   └─ GREEN / UNKNOWN
                                                      (정지 latch 없음) ─▶ FOLLOW_LINE ─▶ cmd_vel_line 통과
```

| 상태 | 조건 | 결과 |
| --- | --- | --- |
| `IDLE` | `motion_enabled == false` | 정지 (zero Twist) |
| `FINAL_STOP` | `path_state == FINAL_STOP` (트랙 종료) | 정지 |
| `LINE_LOST` | 차선 timeout 또는 라인 소실 | 정지 |
| `STOP_FOR_RED` | 신호등 RED | 정지 |
| `WAIT_GREEN` | RED 이후 GREEN 전환 대기 | 정지 |
| `FOLLOW_LINE` | GREEN 또는 UNKNOWN (정지 latch 없음) | `/cmd_vel_line` 통과 → 주행 |

즉 **정상 주행은 `FOLLOW_LINE` 상태에서만** 일어나며, 그 외 모든 안전 조건(신호 RED, 라인 소실, 종료 등)에서는 `zero Twist`로 차량을 정지시킵니다.

## 4. Tech Stack

**Core / Control**
- ROS 2, `rclpy` / `rclcpp`
- OpenCV, `cv_bridge`
- P 제어기 (proportional controller)

**AI / Vision**
- Ultralytics YOLO (v5 ~ v12, YOLO-World, YOLOE)
- TensorRT (FP16 엔진)

**System / Infra**
- 메인 보드: NVIDIA Jetson Nano (JetPack, TensorRT)
- 펌웨어/구동: Raspberry Pi Pico 2 + `micro_ros_agent`
- 센서: Pi Camera V2 × 2 (상단/하단)
- colcon 빌드 시스템

## 5. Dataset

<!-- TODO: 학습에 사용한 데이터셋 정보를 채워주세요 -->

| 항목 | 설명 | 데이터 수 |
| --- | --- | --- |
| (TODO) | (TODO) | (TODO) |

**라벨 구조 (TODO)**
- (사용한 클래스 목록을 기입)

## 6. Methodology

**Step 1 — 영상 전처리 & 라인 검출**
영상 하단 1/3 영역을 크롭하고 BGR 색상 범위로 필터링하여 트랙 라인의 이진 마스크를 생성합니다.

**Step 2 — P 제어 주행**
가장 큰 컨투어의 중심(centroid)과 화면 중앙의 차이를 오차(error)로 정의하고, 비례 상수 `KP`를 곱해 각속도를 산출합니다.

```python
error = x - width // 2
message.angular.z = float(error) * -KP
message.linear.x  = LINEAR_SPEED
```

라인을 놓치면 마지막 오차에 `LOSS_FACTOR`를 곱해 제자리에서 회전하며 라인을 재탐색하고, 오른쪽 마크를 일정 횟수 인식하면 종료 카운트다운에 진입합니다.

**Step 3 — YOLO 객체 탐지**
`yolo_node`(lifecycle node)가 영상을 구독해 추론을 수행하고, 결과를 `yolo_msgs` 메시지로 발행 → `tracking_node`에서 추적합니다.

## 7. Experiments & Results

<!-- TODO: 실험 환경/결과 수치를 채워주세요 -->

**성능 메트릭 (참고: yolo_bridge 벤치마크, FP16, 추론 only)**

| 모델 | Orin Nano (FPS) | Jetson Nano (FPS) |
| --- | :---: | :---: |
| yolov5nu | 100 | 20 |
| yolov8n | 100 | 19 |
| yolov8s | 100 | 9.25 |

**핵심 성과 (TODO)**
- (라인 추종 성공률, 주행 시간 등 기입)

## 8. How to Run

**사전 준비**
```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

**라인 팔로워**
```bash
ros2 run follower follower
ros2 service call /start_follower std_srvs/srv/Empty   # 주행 시작
ros2 service call /stop_follower  std_srvs/srv/Empty   # 정지
```

**YOLO 추론 (모델별 launch)**
```bash
ros2 launch yolo_bringup yolov8.launch.py    # v5/v9/v10/v11/v12, yolo-world, yoloe 등도 제공
```

**Jetson Nano TensorRT 엔진 변환**
```bash
yolo export model=yolov8s.pt format=onnx opset=11 simplify=True
trtexec --onnx=yolov8s.onnx --saveEngine=yolov8s.engine --fp16
```
상세: [yolo_bridge README](src/yolo_bridge/YoloV8-TensorRT-Jetson_Nano/README.md)

> 주행 파라미터 조정: [src/follower/docs/parameters.md](src/follower/docs/parameters.md) · 알려진 이슈: [warnings.md](src/follower/docs/warnings.md)

## 9. Directory Structure

```text
28th-conference-robo404_plus/
├── README.md
└── src/
    ├── follower/                    # 라인 팔로잉 (ament_python)
    │   ├── follower/
    │   │   ├── follower_node.py     # 라인 검출 + P 제어 주행
    │   │   └── test_topic.py
    │   └── docs/                    # parameters / warnings / about
    ├── yolo_ros/                    # YOLO 추론 노드 (ament_python)
    │   └── yolo_ros/
    │       ├── yolo_node.py         # YOLO lifecycle 추론
    │       ├── tracking_node.py     # 객체 트래킹
    │       ├── detect_3d_node.py    # 3D 검출
    │       └── debug_node.py        # 시각화
    ├── yolo_msgs/                   # 메시지/서비스 정의 (ament_cmake)
    │   ├── msg/                     # Detection, BoundingBox2D/3D, KeyPoint ...
    │   └── srv/SetClasses.srv
    ├── yolo_bringup/                # launch 파일 모음 (ament_cmake)
    │   └── launch/                  # yolov5~v12, yolo-world, yoloe
    └── yolo_bridge/                 # Jetson Nano TensorRT C++ 추론 (ament_cmake)
        ├── src/                     # yolov8.cpp, main.cpp
        └── YoloV8-TensorRT-Jetson_Nano/   # ONNX 모델 + 변환 가이드
```

## 10. ROS 2 Interface

**주요 토픽**

| 토픽 | 방향 | 설명 |
| --- | --- | --- |
| `/image_top` | 카메라 → YOLOv8 Node | 신호등 인식용 raw image |
| `/image_bottom` | 카메라 → Line Tracking Node | 차선 인식용 raw image |
| `/yolo_events` | YOLO/Traffic Light Node → Decision Node | STOP/MOVE 이벤트 메시지 |
| `/cmd_vel_line` | Line Tracking Node → Decision Node | 차선 추종 주행 명령 후보 (geometry) |
| `/cmd_vel` | Decision Node → micro_ros_agent | 최종 주행 속도 명령 |

> 다이어그램의 `/image_top`·`/image_bottom`은 상/하단 2-카메라를 가정한 목표 설계이며, 현재는 **단일 카메라**로 동작합니다.

**주요 서비스**

| 서비스 | 타입 | 설명 |
| --- | --- | --- |
| `/start_follower` | `std_srvs/Empty` | 라인 추종 주행 시작 |
| `/stop_follower` | `std_srvs/Empty` | 주행 정지 |

**주요 파라미터 (follower_node)**

| 파라미터 | 기본값 | 설명 |
| --- | --- | --- |
| `LINEAR_SPEED` | 0.2 | 직진 속도 |
| `KP` | 0.015 | 조향 비례 상수 |
| `LOSS_FACTOR` | 1.2 | 라인 소실 시 오차 보정 계수 |
| `TIMER_PERIOD` | 0.06 | 제어 주기 (초) |

## 11. Limitations & Future Work

**현재 한계**
- 색상 기반 라인 검출 → 조명/표면 변화에 민감
- Jetson Nano에서 큰 YOLO 모델은 FPS 저하

**향후 개선 (TODO)**
- *기술*: 적응형 임계값 / 딥러닝 기반 라인 검출
- *시스템*: SLAM·내비게이션 결합
- *응용*: 탐지 객체 기반 의사결정 주행

## 12. Team

<!-- TODO: 팀원 정보를 채워주세요 -->

| | | |
| :---: | :---: | :---: |
| (이름 / 역할) | (이름 / 역할) | (이름 / 역할) |

---

## License / Credits

본 저장소는 여러 오픈소스를 통합했습니다.
- `follower` — MIT, Gabriel Nascarella Hishida
- `yolo_ros` / `yolo_msgs` / `yolo_bringup` — GPL-3.0, Miguel Ángel González Santamarta
- `yolo_bridge` (TensorRT) — Q-engineering, YoloV8-TensorRT-Jetson_Nano

## Related Repositories

- **하드웨어**: [jiy0-0nv/robo404pp_hardware](https://github.com/jiy0-0nv/robo404pp_hardware) — 4WD 섀시 / 펌웨어(Raspberry Pi Pico 2) 등 하드웨어 구성
