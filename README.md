<div align="center">

# ROBO 404++ - Vision-based 4WD Autonomous Driving Mobility

**비전 센서 기반 4WD 자율주행 · OpenCV 라인 추종 · TensorRT YOLOv8 신호등 판단 · ROS 2 의사결정 파이프라인**

YBIGTA 28기 컨퍼런스 프로젝트 (robo404++)

📦 **하드웨어 레포**: [jiy0-0nv/robo404pp_hardware](https://github.com/jiy0-0nv/robo404pp_hardware)

![ROS2](https://img.shields.io/badge/ROS2-Humble-22314E?logo=ros&logoColor=white)
![YOLO](https://img.shields.io/badge/YOLOv8-TensorRT-00FFFF?logo=yolo&logoColor=black)
![TensorRT](https://img.shields.io/badge/TensorRT-Jetson%20Nano-76B900?logo=nvidia&logoColor=white)
![OpenCV](https://img.shields.io/badge/OpenCV-Vision-5C3EE8?logo=opencv&logoColor=white)
![License](https://img.shields.io/badge/License-Mixed-blue.svg)

</div>

---

## Quick Start

```bash
# 1. 워크스페이스에 클론
git clone https://github.com/YBIGTA/28th-conference-robo404_plus.git
cd 28th-conference-robo404_plus

# 2. 의존성 설치 및 빌드
rosdep install --from-paths src --ignore-src -r -y
colcon build --base-paths src
source install/setup.bash

# 3. 전체 파이프라인 실행
ros2 launch robo404_bringup full_pipeline.launch.py \
  engine_path:=/path/to/yolov8n.engine \
  camera_sensor_id:=0

# 4. 주행 시작
ros2 service call /start_follower std_srvs/srv/Empty '{}'
ros2 service call /start_driving std_srvs/srv/Empty '{}'
```

TensorRT `.engine` 파일이 없다면 실행할 Jetson Nano에서 먼저 생성해야 합니다. 자세한 실행 순서는 [How to Run](#5-how-to-run)과 [run.md](run.md)를 참고하세요.

---

## 목차

1. [Project Overview](#1-project-overview)
2. [Current Pipeline](#2-current-pipeline)
3. [Packages](#3-packages)
4. [Decision Logic](#4-decision-logic)
5. [How to Run](#5-how-to-run)
6. [ROS 2 Interface](#6-ros-2-interface)
7. [Model and Dataset](#7-model-and-dataset)
8. [Debugging](#8-debugging)
9. [Limitations and Future Work](#9-limitations-and-future-work)
10. [License / Credits](#10-license--credits)
11. [Related Repositories](#11-related-repositories)

---

## 1. Project Overview

ROBO 404++는 카메라 영상으로 차선을 추종하고 신호등 상태를 반영해 최종 주행 명령을 만드는 ROS 2 기반 자율주행 파이프라인입니다.

- **입력**: Jetson CSI Camera 1대가 발행하는 `/camera/image_raw`
- **라인 추종**: `follower_node`가 OpenCV로 라인을 검출하고 후보 주행 명령(`/cmd_vel_line`)과 라인 상태(`/path_state`)를 발행
- **신호등 판단**: `yolo_jetson/yolo_node`가 TensorRT YOLOv8로 객체를 검출하고, `traffic_light_node`가 bbox 영역 색상을 분석해 `/traffic_light_state`를 발행
- **의사결정**: `decision_node`가 라인, 신호등, timeout 상태를 종합해 최종 `/cmd_vel`을 발행
- **구동**: 실제 모터 제어 계층은 이 레포 밖의 robot base 또는 micro-ROS 펌웨어가 `/cmd_vel`을 구독해 처리

이 저장소의 1차 범위는 **카메라 이미지 입력부터 최종 `/cmd_vel` 출력까지**입니다. 모터 드라이버, 베이스 컨트롤러, 좌/우회전 판단, 교차로 판단, 3D depth 기반 검출은 현재 범위에서 제외합니다.

## 2. Current Pipeline

```text
CSI Camera
  -> /camera/image_raw
      -> follower_node
           -> /cmd_vel_line
           -> /path_state

      -> yolo_jetson/yolo_node
           -> /yolo/detections

      -> traffic_light_node
           <- /yolo/detections
           -> /traffic_light_state

decision_node
  <- /cmd_vel_line
  <- /path_state
  <- /traffic_light_state
  -> /cmd_vel
  -> /decision_state

debug_monitor_node (optional)
  <- /cmd_vel_line
  <- /path_state
  <- /yolo/detections
  <- /traffic_light_state
  <- /decision_state
  <- /cmd_vel
  -> /debug/pipeline_state
  -> /debug/pipeline_warnings
```

핵심 규칙은 하나입니다.

```text
decision_node만 최종 /cmd_vel을 발행한다.
```

`follower_node`는 최종 명령을 직접 발행하지 않고 라인 추종 기준 후보 속도만 `/cmd_vel_line`으로 냅니다.

## 3. Packages

```text
28th-conference-robo404_plus/
├── README.md
├── run.md
├── explain.md
├── contracts/
│   ├── overview.yaml
│   ├── pipeline.yaml
│   ├── topics.yaml
│   ├── nodes.yaml
│   └── states.yaml
└── src/
    ├── csi_camera/          # Jetson CSI camera publisher
    ├── follower/            # OpenCV line follower, /cmd_vel_line publisher
    ├── yolo_jetson/         # TensorRT YOLOv8 C++ node
    ├── yolo_bringup/        # YOLO launch files
    ├── yolo_debug/          # YOLO bbox debug image / UDP stream
    ├── yolo_msgs/           # DetectionArray 등 YOLO 메시지
    ├── traffic_light/       # Detection bbox + image color 기반 RED/GREEN/UNKNOWN 판단
    ├── decision/            # 최종 /cmd_vel 의사결정
    ├── debug_monitor/       # 통합 파이프라인 상태/경고 모니터
    └── robo404_bringup/     # perception, drive, full pipeline launch
```

구현 기준 계약은 [contracts/](contracts/)의 YAML 파일을 우선합니다.

## 4. Decision Logic

`decision_node`는 주기적으로 입력 freshness를 확인하고 상태 머신으로 최종 명령을 결정합니다.

```text
Decision Cycle
  -> timeout 처리 및 red_latched 갱신
  -> motion_enabled == false          => IDLE, zero Twist
  -> path_state == FINAL_STOP         => FINAL_STOP, zero Twist
  -> line timeout 또는 LINE_LOST      => LINE_LOST, zero Twist
  -> traffic_light_state == RED       => STOP_FOR_RED, zero Twist
  -> RED 이후 GREEN 미확인 상태       => WAIT_GREEN, zero Twist
  -> 주행 가능 상태                   => FOLLOW_LINE, /cmd_vel_line 통과
```

| 상태 | 조건 | 결과 |
| --- | --- | --- |
| `IDLE` | `/start_driving` 호출 전 또는 `/stop_driving` 이후 | 정지 |
| `FINAL_STOP` | `path_state == FINAL_STOP` | 정지 |
| `LINE_LOST` | 라인 상태 또는 `/cmd_vel_line` timeout, `LINE_LOST` | 정지 |
| `STOP_FOR_RED` | 신호등 `RED` | 정지, red latch 설정 |
| `WAIT_GREEN` | RED를 본 뒤 `GREEN`을 아직 확인하지 못함 | 정지 |
| `FOLLOW_LINE` | 라인 정상, 주행 enabled, RED latch 없음 | `/cmd_vel_line`을 `/cmd_vel`로 전달 |

처음부터 신호등이 `UNKNOWN`이면 라인 추종을 유지합니다. 한 번 `RED`를 본 뒤에는 `GREEN`이 확인될 때까지 정지합니다.

## 5. How to Run

### 사전 준비

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --base-paths src
source install/setup.bash
```

### TensorRT Engine 생성

모델 파일은 저장소에 포함하지 않습니다. TensorRT `.engine`은 실행할 Jetson Nano에서 생성하세요.

```bash
trtexec \
  --onnx=src/yolo_jetson/models/yolov8n.onnx \
  --saveEngine=/home/lee/models/yolov8n.engine \
  --fp16
```

다른 장비나 다른 TensorRT 버전에서 생성한 `.engine`은 Jetson Nano에서 로드되지 않을 수 있습니다.

### 전체 파이프라인

```bash
ros2 launch robo404_bringup full_pipeline.launch.py \
  engine_path:=/home/lee/models/yolov8n.engine \
  camera_sensor_id:=0
```

실제 주행은 launch 직후 자동으로 시작하지 않습니다.

```bash
ros2 service call /start_follower std_srvs/srv/Empty '{}'
ros2 service call /start_driving std_srvs/srv/Empty '{}'
```

정지는 다음 서비스를 호출합니다.

```bash
ros2 service call /stop_driving std_srvs/srv/Empty '{}'
ros2 service call /stop_follower std_srvs/srv/Empty '{}'
```

### 단계별 실행

카메라만 확인:

```bash
ros2 launch csi_camera single_csi.launch.py \
  sensor_id:=0 \
  image_topic:=/camera/image_raw
```

YOLO 추론만 확인:

```bash
ros2 launch yolo_bringup yolo.launch.py \
  engine_path:=/home/lee/models/yolov8n.engine \
  input_image_topic:=/camera/image_raw \
  namespace:=yolo
```

카메라 + YOLO + 신호등 판단:

```bash
ros2 launch robo404_bringup perception.launch.py \
  engine_path:=/home/lee/models/yolov8n.engine \
  camera_sensor_id:=0
```

라인 추종 + 의사결정:

```bash
ros2 launch robo404_bringup drive.launch.py \
  follower_image_topic:=/camera/image_raw
```

## 6. ROS 2 Interface

### 주요 토픽

| 토픽 | 타입 | Producer | Consumer |
| --- | --- | --- | --- |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | `csi_camera/camera` | `follower_node`, `yolo_node`, `traffic_light_node` |
| `/cmd_vel_line` | `geometry_msgs/msg/Twist` | `follower_node` | `decision_node`, `debug_monitor_node` |
| `/path_state` | `std_msgs/msg/String` | `follower_node` | `decision_node`, `debug_monitor_node` |
| `/yolo/detections` | `yolo_msgs/msg/DetectionArray` | `yolo_node` | `traffic_light_node`, `debug_node`, `debug_monitor_node` |
| `/traffic_light_state` | `std_msgs/msg/String` | `traffic_light_node` | `decision_node`, `debug_monitor_node` |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | `decision_node` | Robot base, `debug_monitor_node` |
| `/decision_state` | `std_msgs/msg/String` | `decision_node` | debug/logger, `debug_monitor_node` |
| `/debug/pipeline_state` | `std_msgs/msg/String` | `debug_monitor_node` | operator/logger |
| `/debug/pipeline_warnings` | `std_msgs/msg/String` | `debug_monitor_node` | operator/logger |

### 주요 서비스

| 서비스 | 타입 | 설명 |
| --- | --- | --- |
| `/start_follower` | `std_srvs/srv/Empty` | `follower_node`가 `/cmd_vel_line` 후보 속도를 내도록 활성화 |
| `/stop_follower` | `std_srvs/srv/Empty` | `follower_node` 후보 속도 정지 |
| `/start_driving` | `std_srvs/srv/Empty` | `decision_node`의 최종 주행 enable |
| `/stop_driving` | `std_srvs/srv/Empty` | `decision_node` 최종 `/cmd_vel` 정지 |
| `/yolo/enable` | `std_srvs/srv/SetBool` | YOLO 추론 enable/disable |

### 주요 파라미터

| 노드 | 파라미터 | 기본값 | 설명 |
| --- | --- | --- | --- |
| `follower_node` | `image_topic` | `/camera/image_raw` | 라인 검출 입력 이미지 |
| `follower_node` | `publish_debug_image` | `False` | `/follower/debug_image` 발행 |
| `follower_node` | `publish_mask_image` | `False` | `/follower/mask_image` 발행 |
| `follower_node` | `enable_udp_stream` | `False` | follower debug UDP H.264 stream |
| `decision_node` | `start_enabled` | `False` | launch 직후 주행 enable 여부 |
| `decision_node` | `publish_rate_hz` | `20.0` | 최종 명령 publish 주기 |
| `decision_node` | `cmd_vel_line_timeout_sec` | `0.5` | 라인 후보 속도 timeout |
| `decision_node` | `path_state_timeout_sec` | `0.5` | 라인 상태 timeout |
| `decision_node` | `traffic_light_timeout_sec` | `1.0` | 신호등 상태 timeout |
| `yolo_node` | `engine_path` | 필수 | TensorRT `.engine` 경로 |
| `yolo_node` | `threshold` | `0.5` | 검출 confidence threshold |
| `yolo_node` | `iou` | `0.5` | NMS IoU threshold |
| `traffic_light_node` | `traffic_light_class_names` | `["traffic light", "traffic_light"]` | 신호등으로 볼 YOLO class name |

`follower_node.py` 안의 라인 검출 상수(`LINEAR_SPEED`, `KP`, BGR threshold, crop 영역 등)는 실제 트랙과 카메라 위치에 맞춰 보정해야 합니다.

## 7. Model and Dataset

이 저장소에는 학습 데이터셋과 모델 바이너리를 포함하지 않습니다.

- `yolo_jetson`은 TensorRT YOLOv8 `.engine` 파일을 runtime 인자로 받습니다.
- ONNX 모델은 [Qengineering/YoloV8-TensorRT-Jetson_Nano](https://github.com/Qengineering/YoloV8-TensorRT-Jetson_Nano) 또는 별도 학습 산출물에서 준비합니다.
- 기본 `num_labels`는 COCO 80 class 기준입니다.
- 신호등 판단은 YOLO가 찾은 traffic light bbox 영역의 HSV 색상 비율을 분석해 `UNKNOWN`, `RED`, `GREEN` 중 하나로 발행합니다.

## 8. Debugging

전체 파이프라인 상태를 SSH에서 확인하려면 `debug_monitor_node`를 켭니다.

```bash
ros2 launch robo404_bringup full_pipeline.launch.py \
  engine_path:=/home/lee/models/yolov8n.engine \
  camera_sensor_id:=0 \
  use_debug_monitor:=True
```

```bash
ros2 topic echo /debug/pipeline_state
ros2 topic echo /debug/pipeline_warnings
```

YOLO bbox debug stream:

```bash
ros2 launch robo404_bringup full_pipeline.launch.py \
  engine_path:=/home/lee/models/yolov8n.engine \
  camera_sensor_id:=0 \
  use_debug:=True \
  debug_stream_ip:=<receiver_pc_ip>
```

Follower debug image:

```bash
ros2 launch robo404_bringup full_pipeline.launch.py \
  engine_path:=/home/lee/models/yolov8n.engine \
  camera_sensor_id:=0 \
  publish_follower_debug_image:=True \
  publish_follower_mask_image:=True
```

추가 실행 절차와 점검 순서는 [run.md](run.md)에 정리되어 있습니다.

## 9. Limitations and Future Work

**현재 한계**

- 라인 검출은 BGR threshold 기반이라 조명과 트랙 표면 변화에 민감합니다.
- TensorRT `.engine`은 Jetson Nano와 TensorRT 버전에 종속됩니다.
- 현재 1차 범위는 단일 카메라 `/camera/image_raw` 중심입니다. `dual_csi.launch.py`는 있으나 `full_pipeline.launch.py`의 기본 구성은 단일 CSI 카메라입니다.
- 좌/우회전, 교차로 판단, 3D depth 검출, YOLO tracking은 1차 범위에서 제외되어 있습니다.

**향후 개선**

- 라인 검출 threshold 자동 보정 또는 딥러닝 기반 라인 검출
- 신호등 상태 confidence와 timestamp를 포함하는 전용 메시지 도입
- 좌/우회전, 교차로, 표지판 기반 의사결정 확장
- 실험 환경별 FPS, latency, 성공률 benchmark 문서화

## 10. License / Credits

본 저장소는 여러 오픈소스와 프로젝트 코드를 통합했습니다. 패키지별 정확한 라이선스는 각 `package.xml`과 포함된 `LICENSE`/`NOTICE` 파일을 기준으로 확인하세요.

- `follower` - MIT, Gabriel Nascarella Hishida do Nascimento
- `yolo_msgs`, `yolo_bringup`, `yolo_debug` - GPL-3.0, Miguel Ángel González Santamarta
- `yolo_jetson` - BSD-3-Clause package metadata, Q-engineering `YoloV8-TensorRT-Jetson_Nano` 기반 코드 포함
- `csi_camera`, `robo404_bringup` - BSD-3-Clause package metadata
- `decision`, `traffic_light`, `debug_monitor` - MIT package metadata

## 11. Related Repositories

- **하드웨어**: [jiy0-0nv/robo404pp_hardware](https://github.com/jiy0-0nv/robo404pp_hardware) - 4WD 섀시 / 펌웨어(Raspberry Pi Pico 2) 등 하드웨어 구성
- **TensorRT YOLOv8 참고 구현**: [Qengineering/YoloV8-TensorRT-Jetson_Nano](https://github.com/Qengineering/YoloV8-TensorRT-Jetson_Nano)
