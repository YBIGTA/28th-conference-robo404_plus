# Robo404 Pipeline Run Guide

이 문서는 Jetson Nano에서 TensorRT YOLO 기반 전체 1차 파이프라인을 실행하는 순서를 정리한다.

최종 흐름:

```text
Bottom Camera
  -> /camera/image_raw
  -> follower_node
      -> /cmd_vel_line
      -> /path_state

Top Camera
  -> /camera/rgb/image_raw
  -> yolo_jetson/yolo_node
      -> /yolo/detections

traffic_light_node
  <- /yolo/detections
  <- /camera/rgb/image_raw
  -> /traffic_light_state

decision_node
  <- /cmd_vel_line
  <- /path_state
  <- /traffic_light_state
  -> /cmd_vel
  -> /decision_state
```

## 1. 전제

이 레포는 카메라 이미지 입력부터 최종 `/cmd_vel` 출력까지 담당한다.

아래는 별도로 준비되어 있어야 한다.

```text
Jetson Nano
CUDA / TensorRT / OpenCV
ROS 2 workspace 환경
CSI Camera 2개가 nvarguscamerasrc로 접근 가능한 상태
Robot base가 /cmd_vel을 구독하는 상태
```

`yolo_jetson`은 TensorRT `.engine` 파일을 필요로 한다. `.engine` 파일은 실행할 Jetson Nano에서 직접 생성한다.

## 2. TensorRT Engine 생성

예시:

```bash
cd /home/lee/Desktop/28th-conference-robo404_plus

trtexec \
  --onnx=src/yolo_jetson/models/yolov8n.onnx \
  --saveEngine=/home/lee/models/yolov8n.engine \
  --fp16
```

주의:

```text
다른 PC나 다른 TensorRT 버전에서 생성한 .engine은 Jetson Nano에서 로드되지 않을 수 있다.
```

## 3. 빌드

Jetson Nano에서 실행한다.

```bash
cd /home/lee/Desktop/28th-conference-robo404_plus
colcon build --base-paths src
source install/setup.bash
```

`robo/` 같은 개인 보관 폴더가 workspace 안에 있으면 colcon이 추가 패키지로 잡을 수 있다. 기본 실행은 `--base-paths src`를 사용한다.

## 4. 전체 파이프라인 실행

기본 실행:

```bash
cd /home/lee/Desktop/28th-conference-robo404_plus
source install/setup.bash

ros2 launch robo404_bringup full_pipeline.launch.py \
  engine_path:=/home/lee/models/yolov8n.engine \
  bottom_sensor_id:=0 \
  top_sensor_id:=1
```

이 launch는 아래 노드들을 함께 실행한다.

```text
csi_camera/bottom_camera
csi_camera/top_camera
yolo_jetson/yolo_node
traffic_light_node
follower_node
decision_node
```

기본 연결:

```text
sensor_id=0 -> /camera/image_raw     -> follower_node
sensor_id=1 -> /camera/rgb/image_raw -> yolo_jetson/yolo_node + traffic_light_node
```

카메라가 반대로 잡히면 launch 인자만 바꾼다.

```bash
ros2 launch robo404_bringup full_pipeline.launch.py \
  engine_path:=/home/lee/models/yolov8n.engine \
  bottom_sensor_id:=1 \
  top_sensor_id:=0
```

SSH/headless Jetson에서 bbox debug 화면을 PC로 보내려면:

```bash
ros2 launch robo404_bringup full_pipeline.launch.py \
  engine_path:=/home/lee/models/yolov8n.engine \
  bottom_sensor_id:=0 \
  top_sensor_id:=1 \
  use_debug:=True \
  debug_stream_ip:=<receiver_pc_ip>
```

PC에서 수신:

```bash
gst-launch-1.0 -v udpsrc port=5000 \
  caps="application/x-rtp,media=video,encoding-name=H264,payload=96" \
  ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink sync=false
```

ROS image topic도 같이 보고 싶으면 `publish_dbg_image:=True`를 추가한다.

Follower line debug image를 같이 보려면:

```bash
ros2 launch robo404_bringup full_pipeline.launch.py \
  engine_path:=/home/lee/models/yolov8n.engine \
  bottom_sensor_id:=0 \
  top_sensor_id:=1 \
  publish_follower_debug_image:=True
```

Follower line debug 화면을 UDP H.264 stream으로 보려면:

```bash
ros2 launch robo404_bringup full_pipeline.launch.py \
  engine_path:=/home/lee/models/yolov8n.engine \
  bottom_sensor_id:=0 \
  top_sensor_id:=1 \
  enable_follower_debug_stream:=True \
  follower_debug_stream_ip:=<receiver_pc_ip> \
  follower_debug_stream_port:=5001
```

PC에서 수신:

```bash
gst-launch-1.0 -v udpsrc port=5001 \
  caps="application/x-rtp,media=video,encoding-name=H264,payload=96" \
  ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink sync=false
```

Follower threshold mask까지 보려면:

```bash
ros2 launch robo404_bringup full_pipeline.launch.py \
  engine_path:=/home/lee/models/yolov8n.engine \
  bottom_sensor_id:=0 \
  top_sensor_id:=1 \
  publish_follower_debug_image:=True \
  publish_follower_mask_image:=True
```

주의:

```text
full_pipeline.launch.py는 노드만 실행한다.
로봇은 launch 직후 자동으로 움직이지 않는다.
주행 시작은 /start_follower와 /start_driving 서비스 호출로 한다.
```

확인:

```bash
ros2 topic hz /camera/image_raw
ros2 topic hz /camera/rgb/image_raw
ros2 topic info /yolo/detections
ros2 topic echo /traffic_light_state
ros2 topic echo /path_state
ros2 topic echo /decision_state
ros2 topic hz /follower/debug_image
ros2 topic hz /follower/mask_image
```

이미지 확인:

```bash
rqt_image_view /follower/debug_image
rqt_image_view /follower/mask_image
```

## 5. 개별 디버깅: 카메라 실행

새 터미널:

```bash
cd /home/lee/Desktop/28th-conference-robo404_plus
source install/setup.bash

ros2 launch csi_camera dual_csi.launch.py \
  bottom_sensor_id:=0 \
  top_sensor_id:=1
```

기본 연결:

```text
sensor_id=0 -> /camera/image_raw     -> follower_node
sensor_id=1 -> /camera/rgb/image_raw -> yolo_jetson/yolo_node
```

카메라가 반대로 잡히면 launch 인자만 바꾼다.

```bash
ros2 launch csi_camera dual_csi.launch.py \
  bottom_sensor_id:=1 \
  top_sensor_id:=0
```

확인:

```bash
ros2 topic list | grep camera
ros2 topic hz /camera/rgb/image_raw
ros2 topic hz /camera/image_raw
```

필수 토픽:

```text
/camera/rgb/image_raw
/camera/image_raw
```

## 6. 개별 디버깅: YOLO 실행

새 터미널:

```bash
cd /home/lee/Desktop/28th-conference-robo404_plus
source install/setup.bash

ros2 launch yolo_bringup yolov8_trt.launch.py \
  engine_path:=/home/lee/models/yolov8n.engine \
  input_image_topic:=/camera/rgb/image_raw \
  namespace:=yolo
```

SSH/headless Jetson에서 bbox debug 화면을 PC로 보내려면:

```bash
ros2 launch yolo_bringup yolov8_trt.launch.py \
  engine_path:=/home/lee/models/yolov8n.engine \
  input_image_topic:=/camera/rgb/image_raw \
  namespace:=yolo \
  use_debug:=True \
  debug_stream_ip:=<receiver_pc_ip>
```

PC에서 수신:

```bash
gst-launch-1.0 -v udpsrc port=5000 \
  caps="application/x-rtp,media=video,encoding-name=H264,payload=96" \
  ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink sync=false
```

ROS image topic도 같이 보고 싶으면 `publish_dbg_image:=True`를 추가한다.

확인:

```bash
ros2 topic info /yolo/detections
ros2 topic echo /yolo/detections --once
```

기대 타입:

```text
yolo_msgs/msg/DetectionArray
```

## 7. 개별 디버깅: Traffic Light 실행

새 터미널:

```bash
cd /home/lee/Desktop/28th-conference-robo404_plus
source install/setup.bash

ros2 run traffic_light traffic_light_node
```

확인:

```bash
ros2 topic echo /traffic_light_state
```

기대 값:

```text
UNKNOWN
RED
GREEN
```

## 8. 개별 디버깅: Follower 실행

새 터미널:

```bash
cd /home/lee/Desktop/28th-conference-robo404_plus
source install/setup.bash

ros2 run follower follower_node
```

debug image를 켜서 단독 실행하려면:

```bash
ros2 run follower follower_node --ros-args \
  -p publish_debug_image:=true \
  -p publish_mask_image:=true
```

UDP stream까지 켜서 단독 실행하려면:

```bash
ros2 run follower follower_node --ros-args \
  -p enable_udp_stream:=true \
  -p stream_host:=<receiver_pc_ip> \
  -p stream_port:=5001
```

확인:

```bash
ros2 topic echo /path_state
ros2 topic echo /cmd_vel_line
ros2 topic hz /follower/debug_image
ros2 topic hz /follower/mask_image
```

`follower_node`는 `/start_follower` 서비스가 호출되기 전에는 zero Twist를 낸다.

## 9. 개별 디버깅: Decision 실행

새 터미널:

```bash
cd /home/lee/Desktop/28th-conference-robo404_plus
source install/setup.bash

ros2 run decision decision_node
```

확인:

```bash
ros2 topic echo /decision_state
ros2 topic echo /cmd_vel
```

`decision_node`는 기본값으로 정지 상태다. 주행을 시작하려면 서비스를 호출한다.

## 10. 주행 시작 / 정지

주행 시작:

```bash
ros2 service call /start_follower std_srvs/srv/Empty '{}'
ros2 service call /start_driving std_srvs/srv/Empty '{}'
```

주행 정지:

```bash
ros2 service call /stop_driving std_srvs/srv/Empty '{}'
ros2 service call /stop_follower std_srvs/srv/Empty '{}'
```

## 11. 전체 확인 명령

토픽 목록:

```bash
ros2 topic list
```

핵심 토픽 확인:

```bash
ros2 topic info /yolo/detections
ros2 topic echo /traffic_light_state
ros2 topic echo /path_state
ros2 topic echo /decision_state
ros2 topic echo /cmd_vel
ros2 topic hz /follower/debug_image
```

서비스 확인:

```bash
ros2 service list | grep -E 'start|stop|enable'
```

YOLO on/off:

```bash
ros2 service call /yolo/enable std_srvs/srv/SetBool '{data: false}'
ros2 service call /yolo/enable std_srvs/srv/SetBool '{data: true}'
```

## 12. 정상 동작 기준

```text
/yolo/detections가 yolo_msgs/msg/DetectionArray로 발행된다.
traffic light bbox가 잡히면 /traffic_light_state가 RED/GREEN/UNKNOWN으로 바뀐다.
라인이 보이면 /path_state가 LINE_VISIBLE이다.
라인을 잃으면 /path_state가 LINE_LOST이다.
RED 상태에서는 /decision_state가 STOP_FOR_RED 또는 WAIT_GREEN으로 바뀌고 /cmd_vel은 zero Twist다.
GREEN 상태가 들어오면 /decision_state가 FOLLOW_LINE으로 바뀌고 /cmd_vel_line이 /cmd_vel로 통과된다.
publish_follower_debug_image:=True이면 /follower/debug_image가 발행된다.
publish_follower_mask_image:=True이면 /follower/mask_image가 발행된다.
enable_follower_debug_stream:=True이면 follower debug 화면이 UDP port 5001로 송출된다.
```

## 13. 자주 나는 문제

### yolo_jetson 빌드에서 nvcc를 못 찾는 경우

```text
Failed to find nvcc.
```

CUDA toolkit이 설치되어 있지 않거나 Jetson 환경이 아니다. Jetson Nano에서 CUDA/TensorRT 설치 상태를 확인한다.

### .engine 파일 로드 실패

```text
TensorRT engine file does not exist
deserializeCudaEngine 실패
```

경로가 틀렸거나 현재 Jetson의 TensorRT 버전과 맞지 않는 engine일 수 있다. 같은 Jetson에서 `trtexec`로 다시 생성한다.

### /traffic_light_state가 계속 UNKNOWN인 경우

```text
/yolo/detections에 traffic light class가 있는지 확인한다.
/camera/rgb/image_raw가 정상 발행되는지 확인한다.
신호등 bbox가 너무 작거나 색상 threshold에 안 들어올 수 있다.
```

### 카메라 노드가 바로 종료되는 경우

```text
Jetson Nano에서 nvarguscamerasrc가 동작하는지 확인한다.
sensor_id 0/1이 실제 카메라 연결과 맞는지 확인한다.
다른 프로세스가 같은 CSI 카메라를 이미 열고 있지 않은지 확인한다.
필요하면 bottom_sensor_id/top_sensor_id 또는 flip_method launch 인자를 바꾼다.
```

### /cmd_vel이 계속 zero인 경우

```text
/start_follower와 /start_driving 서비스를 둘 다 호출했는지 확인한다.
/path_state가 LINE_VISIBLE인지 확인한다.
/traffic_light_state가 RED 또는 WAIT_GREEN 상태를 만들고 있지 않은지 확인한다.
```
