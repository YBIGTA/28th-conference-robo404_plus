# yolo_jetson

Jetson Nano TensorRT 기반 YOLOv8 ROS 2 노드입니다.

이 패키지는 Q-engineering `YoloV8-TensorRT-Jetson_Nano` C++ TensorRT 코드를 Robo404 파이프라인에 맞게 ROS 2 `ament_cmake` 패키지로 정리한 것입니다.

## Interface

기본 bringup에서는 `namespace:=yolo`, `input_image_topic:=/camera/image_raw`로 실행합니다.

```text
node:
  /yolo/yolo_node

subscribe:
  /camera/image_raw
  type: sensor_msgs/msg/Image

publish:
  /yolo/detections
  type: yolo_msgs/msg/DetectionArray

service:
  /yolo/enable
  type: std_srvs/srv/SetBool
```

노드 내부 topic 이름은 `image_raw`, `detections`, `enable`입니다. Launch에서 namespace와 remapping을 적용해 `/yolo/detections`, `/yolo/enable`, `/camera/image_raw`로 연결합니다.

## Engine 생성

TensorRT `.engine` 파일은 실행할 Jetson Nano에서 직접 생성해야 합니다. TensorRT 버전이나 GPU 환경이 다르면 다른 장비에서 만든 `.engine` 파일이 로드되지 않을 수 있습니다.

```bash
trtexec \
  --onnx=src/yolo_jetson/models/yolov8n.onnx \
  --saveEngine=/home/lee/models/yolov8n.engine \
  --fp16
```

모델 파일은 저장소에 포함하지 않습니다. ONNX 모델은 [Qengineering/YoloV8-TensorRT-Jetson_Nano](https://github.com/Qengineering/YoloV8-TensorRT-Jetson_Nano) 또는 별도 학습 산출물에서 준비하세요.

## 실행

YOLO 노드만 실행:

```bash
ros2 launch yolo_bringup yolo.launch.py \
  engine_path:=/home/lee/models/yolov8n.engine \
  input_image_topic:=/camera/image_raw \
  namespace:=yolo
```

TensorRT launch를 직접 실행할 수도 있습니다.

```bash
ros2 launch yolo_bringup yolov8_trt.launch.py \
  engine_path:=/home/lee/models/yolov8n.engine \
  input_image_topic:=/camera/image_raw \
  namespace:=yolo
```

전체 Robo404 perception 파이프라인에서는 `robo404_bringup/perception.launch.py`가 CSI camera, `yolo_node`, `traffic_light_node`를 함께 실행합니다.

```bash
ros2 launch robo404_bringup perception.launch.py \
  engine_path:=/home/lee/models/yolov8n.engine \
  camera_sensor_id:=0
```

기본 출력은 `/yolo/detections`입니다. `traffic_light_node`는 이 detection bbox와 `/camera/image_raw` 이미지를 사용해 `/traffic_light_state`를 만듭니다.
