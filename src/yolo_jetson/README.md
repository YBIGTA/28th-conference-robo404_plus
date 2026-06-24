# yolo_jetson

Jetson Nano TensorRT 기반 YOLOv8 ROS 2 노드다.

이 패키지는 Q-engineering `YoloV8-TensorRT-Jetson_Nano` C++ TensorRT 코드를 Robo404 파이프라인에 맞게 ROS 2 `ament_cmake` 패키지로 정리한 것이다.

## Interface

```text
node:
  /yolo/yolo_node

subscribe:
  /camera/rgb/image_raw
  type: sensor_msgs/msg/Image

publish:
  /yolo/detections
  type: yolo_msgs/msg/DetectionArray

service:
  /yolo/enable
  type: std_srvs/srv/SetBool
```

## Engine 생성

TensorRT engine은 실행할 Jetson Nano에서 직접 생성해야 한다. TensorRT 버전이 다르면 다른 장비에서 만든 `.engine` 파일이 로드되지 않을 수 있다.

```bash
trtexec \
  --onnx=src/yolo_jetson/models/yolov8n.onnx \
  --saveEngine=/path/to/yolov8n.engine \
  --fp16
```

## 실행

```bash
ros2 launch yolo_bringup yolov8_trt.launch.py \
  engine_path:=/path/to/yolov8n.engine \
  input_image_topic:=/camera/rgb/image_raw \
  namespace:=yolo
```

기본 출력은 `/yolo/detections`이며, `traffic_light_node`는 이 detection bbox와 `/camera/rgb/image_raw` 이미지를 사용해 `/traffic_light_state`를 만든다.
