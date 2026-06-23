# ROS2 Pipeline Explain

이 문서는 Robo404 ROS2 파이프라인의 목적, 현재 구조, 목표 구조, 작업 순서를 설명한다.

README는 핵심만 담는 진입 문서이고, 구체적인 구현 기준은 `contracts/`의 YAML 파일을 기준으로 한다.

## 1. Scope

이 브랜치에서 다루는 범위는 이미지 입력부터 최종 `/cmd_vel` 출력까지이다.

```text
camera image
-> line detection
-> YOLO traffic light detection
-> traffic light state
-> decision state
-> /cmd_vel
```

아래는 1차 범위에서 제외한다.

```text
lino hardware
motor driver
base controller
actual actuator control
left turn decision
right turn decision
intersection decision
YOLO tracking
depth based 3D detection
```

## 2. Design Rule

가장 중요한 규칙은 하나다.

```text
decision_node만 최종 /cmd_vel을 발행한다.
```

기존 구조에서는 `follower_node`가 라인을 보고 바로 `/cmd_vel`을 발행한다. 하지만 신호등, 라인 유실, 정지 상태를 합치려면 최종 명령을 한 노드에서만 결정해야 한다.

따라서 구조를 아래처럼 바꾼다.

```text
현재:
follower_node -> /cmd_vel

목표:
follower_node -> /cmd_vel_line
decision_node -> /cmd_vel
```

`/cmd_vel_line`은 라인 추종 기준 후보 속도이고, 최종 주행 명령이 아니다.

## 3. Current Pipeline

현재 기준 구조는 다음과 같다.

```text
/camera/image_raw
  -> follower_node
      -> /cmd_vel

/camera/rgb/image_raw
  -> /yolo/yolo_node
      -> /yolo/detections
```

문제는 `follower_node`가 최종 `/cmd_vel`을 직접 발행한다는 점이다. 이 상태에서는 YOLO 기반 신호등 판단과 라인 추종 명령을 안정적으로 합치기 어렵다.

## 4. Target Pipeline

1차 목표 구조는 다음과 같다.

```text
Bottom Camera
  -> follower_node
      -> /cmd_vel_line
      -> /path_state

Top Camera
  -> yolo_node
      -> /yolo/detections
  -> traffic_light_node
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

각 노드는 다음 책임만 가진다.

```text
follower_node
  라인 검출
  라인 기준 후보 속도 생성
  라인 상태 발행

yolo_node
  YOLO 객체 검출
  raw detection 발행

traffic_light_node
  YOLO detection에서 traffic light bbox 선택
  RGB 이미지에서 bbox 영역 색상 분석
  UNKNOWN / RED / GREEN 신호등 상태 발행

decision_node
  라인 상태와 신호등 상태를 종합
  최종 /cmd_vel 발행
```

## 5. Source Packages

현재 `src/` 아래 주요 패키지는 다음과 같다.

```text
follower
  기존 라인 추종 패키지
  현재는 /cmd_vel을 직접 발행함
  목표는 /cmd_vel_line, /path_state 발행으로 변경

yolo_msgs
  yolo_msgs/msg/DetectionArray
  yolo_msgs/msg/Detection
  기타 YOLO 결과용 메시지 정의

yolo_jetson
  Jetson Nano TensorRT 기반 YOLO 노드
  yolo_node 포함
  1차 목표에서는 /yolo/detections 발행만 담당

yolo_bringup
  YOLO 관련 launch 파일 관리
  1차 목표에서는 TensorRT yolo_node와 선택적 debug_node 실행만 담당

yolo_ros
  YOLO debug_node만 유지
  Python Ultralytics 기반 yolo_node, tracking_node, detect_3d_node는 제거
```

주의할 점:

```text
yolo_jetson의 /yolo/detections 타입:
  yolo_msgs/msg/DetectionArray
```

기존 `yolo_ros` Python backend를 제거해도 `/yolo/detections` 계약은 유지한다.

## 6. Contracts Folder

`contracts/`는 구현 기준 계약서 모음이다.

```text
contracts/overview.yaml
  1차 범위, 원칙, 제외 대상

contracts/pipeline.yaml
  현재 파이프라인과 목표 파이프라인

contracts/topics.yaml
  토픽별 producer, consumer, message type, status

contracts/nodes.yaml
  노드별 책임, 입력, 출력, 서비스

contracts/states.yaml
  decision_node 상태, traffic_light 상태, 판단 규칙
```

구현할 때는 README보다 `contracts/`를 우선 기준으로 본다.

## 7. Node I/O Summary

핵심 토픽 계약은 다음과 같다.

```text
follower_node
  input:
    /camera/image_raw
  output:
    /cmd_vel_line
    /path_state

yolo_node
  input:
    /camera/rgb/image_raw
  output:
    /yolo/detections

traffic_light_node
  input:
    /yolo/detections
    /camera/rgb/image_raw
  output:
    /traffic_light_state

decision_node
  input:
    /cmd_vel_line
    /path_state
    /traffic_light_state
  output:
    /cmd_vel
    /decision_state
```

## 8. Decision State

1차 decision 상태는 아래 정도로 제한한다.

```text
IDLE
FOLLOW_LINE
STOP_FOR_RED
WAIT_GREEN
LINE_LOST
FINAL_STOP
```

기본 판단 우선순위는 다음과 같다.

```text
1. IDLE이면 zero Twist
2. LINE_LOST이면 zero Twist
3. RED이면 STOP_FOR_RED, zero Twist
4. RED 이후 UNKNOWN이면 WAIT_GREEN, zero Twist
5. GREEN이면 FOLLOW_LINE, /cmd_vel_line 통과
6. 정상 주행 가능하면 /cmd_vel_line 통과
```

처음부터 신호등이 `UNKNOWN`이면 라인 추종을 유지한다. 하지만 RED를 본 뒤 `UNKNOWN`이 되면 YOLO가 잠깐 놓친 것으로 보고 계속 정지한다.
