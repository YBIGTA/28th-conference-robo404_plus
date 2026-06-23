# Robo404 ROS2 Pipeline

이 레포는 Robo404의 ROS2 소프트웨어 파이프라인을 정리하고 구현하기 위한 작업 공간이다.

1차 범위는 **카메라 이미지 입력부터 최종 `/cmd_vel` 출력까지**이다. lino hardware, motor driver, base controller, 실제 actuator 제어는 이 브랜치의 범위에서 제외하고 이후 하드웨어 브랜치와 합친다.

## Phase 1 Goal

```text
라인을 따라 주행한다.
빨간불이면 멈춘다.
초록불이면 다시 간다.
라인을 잃으면 정지하거나 복구 대기한다.
최종 /cmd_vel은 decision_node 하나만 발행한다.
```

1차 구조에서는 좌회전/우회전 판단, 교차로 판단, YOLO tracking, depth 기반 3D detection을 제외한다. YOLO는 우선 신호등 판단에만 사용한다.

## Target Pipeline

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

핵심 규칙:

```text
follower_node -> /cmd_vel_line
decision_node -> /cmd_vel
```

`follower_node`는 라인 기준 후보 속도만 만들고, 최종 주행 명령은 `decision_node`만 발행한다.

## Repository Structure

```text
src/
  follower/       라인 검출 및 라인 추종 후보 속도 생성
  yolo_msgs/      YOLO DetectionArray / Detection 메시지 정의
  yolo_jetson/    Jetson Nano / TensorRT YOLO 검출 노드
  yolo_ros/       YOLO 디버그 노드
  yolo_bringup/   YOLO launch 파일 관리

contracts/        구현 기준 계약서
docs/             설계 설명 문서
explain.md        전체 구조와 작업 순서 상세 설명
```

## Contracts

`contracts/`는 구현할 때 맞춰야 하는 기준이다.

```text
contracts/overview.yaml   1차 범위, 원칙, 제외 대상
contracts/pipeline.yaml   현재 파이프라인과 목표 파이프라인
contracts/topics.yaml     토픽별 producer / consumer / message type
contracts/nodes.yaml      노드별 책임, 입력, 출력, 서비스
contracts/states.yaml     decision 상태, traffic light 상태, 판단 규칙
```

상세 설명은 [explain.md](explain.md)를 먼저 보고, 실제 구현 기준은 [contracts/README.md](contracts/README.md)와 각 YAML 파일을 기준으로 본다.
