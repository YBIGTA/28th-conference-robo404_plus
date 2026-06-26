# Contracts

이 폴더는 ROS2 파이프라인 구현 기준을 문서화하는 계약서 모음이다.

README나 `docs/` 문서는 설명용이고, 이 폴더의 YAML 파일들은 구현할 때 맞춰야 하는 기준으로 사용한다.

## Files

```text
overview.yaml
  프로젝트 1차 범위, 원칙, 제외 대상

pipeline.yaml
  현재 파이프라인과 목표 파이프라인

topics.yaml
  토픽별 producer / consumer / message type / status

nodes.yaml
  노드별 책임, 입력, 출력, 서비스

states.yaml
  decision_node 상태, traffic_light 상태, 판단 규칙
```

## Phase 1 Rule

```text
decision_node만 최종 /cmd_vel을 발행한다.
```

이 규칙은 1차 구조의 핵심 계약이다.
