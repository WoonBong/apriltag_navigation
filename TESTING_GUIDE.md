# Testing Guide - 현장 테스트 가이드

AprilTag Navigation 패키지의 각 부분을 단계별로 테스트하는 방법입니다.

---

## 📋 목차
1. [테스트 전 체크리스트](#테스트-전-체크리스트)
2. [1단계: 하드웨어 테스트](#1단계-하드웨어-테스트)
3. [2단계: 비전 시스템 테스트](#2단계-비전-시스템-테스트)
4. [3단계: 맵 및 경로 계획 테스트](#3단계-맵-및-경로-계획-테스트)
5. [4단계: 주행 알고리즘 테스트](#4단계-주행-알고리즘-테스트)
6. [5단계: 통합 테스트](#5단계-통합-테스트)
7. [문제 해결](#문제-해결)

---

## 테스트 전 체크리스트

### 하드웨어 확인
- [ ] 로봇 전원 ON
- [ ] 배터리 충전 상태 확인
- [ ] 카메라 연결 확인
- [ ] 모터 드라이버 정상 작동

### ROS 환경 확인
```bash
# ROS 마스터 실행 확인
rostopic list

# 필수 토픽 확인
rostopic list | grep -E "(camera|odom|cmd_vel)"
# 결과 예시:
# /camera/image_raw
# /odom
# /cmd_vel
```

### 패키지 빌드
```bash
cd ~/nav_ws
catkin_make
source devel/setup.bash
```

---

## 1단계: 하드웨어 테스트

**목적:** 모터와 오도메트리가 정상 작동하는지 확인

### 1.1 오도메트리 테스트

```bash
# 오도메트리 데이터 확인
rostopic echo /odom

# 로봇을 손으로 밀어보고 위치 값이 변하는지 확인
```

**예상 결과:**
- 로봇을 앞으로 밀면 `pose.position.x` 증가
- 로봇을 옆으로 밀면 `pose.position.y` 증가
- 로봇을 회전하면 `orientation` 변화

### 1.2 모터 제어 테스트

**방법 1: 간단한 테스트 (main_node.py 사용)**
```bash
# 1m 전진 테스트
rosrun apriltag_navigation main_node.py --test-motor --distance 1.0 --direction forward

# 결과 확인
# Target: 1.00m, Actual: 1.05m, Error: 0.05m
```

**방법 2: 상세한 테스트 (diagnostics.py 사용)**
```bash
# 2m 전진 테스트
python scripts/diagnostics.py --test-motor --distance 2.0 --direction forward

# 후진 테스트
python scripts/diagnostics.py --test-motor --distance 1.0 --direction backward
```

**확인 사항:**
- [ ] 로봇이 직선으로 이동하는가?
- [ ] 목표 거리와 실제 거리 오차가 10% 이하인가?
- [ ] 정지 시 미끄러지지 않는가?

### 1.3 회전 테스트

```bash
# 시계 방향 90도 회전
rosrun apriltag_navigation main_node.py --test-pivot --direction cw

# 반시계 방향 90도 회전
rosrun apriltag_navigation main_node.py --test-pivot --direction ccw

# 180도 회전 (상세 테스트)
python scripts/diagnostics.py --test-pivot --angle 180 --direction cw
```

**확인 사항:**
- [ ] 회전 각도 오차가 5도 이하인가?
- [ ] 회전 중심이 로봇 중앙인가?
- [ ] 회전 후 제자리에 멈추는가?

### ✅ 1단계 통과 기준
- 오도메트리 데이터가 정상적으로 수신됨
- 모터가 명령대로 움직임
- 이동/회전 오차가 허용 범위 이내

---

## 2단계: 비전 시스템 테스트

**목적:** AprilTag 감지 및 거리 측정이 정확한지 확인

### 2.1 카메라 확인

```bash
# 카메라 이미지 확인
rosrun image_view image_view image:=/camera/image_raw

# 이미지가 정상적으로 보이는지 확인
# - 초점이 맞는지
# - 밝기가 적절한지
# - 프레임레이트가 안정적인지 (10Hz 이상)
```

### 2.2 AprilTag 감지 테스트

**준비:**
1. AprilTag를 벽에 부착 (높이: 카메라와 같은 높이)
2. 로봇을 태그 앞 2~3m 거리에 배치

```bash
# 10초간 비전 테스트
rosrun apriltag_navigation main_node.py --test-vision --duration 10

# 결과 예시:
# [5.0s] Tag 5: 2.31m | Tag 7: 3.50m
# Detected 2 unique tags: [5, 7]
```

**확인 사항:**
- [ ] 태그가 2~5m 거리에서 감지되는가?
- [ ] 거리 측정 값이 실제와 ±10cm 이내인가?
- [ ] 여러 태그가 동시에 감지되는가?

### 2.3 거리별 감지 테스트

| 거리 | 감지 여부 | 거리 오차 |
|------|-----------|-----------|
| 0.5m | ✓ | ±5cm |
| 1.0m | ✓ | ±5cm |
| 2.0m | ✓ | ±10cm |
| 3.0m | ✓ | ±15cm |
| 4.0m | ? | ? |
| 5.0m | ? | ? |

**테스트 방법:**
```bash
# 각 거리에서 30초 테스트
python scripts/diagnostics.py --test-vision --duration 30
```

### 2.4 각도별 감지 테스트

로봇을 태그에서 30도, 45도, 60도 옆으로 이동시켜 테스트

**확인 사항:**
- [ ] 정면(0도)에서 잘 감지되는가?
- [ ] 30도 각도에서 감지되는가?
- [ ] 45도 각도에서 감지되는가?

### ✅ 2단계 통과 기준
- 태그가 안정적으로 감지됨 (2~4m 거리)
- 거리 측정 오차 ±15cm 이내
- 초당 10회 이상 감지

---

## 3단계: 맵 및 경로 계획 테스트

**목적:** 맵 데이터와 경로 계획이 올바른지 확인

### 3.1 맵 파일 확인

```bash
# 맵 파일 열기
cat config/map.yaml

# 확인 사항:
# - 모든 태그의 좌표가 입력되어 있는가?
# - 엣지(이동 경로)가 정의되어 있는가?
# - 작업(task1, task2)이 정의되어 있는가?
```

### 3.2 경로 계획 테스트 (Python 인터프리터)

```python
# Python 인터프리터 실행
ipython

# 맵 매니저 로드
import sys
sys.path.insert(0, 'src')
from apriltag_navigation.map.map_manager import MapManager

map_mgr = MapManager()

# 태그 위치 확인
print(map_mgr.get_tag_position(0))  # (0.0, 0.0)
print(map_mgr.get_tag_position(5))  # (2.0, 0.0)

# 경로 계획
path = map_mgr.nav_graph.find_path(0, 10)
print(f"Path from 0 to 10: {path}")
# 예상: [0, 5, 7, 10]

# 엣지 정보
edge = map_mgr.nav_graph.get_edge(0, 5)
print(edge)
# {'type': 'move', 'direction': 'forward'}

# 작업 웨이포인트
waypoints = map_mgr.task_manager.get_task_waypoints('task1')
print(f"Task1 waypoints: {waypoints}")
```

**확인 사항:**
- [ ] 경로가 논리적으로 올바른가?
- [ ] 모든 태그 쌍에 대해 경로가 존재하는가?
- [ ] 엣지 타입(move/pivot)이 올바른가?

### ✅ 3단계 통과 기준
- 맵 데이터가 정확함
- 경로 계획이 최단 경로를 반환함
- 모든 작업의 웨이포인트가 정의됨

---

## 4단계: 주행 알고리즘 테스트

**목적:** Pure Pursuit 알고리즘이 태그를 잘 추적하는지 확인

### 4.1 단일 태그 추적 테스트

**준비:**
1. 태그를 3m 전방에 부착
2. 로봇을 시작 위치에 배치

```bash
# Mode 3: 직접 태그로 이동
rosrun apriltag_navigation main_node.py --mode 3 --tag 5
```

**관찰 사항:**
- [ ] 로봇이 태그를 향해 부드럽게 회전하는가?
- [ ] 직선으로 접근하는가? (지그재그 없이)
- [ ] 태그 앞에서 정확히 정지하는가?
- [ ] 정지 거리가 적절한가? (0.3~0.5m)

### 4.2 곡선 경로 추적 테스트

**준비:**
1. 태그를 45도 옆에 배치

```bash
# 옆에 있는 태그로 이동
rosrun apriltag_navigation main_node.py --mode 3 --tag 7
```

**확인 사항:**
- [ ] 부드러운 곡선으로 접근하는가?
- [ ] 급격한 방향 전환이 없는가?
- [ ] 목표에 정확히 도달하는가?

### 4.3 회전 + 이동 테스트

```bash
# Task 1 실행 (여러 태그 방문)
rosrun apriltag_navigation main_node.py --mode 1
```

**확인 사항:**
- [ ] 90도 회전이 정확한가?
- [ ] 회전 후 직선 이동이 부드러운가?
- [ ] 각 웨이포인트에 정확히 도달하는가?

### ✅ 4단계 통과 기준
- 단일 태그로 안정적으로 이동
- 경로 추적 오차가 ±20cm 이내
- 급격한 속도 변화 없음

---

## 5단계: 통합 테스트

**목적:** 전체 시스템이 실제 작업을 완수할 수 있는지 확인

### 5.1 Task 1 완주 테스트

```bash
rosrun apriltag_navigation main_node.py --mode 1
```

**체크리스트:**
- [ ] 모든 웨이포인트를 방문하는가?
- [ ] 웨이포인트 사이 이동이 부드러운가?
- [ ] 오류 없이 작업을 완료하는가?
- [ ] 최종 위치가 정확한가?

**성능 측정:**
- 완주 시간: ______ 초
- 경로 오차: ______ cm
- 성공률: ______ % (10회 중 성공 횟수)

### 5.2 Task 2 완주 테스트

```bash
rosrun apriltag_navigation main_node.py --mode 2
```

동일한 체크리스트로 테스트

### 5.3 다양한 시작 위치 테스트

**시나리오:**
1. 태그 0에서 시작 → Task 1
2. 태그 5에서 시작 → Task 1
3. 태그 7에서 시작 → Task 2

**확인:**
- [ ] 다른 시작 위치에서도 작동하는가?
- [ ] 경로 계획이 자동으로 조정되는가?

### 5.4 Mode 4: 스캔 작업 테스트

**준비:**
1. Excel 파일 작성 (`data/excel/test_scan.xlsx`)
2. 로봇을 시작 위치에 배치

```bash
rosrun apriltag_navigation main_node.py --mode 4
# Excel 파일 선택: 1
```

**확인 사항:**
- [ ] 각 스캔 위치로 정확히 이동하는가?
- [ ] 스캔 대기 상태가 동작하는가?
- [ ] 사용자 신호 후 다음 위치로 이동하는가?

### ✅ 5단계 통과 기준
- Task 1, 2 모두 오류 없이 완주
- 성공률 90% 이상 (10회 중 9회 성공)
- 예상 시간 내 완료

---

## 문제 해결

### 문제: 태그가 감지되지 않음

**원인 및 해결:**
- [ ] 카메라 초점 확인 → 재초점
- [ ] 조명 확인 → 조명 추가 또는 밝기 조정
- [ ] 태그 크기 확인 → 더 큰 태그 사용
- [ ] 거리 확인 → 2~4m 범위로 조정

**디버깅:**
```bash
# 카메라 이미지 확인
rosrun image_view image_view image:=/camera/image_raw

# 태그 감지 로그 확인
rosrun apriltag_navigation main_node.py --test-vision --duration 30
```

### 문제: 로봇이 직선으로 가지 않음

**원인 및 해결:**
- [ ] 바퀴 공기압 확인
- [ ] 모터 출력 균형 확인
- [ ] 오도메트리 보정 필요
- [ ] 바닥 상태 확인 (미끄러움)

**디버깅:**
```bash
# 직선 이동 테스트
python scripts/diagnostics.py --test-motor --distance 3.0
# 로봇 뒤에 줄을 그어 직선성 확인
```

### 문제: 회전이 부정확함

**원인 및 해결:**
- [ ] IMU 센서 보정
- [ ] 회전 중심 확인
- [ ] 바닥 마찰 확인

**디버깅:**
```bash
# 360도 회전 후 원위치 확인
python scripts/diagnostics.py --test-pivot --angle 360 --direction cw
```

### 문제: ROS 토픽이 안 보임

```bash
# ROS 마스터 재시작
roscore

# 노드 다시 실행
rosrun apriltag_navigation main_node.py

# 토픽 확인
rostopic list
rostopic hz /camera/image_raw
rostopic hz /odom
```

### 문제: 경로를 찾지 못함

**원인:**
- 맵 파일에 경로가 정의되지 않음

**해결:**
```bash
# 맵 파일 확인
cat config/map.yaml

# edges 섹션에 경로 추가
# - {from: 0, to: 5, type: move, direction: forward}
```

---

## 테스트 체크리스트 요약

### 빠른 체크 (5분)
```bash
# 1. 하드웨어
rosrun apriltag_navigation main_node.py --test-motor --distance 1.0

# 2. 비전
rosrun apriltag_navigation main_node.py --test-vision --duration 5

# 3. 통합
rosrun apriltag_navigation main_node.py --mode 3 --tag 5
```

### 전체 체크 (30분)
```bash
# 상세 진단 실행
python scripts/diagnostics.py --test-all

# Task 1, 2 각각 3회씩 실행
# 성공률 기록
```

---

## 성능 벤치마크 기록

| 날짜 | Task | 성공률 | 평균 시간 | 경로 오차 | 비고 |
|------|------|--------|-----------|-----------|------|
| 2025-01-13 | Task1 | 10/10 | 45s | ±15cm | |
| 2025-01-13 | Task2 | 9/10 | 52s | ±18cm | 1회 태그 감지 실패 |

---

각 단계를 통과하면 다음 단계로 진행하세요. 문제 발생 시 해당 단계로 돌아가 재테스트하세요!
