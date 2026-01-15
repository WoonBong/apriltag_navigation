# Field Test Protocol

현장에서 AprilTag Navigation 시스템을 테스트하기 위한 체크리스트입니다.
각 파트별로 최소 단위 기능을 검증하여 전체 시스템이 정상 동작함을 보장합니다.

---

## 목차
1. [Level 0: ROS 인프라 점검](#level-0-ros-인프라-점검)
2. [Level 1: Vision 모듈](#level-1-vision-모듈)
3. [Level 2: Map Manager](#level-2-map-manager)
4. [Level 3: Robot Controller](#level-3-robot-controller)
5. [Level 4: Rotation Controller](#level-4-rotation-controller)
6. [Level 5: 통합 동작 테스트](#level-5-통합-동작-테스트)
7. [Level 6: 전체 미션 테스트](#level-6-전체-미션-테스트)
8. [Level 7: 예외 처리 테스트](#level-7-예외-처리-테스트)

---

## Level 0: ROS 인프라 점검

시스템 전체가 동작하려면 ROS 토픽과 설정 파일이 올바르게 준비되어야 합니다.

| 항목 | 테스트 방법 | 통과 기준 | 비고 |
|------|----------|---------|------|
| **Camera 토픽** | `rostopic hz /rgb` | > 10 Hz | 이미지 스트림 확인 |
| **Camera Info** | `rostopic echo /camera_info -n 1` | K matrix 존재 | 캘리브레이션 데이터 |
| **Odometry** | `rostopic hz /odom` | > 20 Hz | 위치 추정 기준 |
| **Cmd Vel** | `rostopic info /cmd_vel` | Subscribers: 1 | 모터 제어 토픽 |
| **Map YAML** | `cat config/map.yaml \| grep tags -A 3` | 73개 태그 존재 | 맵 데이터 로드 |

**실패 시 조치:**
- Camera 토픽 없음 → 카메라 드라이버 재시작
- Odometry 없음 → 로봇 베이스 컨트롤러 재시작
- Map YAML 오류 → 파일 경로 확인

---

## Level 1: Vision 모듈

AprilTag 검출 및 위치 추정 기능을 검증합니다.

### 1.1 태그 검출 (Tag Detection)

| 테스트 | 실행 방법 | 통과 기준 |
|-------|---------|---------|
| 기본 검출 | 태그 508을 1m 거리에 배치 → Vision 활성화 | `get_detected_tags()`에 508 포함 |

**검증 명령:**
```bash
rostopic echo /rgb -n 1  # 이미지 수신 확인
# Vision 모듈 로그에서 "Detected tags: [508]" 확인
```

**예상 출력:**
```python
detected_tags = vision.get_detected_tags()
# 예상 결과: [508]
# 또는 여러 태그가 보이는 경우: [508, 500, 501, ...]
```

태그 508이 1m 거리에서 카메라 시야에 정상적으로 들어오면 `get_detected_tags()` 함수는 508을 포함한 리스트를 반환합니다. 태그가 검출되지 않으면 빈 리스트 `[]`를 반환합니다.

### 1.2 정렬 각도 (Alignment Angle)

| 태그 기울기 | 측정 각도 | 오차 허용 | Pass/Fail |
|-----------|---------|---------|----------|
| 0° (수평) | _____ ° | < 2° | ☐ |
| 5° (CW) | _____ ° | < 2° | ☐ |
| -10° (CCW) | _____ ° | < 2° | ☐ |

**측정 방법:**
```python
align_angle = vision.get_alignment_angle(508)
print(f"Alignment angle: {align_angle:.2f}°")
```

### 1.4 Lateral Offset

| 로봇 위치 | 측정 Lateral | 오차 허용 | Pass/Fail |
|---------|------------|---------|----------|
| 중앙 | _____ m | < 2cm | ☐ |
| 좌측 10cm | _____ m | < 2cm | ☐ |
| 우측 10cm | _____ m | < 2cm | ☐ |

**측정 방법:**
```python
lateral = vision.get_tag_lateral(508)
print(f"Lateral offset: {lateral:.3f}m")
# + = 태그가 오른쪽, - = 태그가 왼쪽
```

---

## Level 2: Map Manager

맵 데이터 로드 및 경로 탐색 기능을 검증합니다.

### 2.1 Tag Database

| 테스트 | 명령 | 예상 결과 |
|-------|------|---------|
| 태그 위치 조회 | `tag_db.get_position(508)` | `(-0.3975, 5.35)` |
| Zone 조회 | `tag_db.get_zone(100)` | `'B'` |
| 존재 여부 | `tag_db.exists(999)` | `False` |

### 2.2 Navigation Graph

| 테스트 | 명령 | 예상 결과 |
|-------|------|---------|
| 경로 탐색 | `nav_graph.find_path(508, 100)` | 경로 길이 ≥ 2 |
| 엣지 조회 | `nav_graph.get_edge(500, 501)` | `{'type': 'pivot', 'direction': 'cw'}` |
| 경로 없음 | `nav_graph.find_path(508, 999)` | `None` |

### 2.3 Task Manager

| 테스트 | 명령 | 예상 결과 |
|-------|------|---------|
| Task 1 로드 | `task_manager.get_task_waypoints('task1')` | 45개 waypoints |
| Task 2 로드 | `task_manager.get_task_waypoints('task2')` | 75개 waypoints |
| Excel 스캔 | `task_manager.get_excel_scan_waypoints(path)` | waypoints + scan_tags 반환 |

---

## Level 3: Robot Controller

Odometry 기반 위치 추정 및 모터 제어를 검증합니다.

### 3.1 Odometry 수신

| 테스트 | 방법 | 통과 기준 |
|-------|------|---------|
| Ready 확인 | `robot.is_ready()` | 5초 내 `True` |
| 위치 조회 | `robot.get_position()` | `(x, y)` 반환 |
| Heading 조회 | `robot.get_heading()` | `-π ~ π` 범위 |

**`is_ready()` 테스트 상황:**
`is_ready()`는 RobotController가 `/odom` 토픽과 `/camera_info` 토픽을 모두 수신했는지 확인하는 함수입니다. 시스템 시작 직후 또는 센서 연결이 끊겼다가 복구된 직후에 이 함수를 호출하여 로봇이 주행 준비가 되었는지 확인합니다. 정상적인 경우 5초 이내에 `True`를 반환해야 합니다.

**테스트 방법:**
```python
robot = RobotInterface()
ready = robot.wait_until_ready(timeout=10.0)
if ready:
    print("Robot is ready to move")
else:
    print("Timeout: Sensors not ready")
```

### 3.2 참조점 (Reference Point)

| 테스트 | 방법 | 확인 사항 |
|-------|------|---------|
| 참조점 설정 | `robot.set_reference_point(1.0, 2.0, 0.5)` | `ref_robot_x = 1.0` 저장 |
| 추정 위치 | `robot.get_estimated_pose()` | Odometry delta 기반 계산 |

### 3.3 속도 명령

| 테스트 | 명령 | 확인 |
|-------|------|------|
| 전진 | `robot.move(0.3, 0)` | `/cmd_vel`: linear.x = 0.3 |
| 후진 | `robot.move(-0.3, 0)` | `/cmd_vel`: linear.x = -0.3 |
| 회전 | `robot.move(0, 0.5)` | `/cmd_vel`: angular.z = 0.5 |
| 정지 | `robot.stop()` | `/cmd_vel`: all zeros |

---

## Level 4: Rotation Controller

90도 회전 제어를 검증합니다.

### 4.1 회전 명령

| 방향 | 초기 Heading | 목표 Heading | 오차 허용 | Pass/Fail |
|------|-----------|------------|---------|----------|
| CW | 0° | -90° | < 5° | ☐ |
| CCW | 0° | +90° | < 5° | ☐ |
| CW (연속) | 90° | 0° | < 5° | ☐ |

**테스트 방법:**
```bash
# 회전 전 heading 기록
initial_heading = _____ °

# 회전 실행
rosrun apriltag_navigation main_node.py --mode 1
# 첫 pivot point에서 회전 관찰

# 회전 후 heading 기록
final_heading = _____ °

# 차이 계산
diff = final_heading - initial_heading
# CW: diff ≈ -90°, CCW: diff ≈ +90°
```

### 4.2 회전 완료 판정

| 오차 | 결과 | 예상 |
|------|------|------|
| < 1° | `is_complete()` = `True` | 회전 완료 |
| > 1° | `is_complete()` = `False` | 계속 회전 |

---

## Level 5: 통합 동작 테스트

상위 함수 3개(`move_to_tag`, `rotate_90`, `align_to_tag`)를 검증합니다.

### 5.1 move_to_tag()

**Setup:**
1. 로봇을 tag 508(Dock)에 배치
2. Tag 100을 경로상에 배치 (Zone B)

**Test:**
```python
robot = RobotInterface()
robot.wait_until_ready()
edge = {'direction': 'forward', 'type': 'move'}
success = robot.move_to_tag(100, edge)
```

**검증 항목:**
- [ ] Vision에서 tag 100 검출
- [ ] Pure Pursuit으로 경로 계산 (angular velocity 변화)
- [ ] 로봇이 tag 100에 도착 (center_y ≈ 360px)
- [ ] Lateral offset < 5cm
- [ ] 도착 후 `return True`

### 5.2 rotate_90()

**Setup:**
1. 로봇을 평평한 바닥에 배치
2. 초기 heading 기록

**Test:**
```python
initial_heading = robot.get_heading()
success = robot.rotate_90('cw')
final_heading = robot.get_heading()
```

**검증 항목:**
- [ ] 회전 시작 (target theta = current - π/2)
- [ ] PID 제어 실행 (angular velocity ≈ 0.2 rad/s)
- [ ] 회전 완료 (|final - initial + π/2| < 5°)
- [ ] 로봇 정지 확인

### 5.3 align_to_tag()

**Setup:**
1. 로봇을 tag 100 앞에 배치
2. 로봇을 약 5° 정도 기울임

**Test:**
```python
success = robot.align_to_tag(100)
```

**검증 항목:**
- [ ] Alignment angle 계산 (코너 기반)
- [ ] 회전 제어 (angular = -angle * 0.8)
- [ ] 정렬 완료 (angle < 0.5°, lateral < 5cm)
- [ ] 참조점 업데이트 확인 (`ref_robot_x/y` 변경)
- [ ] 벽 거리 계산 및 로그 출력

---

## Level 6: 전체 미션 테스트

4가지 모드 전체를 실행하여 시스템 종합 성능을 검증합니다.

### 6.1 Task 1 실행

**명령:**
```bash
rosrun apriltag_navigation main_node.py --mode 1
```

**검증 항목:**
- [ ] 45개 웨이포인트 모두 방문
- [ ] Pivot 구간 정상 회전 (500→501, CW)
- [ ] Dock(508)로 복귀
- [ ] 실행 시간 기록: _____ 분
- [ ] 실패한 웨이포인트: _____ 개
- [ ] Dock 복귀 정확도: lateral offset = _____ cm

**목표:**
- 성공률: 100% (45/45)
- Dock 복귀 오차: < 5cm

### 6.2 Task 2 실행

**명령:**
```bash
rosrun apriltag_navigation main_node.py --mode 2
```

**검증 항목:**
- [ ] 75개 웨이포인트 모두 방문
- [ ] Zone C 구간 정상 동작
- [ ] Backward 이동 정상 동작 (해당 edge 있을 경우)
- [ ] Dock 복귀 정확도: lateral offset = _____ cm

### 6.3 Mode 3: Direct Navigation

**명령:**
```bash
rosrun apriltag_navigation main_node.py --mode 3 --tag 123
```

**검증 항목:**
- [ ] BFS 경로 생성 확인 (로그에서 waypoints 출력)
- [ ] 최단 경로로 이동
- [ ] 목표 태그(123) 도착
- [ ] 연속 입력: 새 목표 태그 입력 후 재실행
- [ ] 'q' 입력 시 Dock 복귀

### 6.4 Mode 4: Excel Scan

**명령:**
```bash
rosrun apriltag_navigation main_node.py --mode 4 --excel test.xlsx
```

**검증 항목:**
- [ ] Excel에서 `group_id` 추출 (로그 확인)
- [ ] Scan tags 생성: _____ 개
- [ ] 전체 경로: 508 시작, 508 종료
- [ ] 각 scan tag에서 정지
- [ ] `/robot_pose` 토픽 발행 (2초마다)
- [ ] Manipulator 좌표 변환 확인 (Isaac → Manip)
- [ ] `/scan_finished` 신호 수신 후 다음 지점 이동

**Manipulator 좌표 검증:**
| Tag ID | Isaac (x, y) | Manip (pub_x, pub_y) | Pass/Fail |
|--------|-------------|---------------------|----------|
| 100 | (-0.3975, 0.45) | (-0.45, 0.3975) | ☐ |
| 104 | _____ | _____ | ☐ |

---

## Level 7: 예외 처리 테스트

시스템이 오류 상황에서도 안전하게 동작하는지 검증합니다.

### 7.1 태그 미검출

**Test:**
1. 로봇 이동 중 태그를 손으로 가림

**예상 동작:**
- [ ] "no tag | straight" 로그 출력
- [ ] 직진 모드로 전환 (Pure Pursuit 중단)
- [ ] 태그 다시 보이면 정상 모드 복귀

### 7.2 경로 없음

**Test:**
```bash
rosrun apriltag_navigation main_node.py --mode 3 --tag 999
```

**예상 동작:**
- [ ] "No path found" 오류 로그
- [ ] 빈 waypoints 반환
- [ ] 프로그램 종료 (크래시 없음)

### 7.3 Excel 파일 오류

**Test:**
```bash
# 잘못된 경로
rosrun apriltag_navigation main_node.py --mode 4 --excel invalid.xlsx

# group_id 컬럼 없는 파일
rosrun apriltag_navigation main_node.py --mode 4 --excel wrong_format.xlsx
```

**예상 동작:**
- [ ] "Failed to read Excel" 오류 로그
- [ ] [508] (Dock만) 반환
- [ ] 프로그램 종료

### 7.4 Odometry Timeout

**Test:**
1. ROS 실행
2. `/odom` 토픽 발행 중단
3. 프로그램 실행

**예상 동작:**
- [ ] "Timeout waiting for sensors" 오류
- [ ] 10초 후 종료
- [ ] `wait_until_ready()` → `False` 반환

### 7.5 회전 Timeout

**Test:**
1. IMU 고장 시뮬레이션 (Odometry의 heading 고정)
2. `rotate_90()` 실행

**예상 동작:**
- [ ] 10초 후 강제 정지
- [ ] "Rotation timeout" 로그
- [ ] 다음 웨이포인트로 진행

---

## 테스트 완료 체크리스트

### 전체 시스템

- [ ] **Level 0**: ROS 인프라 (5개 항목)
- [ ] **Level 1**: Vision 모듈 (3개 테스트)
- [ ] **Level 2**: Map Manager (3개 테스트)
- [ ] **Level 3**: Robot Controller (3개 테스트)
- [ ] **Level 4**: Rotation Controller (2개 테스트)
- [ ] **Level 5**: 통합 동작 (3개 함수)
- [ ] **Level 6**: 전체 미션 (4개 모드)
- [ ] **Level 7**: 예외 처리 (5개 시나리오)

### 핵심 성능 지표

| 지표 | 목표 | 실측 | Pass/Fail |
|------|------|------|----------|
| Task 1 성공률 | 100% (45/45) | _____ % | ☐ |
| Dock 복귀 정확도 | < 5cm lateral | _____ cm | ☐ |
| 회전 정확도 | < 5° 오차 | _____ ° | ☐ |

**특이사항:**
```
(테스트 중 발견된 이슈나 개선사항을 기록)



```

---

## 참고: 주요 ROS 명령어

```bash
# 토픽 모니터링
rostopic hz /rgb          # Camera 프레임 레이트
rostopic echo /odom       # Odometry 출력
rostopic echo /cmd_vel    # 속도 명령 확인
rostopic echo /nav_debug_status  # 디버그 상태

# 로그 확인
rosrun apriltag_navigation main_node.py | grep ALIGN  # 정렬 로그만
rosrun apriltag_navigation main_node.py | grep WALL   # 벽 경고만

# 노드 정보
rosnode list              # 실행 중인 노드
rosnode info /apriltag_navigation  # 노드 상세 정보

# 파라미터 확인
rosparam get /apriltag_navigation  # 모든 파라미터
```
