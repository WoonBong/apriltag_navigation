# System Design Report
## AprilTag Navigation System

**작성일:** 2026-01-15
**시스템:** AprilTag 기반 자율 네비게이션
**개발 환경:** Ubuntu 20.04 LTS, ROS Noetic

---

## 목차
1. [서론](#1-서론)
2. [소프트웨어 아키텍처](#2-소프트웨어-아키텍처)
3. [상세 기능 분해](#3-상세-기능-분해)

---

## 1. 서론

### 1.1 계층적 제어 구조 채택 배경

본 시스템은 **계층적 제어 구조(Hierarchical Control Architecture)**를 채택하여, 복잡한 로봇 네비게이션 작업을 명확한 책임 경계를 가진 모듈로 분리하였다. 이는 다음과 같은 설계 목표를 달성하기 위함이다:

#### 1.1.1 Modularity (모듈화)
각 서브시스템(Vision, Navigation, Control)은 독립적으로 테스트 가능하며, 인터페이스가 명확히 정의되어 있다. 예를 들어, Vision Module은 AprilTag 검출 결과를 3차원 좌표로 반환하는 역할만 수행하며, 이후 경로 계획이나 모터 제어에 관여하지 않는다.

#### 1.1.2 Testability (테스트 가능성)
최소 단위 함수(Atomic Function)부터 전체 미션까지 단계별 검증이 가능하다. Level 0(ROS 인프라) → Level 1(개별 모듈) → Level 2(통합 동작) → Level 3(전체 미션)의 4단계 테스트 전략을 통해 시스템 신뢰성을 보장한다.

#### 1.1.3 Deterministic Execution (결정론적 실행)
State Machine 기반 제어로, 모든 하위 동작은 예측 가능한 순서로 실행된다. 동일한 입력(태그 위치, Odometry)에 대해 항상 동일한 출력(속도 명령)이 보장되며, 이는 현장 디버깅 및 재현성 확보에 필수적이다.

### 1.2 시스템 설계 목표

1. **Closed-loop Control (폐루프 제어):** 센서 피드백(Vision, Odometry)을 통해 오차를 지속적으로 보정
2. **Real-time Performance:** 30Hz 제어 루프로 실시간 응답성 확보
3. **Robustness:** 태그 미검출, 경로 오류 등 예외 상황에서도 안전한 동작
4. **Scalability:** 새로운 Zone, Task 추가 시 코드 수정 최소화 (데이터 기반 설계)

---

## 2. 소프트웨어 아키텍처

### 2.1 패키지 구조

```
apriltag_navigation/
│
├── nodes/
│   └── main_node.py                  # 프로그램 진입점
│
├── src/apriltag_navigation/
│   ├── robot_interface.py            # 최상위 API
│   │
│   ├── map/
│   │   └── map_manager.py            # 맵 데이터 관리
│   │       ├── TagDatabase           # 태그 정보 저장
│   │       ├── NavigationGraph       # 경로 탐색 (BFS)
│   │       └── TaskManager           # 태스크 웨이포인트 생성
│   │
│   ├── perception/
│   │   └── vision_module.py          # AprilTag 검출
│   │       ├── Detector (dt-apriltags)
│   │       ├── Camera calibration
│   │       └── Pose estimation (solvePnP)
│   │
│   ├── navigation/
│   │   └── pure_pursuit.py           # 경로 추종
│   │       ├── PurePursuitController # Pure Pursuit 알고리즘
│   │       └── CoordinateTransformer # 좌표 변환 (Mode 4)
│   │
│   └── hardware/
│       └── robot_controller.py       # 하드웨어 인터페이스
│           ├── RobotController       # Odometry, 속도 명령
│           └── RotationController    # 90도 회전 제어
│
└── config/
    └── map.yaml                      # 맵 데이터 (73개 태그, 엣지, 태스크)
```

### 2.2 모듈별 I/O 정의

#### 2.2.1 VisionModule

| I/O | ROS Topic / Data | 설명 |
|-----|-----------------|------|
| **Input** | `/rgb` (sensor_msgs/Image) | 카메라 이미지 스트림 |
| **Input** | `/camera_info` (sensor_msgs/CameraInfo) | 카메라 캘리브레이션 (K matrix) |
| **Output** | `get_tag_data()` → `{x, y, z, center_x, center_y}` | 태그 3D 위치 + 이미지 좌표 |
| **Output** | `get_alignment_angle()` → `float` (degrees) | 태그 기울기 (코너 기반) |

**핵심 알고리즘:** AprilTag Detector (Quad Decimate, Edge Refinement) + solvePnP (PnP 알고리즘으로 3D 자세 추정)

#### 2.2.2 PurePursuitController

| I/O | Input | Output | 설명 |
|-----|-------|--------|------|
| **Input** | `lateral` (float), `distance` (float) | - | Cross-track error + Look-ahead distance |
| **Output** | - | `angular_velocity` (float) | 각속도 명령 (rad/s) |

**핵심 알고리즘:** Pure Pursuit (Curvature = 2sin(α)/L, α = atan2(-lateral, L))

#### 2.2.3 RobotController

| I/O | ROS Topic / Data | 설명 |
|-----|-----------------|------|
| **Input** | `/odom` (nav_msgs/Odometry) | Odometry (위치 + 방향) |
| **Output** | `/cmd_vel` (geometry_msgs/Twist) | 속도 명령 (linear, angular) |

**핵심 기능:** Odometry 기반 위치 추정 + 참조점 설정 (Drift 보정)

#### 2.2.4 MapManager

| I/O | Input | Output | 설명 |
|-----|-------|--------|------|
| **Input** | `map.yaml` (YAML 파일) | - | 73개 태그, 엣지 관계, 태스크 정의 |
| **Output** | `find_path(start, goal)` | `[tag_id_list]` | BFS 알고리즘으로 최단 경로 반환 |

**핵심 알고리즘:** BFS (Breadth-First Search) 그래프 탐색

### 2.3 데이터 흐름도

```
┌─────────────┐
│ Camera      │
│ /rgb        │
└──────┬──────┘
       │ Image
       ▼
┌─────────────────────┐
│ VisionModule        │
│ - AprilTag Detector │
│ - solvePnP          │
└──────┬──────────────┘
       │ {x, y, z, angle, lateral}
       ▼
┌────────────────────────────┐         ┌──────────────┐
│ PurePursuitController      │◄────────┤ MapManager   │
│ - calculate_forward()      │         │ - tag_db     │
│ - calculate_wall_distance()│         │ - nav_graph  │
└──────┬─────────────────────┘         └──────────────┘
       │ angular_velocity
       ▼
┌─────────────────────┐         ┌──────────────┐
│ RobotController     │◄────────┤ /odom        │
│ - move(linear, ang) │         │ (Odometry)   │
└──────┬──────────────┘         └──────────────┘
       │ Twist
       ▼
┌─────────────────────┐
│ /cmd_vel            │
│ (Motor Command)     │
└─────────────────────┘
```

---

## 3. 상세 기능 분해

본 시스템의 핵심 동작은 **State Machine 기반**으로 구성되며, `IDLE → MOVING → STOPPING → ALIGNING` 순서로 상태가 전이된다. 각 상태에서 실행되는 3가지 주요 동작(`move_to_tag`, `rotate_90`, `align_to_tag`)을 **[Perception → State Estimation → Control → Actuation]** 4단계 흐름으로 분해하여 서술한다.

---

### 3.1 move_to_tag() - 태그로 이동

**목적:** 목표 태그까지 Pure Pursuit 알고리즘을 사용하여 경로 추종 이동

**사용 알고리즘:** Pure Pursuit (Geometric Path Tracking), Closed-loop Feedback Control

#### Step 1: Perception (인지)

`VisionModule`이 카메라 프레임에서 AprilTag의 **코너(Corner) 특징점**을 추출하고, `solvePnP` 알고리즘을 통해 태그의 3차원 상대 좌표 $(x, y, z)$를 획득한다.

**세부 과정:**
1. **이미지 수신:** `/rgb` 토픽에서 640×720 해상도 이미지 획득
2. **태그 검출:** AprilTag Detector가 Quad Decimate(1.0) 및 Edge Refinement(1) 적용하여 코너 검출
3. **3D Pose 추정:** Camera Intrinsic Matrix $K$와 4개 코너 좌표를 사용하여 solvePnP 실행
   - 출력: $(x, y, z)$ - 카메라 좌표계에서의 태그 위치
   - $x$: Lateral offset (+ = 우측)
   - $y$: Vertical offset
   - $z$: Distance to tag

**결과:** `tag_data = {'x': lateral, 'y': vertical, 'z': distance, 'center_x': cx, 'center_y': cy}`

#### Step 2: State Estimation (상태 추정)

획득된 센서 데이터를 바탕으로 로봇과 태그 간의 **상대 관계**를 추정한다.

**세부 과정:**
1. **도착 판정 조건 확인:**
   - `center_y` (이미지 상의 Y좌표)가 화면 중앙 $\pm$ 50px 이내인지 확인
   - 중앙 근처 = 태그가 로봇 진행 방향 정면에 위치 = 도착
2. **Cross-Track Error 계산:**
   - `lateral` 값이 경로로부터의 횡방향 오차(Cross-Track Error)
   - `lateral > 0`: 태그가 오른쪽 → 우회전 필요
   - `lateral < 0`: 태그가 왼쪽 → 좌회전 필요

**결과:** `arrival_status` (True/False), `lateral_error` (미터)

#### Step 3: Control (제어 연산)

Pure Pursuit 제어기에 입력하여, 목표 지점까지의 **곡률(Curvature, $\kappa$)**과 **조향 각속도**를 계산한다.

**수식:**
$$
\alpha = \arctan2(-\text{lateral}, L_{\text{eff}})
$$
$$
\kappa = \frac{2 \sin(\alpha)}{L_{\text{eff}}}
$$
$$
\omega = v \cdot \kappa \cdot k_{\text{gain}}
$$

**파라미터:**
- $L_{\text{eff}}$: Effective look-ahead distance = max(distance, 0.4m)
- $\alpha$: Angle to target
- $v$: Linear speed (0.3 m/s for forward, -0.3 m/s for backward)
- $k_{\text{gain}}$: Proportional gain (1.0 for forward, 0.8 for backward)

**Deadband 적용:**
- `|lateral| < 0.005m` 이면 $\omega = 0$ (직진)

**출력 제한:**
- $\omega \in [-0.3, +0.3]$ rad/s (Clipping)

**결과:** `angular_velocity` (rad/s)

#### Step 4: Actuation (구동)

계산된 선속도($v$)와 각속도($\omega$)를 `RobotController`를 통해 하드웨어 드라이버(`/cmd_vel`)로 전송하여 모터를 구동한다.

**세부 과정:**
1. **속도 명령 생성:**
   ```python
   twist = Twist()
   twist.linear.x = 0.3      # Forward: +0.3, Backward: -0.3
   twist.angular.z = angular  # Calculated by Pure Pursuit
   ```
2. **ROS Topic 발행:**
   - `/cmd_vel` 토픽으로 Twist 메시지 발행
3. **하드웨어 실행:**
   - 로봇 베이스 컨트롤러가 좌/우 바퀴 속도 계산
   - Differential Drive 모델로 모터 PWM 신호 생성

**결과:** 로봇이 목표 태그 방향으로 곡선 경로를 따라 이동

---

### 3.2 rotate_90() - 90도 회전

**목적:** 피봇 포인트에서 로봇을 정확히 90도 회전 (CW 또는 CCW)

**사용 알고리즘:** PID Control (Proportional Control), Odometry-based Feedback

#### Step 1: Perception (인지)

`RobotController`가 `/odom` 토픽으로부터 로봇의 현재 **Heading(방향)**을 획득한다.

**세부 과정:**
1. **Odometry 수신:** `/odom` 토픽에서 `pose.pose.orientation` (Quaternion) 추출
2. **Quaternion → Euler 변환:**
   ```python
   from tf.transformations import euler_from_quaternion
   roll, pitch, yaw = euler_from_quaternion([x, y, z, w])
   current_heading = yaw  # -π ~ π
   ```

**결과:** `current_heading` (radians)

#### Step 2: State Estimation (상태 추정)

회전 목표 각도를 설정하고, 현재 각도와의 오차를 계산한다.

**세부 과정:**
1. **목표 각도 계산:**
   - CW 회전: `target_heading = current_heading - π/2`
   - CCW 회전: `target_heading = current_heading + π/2`
2. **각도 정규화:**
   - `target_heading`을 $[-\pi, \pi]$ 범위로 정규화
   ```python
   while target_heading > π:
       target_heading -= 2π
   while target_heading < -π:
       target_heading += 2π
   ```
3. **오차 계산:**
   - `error = target_heading - current_heading`
   - 정규화 적용

**결과:** `rotation_error` (radians)

#### Step 3: Control (제어 연산)

PID 제어기(현재는 P 제어만 사용)를 통해 각속도를 계산한다.

**수식:**
$$
\omega = k_p \cdot \text{error}
$$

**파라미터:**
- $k_p = 0.5$ (근접 시 감속)
- Minimum velocity: 0.08 rad/s (마찰 극복)
- Maximum velocity: ±0.3 rad/s

**세부 로직:**
```python
if |error| < 0.05 rad (≈3°):
    angular = error * 0.5  # 감속
else:
    angular = error * 1.0

# 최소 속도 보장
if |angular| < 0.08:
    angular = 0.08 * sign(angular)

# 최대 속도 제한
angular = clip(angular, -0.3, +0.3)
```

**완료 판정:**
- `|error| < 1°` → 회전 완료

**결과:** `angular_velocity` (rad/s)

#### Step 4: Actuation (구동)

계산된 각속도를 `/cmd_vel`로 발행하여 제자리 회전을 실행한다.

**세부 과정:**
1. **속도 명령:**
   ```python
   twist = Twist()
   twist.linear.x = 0.0      # 제자리 회전 (이동 없음)
   twist.angular.z = angular
   ```
2. **Closed-loop Feedback:**
   - 30Hz 루프로 Odometry 지속 수신
   - 오차 재계산 → 각속도 업데이트
   - 완료 조건 만족 시 정지

**결과:** 로봇이 90도 회전 완료

---

### 3.3 align_to_tag() - 태그 정렬

**목적:** 태그에 대한 로봇의 각도 오차를 0.5° 이내, Lateral offset을 5cm 이내로 정밀 정렬

**사용 알고리즘:** Proportional Control, Coordinate Transformation (Zone-based Pose Estimation)

#### Step 1: Perception (인지)

`VisionModule`이 태그의 **4개 코너 좌표**를 기반으로 태그의 기울기(Alignment Angle)를 계산한다.

**세부 과정:**
1. **코너 추출:**
   - AprilTag Detection 결과에서 `corners = [top_left, top_right, bottom_right, bottom_left]` 획득
2. **상단 엣지 각도 계산:**
   ```python
   top_left = corners[0]   # (x0, y0)
   top_right = corners[1]  # (x1, y1)
   dx = x1 - x0
   dy = y1 - y0
   align_angle = atan2(dy, dx)  # radians
   align_angle_deg = degrees(align_angle)
   ```
3. **각도 정규화 (Zone C/E는 180° 회전):**
   ```python
   if align_angle_deg > 90:
       align_angle_deg -= 180
   elif align_angle_deg < -90:
       align_angle_deg += 180
   ```

**결과:** `align_angle_deg` (도 단위, -90° ~ +90°)

#### Step 2: State Estimation (상태 추정)

정렬 완료 여부를 판정하고, 필요 시 로봇의 절대 위치를 역산한다.

**세부 과정:**
1. **정렬 판정:**
   - `|align_angle_deg| < 0.5°` AND `|lateral| < 0.05m` → 정렬 완료
2. **좌표 역산 (정렬 완료 시):**
   - Tag 정보 조회: `tag_info = tag_db.get(tag_id)` → `(tag_x, tag_y, zone)`
   - Zone별 좌표 변환:
     - **Zone A / DOCK:** Robot heading = 0° (+X 방향)
       ```python
       robot_x = tag_x
       robot_y = tag_y + lateral
       robot_heading = radians(align_angle_deg)
       ```
     - **Zone B / D:** Robot heading = 90° (+Y 방향)
       ```python
       robot_x = tag_x - lateral
       robot_y = tag_y
       robot_heading = π/2 + radians(align_angle_deg)
       ```
     - **Zone C / E:** Robot heading = -90° (-Y 방향)
       ```python
       robot_x = tag_x + lateral
       robot_y = tag_y
       robot_heading = -π/2 + radians(align_angle_deg)
       ```

**결과:** `alignment_status` (True/False), `robot_pose` (x, y, heading)

#### Step 3: Control (제어 연산)

비례 제어(Proportional Control)로 정렬 각속도를 계산한다.

**수식:**
$$
\omega = -\alpha \cdot k_{\text{align}}
$$

**파라미터:**
- $\alpha$: `align_angle_deg` (태그 기울기)
- $k_{\text{align}} = 0.8$ (비례 이득)

**제어 방향:**
- `align_angle_deg > 0` (태그가 CCW로 기울어짐) → $\omega < 0$ (CW 회전 필요)
- `align_angle_deg < 0` (태그가 CW로 기울어짐) → $\omega > 0$ (CCW 회전 필요)

**최소/최대 속도:**
```python
# 최소 속도 보장 (마찰 극복)
if |angular| < 0.08 and |align_angle_deg| > 0.3:
    angular = 0.08 * sign(angular)

# 최대 속도 제한
angular = clip(angular, -0.2, +0.2)
```

**결과:** `angular_velocity` (rad/s)

#### Step 4: Actuation (구동)

각속도를 `/cmd_vel`로 발행하고, 정렬 완료 시 **참조점(Reference Point)을 업데이트**하여 Odometry Drift를 보정한다.

**세부 과정:**
1. **회전 명령 (정렬 중):**
   ```python
   twist = Twist()
   twist.linear.x = 0.0
   twist.angular.z = angular
   cmd_pub.publish(twist)
   ```
2. **참조점 업데이트 (정렬 완료 시):**
   - Vision 기반으로 역산한 `robot_pose`를 참조점으로 설정
   - 현재 Odometry 값을 기준으로 저장
   ```python
   ref_robot_x = robot_x
   ref_robot_y = robot_y
   ref_robot_heading = robot_heading
   ref_odom_x = current_odom_x
   ref_odom_y = current_odom_y
   ref_odom_theta = current_theta
   ```
   - 이후 위치 추정 시 Odometry Delta를 참조점에 더하여 Drift 보정

**결과:** 태그 정렬 완료 + Odometry 참조점 갱신

---

## 4. 모드별 경로 생성 전략

### 4.1 Mode 1/2: 사전 정의 태스크

**알고리즘:** TaskManager가 `map.yaml`의 `tasks` 섹션에서 waypoints 리스트를 직접 로드

**특징:**
- Task 1: 45개 waypoints (Zone B → Zone C 순회)
- Task 2: 75개 waypoints (전체 Zone 순회)

### 4.2 Mode 3: 직접 네비게이션

**알고리즘:** BFS (Breadth-First Search) 기반 최단 경로 탐색

**동작:**
1. `NavigationGraph.find_path(start, goal)` 호출
2. Queue 기반 BFS 탐색으로 최단 경로 생성
3. 경로 없으면 `None` 반환

### 4.3 Mode 4: Excel 기반 스캔

**알고리즘:** Pandas로 Excel 파싱 → group_id 추출 → 경로 연결

**동작:**
1. `group_id` 컬럼 읽기 → 중복 제거 (연속된 같은 값 하나로 병합)
2. Tag ID 변환: `tag_id = 100 + group_id`
3. 각 scan tag 사이 경로를 BFS로 연결
4. 508(Dock) 시작 → scan tags 순회 → 508 복귀

---

## 5. 요약

### 5.1 핵심 설계 특징

1. **계층적 구조:** High-level API (`RobotInterface`) → Mid-level Controllers → Low-level Hardware
2. **Closed-loop Feedback:** Vision + Odometry 센서 기반 실시간 오차 보정
3. **Deterministic State Machine:** IDLE → MOVING → ALIGNING 순서로 예측 가능한 동작
4. **Data-driven Architecture:** `map.yaml`로 모든 맵 구조 정의, 코드 수정 없이 확장 가능

### 5.2 주요 알고리즘

| 기능 | 알고리즘 | 설명 |
|------|---------|------|
| **태그 검출** | AprilTag Detector + solvePnP | 3D Pose Estimation |
| **경로 추종** | Pure Pursuit | Geometric path tracking, Curvature-based control |
| **회전 제어** | PID Control (P-only) | Odometry feedback, ±1° 정확도 |
| **정렬 제어** | Proportional Control | Corner-based angle estimation |
| **경로 탐색** | BFS | Shortest path in navigation graph |

### 5.3 성능 지표

| 지표 | 목표 | 실측 방법 |
|------|------|----------|
| **Dock 복귀 정확도** | < 5cm lateral offset | Field Test Protocol 참조 |
| **회전 정확도** | < 5° 오차 | Odometry 비교 |
| **Vision 검출 거리** | > 1.5m | 단계별 거리 테스트 |
| **제어 주기** | 30 Hz | rospy.Rate(30) |

---

**문서 종료**

본 시스템 설계 보고서는 AprilTag Navigation의 **계층적 아키텍처**, **모듈별 I/O**, **핵심 알고리즘의 4단계 분해**를 상세히 기술하였다. 현장 테스트 및 API 사용법은 별도 문서([Field_Test_Protocol.md](Field_Test_Protocol.md), [API_Manual.md](API_Manual.md))를 참조하시오.
