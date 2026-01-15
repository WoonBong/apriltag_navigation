# System Design Report
## AprilTag Navigation System

**작성일:** 2026-01-15
**시스템:** AprilTag 기반 자율 네비게이션
**개발 환경:** Ubuntu 20.04 LTS, ROS Noetic

---

## 목차
1. [소프트웨어 아키텍처](#1-소프트웨어-아키텍처)
2. [상세 기능 분해](#2-상세-기능-분해)

---

## 1. 소프트웨어 아키텍처

### 1.1 패키지 구조

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
│   │
│   ├── navigation/
│   │   └── pure_pursuit.py           # 경로 추종
│   │       ├── PurePursuitController
│   │       └── CoordinateTransformer
│   │
│   └── hardware/
│       └── robot_controller.py       # 하드웨어 인터페이스
│           ├── RobotController
│           └── RotationController
│
└── config/
    └── map.yaml                      # 맵 데이터 (73개 태그, 엣지, 태스크)
```

### 1.2 모듈별 I/O 정의

#### 1.2.1 RobotInterface (최상위 통합 모듈)

**하위 구성요소:**
- VisionModule
- RobotController
- RotationController
- PurePursuitController
- MapManager

**주요 함수:**

| 함수 | 입력 | 출력 | 설명 |
|------|------|------|------|
| `__init__()` | `config_path` (str, optional) | - | 모든 서브시스템 초기화 |
| `wait_until_ready()` | - | `bool` | 카메라, Odometry, Dock 태그 검출 대기 |
| `move_to_tag()` | `target_tag` (int), `edge` (dict) | `bool` | Pure Pursuit 기반 태그로 이동 |
| `rotate_90()` | `direction` (str: 'cw'/'ccw') | `bool` | 90도 회전 |
| `align_to_tag()` | `tag_id` (int) | `bool` | 태그 정렬 (각도 < 0.5°) |
| `stop()` | - | - | 로봇 정지 |
| `publish_debug_status()` | `current_wp`, `next_wp`, `total_wps`, `wp_idx` | - | 디버그 상태 발행 |

---

#### 1.2.2 VisionModule (AprilTag 검출)

**ROS I/O:**

| I/O | Topic/Type | 설명 |
|-----|-----------|------|
| **Subscribe** | `/rgb` (sensor_msgs/Image) | 카메라 이미지 (1280×720) |
| **Subscribe** | `/camera_info` (sensor_msgs/CameraInfo) | 카메라 캘리브레이션 |

**내부 함수:**

| 함수 | 입력 | 출력 | 하위 동작 |
|------|------|------|---------|
| `_camera_info_callback()` | `msg` (CameraInfo) | - | K matrix 추출 → `camera_params = [fx, fy, cx, cy]` 저장 |
| `_image_callback()` | `msg` (Image) | - | 1. `cv_bridge`로 이미지 변환<br>2. Grayscale 변환<br>3. `detector.detect()` 호출<br>4. 각 태그에 대해 `solvePnP()` 실행<br>5. `detected_tags` 딕셔너리 업데이트 |
| `is_ready()` | - | `bool` | `camera_info_received` 플래그 확인 |
| `is_tag_visible()` | `tag_id` (int) | `bool` | `tag_id in detected_tags` 확인 |
| `get_tag_data()` | `tag_id` (int) | `dict` or `None` | `{'x': lateral, 'y': vertical, 'z': distance, 'center_x': cx, 'center_y': cy, 'corners': [...]}` 반환 |
| `get_alignment_angle()` | `tag_id` (int) | `float` or `None` | 1. 코너 추출<br>2. Top-left, Top-right로 각도 계산: `atan2(dy, dx)`<br>3. 정규화 (-90° ~ +90°) |
| `get_tag_lateral()` | `tag_id` (int) | `float` or `None` | `tag_data['x']` 반환 (Lateral offset) |
| `get_detected_tags()` | - | `list` | 현재 검출된 태그 ID 리스트 |

**핵심 알고리즘:**
- **AprilTag Detector**: Quad Decimate(1.0), Edge Refinement(1)
- **solvePnP**: Camera Intrinsic Matrix + 4 corners → 3D Pose (x, y, z)

---

#### 1.2.3 MapManager (맵 데이터 관리)

**입력:**
- `map.yaml` (YAML 파일)

**하위 구성요소:**

##### TagDatabase

| 함수 | 입력 | 출력 | 설명 |
|------|------|------|------|
| `_load_tags()` | `tags_dict` (dict) | - | YAML에서 73개 태그 파싱 → `tags` 딕셔너리 저장 |
| `get()` | `tag_id` (int) | `dict` or `None` | `{'x': float, 'y': float, 'type': TagType, 'zone': str}` |
| `exists()` | `tag_id` (int) | `bool` | 태그 존재 여부 |
| `get_zone()` | `tag_id` (int) | `str` | Zone 이름 ('A', 'B', 'C', 'D', 'E', 'DOCK') |
| `get_position()` | `tag_id` (int) | `(float, float)` or `(None, None)` | (x, y) 좌표 |

##### NavigationGraph

| 함수 | 입력 | 출력 | 설명 |
|------|------|------|------|
| `_load_edges()` | `edges_list` (list) | - | Edge 연결 관계 파싱 → `edges` 딕셔너리 저장 |
| `get_edge()` | `from_tag` (int), `to_tag` (int) | `dict` or `None` | `{'to': int, 'direction': str, 'type': str}` |
| `find_path()` | `start` (int), `goal` (int) | `list` or `None` | **BFS 알고리즘**으로 최단 경로 탐색<br>1. Queue 초기화: `[(start, [start])]`<br>2. BFS 루프: 인접 노드 탐색<br>3. 목표 도달 시 경로 반환 |

##### TaskManager

| 함수 | 입력 | 출력 | 설명 |
|------|------|------|------|
| `get_task_waypoints()` | `task_name` (str) | `list` | `tasks[task_name]['waypoints']` 반환 |
| `get_excel_scan_waypoints()` | `excel_path` (str) | `(list, list)` | 1. `pandas.read_excel()` → `group_id` 컬럼 추출<br>2. 중복 제거 (연속 같은 값 병합)<br>3. Tag ID 변환: `100 + group_id`<br>4. BFS로 경로 연결: `508 → tag1 → tag2 → ... → 508`<br>5. `(waypoints, scan_tags)` 반환 |

##### MapManager 공통 함수

| 함수 | 입력 | 출력 | 설명 |
|------|------|------|------|
| `get_param()` | `param_path` (str), `default` | any | Dot-separated 경로로 파라미터 조회 (예: `'speeds.linear'`) |
| `validate_waypoints()` | `waypoints` (list) | `(bool, str)` | 1. 태그 존재 확인<br>2. Edge 연결성 확인 |

---

#### 1.2.4 PurePursuitController (경로 추종)

**주요 함수:**

| 함수 | 입력 | 출력 | 하위 동작 |
|------|------|------|---------|
| `calculate_forward()` | `lateral` (float), `distance_to_tag` (float) | `float` (angular velocity) | 1. Look-ahead: `L_eff = max(distance, 0.4)`<br>2. Alpha: `atan2(-lateral, L_eff)`<br>3. Curvature: `κ = 2sin(α)/L_eff`<br>4. Angular: `ω = v × κ × gain` (gain=1.0)<br>5. Deadband: `\|lateral\| < 0.005m → ω=0`<br>6. Clipping: `ω ∈ [-0.3, +0.3]` |
| `calculate_backward()` | `lateral` (float), `distance_to_tag` (float) | `float` | Forward와 동일하나 gain=0.8 |
| `calculate_absolute()` | `robot_x`, `robot_y`, `robot_heading`, `target_x`, `target_y`, `current_lateral`, `move_dir` | `float` | 1. Delta 계산: `dx = target_x - robot_x`<br>2. Angle to target: `atan2(dy, dx)`<br>3. Alpha (heading error) 계산<br>4. Pure Pursuit 수식 적용<br>5. 근접 시 감속: `distance < 0.3m → ω × (distance/0.3)` |
| `calculate_wall_distance()` | `lateral` (float), `align_angle_deg` (float), `zone` (str) | `float` | 1. Base distance 선택 (Zone A: 0.63m, B/C/D/E: 0.35m)<br>2. Zone별 Lateral 부호 처리:<br>   - Zone A: `base - lateral`<br>   - Zone B/D: `base + lateral`<br>   - Zone C/E: `base + lateral`<br>3. 로봇 회전 보정: 각도, 길이/폭 고려<br>4. 우측면 거리 반환 |
| `get_robot_pose_from_tag()` | `tag_id`, `lateral`, `align_angle_deg` | `(float, float, float)` | 1. Tag 좌표 조회: `(tag_x, tag_y, zone)`<br>2. Zone별 좌표 역산:<br>   - Zone A/DOCK: `robot_y = tag_y + lateral`<br>   - Zone B/D: `robot_x = tag_x - lateral`<br>   - Zone C/E: `robot_x = tag_x + lateral`<br>3. `(robot_x, robot_y, robot_heading)` 반환 |

##### CoordinateTransformer (Mode 4 전용)

| 함수 | 입력 | 출력 | 설명 |
|------|------|------|------|
| `isaac_to_manipulator()` | `isaac_x`, `isaac_y` | `(pub_x, pub_y)` | `pub_x = -isaac_y`, `pub_y = -isaac_x` |
| `apply_robot_center_offset()` | `manip_x`, `manip_y`, `zone`, `camera_offset` | `(robot_x, robot_y)` | Zone별 카메라 offset(0.45m) 적용 |

---

#### 1.2.5 RobotController (하드웨어 인터페이스)

**ROS I/O:**

| I/O | Topic/Type | 설명 |
|-----|-----------|------|
| **Subscribe** | `/odom` (nav_msgs/Odometry) | Odometry (위치, 방향) |
| **Publish** | `/cmd_vel` (geometry_msgs/Twist) | 속도 명령 (linear, angular) |

**주요 함수:**

| 함수 | 입력 | 출력 | 하위 동작 |
|------|------|------|---------|
| `_odom_callback()` | `msg` (Odometry) | - | 1. Position 추출: `odom_x = msg.pose.pose.position.x`<br>2. Quaternion → Euler: `euler_from_quaternion()`<br>3. `current_theta = yaw` 저장 |
| `is_ready()` | - | `bool` | Odometry 수신 여부 확인 |
| `get_position()` | - | `(float, float)` | `(odom_x, odom_y)` 반환 |
| `get_heading()` | - | `float` | `current_theta` 반환 (-π ~ π) |
| `get_estimated_pose()` | - | `(float, float, float)` | 1. Odometry delta 계산:<br>   `Δx = odom_x - ref_odom_x`<br>2. 참조점 기준 위치:<br>   `est_x = ref_robot_x + Δx`<br>3. `(est_x, est_y, est_heading)` 반환 |
| `set_reference_point()` | `robot_x`, `robot_y`, `robot_heading` | - | 참조점 저장 (Vision 기반 위치 → Odometry drift 보정) |
| `move()` | `linear` (float), `angular` (float) | - | `Twist` 메시지 생성 → `/cmd_vel` 발행 |
| `stop()` | - | - | `move(0, 0)` 호출 |

**기본 이동 함수:**

| 함수 | 입력 | 설명 |
|------|------|------|
| `forward()` | `distance` (float) | 전진 이동 (Odometry 기반 거리 측정) |
| `backward()` | `distance` (float) | 후진 이동 |
| `rotate()` | `angle` (float) | 제자리 회전 |

---

#### 1.2.6 RotationController (90도 회전 제어)

**주요 함수:**

| 함수 | 입력 | 출력 | 하위 동작 |
|------|------|------|---------|
| `start_rotation()` | `target_angle_delta` (float) | - | 1. 현재 heading 조회: `robot.get_heading()`<br>2. 목표 계산: `target_theta = current + delta`<br>3. 정규화: [-π, π] 범위<br>4. `rotation_active = True` 설정 |
| `get_rotation_error()` | - | `float` | 1. 현재 heading 조회<br>2. 오차 계산: `target - current`<br>3. 정규화 반환 |
| `is_complete()` | - | `bool` | `\|error\| < 1°` 확인 |
| `update()` | - | `bool` | 1. 오차 계산: `get_rotation_error()`<br>2. P 제어:<br>   - `\|error\| < 3°`: `angular = error × 0.5`<br>   - 그 외: `angular = error × 1.0`<br>3. 최소 속도 보장: `\|angular\| < 0.08 → 0.08`<br>4. 최대 속도 제한: `clip(-0.3, +0.3)`<br>5. `robot.move(0, angular)` 호출<br>6. 완료 시 `True` 반환 |

---

## 2. 상세 기능 분해

프로젝트 실행 후 진행되는 순서대로 3가지 핵심 동작을 **[Perception → State Estimation → Control → Actuation]** 4단계로 분해한다.

---

### 2.1 시스템 초기화 (System Initialization)

**실행 순서:** `main()` → `RobotInterface.__init__()` → `wait_until_ready()`

#### Step 1: ROS 노드 초기화
```python
rospy.init_node('apriltag_navigation', anonymous=True)
```

#### Step 2: 서브시스템 초기화

1. **MapManager 생성**
   - `map.yaml` 파싱
   - TagDatabase 로드 (73개 태그)
   - NavigationGraph 로드 (Edge 관계)
   - TaskManager 초기화

2. **VisionModule 생성**
   - AprilTag Detector 초기화 (families='tag36h11', nthreads=4)
   - `/rgb`, `/camera_info` Subscribe
   - Camera calibration 대기

3. **RobotController 생성**
   - `/odom` Subscribe
   - `/cmd_vel` Publisher 생성
   - Odometry 대기

4. **PurePursuitController 생성**
   - 파라미터 로드 (look_ahead, gain, deadband 등)

5. **RotationController 생성**
   - RobotController 참조

#### Step 3: 준비 대기 (wait_until_ready)

**조건:**
1. `vision.is_ready()` → `/camera_info` 수신 완료
2. `robot.is_ready()` → `/odom` 수신 완료
3. `vision.is_tag_visible(508)` → Dock 태그 검출

**모두 만족 시:**
- Dock 시작 위치 기록 (`record_dock_start()`)
- 초기화 완료

---

### 2.2 경로 생성 (Path Planning)

**Mode별 알고리즘:**

#### Mode 1/2: 사전 정의 태스크
```python
waypoints = task_manager.get_task_waypoints('task1')  # YAML에서 직접 로드
```

#### Mode 3: 직접 네비게이션
```python
waypoints = nav_graph.find_path(508, target_tag)  # BFS 알고리즘
```
**BFS 동작:**
1. Queue 초기화: `[(start, [start])]`
2. 현재 노드의 인접 노드 탐색
3. 미방문 노드를 Queue에 추가
4. 목표 도달 시 경로 반환

#### Mode 4: Excel 스캔
```python
waypoints, scan_tags = task_manager.get_excel_scan_waypoints(excel_path)
```
**동작:**
1. Pandas로 `group_id` 컬럼 추출
2. 중복 제거: `[4,4,4,5,5,6] → [4,5,6]`
3. Tag ID 변환: `tag_id = 100 + group_id`
4. BFS로 경로 연결: `508 → 104 → 105 → 106 → 508`

---

### 2.3 move_to_tag() - 태그로 이동

**목적:** 목표 태그까지 Pure Pursuit 알고리즘으로 경로 추종

**State Machine:** `IDLE → MOVING → STOPPING → ALIGNING`

#### Step 1: Perception (인지)

**VisionModule 동작:**

1. **이미지 수신 (`_image_callback`)**
   - `/rgb` 토픽: 1280×720 해상도 이미지
   - `cv_bridge`로 OpenCV 이미지 변환
   - Grayscale 변환

2. **태그 검출 (`detector.detect`)**
   - AprilTag Detector 실행
   - Quad Decimate(1.0), Edge Refinement(1) 적용
   - 4개 코너 좌표 추출

3. **3D Pose 추정 (`solvePnP`)**
   - Camera Intrinsic Matrix $K = [f_x, f_y, c_x, c_y]$
   - 4개 코너 좌표 입력
   - **출력:** $(x, y, z)$ - 카메라 좌표계 기준
     - $x$: Lateral offset (+ = 우측)
     - $y$: Vertical offset
     - $z$: Distance

4. **이미지 좌표 계산**
   - `center_x`, `center_y`: 태그 중심의 픽셀 좌표

**결과:** `tag_data = {'x': lateral, 'y': vertical, 'z': distance, 'center_x': cx, 'center_y': cy}`

#### Step 2: State Estimation (상태 추정)

**도착 판정:**
- 조건: `center_y`가 화면 중앙 ± 50px 이내
- 의미: 태그가 로봇 정면에 위치 → 도착

**Cross-Track Error 계산:**
- `lateral > 0`: 태그가 오른쪽 → 우회전 필요
- `lateral < 0`: 태그가 왼쪽 → 좌회전 필요

**결과:** `arrival_status` (True/False), `lateral_error` (미터)

#### Step 3: Control (제어 연산)

**Pure Pursuit 알고리즘**


**파라미터:**
- Forward: $v = +0.3$ m/s, $k_{\text{gain}} = 1.0$
- Backward: $v = -0.3$ m/s, $k_{\text{gain}} = 0.8$

**Deadband:**
- `|lateral| < 0.005m` → $\omega = 0$ (직진)

**Clipping:**
- $\omega \in [-0.3, +0.3]$ rad/s

**결과:** `angular_velocity` (rad/s)

#### Step 4: Actuation (구동)

**RobotController.move() 호출:**

1. **Twist 메시지 생성**
   ```python
   twist = Twist()
   twist.linear.x = 0.3      # Forward: +0.3, Backward: -0.3
   twist.angular.z = angular  # Pure Pursuit 계산값
   ```

2. **ROS Topic 발행**
   - `/cmd_vel` 토픽으로 발행
   - 로봇 베이스 컨트롤러가 수신

3. **하드웨어 실행**
   - Differential Drive 모델로 좌/우 바퀴 속도 계산
   - 모터 PWM 신호 생성
   - 로봇 이동

**결과:** 로봇이 목표 태그 방향으로 곡선 경로 추종

---

### 2.4 rotate_90() - 90도 회전

**목적:** 피봇 포인트에서 정확히 90도 회전 (CW 또는 CCW)

**State Machine:** `IDLE → ROTATING`

#### Step 1: Perception (인지)

**RobotController._odom_callback 동작:**

1. **Odometry 수신**
   - `/odom` 토픽에서 메시지 수신
   - `pose.pose.orientation`: Quaternion $(x, y, z, w)$

2. **Quaternion → Euler 변환**
   ```python
   from tf.transformations import euler_from_quaternion
   roll, pitch, yaw = euler_from_quaternion([x, y, z, w])
   current_heading = yaw  # -π ~ π
   ```

**결과:** `current_heading` (radians)

#### Step 2: State Estimation (상태 추정)

**RotationController.start_rotation() 동작:**

1. **목표 각도 계산**
   - CW: `target_heading = current_heading - π/2`
   - CCW: `target_heading = current_heading + π/2`

2. **각도 정규화**
   ```python
   while target_heading > π:
       target_heading -= 2π
   while target_heading < -π:
       target_heading += 2π
   ```

3. **오차 계산 (`get_rotation_error`)**
   - `error = target_heading - current_heading`
   - 정규화 적용 ([-π, π])

**결과:** `rotation_error` (radians)

#### Step 3: Control (제어 연산)

**RotationController.update() - P 제어:**

**수식:**
$$
\omega = k_p \cdot \text{error}
$$

**파라미터:**
- 근접 시 ($|\text{error}| < 3°$): $k_p = 0.5$
- 그 외: $k_p = 1.0$

**최소 속도 보장 (마찰 극복):**
```python
if |angular| < 0.08 and |error| > 3°:
    angular = 0.08 × sign(angular)
```

**최대 속도 제한:**
```python
angular = clip(angular, -0.3, +0.3)
```

**완료 판정:**
- `|error| < 1°` → 회전 완료

**결과:** `angular_velocity` (rad/s)

#### Step 4: Actuation (구동)

**RobotController.move() 호출:**

1. **제자리 회전 명령**
   ```python
   twist = Twist()
   twist.linear.x = 0.0      # 이동 없음
   twist.angular.z = angular
   ```

2. **Closed-loop Feedback**
   - 30Hz 루프로 지속 실행
   - Odometry 재수신 → 오차 재계산 → 각속도 업데이트
   - 완료 조건 만족 시 정지

**결과:** 로봇 90도 회전 완료 (오차 < 5°)

---

### 2.5 align_to_tag() - 태그 정렬

**목적:** 태그에 대한 각도 오차 < 0.5°, Lateral offset < 5cm

**State Machine:** `MOVING → ALIGNING`

#### Step 1: Perception (인지)

**VisionModule.get_alignment_angle() 동작:**

1. **코너 추출**
   - AprilTag Detection 결과에서 4개 코너 획득
   - `corners = [top_left, top_right, bottom_right, bottom_left]`

2. **상단 엣지 각도 계산**
   ```python
   top_left = corners[0]   # (x0, y0)
   top_right = corners[1]  # (x1, y1)
   dx = x1 - x0
   dy = y1 - y0
   align_angle = atan2(dy, dx)  # radians
   align_angle_deg = degrees(align_angle)
   ```

3. **각도 정규화 (Zone C/E는 180° 회전)**
   ```python
   if align_angle_deg > 90:
       align_angle_deg -= 180
   elif align_angle_deg < -90:
       align_angle_deg += 180
   ```

**결과:** `align_angle_deg` (도 단위, -90° ~ +90°)

#### Step 2: State Estimation (상태 추정)

**정렬 완료 판정:**
- 조건: `|align_angle_deg| < 0.5°` AND `|lateral| < 0.05m`

**좌표 역산 (정렬 완료 시):**

**PurePursuitController.get_robot_pose_from_tag() 동작:**

1. **Tag 정보 조회**
   ```python
   tag_info = tag_db.get(tag_id)
   tag_x = tag_info['x']
   tag_y = tag_info['y']
   zone = tag_info['zone']
   ```

2. **Zone별 좌표 역산**
   - **Zone A / DOCK:** Robot heading = 0° (+X)
     ```python
     robot_x = tag_x
     robot_y = tag_y + lateral
     robot_heading = radians(align_angle_deg)
     ```
   - **Zone B / D:** Robot heading = 90° (+Y)
     ```python
     robot_x = tag_x - lateral
     robot_y = tag_y
     robot_heading = π/2 + radians(align_angle_deg)
     ```
   - **Zone C / E:** Robot heading = -90° (-Y)
     ```python
     robot_x = tag_x + lateral
     robot_y = tag_y
     robot_heading = -π/2 + radians(align_angle_deg)
     ```

**결과:** `alignment_status` (True/False), `robot_pose` (x, y, heading)

#### Step 3: Control (제어 연산)


**파라미터:**
- $\alpha$: `align_angle_deg` (태그 기울기)
- $k_{\text{align}} = 0.8$

**제어 방향:**
- `align_angle_deg > 0` (태그가 CCW 기울어짐) → $\omega < 0$ (CW 회전)
- `align_angle_deg < 0` (태그가 CW 기울어짐) → $\omega > 0$ (CCW 회전)

**최소/최대 속도:**
```python
# 최소 속도 보장
if |angular| < 0.08 and |align_angle_deg| > 0.3:
    angular = 0.08 × sign(angular)

# 최대 속도 제한
angular = clip(angular, -0.2, +0.2)
```

**결과:** `angular_velocity` (rad/s)

#### Step 4: Actuation (구동)

**정렬 중:**
```python
twist = Twist()
twist.linear.x = 0.0
twist.angular.z = angular
cmd_pub.publish(twist)
```

**정렬 완료 시 - 참조점 업데이트:**

**RobotController.set_reference_point() 동작:**

1. **Vision 기반 위치 저장**
   ```python
   ref_robot_x = robot_x      # Vision에서 역산한 위치
   ref_robot_y = robot_y
   ref_robot_heading = robot_heading
   ```

2. **현재 Odometry 저장**
   ```python
   ref_odom_x = current_odom_x   # 현재 Odometry 값
   ref_odom_y = current_odom_y
   ref_odom_theta = current_theta
   ```

3. **이후 위치 추정**
   - Odometry Delta 계산: `Δx = odom_x - ref_odom_x`
   - 참조점 기준 보정: `est_x = ref_robot_x + Δx`
   - **Odometry Drift 보정**

**결과:** 태그 정렬 완료 + 참조점 갱신 → Drift 보정

---

## 3. 요약

### 3.1 핵심 설계 특징

1. **계층적 구조:** RobotInterface → 중간 Controllers → 하드웨어 드라이버
2. **Closed-loop Feedback:** Vision + Odometry 기반 실시간 오차 보정
3. **Deterministic State Machine:** IDLE → MOVING → ALIGNING 순서로 예측 가능
4. **Data-driven:** `map.yaml`로 모든 맵 정의

### 3.2 주요 알고리즘

| 기능 | 알고리즘 | 핵심 수식/동작 |
|------|---------|--------------|
| **태그 검출** | AprilTag Detector + solvePnP | 3D Pose Estimation |
| **경로 추종** | Pure Pursuit | $\kappa = 2\sin(\alpha)/L$, $\omega = v \cdot \kappa \cdot k$ |
| **회전 제어** | P Control | $\omega = k_p \cdot \text{error}$ |
| **정렬 제어** | Proportional Control | Corner 기반 각도 + Zone별 좌표 역산 |
| **경로 탐색** | BFS | Queue 기반 최단 경로 |

### 3.3 성능 지표

| 지표 | 목표 |
|------|------|
| **Dock 복귀 정확도** | < 5cm lateral offset |
| **회전 정확도** | < 5° 오차 |
| **제어 주기** | 30 Hz |

---

