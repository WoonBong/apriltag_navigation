# Q&A - AprilTag Navigation System

이 문서는 TEST.md에 대한 21개의 질문과 답변을 정리한 내용입니다.

---

## 질문 1: [Pivot Edge] 에서 순서가 도착하고 align하고 90도 돌고 다시 진행하는거 아니야? 그리고 idle는 무슨뜻이지?

**답변:**

**올바른 실행 순서:**

```
1. MOVING: move_to_tag()로 목표 태그까지 이동
2. ALIGNING: align_to_tag()로 정밀 정렬 (먼저!)
3. ROTATING: rotate_90()으로 90도 회전 (pivot edge만 해당)
4. IDLE: 회전 완료 후 다음 waypoint로 넘어가기 전 대기 상태
```

**IDLE의 의미:**
- IDLE은 "유휴 상태"로, 현재 waypoint 처리가 완료되고 다음 작업을 기다리는 상태입니다
- 이 상태에서 시스템은:
  - 현재 waypoint를 완료로 표시
  - waypoint_index를 증가시킴
  - 다음 waypoint가 있으면 MOVING으로 전환
  - 모든 waypoint 완료 시 COMPLETED로 전환

**왜 ALIGNING이 ROTATING 전에 오는가?**
- Pivot 태그에 도착한 직후: 태그를 정면으로 보고 있지만 위치가 부정확
- **먼저 align_to_tag()로 정밀 정렬**: 정확히 태그 위에 위치
- **그 다음 rotate_90()으로 회전**: 정렬된 상태에서 정확한 각도 제어
- 회전 후: 다음 태그를 향하게 되어 직진 가능

예시 (Tag 503 → 509, pivot edge):
```
1. MOVING: 503을 보며 전진 → 503 위치 도착 (중심 근처)
2. ALIGNING: 503에 정밀 정렬 (lateral < 0.01m, angle < 2°)
   → Reference point 설정: (0.0, 0.6275, 0°)
3. ROTATING: 90도 CW 회전 → Zone B 방향(+Y) 전환
   → Heading: 0° → -90°
4. IDLE: 회전 완료, 다음 waypoint(509)로 이동 준비
```

**State Machine 흐름:**

```python
# Straight Edge:
MOVING → ALIGNING → IDLE

# Pivot Edge:
MOVING → ALIGNING → ROTATING → IDLE
```

---

## 질문 2: edge = nav_graph.get_edge(current_wp, next_wp) 이런건 예를을면 어떤 출력이 나오지?

**답변:**

`get_edge()` 함수는 두 태그 사이의 엣지 정보를 담은 **딕셔너리**를 반환합니다.

**예시 1: Straight Edge (508 → 503)**
```python
edge = nav_graph.get_edge(508, 503)
# 출력:
{
    'to': 503,
    'direction': 'forward',
    'type': 'straight'
}
```

**예시 2: Pivot Edge (503 → 509)**
```python
edge = nav_graph.get_edge(503, 509)
# 출력:
{
    'to': 509,
    'direction': 'forward_pivot_cw',
    'type': 'pivot'
}
```

**예시 3: Backward Edge (513 → 512)**
```python
edge = nav_graph.get_edge(513, 512)
# 출력:
{
    'to': 512,
    'direction': 'backward',
    'type': 'straight'
}
```

**엣지가 존재하지 않는 경우:**
```python
edge = nav_graph.get_edge(508, 999)
# 출력: None
```

**이 정보의 활용:**
```python
if edge:
    move_dir = 'backward' if 'backward' in edge['direction'] else 'forward'

    if edge['type'] == 'pivot':
        # 회전 방향 추출
        if 'cw' in edge['direction']:
            rotation_dir = 'cw'
        else:
            rotation_dir = 'ccw'
```

**map.yaml에서의 정의:**
```yaml
edges:
  - from: 508
    to: 503
    direction: forward
    type: straight

  - from: 503
    to: 509
    direction: forward_pivot_cw
    type: pivot
```

---

## 질문 3: 폴더 구조에서 map/map_manager.py 이건 map폴더안에 map_manager.py가 있단거지? 하나의 파일 이름이 아니고?

**답변:**

**정답: 맞습니다!** `map/map_manager.py`는 `map` 폴더 안에 `map_manager.py` 파일이 있다는 의미입니다.

**실제 폴더 구조:**
```
/home/hyf/nav_ws/src/apriltag_navigation/
├── src/
│   └── apriltag_navigation/
│       ├── map/                    ← 폴더
│       │   ├── __init__.py
│       │   └── map_manager.py      ← 파일
│       ├── navigation/             ← 폴더
│       │   ├── __init__.py
│       │   └── pure_pursuit.py     ← 파일
│       ├── perception/             ← 폴더
│       │   ├── __init__.py
│       │   └── vision_module.py    ← 파일
│       ├── hardware/               ← 폴더
│       │   ├── __init__.py
│       │   └── robot_controller.py ← 파일
│       └── robot_interface.py      ← 파일
├── nodes/
│   └── main_node.py
└── config/
    └── map.yaml
```

**각 폴더의 목적:**
- `map/`: 맵 데이터 관리 (태그 DB, 그래프, 태스크)
- `navigation/`: 경로 추종 알고리즘 (Pure Pursuit, 좌표 변환)
- `perception/`: 센서 데이터 처리 (AprilTag 감지)
- `hardware/`: 하드웨어 인터페이스 (모터 제어, odometry)

**Python import 방식:**
```python
# main_node.py에서:
from apriltag_navigation.robot_interface import RobotInterface
from apriltag_navigation.map.map_manager import MapManager
from apriltag_navigation.navigation.pure_pursuit import PurePursuitController
from apriltag_navigation.perception.vision_module import VisionModule
from apriltag_navigation.hardware.robot_controller import RobotController
```

**왜 이렇게 나눴는가?**
- **모듈화**: 기능별로 명확히 분리
- **재사용성**: 다른 프로젝트에서 특정 폴더만 가져올 수 있음
- **유지보수**: 각 기능의 코드 위치가 명확함
- **테스팅**: 폴더 단위로 독립적 테스트 가능

---

## 질문 4: 카메라 켈리브레이션은 뭐야? 카메라 정보를 받는다는게 정확히 무슨 정보를 받는건데?

**답변:**

**카메라 캘리브레이션(Camera Calibration)**은 카메라의 **내부 파라미터**를 측정하는 과정입니다. 이를 통해 2D 이미지에서 3D 실세계 좌표를 계산할 수 있습니다.

**받는 정보 (Camera Info):**

```python
# /camera_info 토픽에서 받는 데이터:
K = [fx,  0, cx,
      0, fy, cy,
      0,  0,  1]

fx = 895.3  # X축 초점 거리 (focal length)
fy = 895.3  # Y축 초점 거리
cx = 640.0  # 이미지 중심 X (principal point)
cy = 360.0  # 이미지 중심 Y
```

**각 파라미터의 의미:**

1. **fx, fy (Focal Length)**:
   - 카메라 렌즈의 초점 거리를 픽셀 단위로 표현
   - 값이 클수록 "줌인"된 효과 (좁은 시야각)
   - 실세계 거리 → 픽셀 거리 변환에 사용

2. **cx, cy (Principal Point)**:
   - 광학 중심이 이미지에서 어디에 위치하는지
   - 이상적으로는 이미지 중심 (640, 360)
   - 실제로는 제조 오차로 약간 다를 수 있음

**왜 필요한가?**

AprilTag 감지 시 **PnP (Perspective-n-Point) 알고리즘**으로 태그의 3D 위치를 계산합니다:

```python
detections = detector.detect(
    gray,
    estimate_tag_pose=True,
    camera_params=[fx, fy, cx, cy],  # ← 캘리브레이션 정보
    tag_size=0.06  # 태그 실제 크기 6cm
)

# 출력:
pose_t = [x, y, z]  # 태그까지 거리 (미터)
#  x: 좌우 오프셋 (lateral)
#  y: 상하 오프셋
#  z: 전방 거리 (distance)
```

**캘리브레이션 없이는?**
- 픽셀 좌표만 알 수 있음 (태그가 이미지 어디에 있는지)
- 실제 거리를 알 수 없음 (0.5m인지 5m인지 구분 불가)
- Pure Pursuit 제어 불가능

**현장에서 확인 방법:**
```bash
# 카메라 정보 확인
rostopic echo /camera_info

# 출력 예시:
K: [895.3, 0.0, 640.0, 0.0, 895.3, 360.0, 0.0, 0.0, 1.0]
width: 1280
height: 720
distortion_model: "plumb_bob"
D: [0.0, 0.0, 0.0, 0.0, 0.0]  # 렌즈 왜곡 계수
```

**시스템에서의 처리:**
```python
def _camera_info_callback(self, msg):
    if not self.camera_info_received:
        K = msg.K
        fx, fy = K[0], K[4]
        cx, cy = K[2], K[5]
        self.camera_params = [fx, fy, cx, cy]
        self.camera_info_received = True
        rospy.loginfo(f"[VISION] Camera params: fx={fx:.1f} fy={fy:.1f}")
```

**캘리브레이션 수행 (참고):**
ROS에서는 `camera_calibration` 패키지를 사용:
```bash
rosrun camera_calibration cameracalibrator.py \
    --size 8x6 --square 0.108 \
    image:=/camera/image_raw
```
체커보드를 여러 각도로 촬영하여 K, D 파라미터를 계산합니다.

---

## 질문 5: Backward Pure Pursuit이 존재하는거야?

**답변:**

**네, 존재합니다!** `pure_pursuit.py` 파일에 `calculate_backward()` 함수가 구현되어 있습니다.

**코드 위치:**
[pure_pursuit.py:78-113](src/apriltag_navigation/navigation/pure_pursuit.py#L78-L113)

```python
def calculate_backward(self, lateral, distance_to_tag):
    """
    Pure Pursuit for backward movement.

    When backing up:
    - lateral > 0: tag is on RIGHT in camera = robot drifted LEFT of path
    - To get back to center, robot needs to move RIGHT
    - When backing up, to move RIGHT, we turn RIGHT (angular < 0)

    Same angular direction as forward! Only linear velocity is negative.
    """
    # Effective look-ahead
    L_eff = max(distance_to_tag, self.look_ahead_base)

    # Same alpha as forward: lateral > 0 → alpha < 0 → turn right
    alpha = math.atan2(-lateral, L_eff)

    # Curvature
    curvature = (2.0 * math.sin(alpha)) / L_eff

    # Angular velocity - use abs(speed) to keep curvature direction
    w = abs(self.linear_speed) * curvature * self.pp_gain_backward

    # Deadband for stability
    if abs(lateral) < self.pp_deadband_cte:
        w = 0.0

    return np.clip(w, -0.3, 0.3)
```

**Forward vs Backward 차이점:**

| 항목 | Forward | Backward |
|------|---------|----------|
| Linear 속도 | +0.3 m/s | -0.3 m/s |
| Angular 계산 | `w = v * curvature * gain_forward` | `w = abs(v) * curvature * gain_backward` |
| Gain 값 | 1.0 (더 공격적) | 0.8 (더 부드럽게) |
| Alpha 계산 | 동일 (`atan2(-lateral, L_eff)`) | 동일 |
| 회전 방향 | lateral > 0 → 우회전 | lateral > 0 → 우회전 (동일!) |

**왜 Angular는 같은 방향?**

후진 시 직관과 다르게 **회전 방향은 전진과 동일**합니다:

```
상황: 후진 중 태그가 카메라 우측에 보임 (lateral > 0)
의미: 로봇이 경로의 좌측으로 치우침
필요: 우측으로 이동해야 함

전진 시: 우회전(angular < 0) → 우측으로 이동
후진 시: 우회전(angular < 0) → 우측으로 이동 (동일!)
```

**언제 사용되는가?**

```python
# robot_interface.py의 move_to_tag()에서:
edge_info = self.map_manager.nav_graph.get_edge(current_wp, next_wp)
direction = edge_info['direction']

if 'backward' in direction:
    move_dir = 'backward'
    linear = -self.robot.linear_speed
else:
    move_dir = 'forward'
    linear = self.robot.linear_speed
```

**Backward edge 예시:**
```yaml
# map.yaml
- from: 513
  to: 512
  direction: backward
  type: straight

- from: 512
  to: 511
  direction: backward
  type: straight
```

**Gain이 다른 이유:**
- `gain_forward = 1.0`: 전진은 시야가 넓어 빠른 반응 가능
- `gain_backward = 0.8`: 후진은 시야가 제한적이므로 보수적으로 제어

**현장 테스트:**
```python
# 후진 Pure Pursuit 단독 테스트
pp = PurePursuitController(map_manager)
lateral = 0.05  # 태그가 우측 5cm
distance = 0.8  # 80cm 전방

w = pp.calculate_backward(lateral, distance)
# w = -0.12 (우회전 명령)

robot.move(linear=-0.3, angular=w)  # 후진하며 우회전
```

---

## 질문 6: 왜 RobotController랑 RotationController를 분리했어?

**답변:**

**설계 철학: 단일 책임 원칙 (Single Responsibility Principle)**

두 클래스는 **책임**과 **사용 패턴**이 다르기 때문에 분리했습니다.

**RobotController의 역할:**
- **저수준 하드웨어 인터페이스**
- ROS 토픽 pub/sub 관리
- 즉각적인 속도 명령 전송
- Odometry 데이터 수신 및 저장
- 기본 동작 제공: `move()`, `stop()`, `move_forward()`, `rotate_cw()`

```python
# 직접적인 속도 제어
robot.move(linear=0.3, angular=0.1)
robot.stop()
robot.rotate_cw(speed=0.2)
```

**RotationController의 역할:**
- **고수준 목표 지향 제어**
- 목표 각도 설정 및 추적
- 피드백 기반 제어 루프
- 도착 판정 (tolerance 체크)
- 상태 관리 (`is_rotating` 플래그)

```python
# 목표 지향 제어
rotation.start_rotation(90, 'cw')  # "90도 회전해라"
while not rotation.is_complete():
    rotation.update()  # 피드백 루프
```

**분리의 장점:**

**1. 재사용성**
```python
# RobotController는 다양한 상황에서 재사용
move_to_tag():
    robot.move(linear, angular)  # Pure Pursuit 제어

align_to_tag():
    robot.move(0, angular)  # 정렬 제어

emergency_stop():
    robot.stop()  # 비상 정지
```

**2. 테스트 용이성**
```python
# 현장에서 독립적으로 테스트 가능

# Level 1: RobotController 단독 테스트
robot.move_forward(0.2)
time.sleep(2)
robot.stop()

# Level 2: RotationController 통합 테스트
rotation = RotationController(robot)
rotation.start_rotation(90, 'cw')
while not rotation.is_complete():
    rotation.update()
    time.sleep(0.1)
```

**3. 복잡도 관리**

RobotController만 사용 시 rotate_90() 구현:
```python
# 복잡하고 반복적인 코드
def rotate_90(self, direction):
    current = self.robot.get_heading()
    target = current - math.radians(90) if direction == 'cw' else current + math.radians(90)

    # Normalize
    while target > math.pi:
        target -= 2 * math.pi

    timeout = rospy.Time.now() + rospy.Duration(10)
    while rospy.Time.now() < timeout:
        error = target - self.robot.get_heading()
        # Normalize error...
        if abs(error) < math.radians(1):
            break
        angular = 0.2 if error > 0 else -0.2
        if abs(error) < math.radians(10):
            angular *= 0.5
        self.robot.move(0, angular)
        time.sleep(0.1)
```

RotationController 사용 시:
```python
# 깔끔하고 명확한 코드
def rotate_90(self, direction):
    self.rotation.start_rotation(90, direction)

    timeout = rospy.Time.now() + rospy.Duration(10)
    while rospy.Time.now() < timeout:
        if self.rotation.update():
            return True
        time.sleep(0.1)
    return False
```

**4. 상태 캡슐화**

RotationController는 **회전 상태**를 내부에서 관리:
```python
class RotationController:
    def __init__(self, robot_controller):
        self.robot = robot_controller
        self.target_theta = None      # 목표 각도
        self.is_rotating = False       # 회전 중 여부

    def is_complete(self, tolerance_deg=1.0):
        error = self.get_rotation_error()
        return abs(math.degrees(error)) < tolerance_deg

    def cancel(self):
        self.is_rotating = False
        self.robot.stop()
```

이 상태를 RobotController에 넣으면:
- RobotController가 너무 비대해짐
- 여러 목적으로 회전할 때 상태 충돌 가능

**5. 확장성**

나중에 다른 회전 전략 추가 시:
```python
# 새로운 회전 컨트롤러 추가 가능
class AdaptiveRotationController:
    """속도를 동적으로 조절하는 회전 컨트롤러"""
    pass

class IMURotationController:
    """IMU 센서 기반 회전 컨트롤러"""
    pass

# RobotController는 변경 불필요!
```

**클래스 계층:**
```
RobotInterface (상위 레벨)
├── MapManager
├── VisionModule
├── PurePursuitController
├── RobotController (저수준 하드웨어)
│   └── RotationController (고수준 목표 제어)
```

**유사한 설계 패턴:**
- 자동차: `Engine` (저수준) vs `CruiseControl` (고수준)
- 컴퓨터: `CPU` (저수준) vs `ProcessScheduler` (고수준)
- 본 시스템: `RobotController` (저수준) vs `RotationController` (고수준)

---

## 질문 7: 제어기 계층 구조가 헷갈려. 상위제어기가 정확히 뭐야? NavigationMission이 최상위야?

**답변:**

**계층 구조 (Top-down):**

```
Level 0: Mission Controller (최상위)
└── NavigationMission (main_node.py)
    - 전체 미션 관리
    - 모드 선택 (Task 1/2, Direct, Scan)
    - State machine 실행

    ↓ 사용

Level 1: Robot API (통합 인터페이스)
└── RobotInterface (robot_interface.py)
    - 모든 하위 모듈 통합
    - 고수준 동작 제공: move_to_tag(), rotate_90(), align_to_tag()
    - 상태 관리

    ↓ 사용

Level 2: Subsystem Controllers (기능별 모듈)
├── MapManager (map/map_manager.py)
│   ├── TagDatabase
│   ├── NavigationGraph
│   └── TaskManager
│
├── VisionModule (perception/vision_module.py)
│   - AprilTag 감지
│   - 카메라 데이터 처리
│
├── PurePursuitController (navigation/pure_pursuit.py)
│   - 경로 추종 알고리즘
│   - 벽 거리 계산
│   - 좌표 변환
│
└── RobotController (hardware/robot_controller.py)
    - 하드웨어 인터페이스
    - Odometry 관리

    ↓ 사용

Level 3: Helper Controllers (특수 목적)
└── RotationController (robot_controller.py 내부)
    - 목표 각도 회전
    - 피드백 제어

    ↓ 사용

Level 4: ROS Infrastructure (최하위)
├── /cmd_vel (모터 제어)
├── /odom (위치 측정)
├── /rgb (카메라 이미지)
└── /camera_info (카메라 정보)
```

**"상위 제어기"의 정의:**

문서에서 "상위 제어기"는 **사용자가 직접 호출하는 고수준 함수**를 의미합니다:

**NavigationMission 관점 (최상위):**
```python
# 사용자가 실행하는 것
mission = NavigationMission()
mission.run()

# mission.run() 내부에서 호출하는 "상위 제어기들"
robot.load_task('task1')           # A. 태스크 로드
robot.wait_until_ready()           # B. 준비 대기
robot.move_to_tag(target)          # F. 태그로 이동
robot.rotate_90('cw')              # G. 90도 회전
robot.align_to_tag(target)         # H. 정밀 정렬
```

**RobotInterface 관점 (Level 1):**
```python
# RobotInterface가 제공하는 "상위 제어기" (공개 API)
- load_task()
- load_excel_scan()
- wait_until_ready()
- move_to_tag()
- rotate_90()
- align_to_tag()
- stop_and_wait()
- wait_for_scan()
- publish_debug_status()

# 이들이 내부에서 호출하는 "하위 함수들"
- VisionModule.is_tag_visible()
- PurePursuitController.calculate_absolute()
- RobotController.move()
- MapManager.tag_db.get_position()
```

**계층별 역할:**

| Level | 이름 | 역할 | 예시 |
|-------|------|------|------|
| 0 | Mission | 전체 미션 오케스트레이션 | `run()`, `state_machine()` |
| 1 | Robot API | 사용자 대면 인터페이스 | `move_to_tag()`, `rotate_90()` |
| 2 | Subsystems | 기능 모듈 | `calculate_absolute()`, `is_tag_visible()` |
| 3 | Helpers | 특수 목적 도구 | `start_rotation()`, `update()` |
| 4 | ROS | 하드웨어/통신 | `publish()`, `subscribe()` |

**호출 체인 예시:**

```python
# Level 0 (Mission)
mission.run()

    # Level 1 (Robot API)
    → robot.move_to_tag(503)

        # Level 2 (Subsystems)
        → vision.is_tag_visible(503)
        → vision.get_tag_data(503)
        → robot_ctrl.get_estimated_pose()
        → map_manager.tag_db.get_position(503)
        → pp.calculate_absolute(...)

            # Level 2 내부 계산
            → math.sqrt(dx*dx + dy*dy)
            → math.atan2(dy, dx)
            → math.sin(alpha)

        → robot_ctrl.move(linear, angular)

            # Level 4 (ROS)
            → cmd_pub.publish(Twist)
```

**왜 헷갈리는가?**

"제어기"라는 용어가 여러 레벨에서 사용되기 때문입니다:

- **NavigationMission**: 미션 제어기
- **RobotInterface**: 로봇 제어기
- **PurePursuitController**: 경로 추종 제어기
- **RobotController**: 하드웨어 제어기
- **RotationController**: 회전 제어기

**명확한 구분:**

```
Mission Controller (NavigationMission)
  ↓ 명령: "Task 1 실행해라"

Robot Controller (RobotInterface)
  ↓ 명령: "503으로 이동해라"

Navigation Controller (PurePursuitController)
  ↓ 명령: "각속도 0.15 rad/s로 회전해라"

Hardware Controller (RobotController)
  ↓ 명령: "모터에 속도 명령 보내라"

ROS Layer
  ↓ 실제 하드웨어로 전송
```

**정리:**

- **최상위**: `NavigationMission` - 미션 전체 관리
- **상위 제어기** (문서 기준): `RobotInterface`의 공개 메서드들 (move_to_tag, rotate_90 등)
- **하위 함수**: 상위 제어기가 호출하는 내부 함수들
- **최하위**: ROS 토픽, 수학 연산, 콜백 함수

각 "상위 제어기"는 **독립적으로 테스트 가능한 단위 동작**을 제공합니다.

---

## 질문 8: CoordinateTransformer는 왜 pure_pursuit 파일 안에 있어? 별개 클래스 아니야?

**답변:**

**현재 위치:**
[pure_pursuit.py:288-344](src/apriltag_navigation/navigation/pure_pursuit.py#L288-L344)

```python
class CoordinateTransformer:
    """
    Coordinate system transformations for different subsystems.
    Used for Mode 4 pose publishing to manipulator.
    """

    @staticmethod
    def isaac_to_manipulator(isaac_x, isaac_y):
        pub_x = -isaac_y
        pub_y = -isaac_x
        return pub_x, pub_y

    @staticmethod
    def apply_robot_center_offset(manip_x, manip_y, zone, camera_offset=0.45):
        # Camera-to-robot-center offset
        ...
```

**왜 pure_pursuit.py에 있는가?**

**1. 좌표 변환은 "Navigation" 영역**
- `navigation/` 폴더는 경로 추종 + 좌표계 관련 로직을 담당
- Pure Pursuit도 좌표 변환이고, CoordinateTransformer도 좌표 변환
- 같은 "도메인"에 속함

**2. 의존성이 낮음**
```python
# CoordinateTransformer는 static method만 사용
# 다른 클래스에 의존하지 않음
CoordinateTransformer.isaac_to_manipulator(x, y)
CoordinateTransformer.apply_robot_center_offset(x, y, zone)

# 따라서 별도 파일로 분리할 필요성이 적음
```

**3. 사용 빈도**
- Mode 4 (Scan Mode)에서만 사용
- 사용처: `robot_interface.py`의 `publish_robot_pose()`
- 전체 시스템에서 비중이 작음

**4. 파일 구조 단순화**
```
navigation/
├── pure_pursuit.py  (284줄)
│   ├── PurePursuitController
│   └── CoordinateTransformer  ← 여기
```

만약 분리한다면:
```
navigation/
├── pure_pursuit.py  (227줄)
├── coordinate_transformer.py  (57줄)  ← 너무 작은 파일
```

**언제 분리해야 하는가?**

다음 조건들이 만족되면 분리를 고려:

1. **크기**: 100줄 이상의 코드
2. **복잡도**: 여러 메서드와 상태를 가짐
3. **재사용성**: 여러 모듈에서 사용
4. **독립성**: 별도로 테스트할 가치가 있음

CoordinateTransformer는:
- ✅ 작은 크기 (57줄)
- ✅ 단순함 (static 메서드 2개)
- ❌ 재사용 낮음 (Mode 4만 사용)
- ❌ 테스트 간단 (수학 공식 2개)

따라서 현재 위치가 적절합니다.

**사용 예시:**

```python
# robot_interface.py에서:
from apriltag_navigation.navigation.pure_pursuit import CoordinateTransformer

def publish_robot_pose(self, tag_id):
    # Isaac Sim 좌표
    robot_x, robot_y, heading = self.pp.get_robot_pose_from_tag(...)

    # Manipulator 좌표로 변환
    pub_x, pub_y = CoordinateTransformer.isaac_to_manipulator(robot_x, robot_y)
    pub_x, pub_y = CoordinateTransformer.apply_robot_center_offset(
        pub_x, pub_y, zone
    )
```

**현장 테스트:**
```python
# Level 1 테스트: 단독으로 테스트 가능
from apriltag_navigation.navigation.pure_pursuit import CoordinateTransformer

# Isaac: Tag 100 at (-0.3975, 0.45)
isaac_x = -0.3975
isaac_y = 0.45

# Transform
manip_x, manip_y = CoordinateTransformer.isaac_to_manipulator(isaac_x, isaac_y)
# Expected: (-0.45, 0.3975)

print(f"Isaac: ({isaac_x}, {isaac_y})")
print(f"Manip: ({manip_x}, {manip_y})")

# Apply offset
robot_x, robot_y = CoordinateTransformer.apply_robot_center_offset(
    manip_x, manip_y, zone='A', camera_offset=0.45
)
print(f"Robot center: ({robot_x}, {robot_y})")
```

**대안 설계 (만약 복잡했다면):**

```
navigation/
├── pure_pursuit.py
│   └── PurePursuitController
├── coordinate_systems/
│   ├── __init__.py
│   ├── isaac_sim.py
│   ├── manipulator.py
│   └── transformers.py
│       └── CoordinateTransformer
```

하지만 현재 규모에서는 과도한 설계입니다 (Over-engineering).

**결론:**
CoordinateTransformer는 작고 단순한 유틸리티 클래스이므로, 관련 도메인인 `navigation/pure_pursuit.py`에 함께 두는 것이 적절합니다. 나중에 좌표 변환 로직이 복잡해지면 분리를 고려할 수 있습니다.

---

## 질문 9: 태그 딕셔너리는 언제 불러오고, edge는 어떤 패턴으로 만들었어?

**답변:**

### 태그 딕셔너리 로딩 시점

**1. 프로그램 시작 시 한 번만 로딩:**

```python
# main_node.py
if __name__ == '__main__':
    rospy.init_node('apriltag_navigation')

    # NavigationMission 생성
    mission = NavigationMission()
        ↓
    # RobotInterface 생성
    self.robot = RobotInterface()
        ↓
    # MapManager 생성 (여기서 YAML 로딩!)
    self.map_manager = MapManager()
        ↓
    # map.yaml 파일 읽기
    with open('config/map.yaml', 'r') as f:
        self.config = yaml.safe_load(f)
        ↓
    # TagDatabase 생성
    self.tag_db = TagDatabase(self.config)
        ↓
    # 73개 태그 로딩 완료!
```

**2. 메모리에 상주:**

프로그램 실행 후 종료까지 계속 메모리에 있습니다:

```python
# 언제든지 즉시 접근 가능
tag_info = map_manager.tag_db.get(503)
# 출력: {'x': 0.0, 'y': 0.6275, 'type': TagType.PIVOT, 'zone': 'A'}

x, y = map_manager.tag_db.get_position(509)
# 출력: (0.6275, 0.6275)

zone = map_manager.tag_db.get_zone(104)
# 출력: 'B'
```

**3. YAML 구조:**

```yaml
# config/map.yaml
tags:
  508:  # Dock tag
    x: -1.2
    y: 0.6275
    type: DOCK
    zone: DOCK
    name: "Dock Station"

  503:  # Pivot tag
    x: 0.0
    y: 0.6275
    type: PIVOT
    zone: A

  100:  # Work tag (scan point)
    x: -0.3975
    y: 0.45
    type: WORK
    zone: A

  # ... 총 73개 태그
```

**4. 로딩 과정:**

```python
# map_manager.py: TagDatabase._load_tags()
def _load_tags(self, tags_dict):
    for tag_id, tag_data in tags_dict.items():
        tag_id = int(tag_id)  # 문자열 → 정수 변환

        # Type enum 변환
        type_str = tag_data['type']  # 'DOCK', 'PIVOT', 'WORK', 'MOVE'
        tag_type = TagType[type_str]  # Enum으로 변환

        # 딕셔너리에 저장
        self.tags[tag_id] = {
            'x': float(tag_data['x']),
            'y': float(tag_data['y']),
            'type': tag_type,
            'zone': tag_data['zone']
        }
```

### Edge 생성 패턴

**설계 원칙:**

1. **Zone 기반 방향성**
   - Zone A: +X 방향 (forward)
   - Zone B/D: +Y 방향 (forward) or -Y 방향 (backward)
   - Zone C/E: -Y 방향 (forward)

2. **Pivot Point 중심**
   - 방향 전환은 Pivot 태그에서만
   - Pivot edge는 `forward_pivot_cw` or `forward_pivot_ccw`

3. **Straight vs Pivot**
   - 같은 방향 계속: `type: straight`
   - 90도 회전 필요: `type: pivot`

**Zone A 패턴 (Dock → Pivot → Work → Pivot):**

```yaml
edges:
  # Dock → First Pivot (straight)
  - from: 508
    to: 503
    direction: forward
    type: straight

  # Pivot → Zone B entrance (pivot + turn)
  - from: 503
    to: 509
    direction: forward_pivot_cw
    type: pivot

  # Within Zone A (straight)
  - from: 503
    to: 500
    direction: forward
    type: straight
```

**Zone B 패턴 (Backward movement):**

```yaml
# Zone B는 backward로 진입
edges:
  - from: 509
    to: 104
    direction: backward
    type: straight

  - from: 104
    to: 105
    direction: backward
    type: straight

  # Zone B → C transition (pivot)
  - from: 105
    to: 512
    direction: backward_pivot_ccw
    type: pivot
```

**Zone C/E 패턴 (Forward -Y direction):**

```yaml
# Zone C는 forward -Y
edges:
  - from: 512
    to: 106
    direction: forward
    type: straight

  - from: 106
    to: 107
    direction: forward
    type: straight
```

**Pivot Edge 생성 규칙:**

```
현재 Zone → 다음 Zone 전환:

A (+X) → B (+Y): forward_pivot_cw   (CW 90도)
B (+Y) → C (-Y): backward_pivot_ccw (CCW 90도, 후진 상태)
C (-Y) → D (+Y): forward_pivot_ccw  (CCW 90도)
D (+Y) → E (-Y): backward_pivot_ccw (CCW 90도, 후진 상태)
E (-Y) → A (+X): forward_pivot_cw   (CW 90도)
```

**Edge 타입 결정 로직:**

```python
# 내부적으로 이렇게 판단:
def determine_edge_type(from_tag, to_tag):
    from_zone = tag_db.get_zone(from_tag)
    to_zone = tag_db.get_zone(to_tag)

    from_type = tag_db.get(from_tag)['type']
    to_type = tag_db.get(to_tag)['type']

    # Pivot 태그 관여 시 pivot edge
    if from_type == TagType.PIVOT or to_type == TagType.PIVOT:
        if from_zone != to_zone:
            return 'pivot'

    return 'straight'
```

**전체 Edge 개수:**

```python
# map.yaml 로딩 후
rospy.loginfo(f"Loaded {len(nav_graph.edges)} edge groups")

# 예시:
# {
#   508: [{'to': 500, ...}],
#   501: [{'to': 111, ...}, {'to': 500, ...}],
#   111: [{'to': 110, ...}, {'to': 501, ...}],
#   ...
# }
```

**Edge 패턴 시각화 (실제 map.yaml 기준):**

```
Zone A → Zone B 전환:
     500 ----forward----> 401 ----forward----> ... -----> 502
      |
   pivot_cw (500→501)
      |
      v
     501 <---backward---- 111 <---backward---- 110 <-- ... Zone B

Zone B 내부:
     501 <---backward---- 111 <---backward---- 110 <---backward--- 109 ... 100

     (모든 Zone B 작업 태그는 후진으로 이동)
```

**현장에서 확인 (실제 경로):**

```python
# Zone B 태그 104로 가는 경로 확인
path = nav_graph.find_path(508, 104)
print(f"Path: {path}")
# [508, 500, 501, 111, 110, 109, 108, 107, 106, 105, 104]

# Edge 정보 확인
for i in range(len(path)-1):
    edge = nav_graph.get_edge(path[i], path[i+1])
    print(f"{path[i]} → {path[i+1]}: {edge}")

# 출력 예시:
# 508 → 500: {'to': 500, 'direction': 'forward', 'type': 'move'}
# 500 → 501: {'to': 501, 'direction': 'turn_cw_90', 'type': 'pivot'}
# 501 → 111: {'to': 111, 'direction': 'backward', 'type': 'move'}
# 111 → 110: {'to': 110, 'direction': 'backward', 'type': 'move'}
# ... (Zone B 모두 backward)
```

**정리:**

- **태그 로딩**: 프로그램 시작 시 한 번, YAML에서 73개 태그 로딩, 메모리 상주
- **Edge 패턴**: Zone별 방향성 + Pivot 중심 설계, straight/pivot 구분, forward/backward 방향
- **접근 방식**: `tag_db.get(id)`, `nav_graph.get_edge(from, to)`, `find_path(start, goal)`

---

## 질문 10: Reference point 초기화가 뭐야? 언제 하는거지?

**답변:**

**Reference Point의 개념:**

Reference point는 **"마지막으로 알려진 정확한 위치"**를 기록하는 기준점입니다. AprilTag를 볼 수 없을 때 odometry를 사용하여 현재 위치를 추정하기 위해 사용됩니다.

**언제 초기화하는가?**

**1. 시스템 시작 시 (Dock에서):**
```python
# main_node.py: NavigationMission.run()
def run(self):
    # Wait until ready
    if not self.robot.wait_until_ready():
        return

    # wait_until_ready() 내부에서:
    def wait_until_ready(self):
        while not rospy.is_shutdown():
            if self.vision.is_ready() and \
               self.robot.is_ready() and \
               self.vision.is_tag_visible(508):

                # Dock 태그를 볼 수 있으면 reference 설정!
                self.record_dock_start()
                return True

    def record_dock_start(self):
        tag_data = self.vision.get_tag_data(508)
        robot_x, robot_y, heading = self.pp.get_robot_pose_from_tag(
            508, tag_data['x'], 0  # align_angle = 0
        )

        # Reference point 설정!
        self.robot.set_reference_point(robot_x, robot_y, heading)
```

**2. 매 Waypoint 도착 시 (align 완료 후):**
```python
# robot_interface.py: align_to_tag()
def align_to_tag(self, target_tag):
    while not rospy.is_shutdown():
        tag_data = self.vision.get_tag_data(target_tag)
        align_angle = self.vision.get_alignment_angle(target_tag)

        if abs(lateral) < 0.01 and abs(align_angle) < 2.0:
            self.robot.stop()

            # 정렬 완료 → Reference 업데이트!
            robot_x, robot_y, heading = self.pp.get_robot_pose_from_tag(
                target_tag, lateral, align_angle
            )
            self.robot.set_reference_point(robot_x, robot_y, heading)
            return True
```

**Reference Point가 저장하는 정보:**

```python
# robot_controller.py: RobotController class
class RobotController:
    def __init__(self, map_manager):
        # Reference point (world coordinates)
        self.ref_robot_x = 0.0      # 로봇 X (Isaac Sim 좌표)
        self.ref_robot_y = 0.0      # 로봇 Y
        self.ref_robot_heading = 0.0  # 로봇 heading (radians)

        # Odometry at reference time
        self.ref_odom_x = 0.0       # Odometry X
        self.ref_odom_y = 0.0       # Odometry Y
        self.ref_odom_theta = 0.0   # Odometry theta

    def set_reference_point(self, robot_x, robot_y, robot_heading):
        # 세계 좌표 저장
        self.ref_robot_x = robot_x
        self.ref_robot_y = robot_y
        self.ref_robot_heading = robot_heading

        # 그 시점의 odometry 저장
        self.ref_odom_x = self.odom_x
        self.ref_odom_y = self.odom_y
        self.ref_odom_theta = self.current_theta

        rospy.loginfo(f"[REF] Robot=({robot_x:.3f},{robot_y:.3f}) Odom=({self.odom_x:.3f},{self.odom_y:.3f})")
```

**왜 필요한가?**

AprilTag를 볼 수 없는 구간에서 현재 위치를 추정하기 위해:

```python
# robot_controller.py: get_estimated_pose()
def get_estimated_pose(self):
    # Odometry delta 계산
    odom_dx = self.odom_x - self.ref_odom_x
    odom_dy = self.odom_y - self.ref_odom_y

    # World 좌표 추정
    robot_x = self.ref_robot_x + odom_dx
    robot_y = self.ref_robot_y + odom_dy

    # Heading 추정
    theta_delta = self.current_theta - self.ref_odom_theta
    robot_heading = self.ref_robot_heading + theta_delta

    return robot_x, robot_y, robot_heading
```

**사용 예시:**

```
시나리오: 503 → 509로 이동 중 (Pivot Edge)

1. 503에서 정렬 완료:
   set_reference_point(0.0, 0.6275, 0.0)
   → ref_robot = (0.0, 0.6275)
   → ref_odom = (5.2, 3.1)  # 현재 odometry 값

2. 90도 CW 회전 중 (태그 안 보임):
   current_odom = (5.2, 3.1)  # 제자리 회전이므로 변화 없음
   estimated_pose = get_estimated_pose()
   → robot_x = 0.0 + (5.2 - 5.2) = 0.0
   → robot_y = 0.6275 + (3.1 - 3.1) = 0.6275
   → heading = 0.0 + (current_theta - ref_theta) = -π/2

3. 509로 이동 중 (태그 안 보임):
   current_odom = (5.8, 3.1)  # +Y 방향으로 이동
   estimated_pose = get_estimated_pose()
   → robot_x = 0.0 + (5.8 - 5.2) = 0.6
   → robot_y = 0.6275 + (3.1 - 3.1) = 0.6275

4. 509 도착하여 태그 보임:
   AprilTag 기반 정확한 위치 획득
   set_reference_point(0.6275, 0.6275, π/2)
   → Reference 업데이트!
```

**Odometry 좌표계:**

Isaac Sim에서는 Odometry와 World 좌표계가 정렬되어 있습니다:
- Odometry +X = World +X
- Odometry +Y = World +Y
- 따라서 `odom_delta = world_delta` (변환 불필요)

**정리:**

- **언제**: 시스템 시작 시 + 매 waypoint 정렬 완료 시
- **무엇**: 현재 정확한 위치(AprilTag 기반) + 그 시점의 odometry 값
- **왜**: 태그 안 보일 때 odometry로 위치 추정
- **어디서 사용**: `move_to_tag()`의 `calculate_absolute()` 호출 시

---

## 질문 11: Waypoint 리스트가 비어있으면 어떻게 돼?

**답변:**

**정상 동작 경로:**

Waypoint 리스트는 **절대 비어있으면 안 됩니다**. 시스템 설계상 항상 최소 1개 이상의 waypoint가 있어야 합니다.

**Mode별 기본 waypoint:**

```python
# Mode 1/2: Task waypoints
waypoints = [508, 503, 500, 501, 502, 503, 508]  # 최소 7개

# Mode 3: Direct navigation
waypoints = find_path(current, target)  # 최소 2개 (start, goal)

# Mode 4: Excel scan
waypoints = [508, 503, ..., 104, ..., 508]  # 최소 3개 (dock-scan-dock)
```

**빈 리스트가 발생하는 경우와 처리:**

**1. Task 로딩 실패:**
```python
# robot_interface.py: load_task()
def load_task(self, task_name):
    self.waypoints = self.map_manager.task_manager.get_task_waypoints(task_name)

    if not self.waypoints:
        rospy.logerr(f"[ROBOT] Task {task_name} not found or empty!")
        return False

    rospy.loginfo(f"[ROBOT] Loaded task: {self.waypoints}")
    return True

# main_node.py에서 확인
if not robot.load_task('task1'):
    rospy.logerr("Failed to load task!")
    return  # 미션 종료
```

**2. 경로 탐색 실패 (Mode 3):**
```python
# robot_interface.py: load_direct_navigation()
def load_direct_navigation(self, target_tag):
    current_tag = 508  # 항상 dock에서 시작

    path = self.map_manager.task_manager.get_path_to_tag(current_tag, target_tag)

    if not path:
        rospy.logerr(f"[ROBOT] No path found from {current_tag} to {target_tag}!")
        return False

    self.waypoints = path
    return True

# 사용
if not robot.load_direct_navigation(105):
    rospy.logerr("No path to target!")
    return
```

**3. Excel 파일 에러 (Mode 4):**
```python
# map_manager.py: get_excel_scan_waypoints()
def get_excel_scan_waypoints(self, excel_path):
    try:
        df = pd.read_excel(excel_path, usecols=['group_id'])
        # ... process ...
        return waypoints, scan_tags

    except Exception as e:
        rospy.logerr(f"[TASK MANAGER] Failed to read Excel: {e}")
        return [508], []  # 빈 scan_tags이지만 waypoints는 [508] 반환!

# robot_interface.py에서:
waypoints, scan_tags = map_manager.task_manager.get_excel_scan_waypoints(path)

if not scan_tags:
    rospy.logwarn("[ROBOT] No scan tags found in Excel!")
    # waypoints = [508]이므로 dock로 돌아감
```

**State Machine에서의 처리:**

```python
# main_node.py: state_machine()
def state_machine(self):
    while not rospy.is_shutdown():
        if self.current_state == NavigationState.IDLE:
            # Waypoint 리스트 확인
            if self.waypoint_index >= len(self.robot.waypoints):
                # 모든 waypoint 완료
                self.current_state = NavigationState.COMPLETED
                rospy.loginfo("[MISSION] All waypoints completed!")
                break

            # 다음 waypoint 가져오기
            next_wp = self.robot.waypoints[self.waypoint_index]
            self.current_state = NavigationState.MOVING
```

**만약 강제로 빈 리스트를 넣는다면?**

```python
# 테스트: 빈 리스트 강제 설정
robot.waypoints = []
robot.waypoint_index = 0

# state_machine() 실행 시:
if self.waypoint_index >= len(self.robot.waypoints):
    # 0 >= 0 → True
    self.current_state = NavigationState.COMPLETED
    rospy.loginfo("[MISSION] No waypoints to execute!")
    break

# 결과: 즉시 COMPLETED 상태로 전환, 아무 동작 없이 종료
```

**안전 장치:**

모든 waypoint 로딩 함수는 **boolean 반환**:

```python
# 올바른 사용 패턴
if robot.load_task('task1'):
    mission.run()  # 실행
else:
    rospy.logerr("Task loading failed!")
    # 실행하지 않음

if robot.load_direct_navigation(target):
    mission.run()
else:
    rospy.logerr("Path not found!")

waypoints, scan_tags = robot.load_excel_scan(path)
if scan_tags:  # 빈 리스트 체크
    mission.run()
else:
    rospy.logwarn("No scan points, returning to dock")
```

**현장 테스트 시나리오:**

```python
# 잘못된 task 이름
robot.load_task('task999')
# 로그: "[ROBOT] Task task999 not found or empty!"
# 반환: False
# 결과: 미션 실행 안 됨

# 존재하지 않는 태그로 경로 탐색
robot.load_direct_navigation(999)
# 로그: "[ROBOT] No path found from 508 to 999!"
# 반환: False
# 결과: 미션 실행 안 됨

# 잘못된 Excel 파일
robot.load_excel_scan('/wrong/path.xlsx')
# 로그: "[TASK MANAGER] Failed to read Excel: ..."
# 반환: ([508], [])
# 결과: Dock에만 있고 스캔 없음
```

**정리:**

- **정상**: Waypoint 리스트는 항상 1개 이상
- **비정상 시**: 로딩 함수가 `False` 반환 또는 경고 로그
- **State Machine**: 빈 리스트 시 즉시 COMPLETED 전환
- **안전**: 모든 로딩 함수에서 유효성 검사 수행

---

## 질문 12: Task별로 웨이포인트 개수가 다르면 어떻게 처리하지? 검증하는 로직이 있어?

**답변:**

**Waypoint 개수는 유동적이며, 별도 검증 로직은 없습니다.**

**Task별 Waypoint 개수:**

```yaml
# map.yaml
tasks:
  task1:
    waypoints: [508, 503, 500, 501, 502, 503, 508]  # 7개

  task2:
    waypoints: [508, 503, 509, 104, 105, 512, 106, 107, 511, 512, 105, 104, 509, 503, 508]  # 15개

  custom_task:
    waypoints: [508, 503, 508]  # 3개도 가능
```

**왜 검증이 필요 없는가?**

시스템은 **인덱스 기반 순회**를 사용하므로 개수와 무관하게 동작합니다:

```python
# main_node.py: state_machine()
class NavigationMission:
    def __init__(self):
        self.waypoint_index = 0  # 현재 처리 중인 waypoint 인덱스

    def state_machine(self):
        while not rospy.is_shutdown():
            if self.current_state == NavigationState.IDLE:
                # 모든 waypoint 완료 체크
                if self.waypoint_index >= len(self.robot.waypoints):
                    self.current_state = NavigationState.COMPLETED
                    break

                # 다음 waypoint 가져오기
                current_wp = self.robot.waypoints[self.waypoint_index]

                # 마지막이 아니면 next_wp 가져오기
                if self.waypoint_index + 1 < len(self.robot.waypoints):
                    next_wp = self.robot.waypoints[self.waypoint_index + 1]
                else:
                    next_wp = None  # 마지막 waypoint

                self.waypoint_index += 1
                self.current_state = NavigationState.MOVING
```

**동적 개수 처리 예시:**

```python
# Task 1 (7개 waypoints)
robot.load_task('task1')
# waypoints = [508, 503, 500, 501, 502, 503, 508]
# len = 7

# 실행:
# index 0: 508 → 503
# index 1: 503 → 500
# index 2: 500 → 501
# ...
# index 6: 503 → 508
# index 7: 7 >= 7 → COMPLETED

# Task 2 (15개 waypoints)
robot.load_task('task2')
# waypoints = [508, ..., 508]
# len = 15

# 실행:
# index 0~14: 순차 실행
# index 15: 15 >= 15 → COMPLETED
```

**Mode 4 (Excel Scan)의 동적 생성:**

```python
# Excel 파일에 따라 waypoint 개수가 달라짐
def get_excel_scan_waypoints(self, excel_path):
    # Excel: [4, 5, 6, 7]
    scan_tags = [104, 105, 106, 107]

    waypoints = [508]  # Dock 시작

    for i, tag in enumerate(scan_tags):
        if i == 0:
            path = find_path(508, 104)  # [508, 503, 509, 104]
        else:
            path = find_path(scan_tags[i-1], scan_tags[i])

        waypoints.extend(path[1:])

    # Return path
    return_path = find_path(107, 508)
    waypoints.extend(return_path[1:])

    return waypoints, scan_tags

# 결과 waypoint 개수는 scan_tags 개수에 따라 변함!
# 4개 scan → 약 20개 waypoints
# 10개 scan → 약 50개 waypoints
```

**암묵적 검증:**

명시적 검증 로직은 없지만, 다음 조건들이 자연스럽게 검증됩니다:

**1. 최소 1개 필요:**
```python
if not self.waypoints:  # 빈 리스트
    rospy.logerr("No waypoints!")
    return False

# 또는
if self.waypoint_index >= len(self.waypoints):
    # 즉시 COMPLETED
```

**2. Waypoint가 유효한 태그 ID인지:**
```python
# move_to_tag() 내부:
target_tag = waypoints[index]

tag_info = self.map_manager.tag_db.get(target_tag)
if not tag_info:
    rospy.logerr(f"[ROBOT] Unknown tag {target_tag}!")
    # 에러 발생, 미션 중단
```

**3. Edge가 존재하는지:**
```python
# state_machine() 내부:
edge_info = nav_graph.get_edge(current_wp, next_wp)

if not edge_info:
    rospy.logerr(f"[MISSION] No edge from {current_wp} to {next_wp}!")
    # 에러 발생, 미션 중단
```

**현장 테스트 시 확인사항:**

```bash
# Task waypoint 개수 확인
roslaunch apriltag_navigation main.launch mode:=1

# 로그에서 확인:
[ROBOT] Loaded task: [508, 503, 500, 501, 502, 503, 508]
[MISSION] Total waypoints: 7

# 실행 중:
[MISSION] Progress: 1/7
[MISSION] Progress: 2/7
...
[MISSION] Progress: 7/7
[MISSION] All waypoints completed!
```

**잘못된 Waypoint 설정 예시:**

```yaml
# map.yaml - 잘못된 설정
tasks:
  bad_task1:
    waypoints: [508, 999, 508]  # 999는 존재하지 않는 태그

  bad_task2:
    waypoints: [508, 104, 508]  # 508→104 직접 edge 없음
```

실행 시:
```python
# bad_task1
robot.load_task('bad_task1')
mission.run()

# 로그:
[MISSION] Moving to 999
[ROBOT] Unknown tag 999!
# 미션 중단 (예외 발생 또는 무한 대기)

# bad_task2
robot.load_task('bad_task2')
mission.run()

# 로그:
[MISSION] No edge from 508 to 104!
# 미션 중단
```

**추가 가능한 검증 로직 (Optional):**

```python
# map_manager.py: TaskManager에 추가 가능
def validate_waypoints(self, waypoints):
    """Waypoint 유효성 검증"""

    if len(waypoints) < 2:
        rospy.logerr("Task must have at least 2 waypoints!")
        return False

    # 모든 태그가 존재하는지 확인
    for tag_id in waypoints:
        if not self.tag_db.exists(tag_id):
            rospy.logerr(f"Tag {tag_id} not found in database!")
            return False

    # 모든 edge가 존재하는지 확인
    for i in range(len(waypoints) - 1):
        from_tag = waypoints[i]
        to_tag = waypoints[i + 1]

        if not self.nav_graph.get_edge(from_tag, to_tag):
            rospy.logerr(f"No edge from {from_tag} to {to_tag}!")
            return False

    rospy.loginfo(f"Waypoints validated: {len(waypoints)} waypoints")
    return True

# 사용:
def load_task(self, task_name):
    waypoints = self.map_manager.task_manager.get_task_waypoints(task_name)

    if not self.map_manager.task_manager.validate_waypoints(waypoints):
        return False

    self.waypoints = waypoints
    return True
```

**정리:**

- **개수 제한 없음**: 3개~100개까지 모두 가능
- **명시적 검증 없음**: 인덱스 기반 순회로 자동 처리
- **암묵적 검증**: 태그 존재, edge 존재 여부는 실행 중 체크
- **확장 가능**: 필요 시 `validate_waypoints()` 추가 가능

---

## 질문 13: Dock 태그 ID가 508인데, 다른 ID로 바뀌면 어떻게 돼? 하드코딩된 부분이 많아?

**답변:**

**현재 하드코딩 상태: Yes**

Dock 태그 ID `508`이 여러 곳에 하드코딩되어 있습니다.

**하드코딩 위치:**

**1. robot_interface.py (6곳)**

```python
# Line ~80: wait_until_ready()
def wait_until_ready(self):
    while not rospy.is_shutdown():
        if self.vision.is_ready() and \
           self.robot.is_ready() and \
           self.vision.is_tag_visible(508):  # ← 하드코딩
            self.record_dock_start()
            return True

# Line ~140: record_dock_start()
def record_dock_start(self):
    tag_data = self.vision.get_tag_data(508)  # ← 하드코딩

# Line ~150: compare_dock_return()
def compare_dock_return(self):
    if not self.vision.is_tag_visible(508):  # ← 하드코딩
        return

    tag_data = self.vision.get_tag_data(508)  # ← 하드코딩

# Line ~200: load_direct_navigation()
def load_direct_navigation(self, target_tag):
    current_tag = 508  # Always start from dock  # ← 하드코딩
```

**2. map_manager.py (2곳)**

```python
# Line ~226: get_excel_scan_waypoints()
def get_excel_scan_waypoints(self, excel_path):
    waypoints = [508]  # Start from dock  # ← 하드코딩

    for i, tag in enumerate(scan_tags):
        if i == 0:
            path = self.nav_graph.find_path(508, tag)  # ← 하드코딩

    # Return to dock
    if scan_tags:
        return_path = self.nav_graph.find_path(scan_tags[-1], 508)  # ← 하드코딩
```

**3. map.yaml**

```yaml
tags:
  508:  # Dock tag - 변경 시 코드도 수정 필요!
    x: -1.2
    y: 0.6275
    type: DOCK
    zone: DOCK
    name: "Dock Station"

tasks:
  task1:
    waypoints: [508, 503, 500, 501, 502, 503, 508]  # ← 시작/끝

  task2:
    waypoints: [508, 503, ..., 508]  # ← 시작/끝
```

**Dock ID를 변경하려면?**

**방법 1: 전역 검색 후 일괄 변경**

```bash
# 모든 508 찾기
grep -rn "508" src/apriltag_navigation/

# 수동으로 모든 위치 변경:
# 508 → 999 (새로운 Dock ID)
```

**방법 2: 설정 파일로 관리 (권장)**

```python
# map.yaml에 추가:
system:
  dock_tag_id: 508  # Dock 태그 ID

# map_manager.py: MapManager 클래스에 추가
class MapManager:
    def __init__(self, config_path=None):
        # ... 기존 코드 ...

        # Dock tag ID 로딩
        self.dock_tag_id = self.config.get('system', {}).get('dock_tag_id', 508)
        rospy.loginfo(f"[MAP MANAGER] Dock tag ID: {self.dock_tag_id}")

# robot_interface.py 수정:
class RobotInterface:
    def __init__(self):
        self.map_manager = MapManager()
        self.DOCK_ID = self.map_manager.dock_tag_id  # 설정에서 로딩!

    def wait_until_ready(self):
        if self.vision.is_tag_visible(self.DOCK_ID):  # 변수 사용
            self.record_dock_start()

    def record_dock_start(self):
        tag_data = self.vision.get_tag_data(self.DOCK_ID)

    def load_direct_navigation(self, target_tag):
        current_tag = self.DOCK_ID  # 변수 사용
```

**방법 3: Enum으로 관리**

```python
# map/map_manager.py에 추가:
class SpecialTags(Enum):
    DOCK = 508
    # 향후 추가 가능:
    # CHARGING_STATION = 600
    # MAINTENANCE_AREA = 700

# 사용:
if self.vision.is_tag_visible(SpecialTags.DOCK.value):
    ...
```

**방법 4: TagDatabase에서 type으로 검색**

```python
# map_manager.py: TagDatabase에 추가
class TagDatabase:
    def get_tag_by_type(self, tag_type):
        """특정 타입의 첫 번째 태그 반환"""
        for tag_id, tag_info in self.tags.items():
            if tag_info['type'] == tag_type:
                return tag_id
        return None

# robot_interface.py 사용:
DOCK_ID = self.map_manager.tag_db.get_tag_by_type(TagType.DOCK)

if self.vision.is_tag_visible(DOCK_ID):
    ...
```

**변경 영향도 분석:**

만약 Dock ID를 508 → 600으로 변경한다면:

| 파일 | 수정 위치 | 영향 |
|------|-----------|------|
| map.yaml | tags, tasks | CRITICAL - 반드시 수정 |
| robot_interface.py | 6곳 | CRITICAL - 시스템 작동 안 됨 |
| map_manager.py | 2곳 | CRITICAL - Excel scan 실패 |

**현장에서 Dock 변경 시나리오:**

```
상황: 새로운 환경에서 Dock 태그가 888번으로 지정됨

현재 시스템:
1. map.yaml 수정:
   - 508 → 888 변경
   - 모든 task waypoints 수정

2. robot_interface.py 수정:
   - 6개 위치에서 508 → 888

3. map_manager.py 수정:
   - 2개 위치에서 508 → 888

4. 재컴파일 및 테스트

개선된 시스템 (설정 파일 사용):
1. map.yaml만 수정:
   system:
     dock_tag_id: 888

2. 끝! (코드 수정 불필요)
```

**권장 개선안:**

```yaml
# map.yaml
system:
  dock_tag_id: 508
  default_linear_speed: 0.3
  default_angular_speed: 0.25

# 코드에서:
DOCK_ID = map_manager.get_param('system.dock_tag_id', 508)
```

**정리:**

- **현재**: 508이 8곳에 하드코딩
- **변경 시**: 모든 위치 수정 필요 (에러 발생 위험)
- **권장**: 설정 파일 또는 TagType 검색으로 관리
- **향후**: `system` 섹션에 Dock ID 추가하여 중앙 관리

---

## 질문 14: move_to_tag의 리턴값이 True/False인데, 이게 정확히 언제 쓰이는거야?

**답변:**

**move_to_tag() 반환값의 의미:**

```python
# robot_interface.py
def move_to_tag(self, target_tag, edge_info):
    """
    Move towards target tag.

    Returns:
        True if arrived (tag centered in image)
        False if still moving or tag not visible
    """
```

**언제 True를 반환하는가?**

태그가 **이미지 중심에 도착**했을 때:

```python
def move_to_tag(self, target_tag, edge_info):
    # ... 중간 생략 ...

    if self.vision.is_tag_visible(target_tag):
        tag_data = self.vision.get_tag_data(target_tag)
        center_y = tag_data['center_y']

        # 도착 판정: 이미지 세로 중심(360)에서 ±50px 이내
        if abs(center_y - self.vision.image_center_y) < 50:
            self.robot.stop()
            return True  # ← 도착!

    # 아직 도착 안 함
    self.robot.move(linear, angular)
    return False
```

**언제 사용되는가?**

**State Machine의 MOVING 상태에서:**

```python
# main_node.py: state_machine()
def state_machine(self):
    while not rospy.is_shutdown():

        elif self.current_state == NavigationState.MOVING:
            # move_to_tag() 반복 호출
            arrived = self.robot.move_to_tag(next_wp, edge_info)

            if arrived:  # ← 여기서 반환값 사용!
                rospy.loginfo(f"[MISSION] Arrived at {next_wp}")

                # Pivot edge면 회전 필요
                if edge_info['type'] == 'pivot':
                    self.current_state = NavigationState.ROTATING
                else:
                    # Straight edge면 바로 정렬
                    self.current_state = NavigationState.ALIGNING

        # ... 다른 상태들 ...
```

**반환값에 따른 동작:**

```
MOVING 상태 루프:

Iteration 1:
  arrived = move_to_tag(503)
  → False (태그가 멀리 있음, center_y = 200)
  → 계속 MOVING 상태 유지

Iteration 2:
  arrived = move_to_tag(503)
  → False (조금 가까워짐, center_y = 280)
  → 계속 MOVING 상태 유지

Iteration 3:
  arrived = move_to_tag(503)
  → False (거의 도착, center_y = 340)
  → 계속 MOVING 상태 유지

Iteration 4:
  arrived = move_to_tag(503)
  → True (도착! center_y = 365, |365-360| < 50)
  → ROTATING 또는 ALIGNING으로 전환
```

**False 반환 케이스들:**

```python
def move_to_tag(self, target_tag, edge_info):
    # Case 1: 태그가 안 보임
    if not self.vision.is_tag_visible(target_tag):
        # Straight mode로 이동
        self.robot.move(linear, 0)
        return False  # ← 계속 이동 중

    # Case 2: 태그는 보이지만 아직 멀리 있음
    tag_data = self.vision.get_tag_data(target_tag)
    center_y = tag_data['center_y']

    if abs(center_y - 360) >= 50:  # 50px 이상 떨어짐
        # Pure Pursuit 제어로 이동
        self.robot.move(linear, angular)
        return False  # ← 계속 이동 중

    # Case 3: 도착!
    return True
```

**실제 사용 예시:**

```python
# Straight Edge (508 → 503)
state = MOVING
while state == MOVING:
    arrived = robot.move_to_tag(503, edge_info)

    if arrived:
        state = ALIGNING  # 바로 정렬
        break

    time.sleep(0.1)  # 10Hz 제어

# Pivot Edge (503 → 509)
state = MOVING
while state == MOVING:
    arrived = robot.move_to_tag(509, edge_info)

    if arrived:
        state = ROTATING  # 회전 먼저
        break

    time.sleep(0.1)
```

**왜 Boolean 반환인가?**

**간단명료한 상태 전환:**

```python
# 좋은 설계 (현재):
if robot.move_to_tag(target):
    next_state()

# 나쁜 설계 (대안):
result = robot.move_to_tag(target)
if result == "arrived":
    next_state()
elif result == "moving":
    continue
elif result == "lost":
    handle_error()
# 너무 복잡!
```

**단일 책임:**
- `move_to_tag()`: 이동 + 도착 판정만
- `align_to_tag()`: 정밀 정렬
- `rotate_90()`: 회전

각 함수가 명확한 완료 조건을 Boolean으로 반환.

**타이밍 제어:**

```python
# main_node.py: 메인 루프
rate = rospy.Rate(10)  # 10Hz

while not rospy.is_shutdown():
    if state == MOVING:
        if robot.move_to_tag(target, edge_info):
            state = next_state

    rate.sleep()  # ← move_to_tag()를 초당 10회 호출
```

**로깅 및 디버깅:**

```python
# 도착 시점 정확히 파악
if robot.move_to_tag(target, edge_info):
    rospy.loginfo(f"[TIMING] Arrival at {rospy.Time.now()}")
    rospy.loginfo(f"[TIMING] Travel time: {travel_duration}s")
```

**현장 테스트:**

```python
# Level 2 테스트: move_to_tag() 단독
target = 503
edge = {'direction': 'forward', 'type': 'straight'}

start_time = rospy.Time.now()
while not rospy.is_shutdown():
    arrived = robot.move_to_tag(target, edge)

    if arrived:
        duration = (rospy.Time.now() - start_time).to_sec()
        print(f"Arrived in {duration:.2f}s")
        break

    rospy.sleep(0.1)
```

**다른 함수들도 같은 패턴:**

```python
# rotate_90()
if robot.rotate_90('cw'):
    rospy.loginfo("Rotation complete")

# align_to_tag()
if robot.align_to_tag(target):
    rospy.loginfo("Alignment complete")

# wait_for_scan()
if robot.wait_for_scan(timeout=300):
    rospy.loginfo("Scan complete")
else:
    rospy.logwarn("Scan timeout")
```

**정리:**

- **반환값**: True = 도착, False = 이동 중
- **사용처**: State machine의 상태 전환 조건
- **도착 판정**: 태그가 이미지 세로 중심 ±50px 이내
- **호출 주기**: 10Hz (0.1초마다)
- **패턴**: 모든 동작 함수가 Boolean 반환 (일관성)

---

## 질문 15: Odometry는 정확히 어떤 역할을 하는거야?

**답변:**

**Odometry (오도메트리)**: 로봇의 **바퀴 회전수를 측정**하여 이동 거리와 위치를 추정하는 센서입니다.

**ROS에서의 Odometry:**

```python
# /odom 토픽 메시지 구조 (nav_msgs/Odometry)
Odometry:
  header:
    stamp: 현재 시간
    frame_id: "odom"
  child_frame_id: "base_link"
  pose:
    pose:
      position:
        x: 5.234  # 로봇 X 위치 (미터)
        y: 3.156  # 로봇 Y 위치
        z: 0.0
      orientation:  # Quaternion
        x: 0.0
        y: 0.0
        z: 0.707
        w: 0.707  # heading = 90도
  twist:
    twist:
      linear:
        x: 0.3  # 현재 속도 (m/s)
      angular:
        z: 0.1  # 현재 각속도 (rad/s)
```

**시스템에서의 3가지 역할:**

### 1. 준비 상태 확인

```python
# robot_controller.py: _odom_callback()
def _odom_callback(self, msg):
    self.odom_x = msg.pose.pose.position.x
    self.odom_y = msg.pose.pose.position.y

    q = msg.pose.pose.orientation
    euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
    self.current_theta = euler[2]

    self.odom_received = True  # ← 준비 완료 플래그

def is_ready(self):
    return self.odom_received

# robot_interface.py: wait_until_ready()
if self.robot.is_ready():  # Odometry 수신 확인
    return True
```

### 2. Reference Point 기반 위치 추정

**태그가 보이지 않을 때 현재 위치 계산:**

```python
# robot_controller.py: get_estimated_pose()
def get_estimated_pose(self):
    """
    AprilTag를 볼 수 없을 때 odometry로 위치 추정
    """
    # Odometry 변화량 계산
    odom_dx = self.odom_x - self.ref_odom_x
    odom_dy = self.odom_y - self.ref_odom_y

    # 마지막 알려진 위치 + odometry 변화량
    robot_x = self.ref_robot_x + odom_dx
    robot_y = self.ref_robot_y + odom_dy

    # Heading 추정
    theta_delta = self.current_theta - self.ref_odom_theta
    robot_heading = self.ref_robot_heading + theta_delta

    return robot_x, robot_y, robot_heading
```

**사용 시나리오:**

```
상황: 503 → 509로 이동 중

1. 503에서 정렬 완료:
   AprilTag 기반 정확한 위치: (0.0, 0.6275, 0°)
   Odometry: (5.2, 3.1, 0°)
   → set_reference_point(0.0, 0.6275, 0°)

2. 90도 CW 회전 중 (태그 안 보임):
   Odometry: (5.2, 3.1, -90°)  # 제자리 회전
   Estimated pose:
     x = 0.0 + (5.2 - 5.2) = 0.0
     y = 0.6275 + (3.1 - 3.1) = 0.6275
     heading = 0° + (-90° - 0°) = -90°

3. 509로 이동 중 (태그 안 보임):
   Odometry: (5.8, 3.1, -90°)  # +X 방향 60cm 이동
   Estimated pose:
     x = 0.0 + (5.8 - 5.2) = 0.6
     y = 0.6275 + (3.1 - 3.1) = 0.6275
     heading = -90°

4. 509 태그 보임:
   AprilTag로 정확한 위치 재측정
   → 새로운 reference point 설정
```

### 3. 회전 제어

**RotationController에서 목표 각도 도달 확인:**

```python
# robot_controller.py: RotationController
def start_rotation(self, degrees, direction):
    current = self.robot.get_heading()  # ← Odometry에서 현재 heading

    if direction == 'cw':
        self.target_theta = current - math.radians(degrees)
    else:
        self.target_theta = current + math.radians(degrees)

def get_rotation_error(self):
    current = self.robot.get_heading()  # ← Odometry 사용
    error = self.target_theta - current
    return error

def is_complete(self, tolerance_deg=1.0):
    error = self.get_rotation_error()
    return abs(math.degrees(error)) < tolerance_deg
```

**Odometry vs AprilTag:**

| 항목 | Odometry | AprilTag |
|------|----------|----------|
| **정확도** | 낮음 (누적 오차) | 높음 (절대 위치) |
| **사용 범위** | 전역 (항상 사용 가능) | 제한적 (태그 시야 내) |
| **업데이트 주파수** | 높음 (50-100Hz) | 중간 (10-30Hz) |
| **오차 특성** | 시간에 따라 누적 | 독립적 (매번 정확) |
| **사용 목적** | 단기 추정 | 위치 보정 |

**Isaac Sim 환경의 특징:**

```python
# Isaac Sim에서는 Odometry가 매우 정확함
# (실제 바퀴 슬립 없음, 시뮬레이션이므로)

# Odometry 좌표계 = Isaac Sim World 좌표계
# → 변환 불필요!

# 실제 로봇에서는:
# - Odometry는 상대 좌표 (누적 오차 큼)
# - AprilTag로 주기적 보정 필수
```

**Odometry 에러 시나리오:**

```python
# /odom 토픽이 안 오는 경우:
def is_ready(self):
    return self.odom_received  # False

# wait_until_ready()에서 대기
while not self.robot.is_ready():
    rospy.logwarn_throttle(1.0, "[ROBOT] Waiting for odometry...")
    time.sleep(0.1)

# 10초 후 timeout
if timeout:
    rospy.logerr("[ROBOT] Odometry timeout!")
    return False
```

**현장 테스트:**

```bash
# Odometry 확인
rostopic echo /odom

# 출력:
pose:
  pose:
    position:
      x: 5.234
      y: 3.156
    orientation:
      z: 0.707
      w: 0.707  # yaw = 90도

# Odometry 주파수 확인
rostopic hz /odom
# 출력: average rate: 50.0

# Odometry 없이 테스트 (에러 확인)
# 1. /odom 토픽 publish 중단
# 2. 시스템 시작
# 3. 로그: "[ROBOT] Waiting for odometry..."
# 4. 10초 후 timeout
```

**정리:**

- **역할**: 위치/속도 측정, reference point 추적, 회전 제어
- **정확도**: Isaac Sim에서는 높음, 실제 로봇에서는 오차 누적
- **사용 패턴**: AprilTag 안 보일 때 위치 추정, 매 정렬 후 reference 업데이트
- **필수성**: Odometry 없으면 시스템 시작 불가 (`is_ready()` 실패)

---

## 질문 16: Pure Pursuit에서 태그가 안 보이면 어떻게 제어하지?

**답변:**

**Pure Pursuit의 2가지 모드:**

1. **Tag Visible Mode**: AprilTag 기반 정밀 제어
2. **Straight Mode**: 태그 안 보일 때 직진

### Tag Visible Mode (정상)

```python
# robot_interface.py: move_to_tag()
def move_to_tag(self, target_tag, edge_info):
    direction = edge_info['direction']
    move_dir = 'backward' if 'backward' in direction else 'forward'

    if self.vision.is_tag_visible(target_tag):
        # ✅ 태그 보임: Pure Pursuit 제어

        # 1. 태그 정보 가져오기
        tag_data = self.vision.get_tag_data(target_tag)
        lateral = tag_data['x']
        distance = tag_data['z']
        center_y = tag_data['center_y']

        # 2. 도착 판정
        if abs(center_y - 360) < 50:
            self.robot.stop()
            return True

        # 3. 현재 위치 추정 (Odometry 기반)
        robot_x, robot_y, robot_heading = self.robot.get_estimated_pose()

        # 4. 목표 위치 가져오기
        target_x, target_y = self.map_manager.tag_db.get_position(target_tag)

        # 5. Pure Pursuit 계산
        angular = self.pp.calculate_absolute(
            robot_x, robot_y, robot_heading,
            target_x, target_y,
            lateral, move_dir
        )
        # angular: -0.3 ~ +0.3 rad/s

        # 6. 속도 명령
        if move_dir == 'backward':
            linear = -self.robot.linear_speed
        else:
            linear = self.robot.linear_speed

        self.robot.move(linear, angular)
        return False
```

### Straight Mode (태그 안 보임)

```python
# robot_interface.py: move_to_tag()
def move_to_tag(self, target_tag, edge_info):
    direction = edge_info['direction']
    move_dir = 'backward' if 'backward' in direction else 'forward'

    if not self.vision.is_tag_visible(target_tag):
        # ❌ 태그 안 보임: Straight mode

        # 각속도 0, 직진만!
        if move_dir == 'backward':
            linear = -self.robot.linear_speed  # -0.3 m/s
        else:
            linear = self.robot.linear_speed   # +0.3 m/s

        self.robot.move(linear, angular=0.0)  # ← 직진!
        return False  # 계속 이동 중
```

**왜 직진인가?**

1. **이전 정렬이 정확함**
   - 이전 waypoint에서 align_to_tag() 완료
   - 로봇은 이미 다음 태그를 향해 정렬된 상태

2. **태그가 멀리 있음**
   - 카메라 시야각 제한 (FOV)
   - 태그가 2-3m 이상 떨어지면 안 보임
   - 직진하면 태그가 시야에 들어옴

3. **간단하고 안전**
   - 복잡한 추정 불필요
   - Odometry 오차 누적 방지
   - 태그 보이면 즉시 Pure Pursuit 전환

**시나리오: 508 → 503 이동 (1.2m)**

```
초기 상태 (Dock 508에서 출발):
  - 로봇이 503을 향해 정렬됨 (align_to_tag 완료)
  - 거리: 1.2m

0.0s: move_to_tag(503) 시작
  → is_tag_visible(503): False (너무 멀리 있음)
  → Straight mode: move(0.3, 0.0)
  → 직진!

1.0s: 0.3m 전진 (거리: 0.9m)
  → is_tag_visible(503): False
  → Straight mode 계속

2.0s: 0.6m 전진 (거리: 0.6m)
  → is_tag_visible(503): True! (시야에 들어옴)
  → Tag Visible mode 전환
  → Pure Pursuit 시작
  → lateral = -0.02 (왼쪽으로 2cm 치우침)
  → angular = 0.05 (우회전 보정)
  → move(0.3, 0.05)

3.5s: Pure Pursuit으로 미세 조정하며 접근
  → lateral = 0.0
  → distance = 0.2m
  → center_y = 365 (거의 중심)

4.0s: 도착!
  → |center_y - 360| < 50
  → return True
```

**다른 상황: Pivot Edge 회전 후**

```
상황: 503에서 90도 CW 회전 후 509로 이동

회전 완료 직후:
  - 로봇 위치: (0.0, 0.6275)
  - 로봇 heading: -90° (Zone B 방향)
  - 509 위치: (0.6275, 0.6275)
  - 거리: 0.6275m

move_to_tag(509) 시작:
  → is_tag_visible(509): False (회전 후 카메라 방향 변경)
  → Straight mode: move(0.3, 0.0)
  → Zone B 방향(+Y)으로 직진

0.3초 후:
  → 0.1m 전진, 거리: 0.53m
  → is_tag_visible(509): True!
  → Pure Pursuit 전환
```

**Straight Mode의 한계:**

**문제: 초기 정렬이 부정확하면?**

```python
# 만약 이전 align에서 5도 틀어졌다면:
# - Straight mode로 직진
# - 점점 경로에서 벗어남

# 해결: 태그 보이면 Pure Pursuit이 보정
# - lateral 측정으로 오프셋 계산
# - 자동으로 경로로 복귀
```

**문제: 태그가 계속 안 보이면?**

```python
# 극단적 케이스: 태그가 가려졌거나 없음
# → Straight mode로 계속 직진
# → 영원히 return False
# → State machine에서 timeout 필요 (현재는 없음)

# 개선안:
def move_to_tag(self, target_tag, edge_info, timeout=30):
    start_time = rospy.Time.now()

    while (rospy.Time.now() - start_time).to_sec() < timeout:
        if self.vision.is_tag_visible(target_tag):
            # Pure Pursuit...
        else:
            # Straight mode...

    rospy.logerr(f"[ROBOT] Timeout: tag {target_tag} not found!")
    return False
```

**현장 테스트:**

```python
# Level 2 테스트: Straight mode 확인

# 1. 로봇을 503에 정렬
robot.align_to_tag(503)

# 2. 후진 (태그가 시야에서 사라짐)
robot.move_backward()
time.sleep(2)
robot.stop()

# 3. 503으로 다시 이동
while True:
    visible = vision.is_tag_visible(503)
    print(f"Tag visible: {visible}")

    if visible:
        print("→ Pure Pursuit mode")
        # lateral, distance 출력
    else:
        print("→ Straight mode")

    arrived = robot.move_to_tag(503, edge_info)
    if arrived:
        break

    time.sleep(0.1)

# 출력 예시:
# Tag visible: False → Straight mode
# Tag visible: False → Straight mode
# Tag visible: True → Pure Pursuit mode
# Tag visible: True → Pure Pursuit mode (lateral: -0.01)
# Tag visible: True → Pure Pursuit mode (lateral: 0.0)
# Arrived!
```

**로그 출력:**

```python
# robot_interface.py에 로그 추가 시:
if self.vision.is_tag_visible(target_tag):
    rospy.loginfo_throttle(0.5, f"[MOVE] Pure Pursuit: lat={lateral:.3f} dist={distance:.3f}")
else:
    rospy.loginfo_throttle(1.0, f"[MOVE] Straight mode (tag not visible)")
```

**정리:**

- **태그 안 보임**: Straight mode (angular = 0, 직진만)
- **태그 보임**: Tag Visible mode (Pure Pursuit 제어)
- **전환**: 자동 (매 iteration마다 `is_tag_visible()` 체크)
- **안전**: 이전 정렬이 정확하므로 직진으로 태그까지 접근 가능
- **한계**: 태그가 계속 안 보이면 무한 직진 (timeout 필요)

---

## 질문 17: YAML 파일에서 파라미터들은 언제 어떻게 설정하는거야?

**답변:**

**YAML 파라미터 설정 시점: 시스템 시작 전 (오프라인)**

### 파라미터 종류 및 설정 방법

#### 1. Robot 물리 파라미터

```yaml
# config/map.yaml
robot:
  # 로봇 크기 (미터)
  length: 0.80  # 전후 길이
  width: 0.50   # 좌우 폭

  # 속도 설정
  speeds:
    linear: 0.3    # 직진 속도 (m/s)
    angular: 0.25  # 회전 속도 (rad/s)
    slow_factor: 0.3  # 감속 비율

  # Pure Pursuit 파라미터
  pure_pursuit:
    look_ahead_base: 0.4      # 기본 look-ahead 거리 (m)
    gain_forward: 1.0         # 전진 gain (공격성)
    gain_backward: 0.8        # 후진 gain (보수적)
    deadband_cte: 0.005       # 횡방향 오차 deadband (m)
    deadband_heading: 1.0     # 각도 오차 deadband (degrees)

  # 벽 거리 설정
  wall_distances:
    work_zone: 0.35      # Work zone (B/C/D/E) 벽까지 거리
    zone_a: 0.6275       # Zone A 벽까지 거리
    min_clearance: 0.10  # 최소 안전 거리
```

**언제 설정하는가?**

- **설치 시**: 로봇 실제 크기 측정 후 입력
- **튜닝 시**: 성능 테스트 후 조정
- **환경 변경 시**: 새로운 맵에서 벽 거리 재측정

**어떻게 로딩되는가?**

```python
# map_manager.py
class MapManager:
    def __init__(self, config_path=None):
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)  # YAML 로딩

        self.robot_params = self.config.get('robot', {})

    def get_param(self, param_path, default=None):
        """점 표기법으로 파라미터 접근"""
        keys = param_path.split('.')  # 'speeds.linear' → ['speeds', 'linear']
        value = self.robot_params

        for key in keys:
            value = value[key]
        return value

# pure_pursuit.py
class PurePursuitController:
    def __init__(self, map_manager):
        # 파라미터 로딩
        self.look_ahead_base = map_manager.get_param('pure_pursuit.look_ahead_base', 0.4)
        self.pp_gain_forward = map_manager.get_param('pure_pursuit.gain_forward', 1.0)
```

#### 2. 태그 위치 및 속성

```yaml
# config/map.yaml
tags:
  508:  # Dock
    x: -1.2
    y: 0.6275
    type: DOCK
    zone: DOCK
    name: "Dock Station"

  503:  # Pivot
    x: 0.0
    y: 0.6275
    type: PIVOT
    zone: A

  100:  # Work (scan point)
    x: -0.3975
    y: 0.45
    type: WORK
    zone: A
```

**언제 설정하는가?**

- **맵 설계 시**: CAD 도면 또는 실측으로 좌표 결정
- **AprilTag 부착 후**: 실제 위치 측정 및 검증
- **변경 시**: 태그 위치 이동 또는 추가

**측정 방법:**

```bash
# 1. Isaac Sim에서 태그 위치 확인
# Grid 상에서 마우스로 클릭 → (x, y) 좌표 읽기

# 2. 실제 환경에서 측정
# - 레이저 거리계로 측정
# - 또는 AprilTag 감지로 역산:

# 로봇을 알려진 위치에 배치
# 태그를 감지
rostopic echo /tag_detections

# 출력: pose_t = [x, y, z]
# 태그 위치 = 로봇 위치 + 카메라 변환 + pose_t
```

#### 3. 엣지 (경로) 정의

```yaml
# config/map.yaml
edges:
  # Dock → First Pivot
  - from: 508
    to: 503
    direction: forward
    type: straight

  # Pivot → Zone B
  - from: 503
    to: 509
    direction: forward_pivot_cw
    type: pivot

  # Zone B movement
  - from: 509
    to: 104
    direction: backward
    type: straight
```

**언제 설정하는가?**

- **맵 설계 시**: 전체 경로 계획
- **검증 후**: 실제 주행 테스트로 direction/type 확정
- **최적화 시**: 불필요한 edge 제거 또는 shortcut 추가

**설계 프로세스:**

```
1. 태그 배치 확정
2. 로봇 이동 경로 설계 (CAD)
3. 각 구간의 방향 결정 (forward/backward)
4. Pivot 지점 식별 (방향 전환)
5. YAML 작성
6. BFS로 경로 생성 테스트
7. 실제 주행으로 검증
```

#### 4. Task Waypoints

```yaml
# config/map.yaml
tasks:
  task1:
    waypoints: [508, 503, 500, 501, 502, 503, 508]

  task2:
    waypoints: [508, 503, 509, 104, 105, 512, 106, 107, 511, 512, 105, 104, 509, 503, 508]
```

**언제 설정하는가?**

- **작업 정의 시**: 어떤 태그들을 순서대로 방문할지 결정
- **최적화 후**: 경로 길이 최소화
- **요구사항 변경 시**: 새로운 작업 추가

**생성 방법:**

```python
# 수동:
# 1. 목표 태그 리스트 작성: [500, 501, 502]
# 2. BFS로 경로 계산
path = nav_graph.find_path(508, 500)  # [508, 503, 500]
path += nav_graph.find_path(500, 501)[1:]  # [501]
path += nav_graph.find_path(501, 502)[1:]  # [502]
path += nav_graph.find_path(502, 508)[1:]  # [503, 508]
# 최종: [508, 503, 500, 501, 502, 503, 508]

# 3. YAML에 입력
```

### 파라미터 튜닝 가이드

**Pure Pursuit Gain 조정:**

```yaml
# 너무 공격적 (진동 발생):
pure_pursuit:
  gain_forward: 2.0  # ← 너무 높음
  → 좌우로 진동하며 주행

# 너무 보수적 (느린 반응):
pure_pursuit:
  gain_forward: 0.3  # ← 너무 낮음
  → 경로 이탈 시 느리게 복귀

# 적절한 값:
pure_pursuit:
  gain_forward: 1.0
  gain_backward: 0.8  # 후진은 더 부드럽게
```

**Look-ahead Distance 조정:**

```yaml
# 너무 짧음:
pure_pursuit:
  look_ahead_base: 0.1  # ← 고속에서 불안정

# 너무 긺:
pure_pursuit:
  look_ahead_base: 1.0  # ← 반응 느림

# 적절 (속도의 1-1.5배):
pure_pursuit:
  look_ahead_base: 0.4  # 0.3 m/s 속도에 적합
```

**현장에서 파라미터 변경:**

```bash
# 1. YAML 파일 수정
nano ~/nav_ws/src/apriltag_navigation/config/map.yaml

# 2. 변경 사항 확인
cat config/map.yaml | grep -A 5 "pure_pursuit"

# 3. 노드 재시작 (파라미터 재로딩)
rosnode kill /apriltag_navigation
roslaunch apriltag_navigation main.launch

# 4. 로그에서 로딩 확인
# 출력:
# [MAP MANAGER] Loaded 73 tags from .../map.yaml
# [PURE PURSUIT] look_ahead_base: 0.4
# [PURE PURSUIT] gain_forward: 1.0
```

**파라미터 백업:**

```bash
# 현재 설정 백업
cp config/map.yaml config/map_backup_20260114.yaml

# Git으로 버전 관리
git add config/map.yaml
git commit -m "Tuned Pure Pursuit gain: 1.0 → 1.2"
git tag -a "config_v1.0" -m "Working configuration for Site A"
```

**실험용 파라미터 관리:**

```yaml
# config/map_test.yaml (실험용)
robot:
  speeds:
    linear: 0.5  # 빠른 속도 테스트
  pure_pursuit:
    gain_forward: 1.5  # 공격적 튜닝

# 사용:
roslaunch apriltag_navigation main.launch config:=map_test.yaml
```

**정리:**

- **설정 시점**: 시스템 시작 전 (YAML 편집)
- **로딩 시점**: 프로그램 시작 시 한 번
- **변경 방법**: YAML 수정 + 노드 재시작
- **튜닝 순서**: 로봇 크기 → 속도 → Pure Pursuit gain → Deadband
- **백업 필수**: Git 또는 파일 복사로 버전 관리

---

## 질문 18: Lateral offset이 정확히 뭐야? +면 어느쪽이야?

**답변:**

**Lateral Offset (횡방향 오프셋)**: 로봇의 **중심선에서 태그까지의 수평 거리**

### AprilTag 좌표계

```python
# vision_module.py: _image_callback()
detections = detector.detect(gray, estimate_tag_pose=True, ...)

for det in detections:
    pose_t = det.pose_t.flatten()
    # pose_t[0]: x (lateral, 좌우)
    # pose_t[1]: y (vertical, 상하)
    # pose_t[2]: z (depth, 전후)

    self.detected_tags[det.tag_id] = {
        'x': pose_t[0],  # lateral offset
        'y': pose_t[1],  # vertical offset
        'z': pose_t[2],  # distance
    }
```

**카메라 좌표계:**

```
        Camera (로봇 시점)
             ↑ -y (up)
             |
             |
    -x ←----•----→ +x (right)
  (left)    |
            |
            ↓ +y (down)

          ⊙ +z (forward, into scene)
```

**Lateral Offset 부호:**

- **lateral > 0**: 태그가 카메라 **우측**에 보임
- **lateral < 0**: 태그가 카메라 **좌측**에 보임
- **lateral = 0**: 태그가 카메라 **정중앙**

### 예시 시나리오

**상황 1: 로봇이 경로 좌측으로 치우침**

```
        로봇 카메라 시점:
        ┌─────────────┐
        │             │
        │      Tag    │  ← 태그가 우측에 보임
        │        ■    │
        └─────────────┘

lateral = +0.05  (5cm 우측)

의미:
  - 태그가 로봇 우측 5cm 위치
  - 로봇이 경로에서 좌측으로 5cm 벗어남
  - 우회전 필요!
```

**상황 2: 로봇이 경로 우측으로 치우침**

```
        로봇 카메라 시점:
        ┌─────────────┐
        │             │
        │  Tag        │  ← 태그가 좌측에 보임
        │  ■          │
        └─────────────┘

lateral = -0.05  (5cm 좌측)

의미:
  - 태그가 로봇 좌측 5cm 위치
  - 로봇이 경로에서 우측으로 5cm 벗어남
  - 좌회전 필요!
```

**상황 3: 완벽한 정렬**

```
        로봇 카메라 시점:
        ┌─────────────┐
        │             │
        │      ■      │  ← 태그가 정중앙
        │             │
        └─────────────┘

lateral = 0.0

의미:
  - 태그가 정중앙
  - 로봇이 경로 중심
  - 회전 불필요!
```

### Pure Pursuit에서의 사용

```python
# pure_pursuit.py: calculate_forward()
def calculate_forward(self, lateral, distance_to_tag):
    """
    Args:
        lateral: + = tag is right = need to turn right
    """
    # Alpha: angle to target
    # If lateral > 0, tag is on right, alpha < 0
    alpha = math.atan2(-lateral, L_eff)  # ← 부호 주의!

    # lateral > 0:
    #   atan2(-0.05, 0.8) = atan2(-0.05, 0.8) = -0.062 rad (음수)
    #   → 음수 alpha = 우회전 필요

    # lateral < 0:
    #   atan2(+0.05, 0.8) = +0.062 rad (양수)
    #   → 양수 alpha = 좌회전 필요

    curvature = (2.0 * math.sin(alpha)) / L_eff
    w = self.linear_speed * curvature * self.pp_gain_forward

    return w  # 각속도 (+ = CCW, - = CW)
```

**제어 로직:**

| lateral | alpha | curvature | angular (w) | 동작 |
|---------|-------|-----------|-------------|------|
| +0.05 | -0.062 | -0.155 | -0.047 | 우회전 (CW) |
| -0.05 | +0.062 | +0.155 | +0.047 | 좌회전 (CCW) |
| 0.0 | 0.0 | 0.0 | 0.0 | 직진 |

### Zone별 Lateral의 의미 (벽 거리 계산)

**Zone A (로봇 heading +X, 벽은 -Y):**

```
      +Y (left)
        ↑
        |
-X ←----•----→ +X (forward)
        |
        ↓
      -Y (right, wall)

lateral > 0:
  - 태그가 카메라 우측 = 세계 좌표 +Y 방향
  - 로봇이 벽에서 FARTHER (벽까지 거리 증가)
  - wall_distance = base_distance - lateral

lateral < 0:
  - 태그가 카메라 좌측 = 세계 좌표 -Y 방향
  - 로봇이 벽에 CLOSER (벽까지 거리 감소)
  - wall_distance = base_distance - lateral (음수이므로 증가)
```

**Zone B (로봇 heading +Y, 벽은 -X):**

```
      +X (left)
        ↑
        |
-Y ←----•----→ +Y (forward)
        |
        ↓
      -X (right, wall)

lateral > 0:
  - 태그가 카메라 우측 = 세계 좌표 -X 방향
  - 로봇이 벽에 CLOSER
  - wall_distance = base_distance + lateral (양수이지만 벽으로)
```

**계산 코드:**

```python
# pure_pursuit.py: calculate_wall_distance()
def calculate_wall_distance(self, lateral, align_angle_deg, zone):
    # Base distance
    if zone in ['B', 'C', 'D', 'E']:
        base_dist = 0.35  # Work zone
    else:
        base_dist = 0.6275  # Zone A

    # Lateral에 따른 보정
    if zone in ['B', 'D']:
        center_to_wall = base_dist + lateral
    elif zone in ['C', 'E']:
        center_to_wall = base_dist + lateral
    else:  # Zone A
        center_to_wall = base_dist - lateral  # ← 부호 반대!

    # 로봇 크기 보정
    right_side_dist = center_to_wall - half_width * cos(angle) - ...
    return right_side_dist
```

### 실제 값 예시

**현장 테스트 로그:**

```bash
# Zone A, Tag 503
[VISION] Tag 503 detected: lateral=0.02, distance=0.8
[PP] Alpha: -0.025 rad (-1.4°)
[PP] Angular: -0.008 rad/s (우회전)

# Zone A, Tag 100 (벽 거리)
[VISION] Tag 100 detected: lateral=0.03
[PP] Wall distance: 0.60m (base=0.6275, lateral=+0.03)
# 0.6275 - 0.03 = 0.5975 → 안전

# Zone B, Tag 104
[VISION] Tag 104 detected: lateral=-0.01
[PP] Wall distance: 0.34m (base=0.35, lateral=-0.01)
# 0.35 + (-0.01) = 0.34 → 안전

# 벽 충돌 경고
[VISION] Tag 105 detected: lateral=0.25
[PP] Wall distance: 0.08m (base=0.35, lateral=0.25)
# 0.35 - 0.25 = 0.10 미만!
[WARNING] Too close to wall!
```

### Deadband (불감대)

```python
# pure_pursuit.py
if abs(lateral) < self.pp_deadband_cte:  # 0.005m = 5mm
    w = 0.0  # 미세한 오차는 무시

# 이유:
# - 센서 노이즈
# - 불필요한 진동 방지
# - 에너지 절약
```

**정리:**

- **Lateral > 0**: 태그가 카메라 우측 → 우회전 필요
- **Lateral < 0**: 태그가 카메라 좌측 → 좌회전 필요
- **Lateral = 0**: 완벽한 정렬
- **Zone별 의미**: 벽까지 거리 계산 시 방향 고려
- **Deadband**: ±5mm 이내는 무시

---

## 질문 19: stop_and_wait는 언제 쓰는거야?

**답변:**

**stop_and_wait()**: 로봇을 **정지**시키고 **지정된 시간 동안 대기**하는 함수

### 함수 정의

```python
# robot_interface.py
def stop_and_wait(self, duration=1.0):
    """
    Stop robot and wait for specified duration.

    Args:
        duration: Wait time in seconds (default: 1.0)

    Returns:
        True when wait completes
    """
    self.robot.stop()  # 모든 속도 0으로
    rospy.sleep(duration)  # 지정 시간 대기
    return True
```

### 사용 시나리오

#### 1. Pivot Edge 회전 전 안정화

```python
# main_node.py: state_machine()에서 (가상의 개선안)
def state_machine(self):
    if self.current_state == NavigationState.MOVING:
        if self.robot.move_to_tag(next_wp, edge_info):
            # 도착!

            if edge_info['type'] == 'pivot':
                # 회전 전 0.5초 대기 (관성 제거)
                self.robot.stop_and_wait(0.5)  # ← 사용!
                self.current_state = NavigationState.ROTATING
```

**왜 필요한가?**

```
상황: 로봇이 pivot 태그에 도착

1. move_to_tag() 완료
   - 로봇 속도: 0.3 m/s → 0 m/s (급정지)
   - 하지만 관성으로 약간 흔들림 발생

2. stop_and_wait(0.5) 실행
   - 0.5초 동안 완전히 정지
   - 진동 소멸
   - Odometry 안정화

3. rotate_90() 시작
   - 안정된 상태에서 회전 시작
   - 정확한 각도 제어 가능
```

#### 2. 정렬 후 안정화

```python
# robot_interface.py: align_to_tag()에서 (가상의 개선안)
def align_to_tag(self, target_tag):
    while not rospy.is_shutdown():
        # ... 정렬 로직 ...

        if abs(lateral) < 0.01 and abs(align_angle) < 2.0:
            self.robot.stop()

            # 정렬 완료 후 0.3초 대기
            self.stop_and_wait(0.3)  # ← 안정화!

            # Reference point 설정
            robot_x, robot_y, heading = self.pp.get_robot_pose_from_tag(...)
            self.robot.set_reference_point(robot_x, robot_y, heading)
            return True
```

**효과:**

- 미세 진동 제거
- AprilTag 감지 안정화
- 정확한 reference point 기록

#### 3. Mode 4: Scan 대기

```python
# main_node.py: state_machine()
def state_machine(self):
    if self.current_state == NavigationState.WAIT_SCAN:
        # Scan tag에 도착

        # 1. 위치 퍼블리시
        self.robot.publish_robot_pose(scan_tag)

        # 2. 정지 및 대기 (manipulator 준비 시간)
        self.robot.stop_and_wait(2.0)  # ← 2초 대기!

        # 3. Scan 완료 신호 대기
        if self.robot.wait_for_scan(timeout=300):
            rospy.loginfo("[MISSION] Scan complete")
            self.current_state = NavigationState.IDLE
```

**시나리오:**

```
1. 로봇이 scan tag (104)에 도착
2. publish_robot_pose() → /robot_pose 토픽 발행
3. stop_and_wait(2.0) → 2초 대기
   - Manipulator가 위치 수신
   - 카메라 위치 조정
   - 준비 완료
4. wait_for_scan() → /scan_finished 신호 대기
5. 신호 수신 후 다음 waypoint로 이동
```

#### 4. 비상 상황 처리

```python
# 가상의 예외 처리 코드
def handle_tag_loss(self, target_tag):
    """태그를 잃어버렸을 때"""

    rospy.logwarn(f"[ROBOT] Tag {target_tag} lost!")

    # 1. 즉시 정지
    self.robot.stop()

    # 2. 3초 대기 (태그 재감지 시도)
    self.stop_and_wait(3.0)

    # 3. 태그 확인
    if self.vision.is_tag_visible(target_tag):
        rospy.loginfo("[ROBOT] Tag recovered!")
        return True
    else:
        rospy.logerr("[ROBOT] Tag still lost!")
        return False
```

#### 5. 디버깅 / 데모

```python
# 느린 속도로 동작 시연
def demo_mode(self):
    for i, wp in enumerate(self.waypoints):
        rospy.loginfo(f"[DEMO] Moving to waypoint {i}: {wp}")

        self.move_to_tag(wp, edge_info)
        self.align_to_tag(wp)

        # 각 waypoint마다 5초 대기 (관찰 시간)
        self.stop_and_wait(5.0)  # ← 데모용!

        rospy.loginfo(f"[DEMO] Waypoint {i} complete")
```

### 다른 대기 함수와 비교

| 함수 | 목적 | 대기 조건 | 반환 |
|------|------|-----------|------|
| `stop_and_wait(duration)` | 단순 시간 대기 | 지정 시간 경과 | True |
| `wait_until_ready()` | 시스템 준비 대기 | Vision/Odom/Dock tag | True/False |
| `wait_for_scan(timeout)` | Scan 완료 대기 | /scan_finished 토픽 | True/False |
| `rospy.sleep(duration)` | 저수준 대기 | 지정 시간 경과 | None |

**stop_and_wait vs rospy.sleep:**

```python
# stop_and_wait (권장):
self.robot.stop_and_wait(2.0)
# - 로봇 정지 포함
# - 명확한 의도 표현
# - Boolean 반환

# rospy.sleep (저수준):
self.robot.stop()
rospy.sleep(2.0)
# - 수동으로 stop() 호출 필요
# - 반환값 없음
```

### 현장 테스트

```python
# Level 1 테스트: stop_and_wait 단독
robot = RobotController(map_manager)

# 전진
robot.move_forward()
rospy.sleep(2.0)

# 정지 및 대기
start = rospy.Time.now()
robot.stop_and_wait(3.0)
end = rospy.Time.now()

duration = (end - start).to_sec()
print(f"Wait duration: {duration:.2f}s")
# 출력: Wait duration: 3.00s

# 로봇 속도 확인 (응 0이어야 함)
odom = rospy.wait_for_message('/odom', Odometry)
print(f"Linear velocity: {odom.twist.twist.linear.x}")
# 출력: Linear velocity: 0.0
```

### 현재 시스템에서의 사용

**현재는 명시적으로 사용하지 않음!**

하지만 다음 상황에서 **추가하면 유용**:

1. **Pivot 회전 전**: `stop_and_wait(0.5)` - 관성 제거
2. **Align 완료 후**: `stop_and_wait(0.3)` - Reference 안정화
3. **Scan 시작 전**: `stop_and_wait(2.0)` - Manipulator 준비
4. **에러 복구**: `stop_and_wait(3.0)` - 재시도 전 대기

**추가 예시:**

```python
# robot_interface.py: rotate_90() 개선
def rotate_90(self, direction):
    # 회전 전 안정화
    self.stop_and_wait(0.3)  # ← 추가!

    self.rotation.start_rotation(90, direction)

    timeout = rospy.Time.now() + rospy.Duration(10)
    while rospy.Time.now() < timeout:
        if self.rotation.update():
            # 회전 완료 후 안정화
            self.stop_and_wait(0.5)  # ← 추가!
            return True
        rospy.sleep(0.1)

    return False
```

**정리:**

- **목적**: 로봇 정지 + 지정 시간 대기
- **현재 사용**: 명시적으로 사용 안 함 (함수는 존재)
- **권장 추가**: Pivot 전/후, Align 후, Scan 전
- **효과**: 진동 제거, 센서 안정화, 정확도 향상
- **대안**: `rospy.sleep()` (하지만 stop() 별도 호출 필요)

---

## 질문 20: Zone A에서 lateral이 +면 벽에서 멀어지는거야 가까워지는거야?

**답변:**

**Zone A에서 lateral > 0 → 벽에서 FARTHER (멀어짐)**

### Zone A 기하학

**Zone A 설정:**
- 로봇 heading: **+X 방향** (forward)
- 벽 위치: **-Y 방향** (로봇의 우측)
- 카메라: 바닥을 아래로 바라봄

```
      +Y (로봇의 좌측)
        ↑
        |
        |  Robot (heading +X)
        |    →
-X ←----•----------------→ +X (forward)
        |
        |
        ↓
      -Y (로봇의 우측, WALL)
    ═══════════════════════
```

**카메라 좌표계:**

```
카메라가 바닥을 내려다봄:
  - 카메라 우측 (+x) = 세계 +Y 방향
  - 카메라 좌측 (-x) = 세계 -Y 방향
  - 카메라 전방 (+z) = 세계 +X 방향

바닥 태그 감지:
  - lateral > 0 → 태그가 카메라 우측 → 세계 좌표 +Y
  - lateral < 0 → 태그가 카메라 좌측 → 세계 좌표 -Y
```

### Lateral 부호의 의미

**Lateral > 0 (양수):**

```
상황: 태그가 카메라 이미지 우측에 보임

        Camera View (바닥 촬영)
        ┌─────────────┐
        │             │
        │        ■ Tag│  ← 우측에 보임
        │             │
        └─────────────┘

실제 위치:
        +Y (좌측)
          ↑
          |
          |  Robot
          |    →
  --------•--------  Path (경로)
          |
          ■ Tag (우측, +Y 방향 치우침)
          |
          ↓
        -Y (우측, WALL)
      ═══════════

의미:
  - 태그가 세계 좌표 +Y 방향에 있음
  - 로봇이 경로에서 -Y 방향으로 치우침
  - 로봇이 벽(-Y) 쪽으로 가까워짐
  - 하지만! lateral > 0은 로봇이 경로 중심에서 -Y로 치우쳤다는 뜻
  - 따라서 center_to_wall = base - lateral
  - base = 0.6275m (Zone A 기본 거리)
  - lateral = +0.05m → center_to_wall = 0.6275 - 0.05 = 0.5775m
  - 벽까지 거리 DECREASE (감소)

잠깐, 위 설명이 모순인가요?
```

### 정확한 분석

**올바른 이해:**

Zone A에서 **base_distance**는 **경로 중심에서 벽까지의 거리**입니다.

```
                    +Y
                     ↑
                     |
        경로 중심 ---|--- 0.6275m (base_distance)
                     |
                     ▼
                   Wall (-Y)
                 ═════════
```

**Lateral의 의미:**

```python
# pure_pursuit.py: get_robot_pose_from_tag()
if zone == 'A':
    robot_x = tag_x
    robot_y = tag_y + lateral  # ← 로봇 Y 위치

# lateral > 0:
#   robot_y = tag_y + 0.05
#   → 로봇이 태그보다 +Y 방향 (벽에서 멀어짐)

# lateral < 0:
#   robot_y = tag_y - 0.05
#   → 로봇이 태그보다 -Y 방향 (벽에 가까워짐)
```

**벽 거리 계산:**

```python
# pure_pursuit.py: calculate_wall_distance()
if zone == 'A':
    center_to_wall = base_dist - lateral

# lateral > 0 (로봇이 +Y 방향):
#   center_to_wall = 0.6275 - 0.05 = 0.5775m
#   → 벽까지 거리 DECREASE

# 어? 모순 아닌가?
```

### 혼란의 원인

**태그 위치 vs 로봇 위치**

카메라가 감지하는 lateral은 **"태그가 로봇에 대해 어디 있는가"**입니다.

```
Lateral > 0: 태그가 카메라 우측 (+Y 방향)
           → 로봇이 태그에 대해 좌측 (-Y 방향)
           → 로봇이 경로 중심에서 -Y로 치우침
           → 벽(-Y)에 가까워짐!
```

**올바른 답:**

**Zone A에서 lateral > 0 → 벽에 CLOSER (가까워짐)**

### 코드 재확인

```python
# pure_pursuit.py: calculate_wall_distance()
def calculate_wall_distance(self, lateral, align_angle_deg, zone):
    """
    Args:
        lateral: pose_t[0] (+ = tag is right of camera center)

    Zone A: Robot faces +X, wall at -Y (robot's right)
      Camera right = world +Y direction
      lateral > 0 = robot shifted +Y = FARTHER from wall (-Y)
      → center_to_wall = base - lateral
    """

    if zone == 'A':
        center_to_wall = base_dist - lateral

    # lateral > 0:
    #   center_to_wall = 0.6275 - 0.05 = 0.5775m
    # lateral < 0:
    #   center_to_wall = 0.6275 - (-0.05) = 0.6775m
```

**코드 주석과 계산이 모순!**

주석: "lateral > 0 = FARTHER from wall"
계산: `base - lateral` → lateral > 0일 때 거리 감소 (CLOSER)

### 정확한 해석

**주석이 틀렸습니다.**

**올바른 주석:**

```python
# Zone A: Robot faces +X, wall at -Y (robot's right)
#   Camera right = world +Y direction
#   lateral > 0 = tag shifted +Y = robot is at -Y = CLOSER to wall
#   → center_to_wall = base - lateral
```

**실제 예시:**

```
태그 위치: (0.0, 0.6275)  # 경로 중심
벽 위치: -Y 방향 (세계 좌표 y < 0)

Case 1: lateral = +0.05
  robot_y = 0.6275 + 0.05 = 0.6775 (잘못된 계산!)

  올바른 이해:
  - 태그가 카메라 우측(+x) = 세계 +Y
  - 로봇은 태그보다 -Y 방향에 있음
  - robot_y = 0.6275 - 0.05 = 0.6225 (벽에 가까움)
  - center_to_wall = 0.6275 - 0.05 = 0.6225

Case 2: lateral = -0.05
  - 태그가 카메라 좌측(-x) = 세계 -Y
  - 로봇은 태그보다 +Y 방향에 있음
  - robot_y = 0.6275 + 0.05 = 0.6775 (벽에서 멀어짐)
  - center_to_wall = 0.6275 - (-0.05) = 0.6775
```

### 최종 답변

**Zone A에서:**

- **lateral > 0**: 벽에 **CLOSER** (가까워짐)
  - 태그가 카메라 우측 = 로봇이 경로 좌측 = 벽 쪽으로 치우침
  - wall_distance = base - lateral = 0.6275 - 0.05 = 0.5775m

- **lateral < 0**: 벽에서 **FARTHER** (멀어짐)
  - 태그가 카메라 좌측 = 로봇이 경로 우측 = 벽 반대쪽으로 치우침
  - wall_distance = base - lateral = 0.6275 - (-0.05) = 0.6775m

**코드 주석 오류:**

[pure_pursuit.py:198](src/apriltag_navigation/navigation/pure_pursuit.py#L198) 주석이 부호를 반대로 설명하고 있습니다.

**정리:**

- **lateral > 0**: Zone A에서 벽에 가까워짐 (0.6275 - lateral)
- **lateral < 0**: Zone A에서 벽에서 멀어짐 (0.6275 + |lateral|)
- **계산 공식**: `center_to_wall = 0.6275 - lateral`
- **주석 오류**: 코드 주석의 "FARTHER" 설명이 틀림

---

## 질문 21: 벽 거리 계산을 현장에서 어떻게 검증해?

**답변:**

**벽 거리 검증 방법: 3단계 프로세스**

### 1. 정적 측정 (Static Measurement)

**준비물:**
- 레이저 거리계 또는 줄자
- 로봇
- Work zone 태그 (104, 105, etc.)

**절차:**

```bash
# Step 1: 로봇을 태그에 정렬
roslaunch apriltag_navigation main.launch

# ROS 명령으로 특정 태그로 이동
rostopic pub /goal_tag std_msgs/Int32 "data: 104"

# 또는 Python:
robot.move_to_tag(104, edge_info)
robot.align_to_tag(104)

# Step 2: 로봇 정지 후 실제 거리 측정
# 레이저 거리계로:
#   - 로봇 우측 끝에서 벽까지 거리 측정
#   - 예: 실측 = 0.33m

# Step 3: 시스템 계산값 확인
rostopic echo /nav_debug_status

# 출력:
wall_distance: 0.34
lateral_offset: 0.02
zone: "B"

# Step 4: 비교
실측: 0.33m
계산: 0.34m
오차: 0.01m (1cm) → OK!
```

**여러 위치에서 반복:**

```python
# test_wall_distance.py
import rospy
from apriltag_navigation.robot_interface import RobotInterface

robot = RobotInterface()

# Test tags in work zone
test_tags = [104, 105, 106, 107]

for tag_id in test_tags:
    # Move to tag
    robot.move_to_tag(tag_id, ...)
    robot.align_to_tag(tag_id)
    robot.stop_and_wait(2.0)

    # Get calculated distance
    debug = rospy.wait_for_message('/nav_debug_status', ...)
    calc_dist = debug.wall_distance

    # Prompt for manual measurement
    real_dist = float(input(f"Tag {tag_id} - Enter measured wall distance (m): "))

    # Compare
    error = abs(calc_dist - real_dist)
    print(f"Tag {tag_id}: Calc={calc_dist:.3f}m, Real={real_dist:.3f}m, Error={error:.3f}m")

    if error > 0.05:
        print(f"  ⚠️  WARNING: Error > 5cm!")
    else:
        print(f"  ✓ OK")

# 출력 예시:
# Tag 104: Calc=0.340m, Real=0.330m, Error=0.010m
#   ✓ OK
# Tag 105: Calc=0.280m, Real=0.250m, Error=0.030m
#   ✓ OK
# Tag 106: Calc=0.150m, Real=0.120m, Error=0.030m
#   ⚠️  WARNING: Error > 5cm!  # 재확인 필요
```

### 2. 동적 모니터링 (Dynamic Monitoring)

**실시간 로그 확인:**

```python
# robot_interface.py: publish_debug_status() 활용
def move_to_tag(self, target_tag, edge_info):
    while True:
        # ... Pure Pursuit ...

        # Wall distance 계산
        zone = self.map_manager.tag_db.get_zone(target_tag)
        wall_dist = self.pp.calculate_wall_distance(lateral, align_angle, zone)

        # 실시간 로그
        rospy.loginfo_throttle(0.5,
            f"[WALL] Zone {zone}: {wall_dist:.3f}m (lateral={lateral:.3f})")

        # 경고
        if wall_dist < 0.10:
            rospy.logwarn_throttle(0.5,
                f"[WALL] Too close! {wall_dist:.3f}m < 0.10m")

        # ... 계속 이동 ...
```

**RViz 시각화 (Optional):**

```python
# visualization.py (가상의 추가 파일)
from visualization_msgs.msg import Marker

def publish_wall_marker(robot_x, robot_y, heading, wall_dist, zone):
    """
    RViz에 벽까지 거리를 화살표로 표시
    """
    marker = Marker()
    marker.header.frame_id = "map"
    marker.type = Marker.ARROW
    marker.action = Marker.ADD

    # 로봇 위치에서 벽 방향으로 화살표
    start = Point(x=robot_x, y=robot_y, z=0)

    if zone == 'A':
        # 벽은 -Y 방향
        end = Point(x=robot_x, y=robot_y - wall_dist, z=0)
    elif zone in ['B', 'D']:
        # 벽은 -X 방향
        end = Point(x=robot_x - wall_dist, y=robot_y, z=0)
    # ...

    marker.points = [start, end]

    # 색상: 거리에 따라
    if wall_dist < 0.10:
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Red
    elif wall_dist < 0.20:
        marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)  # Yellow
    else:
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # Green

    marker_pub.publish(marker)

# RViz에서:
# - Green arrow: 안전 거리
# - Yellow arrow: 주의
# - Red arrow: 위험!
```

### 3. 파라미터 캘리브레이션

**Base Distance 검증:**

```python
# calibrate_wall_distance.py
import rospy
from apriltag_navigation.robot_interface import RobotInterface

robot = RobotInterface()

# Zone A 캘리브레이션
print("=== Zone A Calibration ===")
robot.move_to_tag(500, ...)
robot.align_to_tag(500)

# Lateral = 0일 때 실측
input("Align robot to tag 500 (lateral=0). Press Enter...")

tag_data = robot.vision.get_tag_data(500)
print(f"Lateral offset: {tag_data['x']:.4f}m")

real_wall_dist = float(input("Enter measured wall distance (m): "))
print(f"Current base_distance in YAML: 0.6275m")
print(f"Suggested base_distance: {real_wall_dist:.4f}m")

# Zone B 캘리브레이션
print("\n=== Zone B Calibration ===")
robot.move_to_tag(104, ...)
robot.align_to_tag(104)

input("Align robot to tag 104 (lateral=0). Press Enter...")

tag_data = robot.vision.get_tag_data(104)
real_wall_dist = float(input("Enter measured wall distance (m): "))
print(f"Current base_distance in YAML: 0.35m")
print(f"Suggested base_distance: {real_wall_dist:.4f}m")

# YAML 업데이트 안내
print("\n=== Update config/map.yaml ===")
print("robot:")
print("  wall_distances:")
print(f"    zone_a: {zone_a_calibrated:.4f}")
print(f"    work_zone: {work_zone_calibrated:.4f}")
```

**로봇 크기 검증:**

```python
# 로봇 실제 크기 측정
length = float(input("Measure robot length (m): "))  # 0.80m
width = float(input("Measure robot width (m): "))    # 0.50m

# YAML과 비교
yaml_length = robot.map_manager.get_param('length')
yaml_width = robot.map_manager.get_param('width')

if abs(length - yaml_length) > 0.01:
    print(f"⚠️  Length mismatch: YAML={yaml_length}, Real={length}")

if abs(width - yaml_width) > 0.01:
    print(f"⚠️  Width mismatch: YAML={yaml_width}, Real={width}")
```

### 4. 자동화된 테스트

**전체 맵 순회 테스트:**

```python
# test_all_wall_distances.py
def test_all_work_tags():
    """모든 work zone 태그에서 벽 거리 확인"""

    work_tags = [104, 105, 106, 107, 108, 109, 110, 111]

    results = []

    for tag_id in work_tags:
        # Move and align
        robot.move_to_tag(tag_id, edge_info)
        robot.align_to_tag(tag_id)
        robot.stop_and_wait(1.0)

        # Get wall distance
        zone = robot.map_manager.tag_db.get_zone(tag_id)
        tag_data = robot.vision.get_tag_data(tag_id)
        lateral = tag_data['x']
        align_angle = robot.vision.get_alignment_angle(tag_id)

        wall_dist = robot.pp.calculate_wall_distance(lateral, align_angle, zone)

        results.append({
            'tag': tag_id,
            'zone': zone,
            'lateral': lateral,
            'wall_dist': wall_dist,
            'safe': wall_dist > 0.10
        })

        print(f"Tag {tag_id} (Zone {zone}): {wall_dist:.3f}m {'✓' if wall_dist > 0.10 else '⚠️ '}")

    # Summary
    unsafe_count = sum(1 for r in results if not r['safe'])
    print(f"\n=== Summary ===")
    print(f"Total tags tested: {len(results)}")
    print(f"Unsafe distances: {unsafe_count}")

    if unsafe_count > 0:
        print("\n⚠️  WARNING: Some tags have unsafe wall distances!")
        for r in results:
            if not r['safe']:
                print(f"  Tag {r['tag']}: {r['wall_dist']:.3f}m")

# 실행:
test_all_work_tags()

# 출력:
# Tag 104 (Zone B): 0.340m ✓
# Tag 105 (Zone B): 0.320m ✓
# Tag 106 (Zone C): 0.150m ✓
# Tag 107 (Zone C): 0.080m ⚠️
# ...
# === Summary ===
# Total tags tested: 8
# Unsafe distances: 1
# ⚠️  WARNING: Some tags have unsafe wall distances!
#   Tag 107: 0.080m
```

### 5. 로그 분석

**Mission 완료 후 분석:**

```bash
# 로그 파일 저장
roslaunch apriltag_navigation main.launch mode:=2 2>&1 | tee mission_log.txt

# 벽 거리만 추출
grep "WALL" mission_log.txt > wall_distances.txt

# 최소 거리 찾기
cat wall_distances.txt | grep -o "[0-9]\+\.[0-9]\+m" | sort -n | head -1
# 출력: 0.085m  # 위험!

# 위험 구간 찾기
grep "Too close" mission_log.txt
# 출력:
# [WALL] Too close! 0.085m < 0.10m (Tag 107)
```

### 6. Isaac Sim에서의 검증

**시뮬레이션 환경:**

```python
# Isaac Sim에서 로봇 위치와 벽 위치 정확히 확인 가능

# 1. 로봇을 Tag 104로 이동
# 2. Isaac Sim UI에서 로봇 위치 읽기: (x, y)
# 3. 벽 위치 읽기
# 4. 직접 계산: wall_dist = |robot_y - wall_y|
# 5. 시스템 계산값과 비교

# 예시:
Isaac robot position: (0.6275, 0.30)
Isaac wall position: y = 0.0
Real distance: |0.30 - 0.0| = 0.30m

System calculation: 0.32m
Error: 0.02m (2cm)  # 로봇 크기 보정 차이
```

**정리:**

- **정적 측정**: 레이저 거리계로 실측, 계산값과 비교
- **동적 모니터링**: 실시간 로그 및 RViz 시각화
- **캘리브레이션**: Base distance, 로봇 크기 검증
- **자동화 테스트**: 모든 태그 순회하며 안전 거리 확인
- **로그 분석**: Mission 후 최소 거리 및 위험 구간 식별
- **허용 오차**: ±5cm 이내 (태그 감지 + 로봇 크기 보정 오차)
- **위험 임계값**: 0.10m (10cm) 미만 시 경고

---

## 질문 22: Pure Pursuit는 태그가 보이는 환경에서만 각도 보정이 실행되는가?

**답변:**

**맞습니다.** Pure Pursuit의 각도 보정(angular velocity correction)은 **AprilTag가 보일 때만** 작동합니다.

### Pure Pursuit의 두 가지 모드

**1. Tag Visible Mode (태그 보임):**

```python
# robot_interface.py: move_to_tag()
if self.vision.is_tag_visible(target_tag):
    # AprilTag detection data available
    lateral, distance, align_angle = self.vision.get_tag_pose(target_tag)

    # Pure Pursuit with lateral offset correction
    if move_dir == 'forward':
        angular = self.pp.calculate_forward(lateral, distance)
    else:
        angular = self.pp.calculate_backward(lateral, distance)

    # angular ≠ 0: 경로 보정 실행
    self.robot.move(linear, angular)
```

- **Lateral offset** 기반으로 Pure Pursuit 계산
- **각도 보정** (angular velocity) 발생
- **실시간 경로 수정** 가능

**2. Straight Mode (태그 안 보임):**

```python
else:
    # Tag not visible - move straight
    angular = 0.0  # No correction!
    self.robot.move(linear, 0)
```

- **각도 보정 없음** (angular = 0)
- **직진만 수행**
- 이전 정렬 상태 유지하며 전진/후진

### 왜 이렇게 설계되었나?

**태그 안 보이는 구간의 특성:**

```
Zone A 긴 복도:
     500 --[1m]-- 400 --[1m]-- 401 --[1m]-- 402

500에서 출발:
  - 500 태그 보임: 정렬 후 출발 (align_to_tag)
  - 이동 중: 400 태그가 아직 멀어서 안 보임
  - 이 구간: angular = 0 (직진만)
  - 400 태그 보임: Pure Pursuit 재개
```

**설계 이유:**

1. **태그 없으면 lateral offset 계산 불가**
   - Pure Pursuit은 lateral 값이 필수
   - 태그 안 보이면 데이터가 없음

2. **Odometry로 위치 추정은 가능하지만 방향 보정은 위험**
   - Odometry drift로 부정확한 보정 가능
   - 잘못된 각도 보정 → 경로 이탈 악화

3. **정확한 출발 정렬이 중요한 이유**
   - 태그 안 보이는 구간은 직진만 하므로
   - 출발 시 정렬이 정확해야 목표 방향 유지

### 코드 흐름

**Move Edge 실행 시:**

```python
# State: MOVING
if self.robot.move_to_tag(target_tag, edge):
    # 태그 도착
    self.robot.state = NavigationState.STOPPING

# move_to_tag() 내부:
def move_to_tag(self, target_tag, edge_info):
    move_dir = edge_info['direction']  # 'forward' or 'backward'

    if self.vision.is_tag_visible(target_tag):
        # 태그 보임: Pure Pursuit
        lateral, distance, align_angle = self.vision.get_tag_pose(target_tag)
        angular = self.pp.calculate_forward(lateral, distance)  # ≠ 0

        if distance < arrival_threshold:
            return True  # 도착!
    else:
        # 태그 안 보임: 직진 모드
        angular = 0.0

        # Odometry 기반 위치 추정으로 도착 여부만 판단
        robot_x, robot_y, _ = self.robot.get_estimated_pose()
        target_x, target_y = self.map_manager.tag_db.get_position(target_tag)
        distance = sqrt((target_x - robot_x)**2 + (target_y - robot_y)**2)

        if distance < arrival_threshold:
            return True  # 도착 추정

    self.robot.move(linear, angular)
    return False
```

### 실전 예시

**시나리오: 500 → 402 이동 (Zone A, 2m 거리)**

```
1. 500 태그에서 정렬 완료 (align_to_tag)
   - Heading: 0° (정확히 +X 방향)
   - Lateral: 0.0m (정중앙)

2. 402 향해 출발 (move_to_tag)
   - is_tag_visible(402): False (멀어서 안 보임)
   - angular = 0.0
   - linear = 0.3 m/s (직진)

3. 0.5초 후 (15cm 이동):
   - is_tag_visible(402): False
   - angular = 0.0 (계속 직진)

4. 1.5초 후 (45cm 이동):
   - is_tag_visible(402): True! (감지 시작)
   - lateral = +0.02m (약간 우측 치우침)
   - angular = -0.05 rad/s (좌회전 보정)

5. 2.0초 후 (60cm 이동):
   - lateral = +0.01m
   - angular = -0.03 rad/s (계속 보정)

6. 도착:
   - distance < 0.05m
   - State → STOPPING
```

### 왜 Odometry로 각도 보정 안 하나?

**Odometry 사용 범위:**

```python
# robot_controller.py: get_estimated_pose()
def get_estimated_pose(self):
    """Odometry delta로 위치 추정"""
    odom_dx = self.odom_x - self.ref_odom_x
    odom_dy = self.odom_y - self.ref_odom_y

    robot_x = self.ref_robot_x + odom_dx
    robot_y = self.ref_robot_y + odom_dy

    # 위치만 사용, 각도 보정은 하지 않음
    return robot_x, robot_y, robot_heading
```

**Odometry 한계:**

- **위치 추정**: 도착 여부 판단용으로만 사용 (괜찮음)
- **각도 보정**: 누적 오차로 부정확 (위험함)

**결론:**

- **태그 보일 때**: lateral 기반 Pure Pursuit (정확함)
- **태그 안 보일 때**: 각도 보정 없이 직진 (안전함)
- **Odometry**: 도착 판단용으로만 사용

### 정리

**Pure Pursuit 각도 보정:**

- ✅ **태그 보임**: 실행 (lateral 기반)
- ❌ **태그 안 보임**: 실행 안 함 (angular = 0)

**태그 안 보이는 구간 대비:**

1. **정확한 정렬 후 출발** (align_to_tag)
2. **직진 유지** (angular = 0)
3. **Odometry로 도착 판단만** 수행
4. **태그 다시 보이면 Pure Pursuit 재개**

**이것이 바로 "정렬(ALIGNING)"이 중요한 이유입니다.**

---

## 질문 23: forward_pivot_cw / forward_pivot_ccw 엣지 direction의 의미는?

**답변:**

`forward_pivot_cw`와 같은 **복합 direction**은 **이동 방향 + 엣지 타입 + 회전 방향**을 하나의 문자열로 표현합니다.

### Direction 구조

**형식:**
```
[이동방향]_[엣지타입]_[회전방향]
```

**구성 요소:**

1. **이동 방향**: `forward` 또는 `backward`
2. **엣지 타입**: `pivot` (회전 포함) 또는 생략 (일반 이동)
3. **회전 방향**: `cw` (clockwise) 또는 `ccw` (counter-clockwise)

### 모든 Direction 종류

**1. 일반 Move Edge:**

```yaml
- from: 500
  to: 400
  direction: forward  # 단순 전진
  type: move

- from: 400
  to: 500
  direction: backward  # 단순 후진
  type: move
```

**2. Pivot Edge (회전 포함):**

```yaml
- from: 500
  to: 501
  direction: forward_pivot_cw  # 전진 후 시계방향 90도 회전
  type: pivot

- from: 500
  to: 501
  direction: turn_cw_90  # 제자리 시계방향 90도 회전
  type: pivot
```

### 실제 예시 (map.yaml 기준)

**예시 1: forward_pivot_cw**

```yaml
# Zone A → Zone B 전환
- from: 500  # Entry_LB (Zone A)
  to: 501    # Exit_LB (Zone B)
  direction: turn_cw_90
  type: pivot
```

**의미:**
- **이동**: 500에서 501로 (실제로는 같은 위치의 논리적 태그)
- **타입**: Pivot (회전 필요)
- **회전**: CW (시계방향) 90도

**실행 순서:**
```
1. 500 태그로 이동 (MOVING)
2. 500 태그에 정렬 (ALIGNING)
3. 90도 CW 회전 (ROTATING)
4. 회전 완료 후 다음 waypoint로 (IDLE)
```

**예시 2: backward**

```yaml
# Zone B 작업 구간
- from: 501
  to: 111
  direction: backward
  type: move
```

**의미:**
- **이동**: 501에서 111로
- **방향**: Backward (후진)
- **타입**: Move (회전 없음)

### Direction 파싱 로직

**코드에서 direction 처리:**

```python
# main_node.py: execute()
edge = self.map.nav_graph.get_edge(current_pos, target_tag)

if edge['type'] == 'pivot':
    # Pivot edge: 회전 방향 추출
    direction = 'ccw' if 'ccw' in edge['direction'] else 'cw'
    self.robot.rotate_90(direction)

elif edge['type'] == 'move':
    # Move edge: 이동 방향 추출
    move_dir = edge['direction']  # 'forward' or 'backward'
    self.robot.move_to_tag(target_tag, edge)
```

**Direction 문자열 검사:**

```python
# 'forward_pivot_cw'에서 정보 추출
if 'pivot' in edge['direction']:
    # Pivot edge
    if 'cw' in edge['direction']:
        rotation = 'cw'
    elif 'ccw' in edge['direction']:
        rotation = 'ccw'

    if 'forward' in edge['direction']:
        move_dir = 'forward'
    elif 'backward' in edge['direction']:
        move_dir = 'backward'
```

### 모든 Direction 조합

**실제 사용되는 Direction:**

1. **`forward`**: 전진 이동
2. **`backward`**: 후진 이동
3. **`turn_cw_90`**: 시계방향 90도 제자리 회전
4. **`turn_ccw_90`**: 반시계방향 90도 제자리 회전
5. **`forward_pivot_cw`**: 전진 후 시계방향 회전 (이론상, 실제 사용 확인 필요)
6. **`backward_pivot_ccw`**: 후진 후 반시계방향 회전 (이론상)

**현재 map.yaml에서 주로 사용:**

```yaml
edges:
  # 일반 이동
  - direction: forward
  - direction: backward

  # 회전
  - direction: turn_cw_90
  - direction: turn_ccw_90
```

### 실전 활용

**Zone 전환 시나리오:**

```yaml
# A → B 전환
- from: 500  # Zone A 마지막
  to: 501    # Zone B 시작
  direction: turn_cw_90
  type: pivot

# B → C 전환
- from: 503  # Zone B 마지막
  to: 407    # Zone C 시작
  direction: turn_ccw_90
  type: pivot
```

**실행:**

```
500 도착:
  1. move_to_tag(500) - 전진
  2. align_to_tag(500) - 정렬
  3. rotate_90('cw') - 시계방향 회전
  4. 현재 위치: 501 (논리적)

다음:
  - 501 → 111: backward로 이동
```

### 정리

**Direction 문법:**

| Direction | 이동 | 타입 | 회전 | 예시 |
|-----------|------|------|------|------|
| `forward` | 전진 | move | - | 500→400 |
| `backward` | 후진 | move | - | 501→111 |
| `turn_cw_90` | - | pivot | CW 90° | 500→501 |
| `turn_ccw_90` | - | pivot | CCW 90° | 503→407 |
| `forward_pivot_cw` | 전진 | pivot | CW 90° | (이론상) |

**핵심:**

- **forward/backward**: 이동 방향
- **pivot**: 회전 포함 여부
- **cw/ccw**: 회전 방향 (Clockwise/Counter-Clockwise)

---

## 질문 24: Odometry가 SLAM 기준일 때 위치값 문제는 없나?

**답변:**

**당신의 생각이 부분적으로 맞습니다.** 하지만 완전히 문제없다고는 할 수 없습니다.

### 당신의 제안 분석

**제안 1: Offset 0으로 초기화**

```python
# 가정: SLAM odom이 SLAM 시작점 (0, 0)에서 시작
# Isaac Sim: Dock at (-0.3975, 5.35)

# Reference point 설정 시:
self.ref_odom_x = 0.0      # SLAM 시작점
self.ref_odom_y = 0.0
self.ref_robot_x = -0.3975  # Isaac Sim 좌표
self.ref_robot_y = 5.35

# Delta 계산:
odom_dx = current_odom_x - 0.0 = current_odom_x
odom_dy = current_odom_y - 0.0 = current_odom_y

robot_x = -0.3975 + odom_dx
robot_y = 5.35 + odom_dy
```

**문제점:**

- ✅ **초기 reference 설정은 가능**
- ❌ **좌표계 정렬 문제**: SLAM odom과 Isaac Sim의 축 방향이 다를 수 있음
  - SLAM: +X가 로봇 전방
  - Isaac Sim: +X가 세계 좌표 동쪽

**예시:**

```
SLAM 좌표계 (로봇 중심):
      +X (전방)
        ↑
        |
   +Y ←-•

Isaac Sim 좌표계 (세계 고정):
        +Y
         ↑
         |
         •--→ +X

→ 좌표계 변환 필요!
```

**제안 2: 선속도/각속도만 사용, 위치는 우리가 생성**

```python
# Odometry에서 속도만 사용
linear_vel = odom_msg.twist.twist.linear.x
angular_vel = odom_msg.twist.twist.angular.z

# 위치는 AprilTag 기반으로 생성
robot_x, robot_y, heading = get_robot_pose_from_tag(tag_id, lateral, align_angle)
```

**장점:**

- ✅ **좌표계 문제 해결**: 위치는 AprilTag 기준으로만 계산
- ✅ **단순함**: 속도만 사용하므로 누적 오차 없음

**문제점:**

- ❌ **태그 안 보일 때 위치 모름**: 속도만으로는 위치 계산 불가
- ❌ **도착 판단 불가**: 태그 안 보이는 구간에서 언제 멈춰야 할지 모름

### 현재 시스템 접근법 (혼합 방식)

**AprilTag + Odometry Delta:**

```python
# robot_controller.py
def set_reference_point(self, robot_x, robot_y, robot_heading):
    """태그 보일 때: AprilTag 기반 정확한 위치로 reference 업데이트"""
    self.ref_robot_x = robot_x      # AprilTag 기반
    self.ref_robot_y = robot_y
    self.ref_robot_heading = robot_heading
    self.ref_odom_x = self.odom_x   # 현재 odom 저장
    self.ref_odom_y = self.odom_y
    self.ref_odom_theta = self.current_theta

def get_estimated_pose(self):
    """태그 안 보일 때: Odometry delta로 위치 추정"""
    odom_dx = self.odom_x - self.ref_odom_x
    odom_dy = self.odom_y - self.ref_odom_y

    # Delta를 Isaac Sim 좌표에 더함
    robot_x = self.ref_robot_x + odom_dx
    robot_y = self.ref_robot_y + odom_dy

    return robot_x, robot_y, robot_heading
```

**핵심 가정:**

```python
# CRITICAL ASSUMPTION (robot_controller.py:122)
# "Odom is global and aligned with Isaac Sim at start"
robot_x = self.ref_robot_x + odom_dx
robot_y = self.ref_robot_y + odom_dy
```

**이 가정이 맞으려면:**

1. **Odometry 좌표계 = Isaac Sim 좌표계**
   - +X 방향이 같아야 함
   - +Y 방향이 같아야 함

2. **프로그램 시작 시 정렬 필요**
   - Robot heading = 0° (Isaac Sim +X 방향)
   - Odometry도 같은 방향을 0°로 설정

### SLAM Odometry 사용 시 문제

**SLAM의 특성:**

```
SLAM 시작 시:
  - (0, 0, 0): SLAM 시작 위치 및 방향
  - +X: 로봇이 보는 방향 (임의)
  - 세계 좌표와 무관!

Isaac Sim:
  - Dock: (-0.3975, 5.35)
  - +X: 세계 동쪽 (고정)
  - 로봇 초기 heading: 0° (+X 방향)
```

**좌표계 불일치:**

```
Case 1: SLAM 시작 시 로봇이 북쪽(+Y)을 향함
  SLAM +X = Isaac Sim +Y
  SLAM +Y = Isaac Sim -X

  odom_dx = 1.0 (SLAM +X로 1m 이동)
  → Isaac Sim에서는 +Y로 1m 이동!
  → 하지만 코드는 +X로 1m로 계산
  → 오류!
```

### 해결 방법

**방법 1: Odometry 좌표계 변환 (권장)**

```python
def get_estimated_pose(self):
    """Odometry delta를 Isaac Sim 좌표계로 변환"""
    # Odometry delta (SLAM 좌표계)
    odom_dx_slam = self.odom_x - self.ref_odom_x
    odom_dy_slam = self.odom_y - self.ref_odom_y

    # Reference 시점의 heading으로 회전 변환
    cos_h = math.cos(self.ref_robot_heading)
    sin_h = math.sin(self.ref_robot_heading)

    # SLAM → Isaac Sim 좌표계 변환
    odom_dx_isaac = odom_dx_slam * cos_h - odom_dy_slam * sin_h
    odom_dy_isaac = odom_dx_slam * sin_h + odom_dy_slam * cos_h

    # Isaac Sim 좌표에 더함
    robot_x = self.ref_robot_x + odom_dx_isaac
    robot_y = self.ref_robot_y + odom_dy_isaac

    return robot_x, robot_y, robot_heading
```

**방법 2: 속도 적분 (덜 권장)**

```python
# Odometry에서 속도만 사용
def update_position_from_velocity(self, dt):
    """속도를 적분하여 위치 업데이트"""
    linear_vel = self.get_linear_velocity()
    angular_vel = self.get_angular_velocity()

    # 현재 heading 기준으로 위치 변화 계산
    dx = linear_vel * math.cos(self.robot_heading) * dt
    dy = linear_vel * math.sin(self.robot_heading) * dt

    self.robot_x += dx
    self.robot_y += dy
    self.robot_heading += angular_vel * dt
```

**문제점:**

- 누적 오차
- dt 계산 부정확성
- 미끄러짐(slip) 고려 안 됨

**방법 3: AprilTag만 사용 (가장 안전)**

```python
# 태그 안 보이는 구간: 추정 없이 직진만
def move_to_tag(self, target_tag):
    if self.vision.is_tag_visible(target_tag):
        # 태그 보임: 정확한 제어
        lateral, distance = self.vision.get_tag_pose(target_tag)
        angular = self.pp.calculate_forward(lateral, distance)

        if distance < 0.05:
            return True  # 도착
    else:
        # 태그 안 보임: 직진만, 도착 판단 안 함
        angular = 0.0

    self.robot.move(linear, angular)
    return False
```

### 당신의 제안 재평가

**"Odom에서 선속도/각속도만 쓰고 위치는 우리가 생성"**

- ✅ **맞는 접근**: 좌표계 혼란 피함
- ❌ **실용성 문제**: 태그 안 보일 때 도착 판단 못 함

**"Offset 0으로 초기화"**

- ⚠️ **부분적 해결**: 초기화는 가능하지만
- ❌ **좌표계 정렬 필요**: SLAM과 Isaac Sim 축 방향 일치 필수

### 최종 답변

**당신의 생각:**

> "가상으로 odom 실행, offset 0 초기화 또는 선속도/각속도만 사용, 위치는 우리가 생성 → 문제없지 않나?"

**평가:**

1. **Offset 0 초기화**:
   - ✅ 가능하지만
   - ❌ **좌표계 정렬 필수** (SLAM +X = Isaac +X, SLAM +Y = Isaac +Y)
   - 정렬 안 되면 delta 계산이 틀림

2. **선속도/각속도만 사용**:
   - ✅ 좌표계 문제 피함
   - ❌ **태그 안 보일 때 도착 판단 불가**
   - 실용성 떨어짐

3. **위치는 우리가 생성**:
   - ✅ AprilTag 기반은 정확함
   - ❌ 태그 안 보이는 구간 대비 필요

**권장 방법:**

```python
# 현재 코드 개선: Odometry 좌표계 변환 추가
def get_estimated_pose(self):
    """Odometry delta를 로봇 heading 고려하여 변환"""
    odom_dx_local = self.odom_x - self.ref_odom_x
    odom_dy_local = self.odom_y - self.ref_odom_y

    # Reference heading으로 회전 변환
    cos_h = math.cos(self.ref_robot_heading)
    sin_h = math.sin(self.ref_robot_heading)

    odom_dx_world = odom_dx_local * cos_h - odom_dy_local * sin_h
    odom_dy_world = odom_dx_local * sin_h + odom_dy_local * cos_h

    robot_x = self.ref_robot_x + odom_dx_world
    robot_y = self.ref_robot_y + odom_dy_world

    return robot_x, robot_y, robot_heading
```

**정리:**

- **현재 코드 가정**: "Odom is global and aligned with Isaac Sim"
- **SLAM 사용 시**: 가정 위반 가능성 높음 → **좌표계 변환 필요**
- **당신의 제안**: 부분적으로 맞지만, **좌표계 정렬 문제 간과**
- **최선의 방법**: AprilTag primary, Odometry는 보조 (좌표계 변환 포함)

---

## 마무리

이 Q&A 문서는 AprilTag Navigation 시스템의 24가지 주요 질문에 대한 상세한 답변을 담고 있습니다.

각 질문은 다음 내용을 포함합니다:
- 개념 설명
- 코드 예시
- 실제 사용 시나리오
- 현장 테스트 방법
- 주의사항 및 권장사항

추가 질문이나 문의사항이 있으시면 TEST.md 문서와 함께 참고하시기 바랍니다.

