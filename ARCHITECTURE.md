# AprilTag Navigation Package - Architecture Documentation

## 📋 목차
1. [개요](#개요)
2. [모듈화 설계 철학](#모듈화-설계-철학)
3. [패키지 구조](#패키지-구조)
4. [계층별 설명](#계층별-설명)
5. [데이터 흐름](#데이터-흐름)
6. [설계 원칙](#설계-원칙)
7. [사용 예시](#사용-예시)

---

## 개요

이 패키지는 **AprilTag 기반 실내 네비게이션 시스템**으로, 단일 파일의 복잡한 코드를 **모듈화되고 유지보수 가능한 구조**로 리팩토링한 결과물입니다.

### 핵심 목표
- ✅ **관심사의 분리 (Separation of Concerns)**: 각 모듈이 명확한 단일 책임을 가짐
- ✅ **높은 가독성**: Pseudocode처럼 읽히는 코드
- ✅ **확장성**: 새로운 기능 추가가 쉬움
- ✅ **테스트 용이성**: 각 모듈을 독립적으로 테스트 가능

---

## 모듈화 설계 철학

### Before: 단일 파일 구조 (Monolithic)
```
single_navigation_script.py (1000+ lines)
├── AprilTag 감지 로직
├── 지도 관리 로직
├── 경로 계획 로직
├── 하드웨어 제어 로직
├── 주행 제어 로직
└── 미션 관리 로직
```

**문제점:**
- 코드 이해가 어려움 (1000줄 이상의 단일 파일)
- 버그 수정 시 다른 부분에 영향을 줄 위험
- 테스트가 어려움
- 팀 협업 시 충돌 발생 가능성 높음

### After: 계층적 모듈 구조 (Modular)
```
apriltag_navigation/
├── nodes/main_node.py          # 최상위: 미션 조율
├── src/apriltag_navigation/
│   ├── robot_interface.py      # High-level API
│   ├── perception/             # 인식 계층
│   ├── map/                    # 지도 계층
│   ├── navigation/             # 주행 계층
│   └── hardware/               # 하드웨어 계층
```

**장점:**
- 각 파일이 200~400줄로 관리 가능한 크기
- 명확한 책임 분리로 버그 추적 용이
- 모듈별 독립 테스트 가능
- 팀원들이 각 모듈을 독립적으로 작업 가능

---

## 패키지 구조

```
apriltag_navigation/
│
├── nodes/                          # ROS 노드 실행 파일
│   └── main_node.py               # 메인 네비게이션 노드 (미션 제어)
│
├── scripts/                        # 유틸리티 스크립트
│   └── diagnostics.py             # 하드웨어 진단 도구
│
├── src/apriltag_navigation/       # 핵심 패키지 소스
│   │
│   ├── __init__.py
│   │
│   ├── robot_interface.py         # 🎯 High-Level Robot API
│   │                              #    (모든 하위 모듈을 통합)
│   │
│   ├── perception/                # 👁️ 인식 계층
│   │   ├── __init__.py
│   │   └── vision_module.py       # AprilTag 감지 및 거리/위치 계산
│   │
│   ├── map/                       # 🗺️ 지도 계층
│   │   ├── __init__.py
│   │   └── map_manager.py         # 맵 로딩, 그래프 관리, 작업 관리
│   │
│   ├── navigation/                # 🚗 주행 계층
│   │   ├── __init__.py
│   │   └── pure_pursuit.py        # Pure Pursuit 알고리즘
│   │
│   └── hardware/                  # ⚙️ 하드웨어 계층
│       ├── __init__.py
│       └── robot_controller.py    # 모터 제어, 오도메트리
│
├── config/                        # 설정 파일
│   └── map.yaml                   # 맵 정의, 태그 좌표, 작업 정의
│
├── data/                          # 데이터 파일
│   └── excel/                     # Mode 4용 Excel 파일 저장
│       └── README.md
│
├── launch/                        # ROS Launch 파일
│   └── navigation.launch
│
├── CMakeLists.txt                 # ROS 빌드 설정
├── package.xml                    # ROS 패키지 메타데이터
├── setup.py                       # Python 패키지 설정
└── README.md                      # 사용자 가이드
```

---

## 계층별 설명

### 1️⃣ **메인 노드 계층** (`nodes/main_node.py`)

**역할:** 미션 조율 및 상태 관리

```python
# Pseudocode 스타일의 가독성 높은 코드
class NavigationMission:
    def execute(self):
        if self.waypoint_index >= len(self.waypoints):
            return MISSION_COMPLETE

        edge = self.map.get_edge(current_pos, next_waypoint)

        if edge.type == 'pivot':
            self.robot.rotate_90(direction)
        elif edge.type == 'move':
            self.robot.move_to_tag(target_tag)
```

**특징:**
- 복잡한 로직은 모두 하위 모듈로 위임
- 영어 문장처럼 읽히는 코드
- 네비게이션 전체 흐름을 한눈에 파악 가능

**주요 기능:**
- 모드 선택 (Task 1/2, Direct Navigation, Scan Mode)
- 미션 로딩 및 실행
- 웨이포인트 관리
- 통합 테스트 모드 (`--test-vision`, `--test-motor` 등)

---

### 2️⃣ **로봇 인터페이스 계층** (`robot_interface.py`)

**역할:** High-level API - 모든 하위 모듈을 통합

```python
class RobotInterface:
    def __init__(self):
        self.map_manager = MapManager()
        self.vision = VisionModule()
        self.controller = RobotController(self.map_manager)
        self.navigation = PurePursuit(self.controller, self.map_manager)

    def move_to_tag(self, tag_id, edge):
        """태그로 이동 - 내부적으로 비전, 네비게이션, 제어 통합"""
        detected_tags = self.vision.get_detected_tags()
        arrived = self.navigation.navigate_to_tag(tag_id, detected_tags, edge)
        return arrived
```

**특징:**
- main_node.py가 하위 모듈을 직접 다루지 않도록 추상화
- 복잡한 상태 관리 및 조율 로직 처리
- 상태 머신 구현 (IDLE, MOVING, STOPPING, ALIGNING, WAIT_SCAN)

**주요 메서드:**
- `move_to_tag()`: 특정 태그로 이동
- `rotate_90()`: 90도 회전
- `align_to_tag()`: 태그에 정렬
- `wait_for_scan()`: 스캔 대기

---

### 3️⃣ **인식 계층** (`perception/vision_module.py`)

**역할:** AprilTag 감지 및 3D 위치 계산

```python
class VisionModule:
    def __init__(self):
        self.bridge = CvBridge()
        self.detector = Detector(...)  # AprilTag 감지기

    def get_detected_tags(self):
        """현재 보이는 모든 태그의 거리/방향 정보 반환"""
        detections = self.detector.detect(gray_image)

        for detection in detections:
            pose = self._estimate_pose(detection)
            tags[tag_id] = {
                'x': pose[0],  # 좌우 거리
                'z': pose[2],  # 전방 거리
                'distance': distance
            }
        return tags
```

**특징:**
- ROS 이미지 토픽 구독
- AprilTag 라이브러리를 사용한 감지
- 카메라 보정 정보 기반 3D 위치 추정
- 실시간 태그 정보 제공

**출력 데이터:**
```python
{
    5: {'x': -0.15, 'z': 2.3, 'distance': 2.31},
    7: {'x': 0.02, 'z': 3.5, 'distance': 3.50}
}
```

---

### 4️⃣ **지도 계층** (`map/map_manager.py`)

**역할:** 맵 데이터 관리 및 경로 계획

```python
class MapManager:
    def __init__(self):
        self.nav_graph = NavigationGraph()    # 그래프 기반 경로 계획
        self.task_manager = TaskManager()      # 작업 정의 관리

class NavigationGraph:
    def find_path(self, start, goal):
        """BFS를 사용한 최단 경로 탐색"""
        return bfs_shortest_path(start, goal)

    def get_edge(self, from_tag, to_tag):
        """두 태그 사이의 이동 방법 반환"""
        return {'type': 'move', 'direction': 'forward', ...}
```

**주요 구성요소:**

1. **NavigationGraph**: 그래프 기반 경로 계획
   - 노드: AprilTag ID
   - 엣지: 이동 방법 (`move`, `pivot`)
   - BFS 알고리즘으로 최단 경로 탐색

2. **TaskManager**: 사전 정의된 작업 관리
   - Task 1: Zone B + C 순회
   - Task 2: Zone D + E 순회
   - Excel 기반 스캔 작업

3. **맵 데이터 (YAML):**
```yaml
tags:
  0: {x: 0.0, y: 0.0}
  5: {x: 2.0, y: 0.0}

edges:
  - {from: 0, to: 5, type: move, direction: forward}
  - {from: 5, to: 7, type: pivot, direction: ccw}

tasks:
  task1:
    waypoints: [0, 5, 7, 9]
```

---

### 5️⃣ **주행 계층** (`navigation/pure_pursuit.py`)

**역할:** Pure Pursuit 알고리즘 기반 주행 제어

```python
class PurePursuit:
    def navigate_to_tag(self, target_tag, detected_tags, edge):
        """Pure Pursuit으로 태그 추적"""

        # 1. 목표 태그 감지 확인
        if target_tag not in detected_tags:
            return self._search_for_tag(edge)

        # 2. Pure Pursuit 제어
        target_x = detected_tags[target_tag]['x']
        target_z = detected_tags[target_tag]['z']

        linear_vel, angular_vel = self._compute_velocities(target_x, target_z)
        self.robot.move(linear_vel, angular_vel)

        # 3. 도착 판단
        return self._check_arrival(target_z)
```

**Pure Pursuit 알고리즘:**
- **Look-ahead distance**: 목표 지점을 일정 거리 앞으로 설정
- **각속도 계산**: 목표와의 각도 차이에 비례
- **속도 제어**: 거리에 따라 감속

**특징:**
- 부드러운 곡선 주행
- 태그 추적 시 자동 보정
- 도착 판단 로직 내장

---

### 6️⃣ **하드웨어 계층** (`hardware/robot_controller.py`)

**역할:** 저수준 하드웨어 제어

```python
class RobotController:
    def __init__(self, map_manager):
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self._odom_callback)

    def move(self, linear, angular):
        """모터에 속도 명령 전송"""
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)

    def get_position(self):
        """현재 로봇 위치 반환"""
        return (self.current_x, self.current_y)
```

**주요 기능:**
1. **모터 제어**: `/cmd_vel` 토픽으로 속도 명령
2. **오도메트리**: `/odom` 토픽에서 위치/자세 수신
3. **회전 제어**: `RotationController`로 정밀한 각도 제어
4. **안전 기능**: 비상 정지, 속도 제한

---

## 데이터 흐름

```
┌─────────────────────────────────────────────────────────────┐
│                      Main Node (Mission)                     │
│  - 모드 선택 (Task 1/2/3/4)                                  │
│  - 미션 실행 루프                                             │
└─────────────────┬───────────────────────────────────────────┘
                  │
                  ↓
┌─────────────────────────────────────────────────────────────┐
│                    Robot Interface                           │
│  - move_to_tag(), rotate_90(), align_to_tag()                │
│  - 상태 머신 관리                                             │
└───┬──────────┬──────────┬──────────┬─────────────────────────┘
    │          │          │          │
    ↓          ↓          ↓          ↓
┌────────┐ ┌────────┐ ┌─────────┐ ┌──────────┐
│ Vision │ │  Map   │ │   Nav   │ │ Hardware │
│ Module │ │Manager │ │ (Pure   │ │Controller│
│        │ │        │ │ Pursuit)│ │          │
└────────┘ └────────┘ └─────────┘ └──────────┘
    │          │          │          │
    ↓          ↓          ↓          ↓
┌────────┐ ┌────────┐ ┌─────────┐ ┌──────────┐
│/camera │ │map.yaml│ │Algorithm│ │ /cmd_vel │
│ /image │ │        │ │  Logic  │ │  /odom   │
└────────┘ └────────┘ └─────────┘ └──────────┘
```

### 실행 흐름 예시 (Mode 1 실행)

```
1. main_node.py
   └─> mission.load_task('task1')
       └─> map_manager.task_manager.get_task_waypoints('task1')
           └─> 반환: [0, 5, 7, 9, 11, ...]

2. mission.execute()
   └─> edge = map.nav_graph.get_edge(0, 5)
       └─> 반환: {type: 'move', direction: 'forward'}

   └─> robot.move_to_tag(5, edge)
       └─> detected_tags = vision.get_detected_tags()
           └─> AprilTag 감지: {5: {x: -0.1, z: 2.3}}

       └─> navigation.navigate_to_tag(5, detected_tags, edge)
           └─> linear, angular = pure_pursuit_algorithm()
           └─> controller.move(linear, angular)
               └─> publish to /cmd_vel

3. 도착 판단 → 다음 웨이포인트로 이동
```

---

## 설계 원칙

### 1. **단일 책임 원칙 (Single Responsibility Principle)**
- 각 클래스는 하나의 명확한 책임만 가짐
- 예: `VisionModule`은 오직 태그 감지만 담당

### 2. **의존성 주입 (Dependency Injection)**
```python
class RobotInterface:
    def __init__(self):
        self.vision = VisionModule()        # 의존성 주입
        self.controller = RobotController()
        self.navigation = PurePursuit(self.controller)
```

### 3. **계층 분리 (Layer Separation)**
- High-level: 미션 로직 (main_node.py)
- Mid-level: 통합 인터페이스 (robot_interface.py)
- Low-level: 개별 기능 모듈 (perception, map, navigation, hardware)

### 4. **설정 외부화 (Configuration Externalization)**
- 하드코딩 대신 YAML 파일 사용
- 맵 변경 시 코드 수정 불필요

### 5. **가독성 우선 (Readability First)**
```python
# Bad (before)
if not (self.state == 3 and self.dist < 0.5):
    self.robot.cmd(0.3, 0.1)

# Good (after)
if not self.robot.has_arrived():
    self.robot.move_to_tag(target_tag)
```

### 6. **테스트 용이성 (Testability)**
- 각 모듈을 독립적으로 테스트 가능
- `diagnostics.py`: 하드웨어 테스트
- `main_node.py --test-vision`: 통합 테스트

---

## 사용 예시

### 기본 실행 (인터랙티브 모드)
```bash
rosrun apriltag_navigation main_node.py
# 1: Task 1 (Zone B + C)
# 2: Task 2 (Zone D + E)
# 3: Direct Navigation (Go to specific tag)
# 4: Scan Mode (from Excel)
```

### 커맨드라인 모드
```bash
# Task 1 실행
rosrun apriltag_navigation main_node.py --mode 1

# 특정 태그로 직접 이동 (Mode 3)
rosrun apriltag_navigation main_node.py --mode 3 --tag 5

# Excel 파일로 스캔 모드 (Mode 4)
rosrun apriltag_navigation main_node.py --mode 4 --excel /path/to/scan.xlsx
```

### 테스트/진단 모드
```bash
# 비전 시스템 테스트 (10초)
rosrun apriltag_navigation main_node.py --test-vision --duration 10

# 모터 테스트 (2미터 전진)
rosrun apriltag_navigation main_node.py --test-motor --distance 2.0

# 회전 테스트 (90도 시계방향)
rosrun apriltag_navigation main_node.py --test-pivot --direction cw
```

### 독립적인 하드웨어 진단
```bash
# 상세한 진단 도구
python scripts/diagnostics.py --test-all
python scripts/diagnostics.py --test-motor --distance 3.0 --direction forward
python scripts/diagnostics.py --test-pivot --angle 180 --direction ccw
```

---

## 확장 가능성

### 새로운 주행 알고리즘 추가
```python
# navigation/dwa.py (Dynamic Window Approach)
class DWA:
    def navigate_to_tag(self, target_tag, detected_tags, edge):
        # DWA 알고리즘 구현
        pass

# robot_interface.py에서 교체
self.navigation = DWA(self.controller, self.map_manager)
```

### 새로운 센서 추가
```python
# perception/lidar_module.py
class LidarModule:
    def get_obstacles(self):
        # 장애물 감지
        pass

# robot_interface.py에 통합
self.lidar = LidarModule()
```

### 새로운 작업 모드 추가
```python
# main_node.py
def select_mode():
    print("5: Patrol Mode (Auto loop)")
    ...
    elif choice == '5':
        return 5, None

# NavigationMission에 추가
def load_patrol_task(self):
    self.waypoints = self.map.task_manager.get_patrol_waypoints()
```

---

## 요약

| 항목 | Before (Monolithic) | After (Modular) |
|------|---------------------|-----------------|
| **파일 수** | 1개 (1000+ 줄) | 10개 (각 200~400줄) |
| **가독성** | 낮음 | 높음 (Pseudocode 스타일) |
| **유지보수** | 어려움 | 쉬움 (명확한 책임 분리) |
| **테스트** | 전체 테스트만 가능 | 모듈별 독립 테스트 가능 |
| **확장성** | 낮음 (코드 전체 수정) | 높음 (모듈 교체/추가) |
| **협업** | 충돌 위험 높음 | 모듈별 독립 작업 가능 |

이 설계를 통해 **복잡한 로봇 네비게이션 시스템**을 **유지보수 가능하고 확장 가능한 구조**로 만들었습니다.
