# API Reference - AprilTag Navigation

패키지의 주요 함수와 메서드를 계층별로 정리한 레퍼런스입니다.

---

## 목차
1. [High-Level API (RobotInterface)](#high-level-api-robotinterface)
2. [Vision Module API](#vision-module-api)
3. [Map Manager API](#map-manager-api)
4. [Navigation API](#navigation-api)
5. [Hardware Controller API](#hardware-controller-api)
6. [Mission Control API](#mission-control-api)
7. [UI Helper API](#ui-helper-api)
8. [Test Helper API](#test-helper-api)

---

## High-Level API (RobotInterface)

**위치:** `src/apriltag_navigation/robot_interface.py`

가장 많이 사용하는 통합 인터페이스입니다.

### 초기화
```python
robot = RobotInterface()
```

### 시스템 준비
```python
robot.wait_until_ready()
# Returns: True if all systems ready, False on timeout
# 카메라, 오도메트리 등 모든 시스템이 준비될 때까지 대기
```

### 이동 관련
```python
robot.move_to_tag(tag_id, edge)
# Args:
#   tag_id: 목표 AprilTag ID (int)
#   edge: 이동 방법 정보 (dict)
# Returns: True if arrived, False if still moving
# Pure Pursuit 알고리즘으로 태그까지 이동

robot.rotate_90(direction)
# Args:
#   direction: 'cw' (시계방향) or 'ccw' (반시계방향)
# Returns: True if rotation complete
# 90도 회전 수행

robot.align_to_tag(tag_id)
# Args:
#   tag_id: 정렬할 태그 ID
# Returns: True if aligned
# 태그와 정렬 (정면으로 향하도록)

robot.stop()
# 로봇 정지

robot.stop_and_wait()
# Returns: True after stabilization
# 정지 후 안정화까지 대기
```

### 위치/상태 조회
```python
robot.get_current_position()
# Returns: 현재 위치 (태그 ID)

robot.get_state()
# Returns: NavigationState enum
# States: IDLE, MOVING, STOPPING, ALIGNING, WAIT_SCAN

robot.set_current_position(tag_id)
# Args:
#   tag_id: 설정할 위치
```

### 스캔 모드 (Mode 4)
```python
robot.enable_scan_mode()
# 스캔 모드 활성화

robot.wait_for_scan(tag_id)
# Args:
#   tag_id: 스캔 위치
# Returns: True when scan complete
# 사용자가 스캔 완료 신호를 보낼 때까지 대기
```

### 디버깅
```python
robot.publish_debug_status(current_wp, next_wp, total_wps, wp_idx)
# 디버그 정보를 ROS 토픽으로 발행
```

---

## Vision Module API

**위치:** `src/apriltag_navigation/perception/vision_module.py`

AprilTag 감지 및 위치 추정 기능

### 초기화
```python
from apriltag_navigation.perception.vision_module import VisionModule
vision = VisionModule()
```

### 태그 감지
```python
vision.get_detected_tags()
# Returns: dict
# {
#     tag_id: {
#         'x': 좌우 거리 (m),
#         'z': 전방 거리 (m),
#         'distance': 직선 거리 (m)
#     }
# }
# 현재 카메라에 보이는 모든 AprilTag 정보 반환

vision.is_ready()
# Returns: True if camera is receiving images
# 카메라가 준비되었는지 확인
```

### 예시
```python
tags = vision.get_detected_tags()
if 5 in tags:
    print(f"Tag 5: {tags[5]['z']:.2f}m 전방, {tags[5]['x']:.2f}m 측면")
```

---

## Map Manager API

**위치:** `src/apriltag_navigation/map/map_manager.py`

맵 데이터 관리 및 경로 계획

### 초기화
```python
from apriltag_navigation.map.map_manager import MapManager
map_manager = MapManager()
```

### 파라미터 조회
```python
map_manager.get_param(param_name, default_value=None)
# Args:
#   param_name: 파라미터 이름 (예: 'speeds.linear')
# Returns: 파라미터 값
# 맵 설정에서 파라미터 가져오기

# 예시
linear_speed = map_manager.get_param('speeds.linear', 0.3)
```

### 태그 정보
```python
map_manager.get_tag_position(tag_id)
# Args:
#   tag_id: 태그 ID
# Returns: (x, y) 좌표

map_manager.get_all_tags()
# Returns: dict - 모든 태그 정보
```

### NavigationGraph - 경로 계획
```python
map_manager.nav_graph.find_path(start_tag, goal_tag)
# Args:
#   start_tag: 시작 태그 ID
#   goal_tag: 목표 태그 ID
# Returns: list - 경로 상의 태그 ID 리스트
# BFS 알고리즘으로 최단 경로 탐색

map_manager.nav_graph.get_edge(from_tag, to_tag)
# Args:
#   from_tag: 출발 태그
#   to_tag: 도착 태그
# Returns: dict
# {
#     'type': 'move' or 'pivot',
#     'direction': 'forward', 'backward', 'cw', 'ccw'
# }
```

### TaskManager - 작업 관리
```python
map_manager.task_manager.get_task_waypoints(task_name)
# Args:
#   task_name: 'task1' or 'task2'
# Returns: list - 웨이포인트 리스트

map_manager.task_manager.get_excel_scan_waypoints(excel_path)
# Args:
#   excel_path: Excel 파일 경로
# Returns: (waypoints, scan_tags)
#   waypoints: 이동 경로
#   scan_tags: 스캔할 태그 리스트
```

---

## Navigation API

**위치:** `src/apriltag_navigation/navigation/pure_pursuit.py`

Pure Pursuit 주행 알고리즘

### 초기화
```python
from apriltag_navigation.navigation.pure_pursuit import PurePursuit
navigation = PurePursuit(robot_controller, map_manager)
```

### 주행
```python
navigation.navigate_to_tag(target_tag, detected_tags, edge)
# Args:
#   target_tag: 목표 태그 ID
#   detected_tags: vision.get_detected_tags() 결과
#   edge: 이동 방법 정보
# Returns: True if arrived, False if still navigating
# Pure Pursuit 알고리즘으로 목표까지 주행
```

---

## Hardware Controller API

**위치:** `src/apriltag_navigation/hardware/robot_controller.py`

하드웨어 제어 (모터, 센서)

### 초기화
```python
from apriltag_navigation.hardware.robot_controller import RobotController
controller = RobotController(map_manager)
```

### 모터 제어
```python
controller.move(linear_velocity, angular_velocity)
# Args:
#   linear_velocity: 전진 속도 (m/s)
#   angular_velocity: 회전 속도 (rad/s)
# /cmd_vel 토픽으로 속도 명령 전송

controller.stop()
# 모터 정지
```

### 위치/자세 조회
```python
controller.get_position()
# Returns: (x, y) - 현재 위치 (m)

controller.get_heading()
# Returns: 현재 방향 (rad)

controller.is_ready()
# Returns: True if odometry is being received
# 오도메트리가 수신되고 있는지 확인
```

### RotationController - 회전 제어
```python
from apriltag_navigation.hardware.robot_controller import RotationController
rotation_ctrl = RotationController(robot_controller)

rotation_ctrl.start_rotation(angle, direction)
# Args:
#   angle: 회전 각도 (degrees)
#   direction: 'cw' or 'ccw'
# 회전 시작

rotation_ctrl.update()
# Returns: True if rotation complete
# 회전 제어 루프 (30Hz로 호출)

rotation_ctrl.cancel()
# 회전 취소
```

---

## Mission Control API

**위치:** `nodes/main_node.py`

미션 실행 및 관리

### NavigationMission 클래스
```python
mission = NavigationMission(robot, map_manager)
```

### 미션 로딩
```python
mission.load_task(task_name)
# Args:
#   task_name: 'task1' or 'task2'
# 사전 정의된 작업 로딩

mission.load_direct_navigation(target_tag)
# Args:
#   target_tag: 목표 태그 ID
# Mode 3: 특정 태그로 직접 이동

mission.load_scan_task(excel_path)
# Args:
#   excel_path: Excel 파일 경로
# Mode 4: Excel 기반 스캔 작업
```

### 미션 실행
```python
mission.execute()
# Returns: True if mission complete
# 미션 한 스텝 실행 (30Hz 루프에서 호출)
```

---

## UI Helper API

**위치:** `src/apriltag_navigation/ui_helper.py`

사용자 인터페이스 헬퍼

```python
from apriltag_navigation.ui_helper import *

# 모드 선택
mode, param = select_mode()
# Returns: (mode, param)
#   mode: 1, 2, 3, 4
#   param: None, target_tag, or excel_path

# Excel 파일 선택
excel_path = select_excel_file(package_dir)
# Returns: 선택된 Excel 파일 경로

# Excel 파일 목록
files = list_excel_files(package_dir)
# Returns: list of Excel file paths

# 배너 출력
print_banner("Title")
```

---

## Test Helper API

**위치:** `src/apriltag_navigation/test_helper.py`

테스트 및 진단 기능

```python
from apriltag_navigation.test_helper import *

# 통합 진단 실행
run_diagnostics(args)
# Args: argparse.Namespace
# 여러 테스트를 한 번에 실행

# 개별 테스트
test_vision(vision_module, duration)
# 비전 시스템 테스트

test_motor(robot_controller, distance, direction)
# 모터 이동 테스트

test_pivot(robot_controller, rotation_controller, direction)
# 회전 테스트
```

---

## 사용 예시

### 예시 1: 특정 태그로 이동
```python
import rospy
from apriltag_navigation.robot_interface import RobotInterface

rospy.init_node('my_navigation')
robot = RobotInterface()

if robot.wait_until_ready():
    # 현재 위치에서 태그 5로 이동
    path = robot.map_manager.nav_graph.find_path(
        robot.get_current_position(),
        5
    )

    for waypoint in path:
        edge = robot.map_manager.nav_graph.get_edge(
            robot.get_current_position(),
            waypoint
        )

        if edge['type'] == 'move':
            while not robot.move_to_tag(waypoint, edge):
                rospy.sleep(0.033)  # 30Hz

        robot.set_current_position(waypoint)
```

### 예시 2: 비전 데이터 활용
```python
from apriltag_navigation.perception.vision_module import VisionModule

vision = VisionModule()
rospy.sleep(1.0)  # 카메라 준비 대기

tags = vision.get_detected_tags()
for tag_id, data in tags.items():
    distance = data['distance']
    lateral = data['x']
    print(f"Tag {tag_id}: {distance:.2f}m away, {lateral:.2f}m lateral")
```

### 예시 3: 맵 정보 조회
```python
from apriltag_navigation.map.map_manager import MapManager

map_mgr = MapManager()

# 태그 위치 조회
pos = map_mgr.get_tag_position(5)
print(f"Tag 5 at: {pos}")

# 경로 계획
path = map_mgr.nav_graph.find_path(0, 10)
print(f"Path from 0 to 10: {path}")

# 파라미터 조회
speed = map_mgr.get_param('speeds.linear')
print(f"Linear speed: {speed}")
```

---

## ROS 토픽 및 메시지

### 구독 (Subscribe)
- `/camera/image_raw` (sensor_msgs/Image) - 카메라 이미지
- `/odom` (nav_msgs/Odometry) - 오도메트리

### 발행 (Publish)
- `/cmd_vel` (geometry_msgs/Twist) - 속도 명령
- `/nav_debug` (std_msgs/String) - 디버그 정보

---

## 설정 파일

### config/map.yaml
```yaml
tags:
  0: {x: 0.0, y: 0.0}
  5: {x: 2.0, y: 0.0}

edges:
  - {from: 0, to: 5, type: move, direction: forward}

tasks:
  task1:
    waypoints: [0, 5, 7]

speeds:
  linear: 0.3
  angular: 0.5
```

---

이 API 레퍼런스를 통해 각 모듈의 주요 함수와 사용법을 빠르게 찾을 수 있습니다!
