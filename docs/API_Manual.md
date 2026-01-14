# API Manual

현장에서 코드를 수정하거나 기능을 호출할 때 참고할 실무 문서입니다.
각 클래스의 주요 함수별로 **입력 인자 / 반환값 / 로봇 행동(Side Effect) / 복붙용 실행 코드**를 명시합니다.

---

## 목차
1. [Quick Start: 최소 코드로 시작하기](#quick-start-최소-코드로-시작하기)
2. [RobotInterface API](#robotinterface-api)
3. [VisionModule API](#visionmodule-api)
4. [MapManager API](#mapmanager-api)
5. [PurePursuitController API](#purepursuitcontroller-api)
6. [RobotController API](#robotcontroller-api)
7. [RotationController API](#rotationcontroller-api)
8. [Wall Distance 계산](#wall-distance-계산)
9. [디버깅 Tips](#디버깅-tips)

---

## Quick Start: 최소 코드로 시작하기

Python 빈 페이지에서 바로 실행 가능한 최소 코드입니다.

### 기본 Setup

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import os

# Add src to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from apriltag_navigation.robot_interface import RobotInterface

# ROS 초기화
rospy.init_node('my_test_node', anonymous=True)

# Robot Interface 생성
robot = RobotInterface()

# 시스템 준비 대기 (카메라, Odometry, Dock 태그 검출)
if not robot.wait_until_ready():
    rospy.logerr("Failed to initialize")
    exit(1)

# 이제 robot API 사용 가능!
print("Ready to use robot API")
```

### 예제 1: Tag 100으로 이동

```python
# ... (위 Setup 코드 실행 후)

# 이동 edge 정의
edge = {'direction': 'forward', 'type': 'move'}

# Tag 100으로 이동
rate = rospy.Rate(30)  # 30 Hz
while not rospy.is_shutdown():
    if robot.move_to_tag(100, edge):
        print("Arrived at tag 100!")
        break
    rate.sleep()

# 정렬
while not rospy.is_shutdown():
    if robot.align_to_tag(100):
        print("Aligned to tag 100!")
        break
    rate.sleep()

robot.stop()
```

### 예제 2: 90도 회전

```python
# ... (Setup 코드 실행 후)

rate = rospy.Rate(30)
while not rospy.is_shutdown():
    if robot.rotate_90('cw'):  # 'cw' or 'ccw'
        print("Rotation complete!")
        break
    rate.sleep()

robot.stop()
```

### 예제 3: Vision 데이터 읽기

```python
# ... (Setup 코드 실행 후)

# Tag 508 데이터 읽기
tag_data = robot.vision.get_tag_data(508)
if tag_data:
    print(f"Distance: {tag_data['z']:.3f}m")
    print(f"Lateral: {tag_data['x']:.3f}m")
    print(f"Center: ({tag_data['center_x']}, {tag_data['center_y']})")
else:
    print("Tag 508 not visible")
```

---

## RobotInterface API

`RobotInterface`는 로봇 제어의 최상위 API입니다. 모든 기능은 이 클래스를 통해 접근합니다.

### __init__()

**설명:** RobotInterface 초기화

**입력:**
- `config_path` (str, optional): map.yaml 파일 경로. None이면 기본 경로 사용.

**반환:** 없음

**Side Effect:**
- MapManager, VisionModule, RobotController 등 모든 서브시스템 초기화
- ROS Subscribers/Publishers 생성 (`/nav_debug_status` 등)

**복붙 코드:**
```python
from apriltag_navigation.robot_interface import RobotInterface

robot = RobotInterface()  # 기본 map.yaml 사용
# or
robot = RobotInterface('/custom/path/to/map.yaml')  # 커스텀 경로
```

---

### wait_until_ready()

**설명:** 모든 서브시스템이 준비될 때까지 대기 (카메라, Odometry, Dock 태그 검출)

**입력:** 없음

**반환:**
- `True`: 준비 완료
- `False`: Timeout 또는 rospy shutdown

**Side Effect:**
- Blocking (대기)
- Dock 시작 위치 기록 (`record_dock_start()` 호출)

**복붙 코드:**
```python
if not robot.wait_until_ready():
    rospy.logerr("System not ready, exiting")
    exit(1)

# 준비 완료, 이후 코드 실행 가능
```

---

### move_to_tag()

**설명:** 목표 태그로 이동 (Pure Pursuit 기반)

**입력:**
- `target_tag` (int): 목표 태그 ID
- `edge` (dict): Edge 정보
  - `'direction'`: `'forward'` 또는 `'backward'`
  - `'type'`: `'move'`, `'pivot'`, 등

**반환:**
- `True`: 태그에 도착 (center_y가 화면 중앙에 도달)
- `False`: 이동 중

**Side Effect:**
- `/cmd_vel` 토픽 발행 (로봇 이동)
- `current_lateral`, `current_align_angle` 업데이트

**복붙 코드:**
```python
edge = {'direction': 'forward', 'type': 'move'}

rate = rospy.Rate(30)
while not rospy.is_shutdown():
    if robot.move_to_tag(100, edge):
        rospy.loginfo("Arrived at tag 100!")
        break
    rate.sleep()
```

**Backward 이동:**
```python
edge = {'direction': 'backward', 'type': 'move'}

while not rospy.is_shutdown():
    if robot.move_to_tag(500, edge):
        rospy.loginfo("Backward movement complete!")
        break
    rate.sleep()
```

**주의:**
- 태그가 안 보이면 직진 모드로 자동 전환 ("no tag | straight" 로그)
- 도착 판정: `center_y`가 화면 중앙±50px 이내

---

### rotate_90()

**설명:** 90도 회전 (CW 또는 CCW)

**입력:**
- `direction` (str): `'cw'` (시계방향) 또는 `'ccw'` (반시계방향)

**반환:**
- `True`: 회전 완료 (오차 < 1°)
- `False`: 회전 중

**Side Effect:**
- `/cmd_vel` 토픽 발행 (로봇 회전)
- RotationController 상태 업데이트

**복붙 코드:**
```python
# CW 회전
rate = rospy.Rate(30)
while not rospy.is_shutdown():
    if robot.rotate_90('cw'):
        rospy.loginfo("CW rotation complete!")
        break
    rate.sleep()

# CCW 회전
while not rospy.is_shutdown():
    if robot.rotate_90('ccw'):
        rospy.loginfo("CCW rotation complete!")
        break
    rate.sleep()
```

**주의:**
- 회전 시작 전 자동으로 목표 각도 설정
- PID 제어로 부드러운 회전
- Timeout 없음 (main loop에서 별도 관리 필요)

---

### align_to_tag()

**설명:** 태그에 정밀 정렬 (각도 < 0.5°, lateral < 5cm)

**입력:**
- `tag_id` (int): 정렬 대상 태그 ID

**반환:**
- `True`: 정렬 완료
- `False`: 정렬 중

**Side Effect:**
- `/cmd_vel` 토픽 발행 (회전 제어)
- 참조점 업데이트 (`set_reference_point()` 호출)
- 벽 거리 계산 및 로그 출력

**복붙 코드:**
```python
rate = rospy.Rate(30)
while not rospy.is_shutdown():
    if robot.align_to_tag(100):
        rospy.loginfo("Alignment complete!")
        break
    rate.sleep()
```

**주의:**
- 태그가 안 보이면 정렬 스킵 (True 반환)
- 정렬 완료 시 Odometry 참조점 자동 업데이트
- 벽 거리 < 10cm이면 "WARN!" 로그

---

### stop()

**설명:** 로봇 정지

**입력:** 없음

**반환:** 없음

**Side Effect:**
- `/cmd_vel` 토픽 발행 (linear=0, angular=0)

**복붙 코드:**
```python
robot.stop()
```

---

### stop_and_wait()

**설명:** 로봇 정지 후 일정 시간 대기 (관성 제거)

**입력:**
- `duration` (float, optional): 대기 시간 (초), 기본값 0.8초

**반환:**
- `True`: 대기 완료
- `False`: 대기 중

**Side Effect:**
- `/cmd_vel` 토픽 발행 (정지)
- 타이머 시작

**복붙 코드:**
```python
rate = rospy.Rate(30)
while not rospy.is_shutdown():
    if robot.stop_and_wait(duration=1.0):  # 1초 대기
        rospy.loginfo("Stop complete!")
        break
    rate.sleep()
```

---

### publish_debug_status()

**설명:** 디버그 상태 정보 발행 (`/nav_debug_status` 토픽)

**입력:**
- `current_wp` (int): 현재 웨이포인트
- `next_wp` (int): 다음 웨이포인트
- `total_wps` (int): 전체 웨이포인트 개수
- `wp_idx` (int): 현재 인덱스

**반환:** 없음

**Side Effect:**
- `/nav_debug_status` 토픽 발행 (JSON 형식)

**복붙 코드:**
```python
robot.publish_debug_status(
    current_wp=508,
    next_wp=500,
    total_wps=45,
    wp_idx=1
)
```

**발행되는 JSON 예시:**
```json
{
  "state": "MOVING",
  "current_wp": 508,
  "next_wp": 500,
  "progress": "1/45",
  "zone": "A",
  "lateral": 0.015,
  "align_angle": -1.2,
  "wall_dist": 0.38,
  "warning": "OK"
}
```

---

### get_current_position()

**설명:** 현재 로봇 위치 (태그 ID) 반환

**입력:** 없음

**반환:**
- `int`: 현재 태그 ID

**Side Effect:** 없음

**복붙 코드:**
```python
current = robot.get_current_position()
print(f"Current position: tag {current}")
```

---

### set_current_position()

**설명:** 현재 위치 설정

**입력:**
- `tag_id` (int): 새 위치 태그 ID

**반환:** 없음

**Side Effect:**
- `current_tag` 변수 업데이트

**복붙 코드:**
```python
robot.set_current_position(100)
```

---

## VisionModule API

`robot.vision`을 통해 접근합니다.

### is_tag_visible()

**설명:** 특정 태그가 현재 보이는지 확인

**입력:**
- `tag_id` (int): 태그 ID

**반환:**
- `True`: 태그 검출됨
- `False`: 태그 안 보임

**Side Effect:** 없음

**복붙 코드:**
```python
if robot.vision.is_tag_visible(508):
    print("Tag 508 is visible")
else:
    print("Tag 508 not found")
```

---

### get_tag_data()

**설명:** 태그의 3D 위치 및 이미지 중심 좌표 반환

**입력:**
- `tag_id` (int): 태그 ID

**반환:**
- `dict` or `None`:
  ```python
  {
      'x': float,       # Lateral offset (+ = 오른쪽)
      'y': float,       # Vertical offset
      'z': float,       # Distance to tag
      'center_x': int,  # Image center X
      'center_y': int,  # Image center Y
      'corners': list   # 4개 코너 좌표
  }
  ```

**Side Effect:** 없음

**복붙 코드:**
```python
tag_data = robot.vision.get_tag_data(100)
if tag_data:
    print(f"Distance: {tag_data['z']:.3f}m")
    print(f"Lateral: {tag_data['x']:.3f}m")
    print(f"Center: ({tag_data['center_x']}, {tag_data['center_y']})")
else:
    print("Tag not visible")
```

---

### get_alignment_angle()

**설명:** 태그 정렬 각도 반환 (태그의 top edge 기울기)

**입력:**
- `tag_id` (int): 태그 ID

**반환:**
- `float` or `None`: 각도 (도 단위, -90° ~ +90°)
  - `+`: 태그가 CCW로 기울어짐 → CCW 회전 필요
  - `-`: 태그가 CW로 기울어짐 → CW 회전 필요

**Side Effect:** 없음

**복붙 코드:**
```python
angle = robot.vision.get_alignment_angle(100)
if angle is not None:
    print(f"Alignment angle: {angle:.2f}°")
    if abs(angle) < 0.5:
        print("Already aligned!")
else:
    print("Cannot calculate angle")
```

---

### get_tag_lateral()

**설명:** Lateral offset 반환 (태그의 X축 offset)

**입력:**
- `tag_id` (int): 태그 ID

**반환:**
- `float` or `None`: Lateral offset (미터)
  - `+`: 태그가 오른쪽
  - `-`: 태그가 왼쪽

**Side Effect:** 없음

**복붙 코드:**
```python
lateral = robot.vision.get_tag_lateral(508)
if lateral is not None:
    print(f"Lateral offset: {lateral:.3f}m")
    if lateral > 0:
        print("Tag is on the right")
    else:
        print("Tag is on the left")
```

---

## MapManager API

`robot.map_manager`를 통해 접근합니다.

### tag_db.get()

**설명:** 태그 정보 조회

**입력:**
- `tag_id` (int): 태그 ID

**반환:**
- `dict` or `None`:
  ```python
  {
      'x': float,       # World X coordinate
      'y': float,       # World Y coordinate
      'type': TagType,  # DOCK, PIVOT, WORK, MOVE
      'zone': str,      # 'A', 'B', 'C', 'D', 'E', 'DOCK'
      'name': str       # Optional
  }
  ```

**Side Effect:** 없음

**복붙 코드:**
```python
tag_info = robot.map_manager.tag_db.get(508)
if tag_info:
    print(f"Tag 508: ({tag_info['x']}, {tag_info['y']}), Zone {tag_info['zone']}")
else:
    print("Tag not found")
```

---

### nav_graph.find_path()

**설명:** BFS 알고리즘으로 최단 경로 탐색

**입력:**
- `start` (int): 시작 태그 ID
- `goal` (int): 목표 태그 ID

**반환:**
- `list` or `None`: 경로 (태그 ID 리스트), 경로 없으면 None

**Side Effect:** 없음

**복붙 코드:**
```python
path = robot.map_manager.nav_graph.find_path(508, 123)
if path:
    print(f"Path: {path}")
    print(f"Distance: {len(path) - 1} hops")
else:
    print("No path found")
```

---

### get_param()

**설명:** 설정 파라미터 조회

**입력:**
- `param_path` (str): 파라미터 경로 (dot-separated, 예: `'speeds.linear'`)
- `default` (any, optional): 기본값

**반환:**
- 파라미터 값 or 기본값

**Side Effect:** 없음

**복붙 코드:**
```python
linear_speed = robot.map_manager.get_param('speeds.linear', 0.3)
robot_length = robot.map_manager.get_param('length', 0.80)
wall_clearance = robot.map_manager.get_param('wall_distances.min_clearance', 0.10)

print(f"Linear speed: {linear_speed} m/s")
print(f"Robot length: {robot_length} m")
print(f"Min wall clearance: {wall_clearance} m")
```

---

## PurePursuitController API

`robot.pursuit`를 통해 접근합니다.

### calculate_forward()

**설명:** Forward Pure Pursuit 알고리즘으로 각속도 계산

**입력:**
- `lateral` (float): Lateral offset (미터)
- `distance_to_tag` (float): 태그까지 거리 (미터)

**반환:**
- `float`: Angular velocity (rad/s), -0.3 ~ +0.3 범위로 clipping

**Side Effect:** 없음

**복붙 코드:**
```python
lateral = 0.1  # 태그가 오른쪽 10cm
distance = 1.5  # 1.5m 거리

angular = robot.pursuit.calculate_forward(lateral, distance)
print(f"Angular velocity: {angular:.3f} rad/s")

# 실제 모터 명령
robot.robot.move(0.3, angular)  # linear=0.3 m/s
```

---

### calculate_backward()

**설명:** Backward Pure Pursuit 알고리즘으로 각속도 계산

**입력:**
- `lateral` (float): Lateral offset (미터)
- `distance_to_tag` (float): 태그까지 거리 (미터)

**반환:**
- `float`: Angular velocity (rad/s)

**Side Effect:** 없음

**복붙 코드:**
```python
angular = robot.pursuit.calculate_backward(lateral, distance)

# 후진 명령
robot.robot.move(-0.3, angular)  # linear=-0.3 m/s (backward)
```

---

## RobotController API

`robot.robot`를 통해 접근합니다.

### move()

**설명:** 속도 명령 발행

**입력:**
- `linear` (float): 선속도 (m/s)
- `angular` (float): 각속도 (rad/s)

**반환:** 없음

**Side Effect:**
- `/cmd_vel` 토픽 발행

**복붙 코드:**
```python
# 전진
robot.robot.move(0.3, 0)

# 후진
robot.robot.move(-0.3, 0)

# 좌회전
robot.robot.move(0, 0.5)

# 우회전
robot.robot.move(0, -0.5)

# 전진하며 좌회전
robot.robot.move(0.3, 0.2)
```

---

### get_position()

**설명:** Odometry 기반 현재 위치 반환

**입력:** 없음

**반환:**
- `tuple`: `(x, y)` in meters

**Side Effect:** 없음

**복붙 코드:**
```python
x, y = robot.robot.get_position()
print(f"Current position: ({x:.3f}, {y:.3f})")
```

---

### get_heading()

**설명:** 현재 heading (방향) 반환

**입력:** 없음

**반환:**
- `float`: Heading in radians (-π ~ π)

**Side Effect:** 없음

**복붙 코드:**
```python
heading = robot.robot.get_heading()
print(f"Current heading: {math.degrees(heading):.1f}°")
```

---

## RotationController API

`robot.rotation_ctrl`를 통해 접근합니다.

### start_rotation()

**설명:** 회전 시작 (목표 각도 설정)

**입력:**
- `target_angle_delta` (float): 회전할 각도 (radians)
  - `+π/2`: 90° CCW
  - `-π/2`: 90° CW

**반환:** 없음

**Side Effect:**
- `target_theta` 설정
- `rotation_active = True`

**복붙 코드:**
```python
import math

# 90° CW 회전
robot.rotation_ctrl.start_rotation(-math.pi/2)

# 90° CCW 회전
robot.rotation_ctrl.start_rotation(math.pi/2)

# 45° CW 회전
robot.rotation_ctrl.start_rotation(-math.pi/4)
```

---

### update()

**설명:** PID 제어 루프 업데이트 (매 iteration 호출)

**입력:** 없음

**반환:**
- `True`: 회전 완료
- `False`: 회전 중

**Side Effect:**
- `/cmd_vel` 토픽 발행

**복붙 코드:**
```python
robot.rotation_ctrl.start_rotation(-math.pi/2)

rate = rospy.Rate(30)
while not rospy.is_shutdown():
    if robot.rotation_ctrl.update():
        print("Rotation complete!")
        break
    rate.sleep()
```

---

## Wall Distance 계산

⚠️ **현장 점검 필수**: 각 Zone에서 벽 거리를 정확히 확인해야 충돌 방지 가능!

### calculate_wall_distance()

**설명:** 로봇 우측면에서 벽까지 거리 계산 (Zone별 로직 다름)

**입력:**
- `lateral` (float): Lateral offset (미터)
- `align_angle_deg` (float): 정렬 각도 (도 단위)
- `zone` (str): Zone 이름 ('A', 'B', 'C', 'D', 'E', 'DOCK')

**반환:**
- `float`: 벽까지 거리 (미터)

**Side Effect:** 없음

**복붙 코드:**
```python
lateral = 0.05  # 5cm offset
align_angle = 2.0  # 2° 기울어짐
zone = 'B'

wall_dist = robot.pursuit.calculate_wall_distance(lateral, align_angle, zone)
print(f"Wall distance: {wall_dist:.3f}m")

# 충돌 경고
min_clearance = 0.10  # 10cm
if wall_dist < min_clearance:
    print(f"WARNING: Too close to wall! ({wall_dist*100:.1f}cm)")
```

### Zone별 계산 로직 요약

| Zone | 로봇 Heading | 벽 위치 | Lateral 부호 해석 | 계산식 |
|------|-----------|--------|---------------|-------|
| **A / DOCK** | +X (0°) | 우측 (-Y) | `+` = 벽 가까움 | `base - lateral` |
| **B / D** | +Y (90°) | 우측 (-X) | `+` = 벽 멀어짐 | `base + lateral` |
| **C / E** | -Y (-90°) | 우측 (+X) | `+` = 벽 가까움 | `base + lateral` |

**주의사항:**
- Zone A: `lateral > 0` → 로봇이 벽에 가까워짐 (위험!)
- Zone B/D: `lateral > 0` → 로봇이 벽에서 멀어짐 (안전)
- Zone C/E: `lateral > 0` → 로봇이 벽에 가까워짐 (위험!)

---

## 디버깅 Tips

### 1. 로그 레벨 조정

```python
# 특정 태그만 로그 출력
rospy.loginfo_throttle(0.5, f"[DEBUG] lateral={lateral:.3f}")

# 경고 출력
rospy.logwarn(f"[WALL] Distance too close: {wall_dist:.3f}m")

# 에러 출력
rospy.logerr(f"[ERROR] Tag {tag_id} not found")
```

### 2. ROS 토픽 모니터링

```bash
# 디버그 상태 확인
rostopic echo /nav_debug_status

# 속도 명령 확인
rostopic echo /cmd_vel

# Odometry 확인
rostopic echo /odom
```

### 3. Vision 데이터 확인

```python
# 모든 검출된 태그 출력
detected = robot.vision.get_detected_tags()
print(f"Detected tags: {detected}")

# 특정 태그 상세 정보
for tag_id in detected:
    data = robot.vision.get_tag_data(tag_id)
    print(f"Tag {tag_id}: dist={data['z']:.2f}m, lateral={data['x']:.3f}m")
```

### 4. 참조점 (Reference Point) 확인

```python
# 현재 참조점 조회
ref_x = robot.robot.ref_robot_x
ref_y = robot.robot.ref_robot_y
ref_heading = robot.robot.ref_robot_heading

print(f"Reference: ({ref_x:.3f}, {ref_y:.3f}), heading={math.degrees(ref_heading):.1f}°")

# Odometry와 추정 위치 비교
odom_x, odom_y = robot.robot.get_position()
est_x, est_y, est_heading = robot.robot.get_estimated_pose()

print(f"Odometry: ({odom_x:.3f}, {odom_y:.3f})")
print(f"Estimated: ({est_x:.3f}, {est_y:.3f}), heading={math.degrees(est_heading):.1f}°")
```

### 5. 벽 거리 실시간 모니터링

```python
rate = rospy.Rate(5)  # 5 Hz
while not rospy.is_shutdown():
    if robot.vision.is_tag_visible(robot.current_tag):
        lateral = robot.vision.get_tag_lateral(robot.current_tag)
        angle = robot.vision.get_alignment_angle(robot.current_tag)
        zone = robot.map_manager.tag_db.get_zone(robot.current_tag)

        if lateral is not None and angle is not None:
            wall_dist = robot.pursuit.calculate_wall_distance(lateral, angle, zone)
            print(f"[{zone}] Wall: {wall_dist*100:.1f}cm, Lateral: {lateral*100:.1f}cm")

    rate.sleep()
```

### 6. State Machine 상태 확인

```python
from apriltag_navigation.robot_interface import NavigationState

print(f"Current state: {robot.state}")

# State별 처리
if robot.state == NavigationState.MOVING:
    print("Robot is moving")
elif robot.state == NavigationState.ALIGNING:
    print("Robot is aligning")
elif robot.state == NavigationState.ROTATING:
    print("Robot is rotating")
```

---

## 전체 예제: Task 실행

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from apriltag_navigation.robot_interface import RobotInterface

def main():
    # ROS 초기화
    rospy.init_node('custom_task', anonymous=True)

    # Robot 초기화
    robot = RobotInterface()
    if not robot.wait_until_ready():
        rospy.logerr("System not ready")
        return

    # 경로 정의 (508 → 500 → 100)
    waypoints = [508, 500, 100]

    # 웨이포인트 순회
    for i in range(len(waypoints) - 1):
        current = waypoints[i]
        target = waypoints[i + 1]

        rospy.loginfo(f"Moving from {current} to {target}")

        # Edge 조회
        edge = robot.map_manager.nav_graph.get_edge(current, target)
        if not edge:
            rospy.logerr(f"No edge from {current} to {target}")
            continue

        # 이동
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if robot.move_to_tag(target, edge):
                break
            rate.sleep()

        # 정렬
        while not rospy.is_shutdown():
            if robot.align_to_tag(target):
                break
            rate.sleep()

        # Pivot 처리
        if edge['type'] == 'pivot':
            direction = 'cw' if 'cw' in edge['direction'] else 'ccw'
            while not rospy.is_shutdown():
                if robot.rotate_90(direction):
                    break
                rate.sleep()

        # 현재 위치 업데이트
        robot.set_current_position(target)

    # 완료
    robot.stop()
    rospy.loginfo("Task complete!")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
```

---

## 요약

이 API Manual은 현장에서 바로 사용할 수 있는 실무 문서입니다:

1. **Quick Start**: Python 빈 페이지에서 바로 실행 가능한 최소 코드
2. **함수별 상세 스펙**: 입력/출력/Side Effect/복붙 코드
3. **Wall Distance**: Zone별 계산 로직 및 주의사항
4. **디버깅 Tips**: 실시간 모니터링 및 문제 해결

**주요 클래스:**
- `RobotInterface`: 최상위 API (이동, 회전, 정렬)
- `VisionModule`: AprilTag 검출 및 위치 추정
- `MapManager`: 맵 데이터 및 경로 탐색
- `PurePursuitController`: 경로 추종 알고리즘
- `RobotController`: 하드웨어 인터페이스

**현장 필수 체크:**
- [ ] Wall distance < 10cm 경고 확인
- [ ] Zone별 lateral 부호 해석
- [ ] Dock 복귀 정확도 (< 5cm)
