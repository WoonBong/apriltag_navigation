# AprilTag Navigation - Code Structure & Field Testing Report

본 문서는 AprilTag Navigation 시스템의 **상위 제어기 구조**와 각 상위 제어기가 호출하는 **하위 함수들의 실행 순서**를 설명합니다. 현장 테스트 시 각 하위 함수를 독립적으로 검증할 수 있도록 구성되었습니다.

---

## 목차
1. [시스템 개요](#시스템-개요)
2. [전체 동작 흐름](#전체-동작-흐름)
3. [상위 제어기 목록](#상위-제어기-목록)
4. [상위 제어기별 하위 함수 호출 구조](#상위-제어기별-하위-함수-호출-구조)
5. [최소 단위 함수 트리](#최소-단위-함수-트리)
6. [예외 처리 로직](#예외-처리-로직)
7. [현장 테스트 단위](#현장-테스트-단위)

---

## 시스템 개요

### 전체 주행 완성을 위한 기본 동작

**로봇 움직임 3가지 기본 동작:**

```
1. move_to_tag()    - 목표 태그까지 이동
2. rotate_90()      - 90도 회전 (피봇 포인트)
3. align_to_tag()   - 태그에 정밀 정렬
```

**전체 프로젝트 관점의 핵심 동작:**

```
[초기화 단계]
1. 시스템 초기화      - ROS 노드, 서브시스템 초기화
2. 맵 로딩           - map.yaml에서 태그/엣지/태스크 로드
3. 준비 대기         - 카메라, Odometry, 초기 태그(508) 검출 대기

[경로 계획 단계]
4. 태스크 선택       - Mode 1/2: 사전 정의 태스크, Mode 3: 직접 입력, Mode 4: Excel 스캔
5. 경로 생성         - BFS 알고리즘으로 웨이포인트 리스트 생성

[실행 단계]
6. 웨이포인트 순회   - 각 웨이포인트별로 이동/회전/정렬 조합
7. 실시간 모니터링   - 벽 거리, lateral offset, 상태 정보 Publish

[종료 단계]
8. 도킹 복귀         - 모든 태스크 완료 후 508(Dock)로 복귀
9. 정확도 검증       - 시작 위치 vs 종료 위치 비교
```

### 상위 제어기 계층 구조

```
main_node.py (프로그램 시작점)
│
├─ [Phase 1: 초기화]
│   └─ RobotInterface.__init__()
│       ├─ MapManager (맵 로딩)
│       ├─ VisionModule (카메라 초기화)
│       ├─ RobotController (Odometry, 모터 인터페이스)
│       ├─ RotationController (회전 제어)
│       └─ PurePursuitController (경로 추종)
│
├─ [Phase 2: 경로 계획]
│   ├─ TaskManager.get_task_waypoints() (Task 1/2)
│   ├─ TaskManager.get_excel_scan_waypoints() (Mode 4)
│   └─ NavigationGraph.find_path() (Mode 3)
│
├─ [Phase 3: 실행]
│   └─ NavigationMission.execute() (State Machine)
│       ├─ RobotInterface.move_to_tag()
│       ├─ RobotInterface.rotate_90()
│       ├─ RobotInterface.align_to_tag()
│       ├─ RobotInterface.stop_and_wait()
│       └─ RobotInterface.wait_for_scan() (Mode 4 only)
│
└─ [Phase 4: 모니터링 & 디버깅]
    ├─ RobotInterface.publish_debug_status() (실시간 상태)
    ├─ RobotInterface.publish_robot_pose() (Mode 4)
    └─ RobotInterface.compare_dock_return() (정확도 검증)
```

---

## 전체 동작 흐름

### 프로그램 시작부터 미션 완료까지

```
┌─────────────────────────────────────────────────────────────┐
│ 1. 프로그램 시작 (main_node.py)                             │
│    rosrun apriltag_navigation main_node.py [--mode N]       │
└────────────────┬────────────────────────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────────────────────────┐
│ 2. 시스템 초기화                                             │
│    ├─ rospy.init_node('apriltag_navigation')               │
│    ├─ RobotInterface.__init__()                            │
│    │   ├─ MapManager: map.yaml 파싱 (73개 태그, 엣지, 태스크)│
│    │   ├─ VisionModule: 카메라 Subscribe (/rgb, /camera_info)│
│    │   ├─ RobotController: Odom/Cmd Subscribe/Publish      │
│    │   └─ PurePursuitController: 파라미터 로드              │
│    └─ RobotInterface.wait_until_ready()                    │
│        ├─ vision.is_ready() → /camera_info 수신 대기       │
│        ├─ robot.is_ready() → /odom 수신 대기               │
│        └─ vision.is_tag_visible(508) → Dock 태그 검출 대기  │
└────────────────┬────────────────────────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────────────────────────┐
│ 3. 모드 선택 & 경로 생성                                      │
│    ├─ Mode 1: load_task('task1')                           │
│    │   └─ TaskManager.get_task_waypoints('task1')          │
│    │       → [508, 500, 400, ..., 508] (45개)              │
│    │                                                         │
│    ├─ Mode 2: load_task('task2')                           │
│    │   └─ TaskManager.get_task_waypoints('task2')          │
│    │       → [508, 500, 400, ..., 508] (75개)              │
│    │                                                         │
│    ├─ Mode 3: load_direct_navigation(target_tag)           │
│    │   └─ NavigationGraph.find_path(508, target_tag)       │
│    │       → BFS 알고리즘으로 최단 경로                       │
│    │                                                         │
│    └─ Mode 4: load_excel_scan(excel_path)                  │
│        └─ TaskManager.get_excel_scan_waypoints(excel_path) │
│            ├─ pandas.read_excel(): group_id 추출            │
│            ├─ 중복 제거: [4,4,5,5,6] → [4,5,6]             │
│            ├─ Tag ID 변환: [4,5,6] → [104,105,106]         │
│            └─ 경로 연결: 508→104→105→106→508               │
└────────────────┬────────────────────────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────────────────────────┐
│ 4. 웨이포인트 순회 실행 (State Machine Loop)                 │
│                                                              │
│    for waypoint_index in range(len(waypoints) - 1):        │
│        current_wp = waypoints[waypoint_index]               │
│        next_wp = waypoints[waypoint_index + 1]              │
│                                                              │
│        ┌─ Edge 타입 확인                                     │
│        │   edge = nav_graph.get_edge(current_wp, next_wp)  │
│        │                                                     │
│        ├─ [Straight Edge]                                   │
│        │   State: MOVING → ALIGNING → IDLE                 │
│        │   1) move_to_tag(next_wp, edge)                   │
│        │   2) align_to_tag(next_wp)                        │
│        │                                                     │
│        ├─ [Pivot Edge]                                      │
│        │   State: MOVING → ALIGNING → ROTATING → IDLE      │
│        │   1) move_to_tag(next_wp, edge)                   │
│        │   2) align_to_tag(next_wp)                        │
│        │   3) rotate_90(direction)                         │
│        │                                                     │
│        ├─ [Backward Edge]                                   │
│        │   State: MOVING → ALIGNING → IDLE                 │
│        │   1) move_to_tag(next_wp, edge)  # 후진           │
│        │   2) align_to_tag(next_wp)                        │
│        │                                                     │
│        └─ [Scan Mode Only]                                  │
│            State: WAIT_SCAN                                 │
│            1) wait_for_scan(next_wp)                       │
│               ├─ publish_robot_pose() 주기적 Publish         │
│               └─ /scan_finished 신호 대기                   │
└────────────────┬────────────────────────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────────────────────────┐
│ 5. 미션 완료 & 검증                                          │
│    ├─ Dock(508)로 복귀 완료                                 │
│    ├─ compare_dock_return()                                │
│    │   └─ 시작 위치 vs 종료 위치 lateral offset 비교         │
│    └─ 프로그램 종료 또는 새 미션 대기                         │
└─────────────────────────────────────────────────────────────┘
```

---

## 상위 제어기 목록

### 1. **NavigationMission** (main_node.py)
- **역할:** 전체 미션 제어 (모드 선택, 웨이포인트 순회)
- **주요 기능:**
  - 태스크 로드 (Task 1/2, Direct, Excel Scan)
  - 웨이포인트 순차 실행
  - 상태 머신 관리 (IDLE → MOVING → ALIGNING → ROTATING → WAIT_SCAN)
  - 미션 완료 판정

### 2. **RobotInterface** (robot_interface.py)
- **역할:** 로봇 API 제공
- **주요 기능:**
  - **초기화:** `__init__()`, `wait_until_ready()`
  - **로봇 움직임:** `move_to_tag()`, `rotate_90()`, `align_to_tag()`
  - **제어:** `stop_and_wait()`, `wait_for_scan()`
  - **모니터링:** `publish_debug_status()`, `publish_robot_pose()`
  - **검증:** `record_dock_start()`, `compare_dock_return()`

### 3. **MapManager** (map/map_manager.py)
- **역할:** 맵 데이터 관리 (태그, 엣지, 태스크)
- **하위 구성요소:**
  - `TagDatabase` - 태그 정보 (위치, 존, 타입)
  - `NavigationGraph` - 엣지 & 경로 탐색
  - `TaskManager` - 태스크 웨이포인트 생성

### 4. **VisionModule** (perception/vision_module.py)
- **역할:** AprilTag 검출 & 포즈 추정
- **주요 기능:**
  - 카메라 캘리브레이션 (`_camera_info_callback`)
  - 태그 검출 (`_image_callback`, `detector.detect`)
  - 정렬 각도 계산 (코너 기반)
  - 태그 가시성 확인

### 5. **PurePursuitController** (navigation/pure_pursuit.py)
- **역할:** Pure Pursuit 알고리즘 기반 경로 추종
- **주요 기능:**
  - Forward/Backward Pure Pursuit
  - 절대 좌표 기반 제어
  - 벽 거리 계산
  - 태그 기반 로봇 위치 추정

### 6. **RobotController** (hardware/robot_controller.py)
- **역할:** 하드웨어 인터페이스 (Odometry, 속도 명령)
- **주요 기능:**
  - Odometry 수신 (`_odom_callback`)
  - 속도 명령 발행 (`move`, `stop`)
  - 참조점 기반 위치 추정
  - 기본 이동 함수 (forward, backward, rotate)

### 7. **RotationController** (hardware/robot_controller.py)
- **역할:** 정밀 회전 제어
- **주요 기능:**
  - 목표 각도 설정
  - PID 기반 회전 제어
  - 완료 판정 (오차 허용치 내)

### 8. **CoordinateTransformer** (navigation/pure_pursuit.py)
- **역할:** 좌표계 변환 (Mode 4 전용)
- **주요 기능:**
  - Isaac Sim → Manipulator 좌표 변환
  - 카메라 위치 → 로봇 중심 offset 적용

---

## 상위 제어기별 하위 함수 호출 구조

### A. 시스템 초기화

**상위 함수:** `RobotInterface.__init__()`

**하위 함수 호출 순서:**

```python
1. MapManager 초기화
   ├─ 1.1 YAML 파일 읽기
   │      with open('config/map.yaml', 'r') as f:
   │          config = yaml.safe_load(f)
   │
   ├─ 1.2 TagDatabase 생성
   │      ├─ tags 딕셔너리 파싱
   │      │   for tag_id, tag_data in config['tags'].items():
   │      │       tags[int(tag_id)] = {
   │      │           'x': float(tag_data['x']),
   │      │           'y': float(tag_data['y']),
   │      │           'type': TagType[tag_data['type']],
   │      │           'zone': tag_data['zone']
   │      │       }
   │      │
   │      └─ 총 73개 태그 로드 완료
   │
   ├─ 1.3 NavigationGraph 생성
   │      ├─ edges 리스트 파싱
   │      │   for edge_data in config['edges']:
   │      │       from_tag = int(edge_data['from'])
   │      │       to_tag = int(edge_data['to'])
   │      │       edges[from_tag].append({
   │      │           'to': to_tag,
   │      │           'direction': edge_data['direction'],
   │      │           'type': edge_data['type']
   │      │       })
   │      │
   │      └─ 엣지 연결 관계 저장
   │
   └─ 1.4 TaskManager 생성
          ├─ tasks 섹션 로드
          │   task1: {waypoints: [508, 500, ...]}
          │   task2: {waypoints: [508, 500, ...]}
          │
          └─ TagDatabase, NavigationGraph 참조 저장

2. VisionModule 초기화
   ├─ 2.1 AprilTag Detector 생성
   │      detector = Detector(
   │          families='tag36h11',
   │          nthreads=4,
   │          quad_decimate=1.0,
   │          refine_edges=1
   │      )
   │
   ├─ 2.2 ROS Subscriber 생성
   │      image_sub = rospy.Subscriber('/rgb', Image, _image_callback)
   │      camera_info_sub = rospy.Subscriber('/camera_info', CameraInfo, _camera_info_callback)
   │
   └─ 2.3 cv_bridge 초기화
          bridge = CvBridge()

3. RobotController 초기화
   ├─ 3.1 파라미터 로드
   │      linear_speed = map_manager.get_param('speeds.linear', 0.3)
   │      angular_speed = map_manager.get_param('speeds.angular', 0.25)
   │
   ├─ 3.2 ROS Publisher/Subscriber 생성
   │      cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
   │      odom_sub = rospy.Subscriber('/odom', Odometry, _odom_callback)
   │
   └─ 3.3 참조점 초기화
          ref_odom_x = 0.0
          ref_odom_y = 0.0
          ref_robot_x = 0.0
          ref_robot_y = 0.0

4. PurePursuitController 초기화
   ├─ 4.1 파라미터 로드
   │      look_ahead_base = map_manager.get_param('pure_pursuit.look_ahead_base', 0.4)
   │      pp_gain_forward = map_manager.get_param('pure_pursuit.gain_forward', 1.0)
   │      wall_dist_work_zone = map_manager.get_param('wall_distances.work_zone', 0.35)
   │
   └─ 4.2 TagDatabase 참조 저장
          tag_db = map_manager.tag_db

5. RotationController 초기화
   └─ 5.1 RobotController 참조 저장
          robot = robot_controller
          is_rotating = False
```

**현장 테스트 단위:**
- **1.1** YAML 파싱: `map.yaml` 파일 존재 및 문법 검증
- **1.2** 태그 로드: 73개 태그 모두 로드, 좌표/존 정확성
- **1.3** 엣지 로드: 모든 엣지 from-to 관계 유효성
- **1.4** 태스크 로드: Task 1/2 웨이포인트 존재
- **2.1** Detector 생성: dt_apriltags 라이브러리 정상 import
- **2.2** ROS Subscriber: 토픽 연결 확인
- **3.2** ROS Pub/Sub: `/cmd_vel`, `/odom` 토픽 활성화
- **4.1** 파라미터 검증: 모든 필수 파라미터 로드 확인

---

### B. 준비 대기

**상위 함수:** `RobotInterface.wait_until_ready()`

**하위 함수 호출 순서:**

```python
1. 서브시스템 준비 상태 확인 (Loop, 10Hz)
   │
   ├─ 1.1 VisionModule.is_ready()
   │      └─ return camera_info_received
   │          (/camera_info 토픽에서 K matrix 수신 확인)
   │
   ├─ 1.2 RobotController.is_ready()
   │      └─ return odom_received
   │          (/odom 토픽에서 odometry 메시지 수신 확인)
   │
   └─ 1.3 VisionModule.is_tag_visible(508)
          └─ return 508 in detected_tags
              (Dock 태그가 카메라 시야에 있는지 확인)

2. 모든 조건 충족 시
   ├─ record_dock_start()
   │   ├─ tag_data = vision.get_tag_data(508)
   │   ├─ dock_start_lateral = tag_data['x']
   │   ├─ dock_start_center_x = tag_data['center_x']
   │   └─ dock_start_recorded = True
   │
   └─ return True

3. Timeout 처리
   └─ if rospy.is_shutdown():
          return False
```

**현장 테스트 단위:**
- **1.1** 카메라 준비: `/camera_info` 메시지 수신 (5초 내)
- **1.2** Odometry 준비: `/odom` 메시지 수신 (5초 내)
- **1.3** Dock 태그 검출: Tag 508이 시야에 있는지
- **2** Dock 초기 상태 기록: lateral, center_x 값 저장

---

### C. NavigationMission - Task 로드

**상위 함수:** `load_task(task_name)`

**하위 함수 호출 순서:**

```python
1. TaskManager.get_task_waypoints(task_name)
   │
   ├─ 1.1 태스크 존재 여부 확인
   │      if task_name not in self.tasks:
   │          return []  # 빈 리스트 반환
   │
   ├─ 1.2 YAML에서 waypoints 필드 읽기
   │      waypoints = self.tasks[task_name]['waypoints']
   │
   └─ 1.3 웨이포인트 리스트 반환
          return waypoints
          # Task 1: [508, 500, 400, 401, ..., 508] (45개)
          # Task 2: [508, 500, 400, 401, ..., 508] (75개)
```

**현장 테스트 단위:**
- **1.1** 태스크 존재: `map.yaml`에 'task1', 'task2' 정의 확인
- **1.2** 웨이포인트 필드: waypoints 리스트 형식 검증
- **1.3** 개수 검증: Task 1=45, Task 2=75

---

### D. NavigationMission - Excel 스캔 경로 생성

**상위 함수:** `load_excel_scan(excel_path)`

**하위 함수 호출 순서:**

```python
1. TaskManager.get_excel_scan_waypoints(excel_path)
   │
   ├─ 1.1 pandas.read_excel(excel_path, usecols=['group_id'])
   │      ├─ Excel 파일 존재 확인
   │      ├─ group_id 컬럼 읽기
   │      └─ DataFrame 반환: df['group_id'] = [4, 4, 4, 5, 5, 6, ...]
   │
   ├─ 1.2 group_id 중복 제거 (연속된 같은 값 하나로)
   │      group_order = []
   │      prev_gid = None
   │      for gid in df['group_id']:
   │          if gid != prev_gid:
   │              group_order.append(gid)
   │              prev_gid = gid
   │      # 결과: [4, 5, 6, 7, 6] (연속 중복 제거)
   │
   ├─ 1.3 group_id → tag_id 변환
   │      scan_tags = [100 + int(gid) for gid in group_order]
   │      # 결과: [104, 105, 106, 107, 106]
   │
   ├─ 1.4 각 scan_tag 사이 경로 탐색
   │      waypoints = [508]  # 시작점
   │
   │      for i, tag in enumerate(scan_tags):
   │          if i == 0:
   │              path = NavigationGraph.find_path(508, tag)
   │          else:
   │              path = NavigationGraph.find_path(scan_tags[i-1], tag)
   │
   │          if path:
   │              waypoints.extend(path[1:])  # 첫 요소 제외하고 추가
   │
   │      # 예: 508 → [508,500,400,401,402,403,404] → 104
   │      #     104 → [104,105] → 105
   │
   └─ 1.5 마지막 태그 → 508 경로 추가
          if scan_tags:
              return_path = NavigationGraph.find_path(scan_tags[-1], 508)
              if return_path:
                  waypoints.extend(return_path[1:])

          return waypoints, scan_tags
```

**현장 테스트 단위:**
- **1.1** Excel 파일: 존재 여부, group_id 컬럼 존재
- **1.2** 중복 제거: [4,4,5,5] → [4,5] 변환 로직
- **1.3** Tag ID 변환: group_id + 100
- **1.4** 경로 생성: 각 태그 간 유효한 경로 존재
- **1.5** 전체 구조: 시작=508, 끝=508

---

### E. NavigationMission - 웨이포인트 실행

**상위 함수:** `execute()` - Main state machine loop

**하위 함수 호출 순서:**

```python
1. 완료 확인
   if waypoint_index >= len(waypoints):
       return True  # 모든 웨이포인트 완료

2. 현재 & 다음 웨이포인트 확인
   current_wp = waypoints[waypoint_index]
   next_wp = waypoints[waypoint_index + 1]

3. NavigationGraph.get_edge(current_wp, next_wp)
   │
   ├─ 3.1 엣지 존재 확인
   │      if current_wp not in edges:
   │          return None
   │
   │      for edge in edges[current_wp]:
   │          if edge['to'] == next_wp:
   │              return edge
   │
   └─ 3.2 Edge 정보 반환
          return {
              'to': next_wp,
              'direction': 'forward'/'backward'/'cw'/'ccw',
              'type': 'straight'/'pivot'/'backward'
          }

4. Edge 타입에 따라 상태 머신 실행

   ┌─ 4.1 Straight Edge (직진)
   │   ├─ State: IDLE → MOVING
   │   │   if state == IDLE:
   │   │       state = MOVING
   │   │
   │   ├─ State: MOVING → ALIGNING
   │   │   if state == MOVING:
   │   │       if RobotInterface.move_to_tag(next_wp, edge):
   │   │           state = ALIGNING
   │   │
   │   └─ State: ALIGNING → IDLE (다음 웨이포인트)
   │       if state == ALIGNING:
   │           if RobotInterface.align_to_tag(next_wp):
   │               robot.set_current_position(next_wp)
   │               waypoint_index += 1
   │               state = IDLE
   │
   ├─ 4.2 Pivot Edge (회전)
   │   ├─ State: IDLE → MOVING
   │   ├─ State: MOVING → ROTATING
   │   │   if RobotInterface.move_to_tag(next_wp, edge):
   │   │       state = ROTATING
   │   │
   │   ├─ State: ROTATING → ALIGNING
   │   │   if RobotInterface.rotate_90(edge['direction']):
   │   │       state = ALIGNING
   │   │
   │   └─ State: ALIGNING → IDLE
   │       if RobotInterface.align_to_tag(next_wp):
   │           waypoint_index += 1
   │           state = IDLE
   │
   ├─ 4.3 Backward Edge (후진)
   │   (Straight Edge와 동일, edge['direction']에 'backward' 포함)
   │
   └─ 4.4 Scan Mode (Mode 4 Only)
       if scan_mode_active and next_wp in scan_tags:
           ├─ State: ALIGNING → WAIT_SCAN
           │   state = WAIT_SCAN
           │
           └─ State: WAIT_SCAN → IDLE
               if RobotInterface.wait_for_scan(next_wp):
                   waypoint_index += 1
                   state = IDLE

5. 실시간 디버깅 상태 발행 (매 루프)
   RobotInterface.publish_debug_status(
       current_wp=current_wp,
       next_wp=next_wp,
       total_wps=len(waypoints),
       wp_idx=waypoint_index
   )
```

**현장 테스트 단위:**
- **2** 웨이포인트 추출: 인덱스 범위 검증
- **3** Edge 조회: `map.yaml`에서 from-to 엣지 존재
- **4.1** Straight 실행: IDLE → MOVING → ALIGNING 상태 전환
- **4.2** Pivot 실행: IDLE → MOVING → ALIGNING → ROTATING
- **4.3** Backward 실행: 후진 방향 정확성
- **4.4** Scan 실행: /scan_finished 신호 대기
- **5** Debug 발행: `/nav_debug_status` 토픽 발행 확인

---

### F. RobotInterface.move_to_tag()

**상위 함수:** `move_to_tag(target_tag, edge_info)`

**하위 함수 호출 순서:**

```python
1. Edge 정보에서 방향 추출
   direction = edge_info['direction']
   move_dir = 'backward' if 'backward' in direction else 'forward'

2. VisionModule.is_tag_visible(target_tag)
   │
   ├─ return target_tag in detected_tags
   │
   ├─ True: 태그 보임 → 3번으로
   │
   └─ False: 태그 안 보임 → 직진 모드
       ├─ linear = linear_speed if move_dir == 'forward' else -linear_speed
       ├─ RobotController.move(linear, 0)
       └─ return False  # 계속 이동

3. VisionModule.get_tag_data(target_tag)
   │
   ├─ return detected_tags[target_tag]
   │
   └─ 반환값: {
          'x': lateral,         # 카메라 중심 대비 좌우 offset
          'y': vertical,        # 상하 offset
          'z': distance,        # 거리
          'center_x': pixel_x,  # 픽셀 x 좌표
          'center_y': pixel_y,  # 픽셀 y 좌표 ← 도착 판정용
          'yaw': yaw_angle,
          'corners': corners
      }

4. 도착 판정
   lateral = tag_data['x']
   center_y = tag_data['center_y']

   if abs(center_y - image_center_y) < center_y_tolerance:  # default: 50px
       rospy.loginfo(f">>> ARRIVED at tag {target_tag}")
       return True

5. RobotController.get_estimated_pose()
   │
   ├─ 5.1 Odometry delta 계산
   │      odom_dx = odom_x - ref_odom_x
   │      odom_dy = odom_y - ref_odom_y
   │      theta_delta = current_theta - ref_odom_theta
   │
   └─ 5.2 로봇 위치 추정
          robot_x = ref_robot_x + odom_dx
          robot_y = ref_robot_y + odom_dy
          robot_heading = ref_robot_heading + theta_delta

          return robot_x, robot_y, robot_heading

6. MapManager.tag_db.get_position(target_tag)
   │
   ├─ tag_info = tags[target_tag]
   │
   └─ return tag_info['x'], tag_info['y']

7. PurePursuitController.calculate_absolute(
       robot_x, robot_y, robot_heading,
       target_x, target_y, lateral, move_dir
   )
   │
   ├─ 7.1 목표까지 거리 & 각도 계산
   │      dx = target_x - robot_x
   │      dy = target_y - robot_y
   │      distance = sqrt(dx² + dy²)
   │      angle_to_target = atan2(dy, dx)
   │
   ├─ 7.2 Alpha (각도 오차) 계산
   │      if move_dir == 'backward':
   │          alpha = angle_to_target - (robot_heading + π)
   │      else:
   │          alpha = angle_to_target - robot_heading
   │
   │      # Normalize to [-π, π]
   │      while alpha > π: alpha -= 2π
   │      while alpha < -π: alpha += 2π
   │
   ├─ 7.3 Curvature 계산
   │      L_eff = max(distance, look_ahead_base)  # min 0.4m
   │      curvature = (2 * sin(alpha)) / L_eff
   │
   ├─ 7.4 Angular velocity 계산
   │      if move_dir == 'backward':
   │          w = -abs(linear_speed) * curvature * gain_backward
   │      else:
   │          w = linear_speed * curvature * gain_forward
   │
   │      # 목표 근처에서 감속
   │      if distance < 0.3:
   │          w *= (distance / 0.3)
   │
   └─ 7.5 클램핑
          return clip(w, -0.3, 0.3)

8. PurePursuitController.calculate_wall_distance(lateral, align_angle, zone)
   │
   ├─ 8.1 Zone별 기준 거리 설정
   │      if zone in ['B', 'C', 'D', 'E']:
   │          base_dist = 0.35m  # Work zone
   │      else:
   │          base_dist = 0.6275m  # Zone A (corridor)
   │
   ├─ 8.2 Lateral offset 보정
   │      if zone in ['B', 'D']:
   │          center_to_wall = base_dist + lateral  # lateral>0 = 벽에 가까움
   │      elif zone in ['C', 'E']:
   │          center_to_wall = base_dist + lateral
   │      else:  # Zone A
   │          center_to_wall = base_dist - lateral  # lateral>0 = 벽에서 멀어짐
   │
   └─ 8.3 로봇 회전 보정
          angle_rad = radians(align_angle_deg)
          half_length = robot_length / 2  # 0.4m
          half_width = robot_width / 2    # 0.25m

          right_side_dist = center_to_wall
                          - half_width * cos(angle_rad)
                          - half_length * abs(sin(angle_rad))

          return right_side_dist

9. 속도 명령 발행
   ├─ 9.1 Linear 속도 설정
   │      linear = linear_speed
   │      if move_dir == 'backward':
   │          linear = -linear
   │      if target_tag == 508:  # Dock 접근 시 감속
   │          linear *= slow_factor  # 0.3
   │
   └─ 9.2 RobotController.move(linear, angular)
          twist = Twist()
          twist.linear.x = linear
          twist.angular.z = angular
          cmd_pub.publish(twist)
```

**현장 테스트 단위:**

- **2** 태그 가시성: 카메라 시야에 태그 있는지
- **3** 태그 데이터: x, y, z, center_x, center_y 정확도
- **4** 도착 판정: center_y가 이미지 중심(360px) 근처
- **5** Odometry 추정: 참조점 대비 delta 계산
- **6** 목표 좌표: `map.yaml`에서 정확한 (x, y) 반환
- **7.1~7.5** Pure Pursuit: Alpha, Curvature, Angular 수식
- **8.1~8.3** 벽 거리: Zone별 다른 계산식
- **9** 속도 발행: `/cmd_vel` 토픽 정상 발행

---

### G. RobotInterface.rotate_90()

**상위 함수:** `rotate_90(direction)`

**하위 함수 호출 순서:**

```python
1. 회전 시작 확인
   if not rotation_ctrl.is_rotating:
       rotation_ctrl.start_rotation(90, direction)
       ├─ 1.1 현재 heading 획득
       │      current = RobotController.get_heading()
       │      # return current_theta (from /odom callback)
       │
       ├─ 1.2 목표 heading 계산
       │      angle_rad = radians(90)
       │      if direction == 'cw':
       │          target_theta = current - angle_rad
       │      else:
       │          target_theta = current + angle_rad
       │
       ├─ 1.3 Normalize to [-π, π]
       │      while target_theta > π: target_theta -= 2π
       │      while target_theta < -π: target_theta += 2π
       │
       └─ 1.4 상태 플래그 설정
              is_rotating = True

2. RotationController.update() - 제어 루프 (30Hz)
   │
   ├─ 2.1 회전 상태 확인
   │      if not is_rotating:
   │          return True  # 이미 완료
   │
   ├─ 2.2 회전 오차 계산
   │      current = RobotController.get_heading()
   │      error = target_theta - current
   │
   │      # Normalize error to [-π, π]
   │      while error > π: error -= 2π
   │      while error < -π: error += 2π
   │
   ├─ 2.3 완료 판정
   │      if abs(degrees(error)) < tolerance_deg:  # default: 1°
   │          RobotController.stop()
   │          is_rotating = False
   │          rospy.loginfo("[ROTATION] Complete")
   │          return True
   │
   ├─ 2.4 PID 제어 (간소화된 P-controller)
   │      angular = 0.2 if error > 0 else -0.2
   │
   │      # 목표 근처에서 감속
   │      if abs(degrees(error)) < 10°:
   │          angular *= 0.5
   │
   └─ 2.5 속도 명령
          RobotController.move(0, angular)
          rospy.loginfo_throttle(0.3, f"[ROTATION] Error: {degrees(error):.1f}°")
          return False  # 아직 회전 중
```

**현장 테스트 단위:**

- **1.1** 현재 heading: `/odom` 토픽에서 quaternion → euler 변환
- **1.2** 목표 계산: CW는 음수(-π/2), CCW는 양수(+π/2)
- **1.3** 정규화: [-π, π] 범위 유지
- **2.2** 오차 계산: 목표 - 현재, 정규화
- **2.3** 완료 판정: 오차 < 1°
- **2.4** P 제어: 근접 시 0.5배 감속
- **2.5** 모터 명령: `/cmd_vel` 발행

---

### H. RobotInterface.align_to_tag()

**상위 함수:** `align_to_tag(tag_id)`

**하위 함수 호출 순서:**

```python
1. VisionModule.is_tag_visible(tag_id)
   │
   ├─ True: 태그 보임 → 2번으로
   │
   └─ False: 태그 안 보임
       rospy.loginfo(f"[ALIGN] Tag {tag_id} not visible, skip")
       return True  # 정렬 스킵

2. VisionModule.get_alignment_angle(tag_id)
   │
   ├─ 2.1 태그 데이터 확인
   │      tag_data = detected_tags.get(tag_id)
   │      if not tag_data:
   │          return None
   │
   ├─ 2.2 태그 코너 추출
   │      corners = tag_data['corners']
   │      if corners is None:
   │          return None
   │      # corners: [top-left, top-right, bottom-right, bottom-left]
   │
   ├─ 2.3 상단 엣지 각도 계산
   │      top_left = corners[0]   # (x0, y0)
   │      top_right = corners[1]  # (x1, y1)
   │      dx = top_right[0] - top_left[0]
   │      dy = top_right[1] - top_left[1]
   │      align_angle = atan2(dy, dx)
   │      align_angle_deg = degrees(align_angle)
   │
   └─ 2.4 각도 정규화 (Zone C/E는 180° 회전)
          if align_angle_deg > 90:
              align_angle_deg -= 180
          elif align_angle_deg < -90:
              align_angle_deg += 180

          return align_angle_deg

3. VisionModule.get_tag_lateral(tag_id)
   │
   └─ return tag_data['x']  # Lateral offset

4. 정렬 완료 판정
   if abs(align_angle_deg) < align_angle_threshold:  # default: 0.5°
       goto 5 (참조점 업데이트)
   else:
       goto 6 (정렬 제어)

5. 정렬 완료 시 참조점 업데이트
   │
   ├─ 5.1 PurePursuitController.get_robot_pose_from_tag(tag_id, lateral, align_angle_deg)
   │      │
   │      ├─ 5.1.1 태그 정보 조회
   │      │        tag_info = tag_db.get(tag_id)
   │      │        tag_x = tag_info['x']
   │      │        tag_y = tag_info['y']
   │      │        zone = tag_info['zone']
   │      │
   │      └─ 5.1.2 Zone별 로봇 위치 역산
   │               # Zone A / DOCK: 로봇 heading = +X 방향
   │               if zone == 'A' or zone == 'DOCK':
   │                   robot_x = tag_x
   │                   robot_y = tag_y + lateral
   │                   robot_heading = radians(align_angle_deg)
   │
   │               # Zone B / D: 로봇 heading = +Y 방향
   │               elif zone == 'B' or zone == 'D':
   │                   robot_x = tag_x - lateral
   │                   robot_y = tag_y
   │                   robot_heading = π/2 + radians(align_angle_deg)
   │
   │               # Zone C / E: 로봇 heading = -Y 방향
   │               elif zone == 'C' or zone == 'E':
   │                   robot_x = tag_x + lateral
   │                   robot_y = tag_y
   │                   robot_heading = -π/2 + radians(align_angle_deg)
   │
   │               return robot_x, robot_y, robot_heading
   │
   ├─ 5.2 RobotController.set_reference_point(robot_x, robot_y, robot_heading)
   │      ├─ ref_robot_x = robot_x
   │      ├─ ref_robot_y = robot_y
   │      ├─ ref_robot_heading = robot_heading
   │      ├─ ref_odom_x = current_odom_x  # Odometry 현재 값
   │      ├─ ref_odom_y = current_odom_y
   │      └─ ref_odom_theta = current_theta
   │
   ├─ 5.3 벽 거리 계산 (로깅용)
   │      zone = tag_db.get_zone(tag_id)
   │      wall_dist = pursuit.calculate_wall_distance(lateral, align_angle_deg, zone)
   │      min_clearance = 0.10m
   │      wall_status = "OK" if wall_dist >= min_clearance else "WARN!"
   │
   └─ 5.4 완료 로그
          rospy.loginfo(f"[ALIGN] Done! angle:{align_angle_deg:.2f}° "
                        f"lateral:{lateral:.3f}m wall:{wall_dist:.3f}m {wall_status}")
          return True

6. 정렬 제어 (align_angle_deg가 threshold 초과 시)
   │
   ├─ 6.1 Angular velocity 계산
   │      angular = -align_angle_deg * 0.8
   │      # Positive angle = 태그가 CW로 기울어짐 = CCW로 회전 필요 = positive angular
   │      # 하지만 제어 방향은 반대 (negative)
   │
   ├─ 6.2 최소 속도 보장 (마찰 극복)
   │      if abs(angular) < 0.08 and abs(align_angle_deg) > 0.3:
   │          angular = 0.08 if angular > 0 else -0.08
   │
   ├─ 6.3 최대 속도 제한
   │      angular = max(-0.2, min(0.2, angular))
   │
   └─ 6.4 속도 명령
          RobotController.move(0, angular)
          rospy.loginfo_throttle(0.3, f"[ALIGN] angle:{align_angle_deg:.2f}° "
                                       f"lat:{lateral:.3f}m ang:{angular:.2f}")
          return False  # 계속 정렬 중
```

**현장 테스트 단위:**

- **1** 태그 가시성: Vision에서 태그 검출
- **2.2** 코너 추출: AprilTag 검출기의 4개 코너
- **2.3** 각도 계산: Top-left, Top-right로 기울기
- **2.4** 정규화: Zone C/E는 180° 보정
- **3** Lateral: 태그 중심에서 offset
- **4** 판정: Angle < 0.5°
- **5.1.2** 위치 역산: Zone별 좌표 변환
- **5.2** 참조점 설정: Odometry drift 보정 기준
- **6.1~6.4** P 제어: 비례 제어, 최소/최대 제한

---

### I. RobotInterface.stop_and_wait()

**상위 함수:** `stop_and_wait(duration)`

**하위 함수 호출 순서:**

```python
1. 타이머 시작 확인
   if stop_start_time is None:
       ├─ stop_start_time = rospy.Time.now()
       └─ RobotController.stop()
              twist = Twist()  # 모든 값 0
              cmd_pub.publish(twist)

2. 경과 시간 계산
   elapsed = (rospy.Time.now() - stop_start_time).to_sec()

3. 대기 완료 판정
   if elapsed >= duration:  # default: 0.8s
       ├─ stop_start_time = None  # 타이머 리셋
       └─ return True

4. 대기 중
   return False
```

**현장 테스트 단위:**
- **1** 정지 명령: `/cmd_vel` (0, 0) 발행
- **2** 시간 측정: rospy.Time 정확도
- **3** 완료 판정: duration 경과 확인

---

### J. RobotInterface.wait_for_scan() (Mode 4 Only)

**상위 함수:** `wait_for_scan(tag_id)`

**하위 함수 호출 순서:**

```python
1. 로봇 정지
   RobotController.stop()

2. 스캔 완료 신호 확인
   if scan_finished:  # /scan_finished 토픽 콜백에서 설정
       ├─ rospy.loginfo(f"[SCAN MODE] Scan completed at tag {tag_id}")
       ├─ scan_finished = False  # 플래그 리셋
       └─ return True

3. 로봇 위치 발행 (주기적, 2초마다)
   ├─ 3.1 현재 위치 추정
   │      robot_x, robot_y, robot_heading = RobotController.get_estimated_pose()
   │
   └─ 3.2 publish_robot_pose(tag_id, robot_x, robot_y, degrees(robot_heading))
          ├─ 3.2.1 Isaac → Manipulator 좌표 변환
          │        cam_manip_x, cam_manip_y = CoordinateTransformer.isaac_to_manipulator(robot_x, robot_y)
          │        # manip_x = -isaac_y
          │        # manip_y = -isaac_x
          │
          ├─ 3.2.2 카메라 위치 → 로봇 중심 offset 적용
          │        zone = tag_db.get_zone(tag_id)
          │        pub_x, pub_y = CoordinateTransformer.apply_robot_center_offset(
          │            cam_manip_x, cam_manip_y, zone, camera_offset=0.45
          │        )
          │        # Zone A/DOCK: robot_y = camera_y + 0.45
          │        # Zone B/D: robot_x = camera_x + 0.45
          │        # Zone C/E: robot_x = camera_x - 0.45
          │
          ├─ 3.2.3 Pose2DWithFlag 메시지 생성
          │        pm = Pose2DWithFlag()
          │        pm.header.stamp = rospy.Time.now()
          │        pm.x = pub_x
          │        pm.y = pub_y
          │        pm.theta = theta_deg
          │        pm.flag = True
          │        pm.id = tag_id
          │
          └─ 3.2.4 /robot_pose 토픽 발행
                   pose_pub.publish(pm)

4. 대기 중 로그
   rospy.loginfo_throttle(2.0, f"[SCAN MODE] Waiting for /scan_finished at tag {tag_id}...")
   return False
```

**현장 테스트 단위:**
- **1** 정지: 로봇 정지 확인
- **2** 신호 확인: `/scan_finished` 토픽 수신
- **3.2.1** 좌표 변환: Isaac → Manipulator
- **3.2.2** Offset 적용: Zone별 카메라-로봇 중심 offset
- **3.2.3** 메시지 생성: Pose2DWithFlag 필드
- **3.2.4** 발행: `/robot_pose` 토픽 발행

---

### K. RobotInterface.publish_debug_status()

**상위 함수:** `publish_debug_status(current_wp, next_wp, total_wps, wp_idx)`

**하위 함수 호출 순서:**

```python
1. 현재 Zone 조회
   zone = MapManager.tag_db.get_zone(current_tag)

2. 벽 거리 계산
   wall_dist = PurePursuitController.calculate_wall_distance(
       current_lateral, current_align_angle, zone
   )

3. Drift 방향 판정
   if zone in ['B', 'C', 'D', 'E']:
       drift_toward_wall = (current_lateral > 0)  # Work zone
   else:
       drift_toward_wall = (current_lateral < 0)  # Zone A

   drift_direction = "TOWARD_WALL" if drift_toward_wall else "AWAY_FROM_WALL"

4. 벽 거리 상태
   min_clearance = 0.10m
   wall_status = "OK" if wall_dist >= min_clearance else "WARNING"

5. JSON 상태 메시지 생성
   status = {
       'timestamp': rospy.Time.now().to_sec(),
       'state': self.state.name,  # IDLE, MOVING, ALIGNING, ROTATING, WAIT_SCAN
       'zone': zone,
       'current_tag': current_tag,
       'target_tag': current_wp,
       'next_tag': next_wp,
       'waypoint_progress': f"{wp_idx}/{total_wps}",
       'lateral_offset_m': round(current_lateral, 4),
       'drift_direction': drift_direction,
       'wall_distance_m': round(wall_dist, 4),
       'wall_status': wall_status,
       'align_angle_deg': round(current_align_angle, 2),
   }

6. /nav_debug_status 토픽 발행
   msg = String()
   msg.data = json.dumps(status)
   debug_status_pub.publish(msg)
```

**현장 테스트 단위:**
- **1** Zone 조회: 현재 태그의 Zone 확인
- **2** 벽 거리: Zone별 계산식
- **3** Drift 판정: Zone별 다른 로직
- **4** 상태 판정: 안전 거리 확인
- **5** JSON 생성: 모든 필드 포함
- **6** 토픽 발행: `/nav_debug_status` 발행

---

### L. MapManager - 경로 탐색 (BFS)

**상위 함수:** `NavigationGraph.find_path(start, goal)`

**하위 함수 호출 순서:**

```python
1. 예외 처리
   ├─ 1.1 시작 = 목표
   │      if start == goal:
   │          return [start]
   │
   └─ 1.2 시작점 엣지 없음
          if start not in edges:
              return None

2. BFS (Breadth-First Search) 초기화
   ├─ visited = {start}  # 방문한 노드 집합
   └─ queue = deque([(start, [start])])  # (현재 노드, 경로)

3. BFS 탐색 루프
   while queue:
       ├─ 3.1 큐에서 현재 노드 & 경로 추출
       │      current, path = queue.popleft()
       │
       ├─ 3.2 현재 노드의 엣지 확인
       │      if current not in edges:
       │          continue
       │
       ├─ 3.3 이웃 노드 순회
       │      for edge in edges[current]:
       │          next_tag = edge['to']
       │
       │          ├─ 3.3.1 방문 여부 확인
       │          │        if next_tag in visited:
       │          │            continue  # 이미 방문
       │          │
       │          ├─ 3.3.2 새 경로 생성
       │          │        new_path = path + [next_tag]
       │          │
       │          ├─ 3.3.3 목표 도달 확인
       │          │        if next_tag == goal:
       │          │            return new_path  # 경로 발견!
       │          │
       │          └─ 3.3.4 큐에 추가
       │                   visited.add(next_tag)
       │                   queue.append((next_tag, new_path))

4. 경로 없음
   return None  # 큐가 비었는데 목표를 못 찾음
```

**현장 테스트 단위:**
- **1.1** 예외: 시작=목표 처리
- **1.2** 예외: 엣지 없는 노드
- **2** 초기화: visited, queue 구조
- **3.1** Deque 연산: popleft()
- **3.3.1** 중복 방지: visited 집합
- **3.3.2** 경로 누적: path + [next_tag]
- **3.3.3** 목표 도달: 즉시 반환
- **4** 실패: 연결 안 된 노드

---

### M. VisionModule - AprilTag 검출

**상위 함수:** `_image_callback(msg)` - 카메라 이미지 수신 콜백

**하위 함수 호출 순서:**

```python
1. 카메라 캘리브레이션 확인
   if not camera_info_received:
       return  # 캘리브레이션 정보 없으면 스킵

2. ROS Image → OpenCV 변환
   ├─ 2.1 cv_bridge를 통한 변환
   │      cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
   │      # sensor_msgs/Image → numpy array (H, W, 3)
   │
   └─ 2.2 Grayscale 변환
          gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
          # (H, W, 3) → (H, W)

3. AprilTag 검출 (dt_apriltags 라이브러리)
   detections = detector.detect(
       gray,
       estimate_tag_pose=True,
       camera_params=[fx, fy, cx, cy],
       tag_size=0.06  # 6cm
   )
   # PnP 알고리즘으로 3D pose 추정

4. 검출 결과 처리
   ├─ 4.1 이전 검출 초기화
   │      detected_tags.clear()
   │
   └─ 4.2 검출된 각 태그 처리
          for det in detections:
              if det.pose_t is not None:
                  │
                  ├─ 4.2.1 Pose 추출
                  │        pose_t = det.pose_t.flatten()  # [x, y, z]
                  │        pose_R = det.pose_R  # 3x3 rotation matrix
                  │
                  ├─ 4.2.2 Yaw 각도 계산
                  │        yaw = atan2(pose_R[1,0], pose_R[0,0])
                  │        # Rotation matrix에서 Z축 회전 추출
                  │
                  └─ 4.2.3 검출 정보 저장
                           detected_tags[det.tag_id] = {
                               'x': pose_t[0],        # Lateral offset
                               'y': pose_t[1],        # Vertical offset
                               'z': pose_t[2],        # Distance to tag
                               'yaw': yaw,            # Tag rotation
                               'pose_R': pose_R,
                               'center_x': det.center[0],  # Pixel x
                               'center_y': det.center[1],  # Pixel y
                               'corners': det.corners,     # 4 corners
                           }
```

**현장 테스트 단위:**
- **1** 캘리브레이션: `/camera_info` 수신 확인
- **2.1** 이미지 변환: ROS msg → OpenCV
- **2.2** Grayscale: BGR → Gray 변환
- **3** AprilTag 검출: dt_apriltags PnP 알고리즘
- **4.2.1** Pose 추출: Translation vector
- **4.2.2** Yaw 계산: Rotation matrix
- **4.2.3** 저장: Dictionary 구조

---

## 최소 단위 함수 트리

### ROS 콜백 함수 (가장 작은 단위)

```
┌─ ROS Subscribers (비동기 콜백)
│
├─ VisionModule._camera_info_callback(msg)
│   ├─ K = msg.K  # 3x3 camera matrix
│   ├─ fx = K[0], fy = K[4]
│   ├─ cx = K[2], cy = K[5]
│   ├─ camera_params = [fx, fy, cx, cy]
│   └─ camera_info_received = True
│
├─ VisionModule._image_callback(msg)
│   └─ (위 M섹션 참조)
│
├─ RobotController._odom_callback(msg)
│   ├─ odom_x = msg.pose.pose.position.x
│   ├─ odom_y = msg.pose.pose.position.y
│   ├─ q = msg.pose.pose.orientation  # Quaternion
│   ├─ euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
│   ├─ current_theta = euler[2]  # yaw angle
│   └─ odom_received = True
│
└─ RobotInterface._scan_finished_callback(msg)  # Mode 4
    └─ if msg.data:
           scan_finished = True
```

### 수학 함수 (순수 계산, 부작용 없음)

```
┌─ 각도 정규화
│   def _normalize_angle(angle):
│       while angle > π: angle -= 2π
│       while angle < -π: angle += 2π
│       return angle
│
├─ 각도 차이 계산
│   def _angle_difference(target, current):
│       diff = target - current
│       return _normalize_angle(diff)
│
├─ 클램핑
│   def _clamp(value, min_val, max_val):
│       return max(min_val, min(value, max_val))
│
├─ 유클리드 거리
│   def _distance(x1, y1, x2, y2):
│       dx = x2 - x1
│       dy = y2 - y1
│       return sqrt(dx*dx + dy*dy)
│
├─ Quaternion → Euler 변환
│   def _quaternion_to_euler(qx, qy, qz, qw):
│       import tf.transformations as tft
│       return tft.euler_from_quaternion([qx, qy, qz, qw])
│
└─ 좌표 변환
    ├─ Isaac → Manipulator
    │   def isaac_to_manipulator(isaac_x, isaac_y):
    │       return -isaac_y, -isaac_x
    │
    └─ Camera → Robot Center
        def apply_robot_center_offset(manip_x, manip_y, zone, offset=0.45):
            if zone in ['A', 'DOCK']:
                return manip_x, manip_y + offset
            elif zone in ['B', 'D']:
                return manip_x + offset, manip_y
            elif zone in ['C', 'E']:
                return manip_x - offset, manip_y
```

### Pure Pursuit 수식 (단계별 분해)

```
PurePursuitController.calculate_absolute(robot_x, robot_y, robot_heading,
                                          target_x, target_y, lateral, move_dir)
│
├─ Step 1: 목표까지 벡터 계산
│   dx = target_x - robot_x
│   dy = target_y - robot_y
│
├─ Step 2: 거리 계산
│   distance = sqrt(dx*dx + dy*dy)
│
├─ Step 3: 목표 방향 각도
│   angle_to_target = atan2(dy, dx)
│
├─ Step 4: Alpha (각도 오차) 계산
│   if move_dir == 'backward':
│       alpha = angle_to_target - (robot_heading + π)
│   else:
│       alpha = angle_to_target - robot_heading
│   alpha = _normalize_angle(alpha)
│
├─ Step 5: Look-ahead distance
│   L_eff = max(distance, look_ahead_base)
│
├─ Step 6: Curvature 계산
│   curvature = (2 * sin(alpha)) / L_eff
│
├─ Step 7: Angular velocity 계산
│   if move_dir == 'backward':
│       w = -abs(linear_speed) * curvature * gain_backward
│   else:
│       w = linear_speed * curvature * gain_forward
│
├─ Step 8: 근접 감속
│   if distance < 0.3:
│       w *= (distance / 0.3)
│
├─ Step 9: Deadband
│   if abs(lateral) < deadband_cte and abs(alpha) < deadband_heading:
│       w = 0.0
│
└─ Step 10: 클램핑
    return _clamp(w, -0.3, 0.3)
```

### 벽 거리 계산 수식 (단계별 분해)

```
PurePursuitController.calculate_wall_distance(lateral, align_angle_deg, zone)
│
├─ Step 1: Zone별 기준 거리
│   if zone in ['B', 'C', 'D', 'E']:
│       base_dist = 0.35  # Work zone
│   else:
│       base_dist = 0.6275  # Zone A (corridor)
│
├─ Step 2: Lateral offset 부호 결정
│   # Zone A: lateral > 0 = 벽에서 멀어짐
│   # Zone B/D/C/E: lateral > 0 = 벽에 가까워짐
│
│   if zone in ['B', 'D', 'C', 'E']:
│       center_to_wall = base_dist + lateral
│   else:
│       center_to_wall = base_dist - lateral
│
├─ Step 3: 로봇 회전 고려
│   angle_rad = radians(align_angle_deg)
│   half_length = robot_length / 2  # 0.4m
│   half_width = robot_width / 2    # 0.25m
│
├─ Step 4: 벽까지 거리 (우측면 기준)
│   right_side_dist = center_to_wall
│                   - half_width * cos(angle_rad)
│                   - half_length * abs(sin(angle_rad))
│
└─ Step 5: 반환
    return right_side_dist
```

### 로봇 위치 역산 (태그 기반)

```
PurePursuitController.get_robot_pose_from_tag(tag_id, lateral, align_angle_deg)
│
├─ Step 1: 태그 정보 조회
│   tag_x, tag_y = tag_db.get_position(tag_id)
│   zone = tag_db.get_zone(tag_id)
│
├─ Step 2: Zone별 좌표 변환
│
│   ┌─ Zone A / DOCK (로봇 heading = 0°, +X 방향)
│   │   robot_x = tag_x
│   │   robot_y = tag_y + lateral
│   │   robot_heading = radians(align_angle_deg)
│   │
│   ├─ Zone B / D (로봇 heading = 90°, +Y 방향)
│   │   robot_x = tag_x - lateral
│   │   robot_y = tag_y
│   │   robot_heading = π/2 + radians(align_angle_deg)
│   │
│   └─ Zone C / E (로봇 heading = -90°, -Y 방향)
│       robot_x = tag_x + lateral
│       robot_y = tag_y
│       robot_heading = -π/2 + radians(align_angle_deg)
│
└─ Step 3: 반환
    return robot_x, robot_y, robot_heading
```

---

## 예외 처리 로직

### 1. 태그 미검출 예외

**발생 위치:** `move_to_tag()`, `align_to_tag()`

**처리 방법:**

```python
# move_to_tag() - 태그 안 보일 때
if not vision.is_tag_visible(target_tag):
    # 예외 처리: 직진 모드
    linear = linear_speed if move_dir == 'forward' else -linear_speed
    robot.move(linear, 0)
    rospy.loginfo_throttle(1.0, f"[MOVE] -> {target_tag} | no tag | straight | {move_dir}")
    return False  # 계속 이동

# align_to_tag() - 태그 안 보일 때
if not vision.is_tag_visible(tag_id):
    # 예외 처리: 정렬 스킵
    rospy.loginfo(f"[ALIGN] Tag {tag_id} not visible, skip")
    return True  # 정렬 완료로 간주
```

**현장 테스트:**
- 태그를 가리고 로봇이 직진하는지 확인
- 태그 시야에서 벗어날 때 정렬 스킵 확인

---

### 2. 경로 없음 예외

**발생 위치:** `NavigationGraph.find_path()`

**처리 방법:**

```python
# 경로 탐색 실패
path = nav_graph.find_path(start, goal)
if path is None:
    rospy.logerr(f"[NAV] No path found from {start} to {goal}")
    return []  # 빈 리스트 반환

# Mission에서 처리
if not waypoints:
    rospy.logerr("[MISSION] No waypoints, cannot start")
    return False
```

**현장 테스트:**
- 존재하지 않는 태그 ID 입력 시 오류 처리
- 연결 안 된 노드 쌍 테스트

---

### 3. Excel 파일 오류 예외

**발생 위치:** `TaskManager.get_excel_scan_waypoints()`

**처리 방법:**

```python
try:
    import pandas as pd
    df = pd.read_excel(excel_path, usecols=['group_id'])

    # ... 처리 ...

except FileNotFoundError:
    rospy.logerr(f"[TASK MANAGER] Excel file not found: {excel_path}")
    return [508], []  # Dock만 반환

except KeyError:
    rospy.logerr("[TASK MANAGER] 'group_id' column not found in Excel")
    return [508], []

except Exception as e:
    rospy.logerr(f"[TASK MANAGER] Failed to read Excel: {e}")
    return [508], []
```

**현장 테스트:**
- 존재하지 않는 파일 경로
- group_id 컬럼 없는 Excel
- 손상된 Excel 파일

---

### 4. Odometry 미수신 예외

**발생 위치:** `wait_until_ready()`

**처리 방법:**

```python
timeout = rospy.Time.now() + rospy.Duration(10.0)  # 10초 타임아웃

while not rospy.is_shutdown():
    if vision.is_ready() and robot.is_ready() and vision.is_tag_visible(508):
        return True

    if rospy.Time.now() > timeout:
        rospy.logerr("[ROBOT INTERFACE] Timeout waiting for sensors")
        return False

    rate.sleep()
```

**현장 테스트:**
- `/odom` 토픽 발행 안 할 때
- `/camera_info` 토픽 발행 안 할 때
- Timeout 10초 확인

---

### 5. 회전 Timeout 예외

**발생 위치:** `RotationController.update()`

**처리 방법:**

```python
# Main loop에서 timeout 관리
timeout = rospy.Time.now() + rospy.Duration(10.0)

while not rospy.is_shutdown():
    if robot.rotate_90(direction):
        break  # 회전 완료

    if rospy.Time.now() > timeout:
        rospy.logerr(f"[ROTATION] Timeout after 10s")
        robot.stop()
        break  # 실패로 간주하고 다음 단계

    rate.sleep()
```

**현장 테스트:**
- IMU 고장 시뮬레이션
- 회전 막힌 상황 (장애물)

---

### 6. 벽 충돌 경고

**발생 위치:** `move_to_tag()`, `publish_debug_status()`

**처리 방법:**

```python
# 벽 거리 계산
wall_dist = pursuit.calculate_wall_distance(lateral, align_angle, zone)
min_clearance = map_manager.get_param('wall_distances.min_clearance', 0.10)

if wall_dist < min_clearance:
    rospy.logwarn(f"[WALL] Distance {wall_dist:.3f}m < {min_clearance}m WARN!")
    # 경고만 출력, 주행은 계속 (실제 충돌은 하드웨어에서 감지)
```

**현장 테스트:**
- 로봇을 벽 근처(< 10cm)로 이동
- 경고 메시지 출력 확인
- `/nav_debug_status`에 "WARNING" 표시

---

### 7. Scan Mode 신호 Timeout

**발생 위치:** `wait_for_scan()`

**처리 방법:**

```python
# Main loop에서 timeout 관리
scan_timeout = rospy.Time.now() + rospy.Duration(300.0)  # 5분

while not rospy.is_shutdown():
    if robot.wait_for_scan(tag_id):
        break  # 스캔 완료

    if rospy.Time.now() > scan_timeout:
        rospy.logwarn(f"[SCAN] Timeout waiting for /scan_finished at tag {tag_id}")
        break  # 다음 웨이포인트로 강제 진행

    rate.sleep()
```

**현장 테스트:**
- `/scan_finished` 신호 발행 안 하기
- 5분 timeout 확인
- 강제 진행 동작 확인

---

## 현장 테스트 단위

### 테스트 레벨 구조

```
Level 0: ROS 인프라 (최하위)
├─ /camera_info 토픽 발행 확인
├─ /odom 토픽 발행 확인
├─ /cmd_vel 토픽 구독 확인
└─ map.yaml 파일 존재 및 문법

Level 1: 개별 하위 함수 (단위 테스트)
├─ Vision: 태그 검출, 각도 계산, lateral offset
├─ Map: 태그 조회, 경로 탐색, 태스크 로드
├─ Pure Pursuit: Forward/Backward 계산, 벽 거리
├─ Robot Controller: Odometry, 속도 명령, 회전 제어
└─ Coordinate Transformer: 좌표 변환

Level 2: 상위 함수 (통합 테스트)
├─ move_to_tag(): Vision + Pure Pursuit + Robot Controller
├─ rotate_90(): Rotation Controller + Robot Controller
├─ align_to_tag(): Vision + Pure Pursuit + Robot Controller
├─ wait_for_scan(): Pose publish + Scan signal
└─ publish_debug_status(): Status monitoring

Level 3: 전체 미션 (시스템 테스트)
├─ Task 1 실행
├─ Task 2 실행
├─ Mode 3 (Direct navigation)
└─ Mode 4 (Excel Scan)

Level 4: 예외 처리 (Robustness 테스트)
├─ 태그 미검출 처리
├─ 경로 없음 처리
├─ Excel 오류 처리
├─ Odometry timeout
├─ 회전 timeout
└─ 벽 충돌 경고
```

### 개별 하위 함수 테스트 목록

#### Level 0: ROS 인프라

| 항목 | 테스트 방법 | 통과 기준 |
|------|----------|---------|
| Camera 토픽 | `rostopic hz /rgb` | > 10 Hz |
| Camera Info | `rostopic echo /camera_info -n 1` | K matrix 존재 |
| Odometry | `rostopic hz /odom` | > 20 Hz |
| Cmd Vel | `rostopic info /cmd_vel` | Subscribers: 1 |
| Map YAML | `cat config/map.yaml` | 73개 태그, edges, tasks |

#### Level 1: Vision Module

| 함수 | 테스트 내용 | 검증 방법 |
|------|------------|---------|
| `is_ready()` | 카메라 캘리브레이션 수신 | `/camera_info` 토픽 확인 |
| `get_detected_tags()` | 태그 검출 | 태그를 1m 거리에 놓고 검출 여부 |
| `get_tag_data()` | 거리 측정 정확도 | 0.5m, 1.0m, 2.0m에서 오차 < 10% |
| `get_alignment_angle()` | 정렬 각도 계산 | 태그 5°, 10° 기울여서 측정 |
| `get_tag_lateral()` | Lateral offset | 로봇 10cm 좌/우 이동 후 측정 |
| `_image_callback()` | 콜백 실행 | 이미지 수신 시 detected_tags 업데이트 |

#### Level 1: Map Manager

| 함수 | 테스트 내용 | 검증 방법 |
|------|------------|---------|
| `tag_db.get_position()` | 태그 좌표 조회 | Tag 508: (-0.3975, 5.35) |
| `tag_db.get_zone()` | Zone 분류 | Tag 100 → 'B', Tag 123 → 'C' |
| `tag_db.exists()` | 태그 존재 확인 | Tag 508: True, Tag 999: False |
| `nav_graph.find_path()` | BFS 경로 탐색 | 508→100 경로 길이 = 3 |
| `nav_graph.get_edge()` | 엣지 조회 | 500→501: pivot, cw |
| `task_manager.get_task_waypoints()` | 태스크 로드 | Task 1: 45개 웨이포인트 |
| `task_manager.get_excel_scan_waypoints()` | Excel 파싱 | group_id 추출 및 경로 생성 |

#### Level 1: Pure Pursuit Controller

| 함수 | 테스트 내용 | 검증 방법 |
|------|------------|---------|
| `calculate_forward()` | Forward Pure Pursuit | lateral=0 → angular≈0 |
| `calculate_backward()` | Backward Pure Pursuit | lateral=0.1 → angular<0 |
| `calculate_absolute()` | 절대 좌표 제어 | Alpha, Curvature 수식 검증 |
| `calculate_wall_distance()` | 벽 거리 계산 | Zone A: 0.38m, Zone B: 0.10m |
| `get_robot_pose_from_tag()` | 태그 기반 위치 역산 | Zone별 좌표 변환 검증 |

#### Level 1: Robot Controller

| 함수 | 테스트 내용 | 검증 방법 |
|------|------------|---------|
| `is_ready()` | Odometry 수신 | `/odom` 토픽 5초 내 수신 |
| `get_position()` | 위치 조회 | Odometry x, y 반환 |
| `get_heading()` | Heading 조회 | Quaternion → Euler 변환 |
| `get_estimated_pose()` | Odom 기반 추정 | Delta 계산 검증 |
| `set_reference_point()` | 참조점 설정 | ref_* 변수 업데이트 |
| `move()` | 속도 명령 | `/cmd_vel` 토픽 발행 확인 |
| `_odom_callback()` | 콜백 실행 | odom_x, odom_y, current_theta 업데이트 |

#### Level 1: Rotation Controller

| 함수 | 테스트 내용 | 검증 방법 |
|------|------------|---------|
| `start_rotation()` | 목표 각도 설정 | Target theta 계산 검증 |
| `get_rotation_error()` | 회전 오차 계산 | Normalize [-π, π] 확인 |
| `is_complete()` | 완료 판정 | 오차 < 1° 확인 |
| `update()` | PID 제어 루프 | 90° 회전 정확도 ±5° |

#### Level 1: Coordinate Transformer

| 함수 | 테스트 내용 | 검증 방법 |
|------|------------|---------|
| `isaac_to_manipulator()` | 좌표 변환 | (-0.3975, 0.45) → (-0.45, 0.3975) |
| `apply_robot_center_offset()` | Robot center offset | Zone별 0.45m offset 적용 |

### Level 2: 상위 함수 통합 테스트

#### move_to_tag()
1. **Setup**: 로봇을 tag 508에 배치, tag 100 방향
2. **Test**: `move_to_tag(100, edge)`
3. **Verify**:
   - Vision에서 tag 100 검출
   - Pure Pursuit으로 경로 계산
   - 로봇이 tag 100에 도착 (center_y ≈ 360px)
   - Lateral offset < 5cm

#### rotate_90()
1. **Setup**: 로봇을 평평한 바닥에 배치, 초기 heading 기록
2. **Test**: `rotate_90('cw')`
3. **Verify**:
   - 회전 시작 (target theta = current - π/2)
   - 회전 중 PID 제어 (angular velocity ≈ 0.2 rad/s)
   - 회전 완료 (오차 < 5°)
   - 로봇 정지 확인

#### align_to_tag()
1. **Setup**: 로봇을 tag 100 앞에 배치, 5° 정도 기울임
2. **Test**: `align_to_tag(100)`
3. **Verify**:
   - Alignment angle 계산 (코너 기반)
   - 회전 제어 (angular = -angle * 0.8)
   - 정렬 완료 (angle < 0.5°, lateral < 5cm)
   - 참조점 업데이트 확인

#### wait_for_scan() (Mode 4)
1. **Setup**: Mode 4 활성화, scan_tags 설정
2. **Test**: `wait_for_scan(104)`
3. **Verify**:
   - 로봇 정지
   - `/robot_pose` 토픽 2초마다 발행
   - Manipulator 좌표 변환 확인
   - `/scan_finished` 신호 수신 시 완료

#### publish_debug_status()
1. **Setup**: 로봇 이동 중
2. **Test**: `publish_debug_status(...)`
3. **Verify**:
   - `/nav_debug_status` 토픽 발행
   - JSON 형식 확인
   - 모든 필드 존재 (state, zone, lateral, wall_dist 등)

### Level 3: 전체 미션 테스트

#### Task 1 실행
1. **Setup**: 모든 태그 배치 (Zone B + C)
2. **Test**: `load_task('task1')` → `execute()` 루프
3. **Verify**:
   - 45개 웨이포인트 모두 방문
   - Dock(508)로 복귀
   - 실행 시간 기록
   - 실패한 웨이포인트 없음
   - Dock 복귀 정확도 (lateral offset < 5cm)

#### Mode 3: Direct Navigation
1. **Setup**: 로봇 at Dock, 목표 tag 입력
2. **Test**: `rosrun apriltag_navigation main_node.py --mode 3 --tag 123`
3. **Verify**:
   - BFS 경로 생성
   - 최단 경로로 이동
   - 목표 태그 도착

#### Mode 4: Excel Scan
1. **Setup**: `data/excel/test.xlsx` 준비 (group_id 컬럼)
2. **Test**: `rosrun apriltag_navigation main_node.py --mode 4 --excel test.xlsx`
3. **Verify**:
   - Scan tags 추출 확인
   - 전체 경로 생성 (508 시작, 508 종료)
   - 각 scan tag에서 정지 → `/robot_pose` 발행
   - `/scan_finished` 신호 대기
   - Manipulator로 pose 전달 확인

### Level 4: 예외 처리 테스트

#### 태그 미검출
1. **Test**: 태그를 가림
2. **Verify**: 직진 모드로 전환 ("no tag | straight" 로그)

#### 경로 없음
1. **Test**: 존재하지 않는 태그 ID (999)
2. **Verify**: 오류 로그, 빈 waypoints 반환

#### Excel 오류
1. **Test**: 잘못된 파일 경로, 손상된 파일
2. **Verify**: Exception catch, [508] 반환

#### Odometry Timeout
1. **Test**: `/odom` 토픽 stop
2. **Verify**: 10초 후 timeout, 오류 메시지

#### 회전 Timeout
1. **Test**: IMU 고장 시뮬레이션
2. **Verify**: 10초 후 강제 정지

#### 벽 충돌 경고
1. **Test**: 로봇을 벽 < 10cm 위치
2. **Verify**: "WARN!" 로그, `/nav_debug_status`에 "WARNING"

---

## 요약

본 보고서는 AprilTag Navigation 시스템의 **완전한 동작 흐름**을 다음과 같이 기술했습니다:

### 1. 전체 프로젝트 관점의 핵심 동작 (8단계)
1. **초기화** - ROS 노드, 서브시스템 (Map, Vision, Robot Controller)
2. **맵 로딩** - map.yaml에서 73개 태그, 엣지, 태스크
3. **준비 대기** - 카메라, Odometry, Dock 태그 검출
4. **태스크 선택** - Mode 1/2/3/4
5. **경로 생성** - BFS 알고리즘, Excel 파싱
6. **웨이포인트 순회** - 이동/회전/정렬 조합
7. **실시간 모니터링** - Debug status, Pose publish
8. **도킹 복귀** - 정확도 검증

### 2. 로봇 움직임 3가지 기본 동작
- `move_to_tag()` - Vision + Pure Pursuit
- `rotate_90()` - Rotation Controller
- `align_to_tag()` - Vision + Coordinate Transform

### 3. 상위 제어기 8개
1. NavigationMission
2. RobotInterface
3. MapManager (TagDatabase, NavigationGraph, TaskManager)
4. VisionModule
5. PurePursuitController
6. RobotController
7. RotationController
8. CoordinateTransformer

### 4. 최소 단위 함수 트리
- **ROS 콜백** (비동기): `_image_callback`, `_odom_callback`, `_camera_info_callback`
- **수학 함수** (순수 계산): `_normalize_angle`, `_clamp`, `_distance`
- **Pure Pursuit 수식** (10단계 분해)
- **벽 거리 계산** (5단계 분해)

### 5. 예외 처리 7가지
- 태그 미검출, 경로 없음, Excel 오류, Odometry timeout, 회전 timeout, 벽 충돌 경고, Scan timeout

### 6. 현장 테스트 전략
- **Level 0**: ROS 인프라
- **Level 1**: 개별 함수 (40+ 함수)
- **Level 2**: 통합 동작 (5개 상위 함수)
- **Level 3**: 전체 미션 (4개 모드)
- **Level 4**: 예외 처리 (7가지 시나리오)

모든 하위 함수는 **독립적으로 테스트 가능**하도록 설계되어, 현장에서 **문제 발생 시 최하위 함수부터 역으로 추적**하여 빠르게 원인을 파악할 수 있습니다.
