# AprilTag Navigation - Code Structure & Field Testing Report

본 문서는 AprilTag Navigation 시스템의 **상위 제어기 구조**와 각 상위 제어기가 호출하는 **하위 함수들의 실행 순서**를 설명합니다. 현장 테스트 시 각 하위 함수를 독립적으로 검증할 수 있도록 구성되었습니다.

---

## 목차
1. [시스템 개요](#시스템-개요)
2. [상위 제어기 목록](#상위-제어기-목록)
3. [상위 제어기별 하위 함수 호출 구조](#상위-제어기별-하위-함수-호출-구조)
4. [현장 테스트 단위](#현장-테스트-단위)

---

## 시스템 개요

### 전체 주행 완성을 위한 3가지 기본 동작

모든 주행 태스크는 다음 3가지 기본 동작의 조합으로 완성됩니다:

1. `move_to_tag()`    - 목표 태그까지 이동 (Vision + PurePursuit + Motor)
2. `rotate_90()`      - 90도 회전 (IMU/Odom + Motor)
3. `align_to_tag()`   - 태그에 정밀 정렬 (Vision + PID + Motor)

---

## 상위 제어기별 하위 함수 호출 구조 (Decomposition)

   **"작은 단위 동작의 집합"**을 분석한 내용입니다.

### A. `move_to_tag(target_id)` 실행 흐름

이 함수는 단독으로 실행되는 것이 아니라, 아래의 **5단계 하위 함수**들이 순차적으로 성공해야 완료됩니다.

1.  **Tag Detection (Vision)**
    * 함수: `VisionModule.detect_tags(image)`
    * 역할: 카메라 이미지에서 AprilTag의 4개 코너 점을 찾습니다.
2.  **Pose Estimation (Vision)**
    * 함수: `VisionModule.solve_pnp(corners)` (또는 `get_pose`)
    * 역할: 2D 이미지 좌표를 로봇 기준 3D 좌표(x, y, z)로 변환합니다.
3.  **Target Calculation (Map)**
    * 함수: `MapManager.get_target_coordinates(target_id)`
    * 역할: 맵 파일에서 목표 태그의 글로벌 좌표를 가져옵니다.
4.  **Control Calculation (Navigation)**
    * 함수: `PurePursuit.compute_velocity(robot_pose, target_pose)`
    * 역할: 현재 위치와 목표 위치의 차이를 계산하여 필요한 선속도($v$)와 각속도($\omega$)를 산출합니다.
5.  **Actuation (Hardware)**
    * 함수: `RobotController.send_velocity_command(v, w)`
    * 역할: 계산된 속도 값을 로봇의 바퀴(Motor Driver)로 전송합니다.

---

## 현장 테스트 단위 (Unit Test Plan)

현장 도착 시, 상위 기능을 바로 실행하지 않고 아래의 **"최소 단위 함수"**부터 순차적으로 테스트합니다.

| 레벨 | 테스트 항목 | 대상 함수 (Code Level) | 점검 사항 (Checklist) |
|:---:|:---|:---|:---|
| **Level 1** | **하드웨어/센서** | `get_image()` <br> `get_odom()` | - 카메라 영상이 끊김 없이 들어오는가? <br> - 엔코더 값이 바퀴 회전과 일치하는가? |
| **Level 2** | **인식/판단** | `detect_tags()` <br> `calculate_path()` | - 조명 변화에도 태그 ID가 정확히 뜨는가? <br> - 태그 거리가 1m일 때 계산 값도 1.0m인가? |
| **Level 3** | **단위 제어** | `move_to_tag()` <br> `rotate_90()` | - 태그를 향해 직진 후 정확히 멈추는가? <br> - 90도 회전 후 오차가 2도 이내인가? |
| **Level 4** | **전체 미션** | `run_mission()` | - A지점에서 B지점까지 연속 동작이 되는가? |