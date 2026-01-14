# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [1.0.1] - 2026-01-15

### Added
- **Mode 3 연속 네비게이션**: 목적지 도착 후 다음 목적지를 입력받아 계속 네비게이션 가능
  - `_ask_next_destination()` 메서드 추가
  - 'q' 입력으로 종료 및 dock 복귀
  - 잘못된 태그 입력 시 재입력 요청
- **경로 검증 기능**: `MapManager.validate_waypoints()` 추가
  - 태그 존재 여부 확인
  - 엣지 연결성 검증
  - 오류 메시지 반환
- **문서 구조화**: docs/ 폴더 생성 및 문서 이동
  - QA.md (24개 질문)
  - TEST.md (코드 구조)
  - Report.md (간략 보고서)
  - TAG_COORDINATES.md (태그 좌표)
- **QA.md 신규 질문 3개**:
  - 질문 22: Pure Pursuit는 태그가 보이는 환경에서만 각도 보정 실행?
  - 질문 23: forward_pivot_cw / forward_pivot_ccw 엣지 direction의 의미
  - 질문 24: Odometry가 SLAM 기준일 때 위치값 문제
- **README.md 간소화**: 상세 내용을 docs로 이동, 핵심 정보만 유지

### Fixed
- **pure_pursuit.py:198 주석 오류 수정**:
  - 수정 전: `lateral > 0 = FARTHER from wall`
  - 수정 후: `lateral > 0 = CLOSER to wall`
  - Zone A에서 lateral 부호의 의미 정확히 반영
- **QA.md 질문 9 예시 수정**:
  - 가상 예시 (509→104)를 실제 map.yaml 태그로 변경 (111→110)
  - Edge 패턴 시각화를 실제 구조로 업데이트

### Changed
- **main_node.py 리팩토링**:
  - `NavigationMission.__init__()`: mode 파라미터 추가
  - `execute()`: Mode 3일 때 연속 네비게이션 로직 추가
- **문서 정리**:
  - QUICKSTART.md 삭제 (README.md로 통합)
  - API_REFERENCE.md, ARCHITECTURE.md, COLLABORATION.md, TESTING_GUIDE.md 삭제 (TEST.md로 통합)

## [1.0.0] - 2026-01-13

### Added
- **초기 릴리스**: AprilTag 네비게이션 시스템
- **4가지 네비게이션 모드**:
  - Mode 1: Task 1 (Zone B + C)
  - Mode 2: Task 2 (Zone D + E)
  - Mode 3: 직접 네비게이션 (단일 목적지)
  - Mode 4: Excel 기반 스캔 모드
- **Pure Pursuit 경로 추종**:
  - 전진/후진 Pure Pursuit 알고리즘
  - 절대 좌표 기반 Pure Pursuit
  - 벽 거리 계산
- **모듈형 아키텍처**:
  - MapManager: YAML 기반 맵 관리, BFS 경로 탐색
  - VisionModule: AprilTag 감지 및 포즈 추정
  - PurePursuitController: Pure Pursuit 제어 알고리즘
  - RobotController: 모터 제어 및 오도메트리
  - RobotInterface: 고수준 로봇 API
- **진단 도구**:
  - `--test-vision`: 비전 시스템 테스트
  - `--test-motor`: 모터 이동 테스트
  - `--test-pivot`: 회전 테스트
- **데이터 기반 설정**:
  - map.yaml: 73개 태그, 엣지, Task 정의
  - 로봇 파라미터 (속도, 크기, 임계값)
- **상태 머신**:
  - IDLE, MOVING, STOPPING, ALIGNING, ROTATING, WAIT_SCAN
- **CLI 인터페이스**:
  - 대화형 모드 선택
  - 명령줄 인자 지원
- **문서**:
  - README.md: 패키지 개요 및 사용법
  - TEST.md: 코드 구조 상세 문서
  - TAG_COORDINATES.md: 태그 좌표 정보

### Technical Details
- **의존성**: ROS, dt-apriltags, numpy, opencv-python, pandas, pyyaml
- **ROS Topics**:
  - Subscribe: `/rgb`, `/camera_info`, `/odom`
  - Publish: `/cmd_vel`, `/nav_debug_status`, `/robot_pose`, `/scan_finished`
- **좌표계**: Isaac Sim 좌표계 기준
- **Zone 구조**: 5개 Zone (A, B, C, D, E) + DOCK

---

## 변경 이력 형식

### Added
새로운 기능

### Changed
기존 기능의 변경사항

### Deprecated
곧 제거될 기능

### Removed
제거된 기능

### Fixed
버그 수정

### Security
보안 관련 수정

---

[Unreleased]: https://github.com/WoonBong/apriltag_navigation/compare/v1.0.1...HEAD
[1.0.1]: https://github.com/WoonBong/apriltag_navigation/compare/v1.0.0...v1.0.1
[1.0.0]: https://github.com/WoonBong/apriltag_navigation/releases/tag/v1.0.0
