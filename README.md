# AprilTag Navigation

AprilTag 기반 ROS 네비게이션 패키지 - Pure Pursuit 경로 추종 알고리즘 사용

## 주요 기능

- **4가지 네비게이션 모드**: 사전 정의 Task, 직접 네비게이션, Excel 기반 스캔
- **Pure Pursuit 제어**: 전진/후진 경로 추종 및 정밀 정렬
- **데이터 기반 설정**: YAML로 맵, 경로, 파라미터 관리
- **모듈형 구조**: 깔끔한 계층 분리 (perception, navigation, hardware)
- **진단 도구**: 비전, 모터, 회전 독립 테스트

## 빠른 시작

### 설치

```bash
# 1. 작업공간에 클론
cd ~/catkin_ws/src
git clone https://github.com/WoonBong/apriltag_navigation.git

# 2. 의존성 설치
pip install dt-apriltags numpy opencv-python pandas pyyaml

# 3. 빌드
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 실행

```bash
# 네비게이션 시작
roslaunch apriltag_navigation navigation.launch

# 명령줄 모드
rosrun apriltag_navigation main_node.py --mode 1          # Task 1
rosrun apriltag_navigation main_node.py --mode 3 --tag 104  # 직접 네비게이션

# 진단 도구
rosrun apriltag_navigation main_node.py --test-vision --duration 10
rosrun apriltag_navigation main_node.py --test-motor --distance 2.0
```

## 네비게이션 모드

| 모드 | 설명 | 사용법 |
|------|------|--------|
| **Mode 1** | Task 1 (Zone B + C) | `--mode 1` |
| **Mode 2** | Task 2 (Zone D + E) | `--mode 2` |
| **Mode 3** | 직접 네비게이션 (연속) | `--mode 3 --tag <ID>` |
| **Mode 4** | Excel 기반 스캔 | `--mode 4 --excel <path>` |

**Mode 3 특징**: 목적지 도착 후 다음 목적지를 입력하여 연속 네비게이션 가능

## 패키지 구조

```
apriltag_navigation/
├── config/map.yaml          # 맵 설정 (태그, 엣지, Task)
├── nodes/main_node.py       # 메인 네비게이션 노드
├── src/apriltag_navigation/
│   ├── robot_interface.py   # 고수준 로봇 API
│   ├── map/                 # 맵 관리 (YAML 파싱, 경로 탐색)
│   ├── perception/          # AprilTag 감지 및 포즈 추정
│   ├── navigation/          # Pure Pursuit 제어
│   └── hardware/            # 모터 제어 및 오도메트리
├── docs/                    # 상세 문서
│   ├── QA.md               # 24가지 FAQ
│   ├── TEST.md             # 코드 구조 및 설계
│   ├── Report.md           # 간략 보고서
│   └── TAG_COORDINATES.md  # 태그 좌표 정보
└── data/excel/             # Excel 스캔 파일

```

## 의존성

- **ROS**: rospy, sensor_msgs, geometry_msgs, nav_msgs, tf
- **Python**: dt-apriltags, numpy, opencv-python, pandas, pyyaml

## 상세 문서

- **[QA.md](docs/QA.md)**: 24가지 상세 질문 & 답변
- **[TEST.md](docs/TEST.md)**: 전체 코드 구조 및 설계 문서
- **[TAG_COORDINATES.md](docs/TAG_COORDINATES.md)**: 태그 좌표 및 배치 정보

## 개발 및 기여

### 맵 수정

`config/map.yaml` 파일 편집:
- 태그 좌표 수정
- 엣지 추가/삭제
- Task waypoint 변경
- 로봇 파라미터 조정

### 경로 검증

```python
from apriltag_navigation.map.map_manager import MapManager

map_mgr = MapManager()
is_valid, msg = map_mgr.validate_waypoints([508, 500, 104])
print(f"Valid: {is_valid}, Message: {msg}")
```

## 변경 이력

자세한 변경 이력은 [CHANGELOG.md](CHANGELOG.md)를 참조하세요.

## 라이선스

MIT License

## 작성자

WoonBong (woonbong@konkuk.ac.kr)

---

**더 자세한 정보**: [docs/](docs/) 폴더의 문서 참조
