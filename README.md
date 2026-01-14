# AprilTag Navigation

AprilTag 기반 ROS 네비게이션 패키지

## 개발 환경
- Ubuntu 20.04 LTS
- ROS Noetic

## 설치

### Dependencies
```bash
# Python packages
pip install dt-apriltags numpy opencv-python pandas pyyaml
```

### Compile
```bash
cd ~/catkin_ws/src
git clone https://github.com/WoonBong/apriltag_navigation.git
cd ..
catkin_make
source devel/setup.bash
```

## 사용법

### Run the package
```bash
# Interactive mode (모드 선택)
rosrun apriltag_navigation main_node.py

# Mode 1: Task 1/2 실행
rosrun apriltag_navigation main_node.py --mode 1

# Mode 3: 특정 태그로 직접 이동
rosrun apriltag_navigation main_node.py --mode 3 --tag 5

# Mode 4: Excel 기반 스캔 작업
rosrun apriltag_navigation main_node.py --mode 4
```

