# AprilTag Navigation - Modular ROS Package

Research-quality modular navigation system for AprilTag-based robot navigation using Pure Pursuit path following.

## Features

- **Data-Driven Configuration**: All map topology, tag coordinates, and tasks defined in YAML
- **Modular Architecture**: Clean separation of concerns across subsystems
- **High-Level API**: Pseudocode-like main node that's easy to read and maintain
- **Field Diagnostics**: CLI tools for testing motors, vision, and pivot independently
- **Preserved Logic**: Original Pure Pursuit and pose estimation algorithms maintained

## Package Structure

```
apriltag_navigation/
├── config/
│   └── map.yaml              # All map configuration (tags, edges, tasks)
├── launch/
│   └── navigation.launch     # ROS launch file
├── nodes/
│   └── main_node.py          # High-level mission controller (pseudocode-like)
├── src/apriltag_navigation/
│   ├── map/
│   │   └── map_manager.py    # YAML parser, TagDatabase, NavigationGraph
│   ├── perception/
│   │   └── vision_module.py  # AprilTag detection & pose estimation
│   ├── navigation/
│   │   └── pure_pursuit.py   # Pure Pursuit algorithms
│   ├── hardware/
│   │   └── robot_controller.py  # Motor control & odometry
│   └── robot_interface.py    # High-level robot API
├── scripts/
│   └── diagnostics.py        # Field testing tool
├── package.xml
├── CMakeLists.txt
└── README.md
```

## Dependencies

### ROS Packages
- rospy
- sensor_msgs
- geometry_msgs
- nav_msgs
- std_msgs
- tf

### Python Packages
```bash
pip install dt-apriltags numpy opencv-python pandas pyyaml
```

## Installation

1. Clone into your catkin workspace:
```bash
cd ~/catkin_ws/src
cp -r /path/to/apriltag_navigation .
```

2. Build the package:
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Usage

### Running Navigation Missions

#### Task 1 (Zone B + C):
```bash
roslaunch apriltag_navigation navigation.launch
# Then select mode 1
```

#### Task 2 (Zone D + E):
```bash
roslaunch apriltag_navigation navigation.launch
# Then select mode 2
```

#### Scan Mode (from Excel):
```bash
roslaunch apriltag_navigation navigation.launch
# Then select mode 4 and provide Excel file path
```

### Field Diagnostics

Test individual subsystems without running full navigation:

#### Test Motors
```bash
# Move forward 2 meters
rosrun apriltag_navigation diagnostics.py --test-motor --distance 2.0 --direction forward

# Move backward 1 meter
rosrun apriltag_navigation diagnostics.py --test-motor --distance 1.0 --direction backward
```

#### Test Vision
```bash
# Detect AprilTags for 10 seconds
rosrun apriltag_navigation diagnostics.py --test-vision --duration 10
```

#### Test Pivot
```bash
# Rotate 90 degrees counter-clockwise
rosrun apriltag_navigation diagnostics.py --test-pivot --direction ccw --angle 90

# Rotate 90 degrees clockwise
rosrun apriltag_navigation diagnostics.py --test-pivot --direction cw --angle 90
```

#### Run All Tests
```bash
rosrun apriltag_navigation diagnostics.py --test-all
```

## Architecture

### Design Principles

1. **High-Level Abstraction**: Main node reads like English pseudocode
   ```python
   mission.load_task('task1')
   while mission.execute():
       # Clean, readable mission execution
   ```

2. **Data-Driven Topology**: All map data externalized to YAML
   - Tag coordinates and zones
   - Graph connectivity and directions
   - Task waypoint sequences
   - Robot parameters

3. **Separation of Concerns**:
   - **MapManager**: Configuration and pathfinding
   - **VisionModule**: AprilTag detection
   - **PurePursuitController**: Path following math
   - **RobotController**: Hardware interface
   - **RobotInterface**: High-level API

4. **Field Testability**: Diagnostic tools for independent subsystem testing

## Configuration

Edit `config/map.yaml` to modify:
- Tag positions and zones
- Navigation graph edges
- Task definitions
- Robot parameters (speeds, thresholds, dimensions)

## API Overview

### High-Level Robot Interface

```python
from apriltag_navigation.robot_interface import RobotInterface

robot = RobotInterface()
robot.wait_until_ready()

# High-level commands
robot.move_to_tag(target_tag, edge_info)
robot.align_to_tag(tag_id)
robot.rotate_90('ccw')
robot.stop()
```

### Map Management

```python
from apriltag_navigation.map.map_manager import MapManager

map_mgr = MapManager()
path = map_mgr.nav_graph.find_path(start, goal)
tag_info = map_mgr.tag_db.get(tag_id)
waypoints = map_mgr.task_manager.get_task_waypoints('task1')
```

### Vision

```python
from apriltag_navigation.perception.vision_module import VisionModule

vision = VisionModule()
detected_tags = vision.get_detected_tags()
lateral = vision.get_tag_lateral(tag_id)
align_angle = vision.get_alignment_angle(tag_id)
```

## Extending the System

### Adding New Tasks

Edit `config/map.yaml`:
```yaml
tasks:
  my_task:
    description: "Custom task description"
    waypoints: [508, 500, 400, 401, ...]
```

### Adding New Tags

Edit `config/map.yaml`:
```yaml
tags:
  999:
    x: 1.5
    y: 2.5
    type: WORK
    zone: A
```

### Modifying Navigation Graph

Edit `config/map.yaml`:
```yaml
edges:
  - from: 999
    to: 500
    direction: forward
    type: move
```

## Troubleshooting

### Camera not detected
- Check that `/rgb` and `/camera_info` topics are publishing
- Verify camera calibration parameters

### Odometry issues
- Ensure `/odom` topic is publishing
- Check coordinate frame alignment

### Tag detection problems
- Verify lighting conditions
- Check tag size parameter in `config/map.yaml`
- Use diagnostics tool: `--test-vision`

### Motor control issues
- Test with diagnostics: `--test-motor`
- Verify `/cmd_vel` topic is subscribed by robot

## License

MIT License

## Credits

Refactored from original `navigation_purepursuit.py` by [Your Name].
Pure Pursuit algorithms and coordinate transformations preserved from original implementation.
