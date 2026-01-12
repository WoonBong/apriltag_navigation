#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Main Navigation Node - High-Level Mission Controller
Extremely readable, pseudocode-like structure.
All complexity hidden in robot_interface.
"""

import rospy
import sys
import os
import argparse
import glob

# Add src to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from apriltag_navigation.robot_interface import RobotInterface, NavigationState


class NavigationMission:
    """
    High-level navigation mission controller.
    Reads like English pseudocode.
    """

    def __init__(self, robot, map_manager):
        """
        Initialize navigation mission.

        Args:
            robot: RobotInterface instance
            map_manager: MapManager instance
        """
        self.robot = robot
        self.map = map_manager

        # Mission state
        self.waypoints = []
        self.waypoint_index = 0
        self.scan_tags = []  # For Mode 4
        self.scan_index = 0

    def load_task(self, task_name):
        """
        Load a predefined task from map configuration.

        Args:
            task_name: Task name ('task1' or 'task2')
        """
        self.waypoints = self.map.task_manager.get_task_waypoints(task_name)
        self.waypoint_index = 0
        rospy.loginfo(f"[MISSION] Loaded {task_name}: {len(self.waypoints)} waypoints")

    def load_direct_navigation(self, target_tag):
        """
        Load a direct navigation task to a specific tag (Mode 3).

        Args:
            target_tag: Target tag ID
        """
        current_pos = self.robot.get_current_position()
        self.waypoints = self.map.nav_graph.find_path(current_pos, target_tag)
        self.waypoint_index = 0
        rospy.loginfo(f"[MISSION] Direct navigation to tag {target_tag}: {len(self.waypoints)} waypoints")

    def load_scan_task(self, excel_path):
        """
        Load a scan task from Excel file (Mode 4).

        Args:
            excel_path: Path to Excel file
        """
        self.waypoints, self.scan_tags = self.map.task_manager.get_excel_scan_waypoints(excel_path)
        self.waypoint_index = 0
        self.scan_index = 0
        self.robot.enable_scan_mode()
        rospy.loginfo(f"[MISSION] Loaded scan task: {len(self.scan_tags)} scan points")

    def execute(self):
        """
        Execute one step of the mission.
        Main control loop - reads like pseudocode.
        """
        # Check if mission complete
        if self.waypoint_index >= len(self.waypoints):
            rospy.loginfo("[MISSION] Complete!")
            self.robot.stop()
            self.robot.compare_dock_return()
            return True

        # Get current and next waypoint
        current_wp = self.waypoints[self.waypoint_index]
        next_wp = self.waypoints[self.waypoint_index + 1] if self.waypoint_index + 1 < len(self.waypoints) else None

        # Publish debug status
        self.robot.publish_debug_status(
            current_wp=current_wp,
            next_wp=next_wp,
            total_wps=len(self.waypoints),
            wp_idx=self.waypoint_index
        )

        # Skip if already at waypoint
        if current_wp == self.robot.get_current_position():
            self.waypoint_index += 1
            return False

        # Get edge information (how to get from current to next)
        edge = self.map.nav_graph.get_edge(self.robot.get_current_position(), current_wp)
        if not edge:
            rospy.logwarn(f"[MISSION] No path from {self.robot.get_current_position()} to {current_wp}")
            self.waypoint_index += 1
            return False

        # Handle different edge types
        if edge['type'] == 'pivot':
            # Execute 90-degree rotation
            direction = 'ccw' if 'ccw' in edge['direction'] else 'cw'
            if self.robot.rotate_90(direction):
                self._complete_waypoint(current_wp)

        elif edge['type'] == 'move':
            # Execute movement with state machine
            self._execute_movement(current_wp, edge)

        return False

    def _execute_movement(self, target_tag, edge):
        """
        Execute movement to target tag with proper state transitions.

        Args:
            target_tag: Target tag ID
            edge: Edge information
        """
        # State: MOVING
        if self.robot.state == NavigationState.IDLE or self.robot.state == NavigationState.MOVING:
            self.robot.state = NavigationState.MOVING

            # Move towards target
            if self.robot.move_to_tag(target_tag, edge):
                # Arrived at target
                self.robot.state = NavigationState.STOPPING

        # State: STOPPING
        elif self.robot.state == NavigationState.STOPPING:
            if self.robot.stop_and_wait():
                self.robot.state = NavigationState.ALIGNING

        # State: ALIGNING
        elif self.robot.state == NavigationState.ALIGNING:
            if self.robot.align_to_tag(target_tag):
                # Check if this is a scan target (Mode 4)
                if self._should_scan_at_tag(target_tag):
                    self.robot.state = NavigationState.WAIT_SCAN
                else:
                    self._complete_waypoint(target_tag)

        # State: WAIT_SCAN (Mode 4 only)
        elif self.robot.state == NavigationState.WAIT_SCAN:
            if self.robot.wait_for_scan(target_tag):
                self.scan_index += 1
                self._complete_waypoint(target_tag)

    def _should_scan_at_tag(self, tag_id):
        """Check if current tag is a scan target"""
        if self.scan_index < len(self.scan_tags):
            return tag_id == self.scan_tags[self.scan_index]
        return False

    def _complete_waypoint(self, tag_id):
        """Mark waypoint as complete and advance"""
        self.robot.set_current_position(tag_id)
        self.waypoint_index += 1
        self.robot.state = NavigationState.IDLE
        rospy.loginfo(f"[MISSION] Waypoint {tag_id} complete")


def run_diagnostics(args):
    """
    Run diagnostic tests based on command-line arguments.

    Args:
        args: Parsed command-line arguments
    """
    # Import diagnostic modules
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
    from apriltag_navigation.map.map_manager import MapManager
    from apriltag_navigation.perception.vision_module import VisionModule
    from apriltag_navigation.hardware.robot_controller import RobotController, RotationController

    print("=" * 60)
    print("AprilTag Navigation - Diagnostic Mode")
    print("=" * 60)

    # Initialize subsystems
    print("\nInitializing subsystems...")
    map_manager = MapManager()
    vision = VisionModule()
    robot = RobotController(map_manager)
    rotation_ctrl = RotationController(robot)

    print("[OK] All subsystems initialized")

    # Wait for ready
    print("\nWaiting for camera and odometry...")
    rate = rospy.Rate(10)
    timeout = rospy.Time.now() + rospy.Duration(10.0)

    while not rospy.is_shutdown() and rospy.Time.now() < timeout:
        if vision.is_ready() and robot.is_ready():
            print("[OK] All systems ready\n")
            break
        rate.sleep()
    else:
        print("[ERROR] Timeout waiting for systems")
        return

    # Run selected tests
    if args.test_vision:
        test_vision(vision, args.duration)

    if args.test_motor:
        test_motor(robot, args.distance, args.direction)

    if args.test_pivot:
        test_pivot(robot, rotation_ctrl, args.direction)

    print("\nDiagnostics complete!")


def test_vision(vision, duration):
    """Test vision system"""
    print("=" * 60)
    print(f"VISION TEST: Detecting AprilTags for {duration}s")
    print("=" * 60)

    rate = rospy.Rate(10)
    start_time = rospy.Time.now()
    detected_tags_history = set()

    try:
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed >= duration:
                break

            detected_tags = vision.get_detected_tags()
            detected_tags_history.update(detected_tags.keys())

            print(f"\r[{elapsed:.1f}s] ", end='')
            if detected_tags:
                tag_strs = [f"Tag {tid}: {d['z']:.2f}m" for tid, d in detected_tags.items()]
                print(" | ".join(tag_strs), end='')
            else:
                print("No tags detected", end='')
            sys.stdout.flush()
            rate.sleep()

    except KeyboardInterrupt:
        print("\n[INTERRUPTED]")

    print("\n" + "=" * 60)
    print(f"Detected {len(detected_tags_history)} unique tags: {sorted(detected_tags_history)}")
    print("=" * 60)


def test_motor(robot, distance, direction):
    """Test motor movement"""
    import time
    print("=" * 60)
    print(f"MOTOR TEST: Move {direction} {distance}m")
    print("=" * 60)

    start_x, start_y = robot.get_position()
    speed = 0.3 if direction == 'forward' else -0.3

    rate = rospy.Rate(30)
    try:
        while not rospy.is_shutdown():
            robot.move(speed, 0)
            current_x, current_y = robot.get_position()
            traveled = ((current_x - start_x)**2 + (current_y - start_y)**2)**0.5

            if traveled >= distance:
                break
            rate.sleep()
    except KeyboardInterrupt:
        print("\n[INTERRUPTED]")
    finally:
        robot.stop()
        time.sleep(0.5)
        end_x, end_y = robot.get_position()
        actual = ((end_x - start_x)**2 + (end_y - start_y)**2)**0.5
        print(f"\nTarget: {distance:.2f}m, Actual: {actual:.2f}m, Error: {abs(actual-distance):.2f}m")


def test_pivot(robot, rotation_ctrl, direction):
    """Test pivot rotation"""
    import time
    print("=" * 60)
    print(f"PIVOT TEST: Rotate 90° {direction.upper()}")
    print("=" * 60)

    start_heading = robot.get_heading()
    rotation_ctrl.start_rotation(90, direction)

    rate = rospy.Rate(30)
    try:
        while not rospy.is_shutdown():
            if rotation_ctrl.update():
                break
            rate.sleep()
    except KeyboardInterrupt:
        print("\n[INTERRUPTED]")
        rotation_ctrl.cancel()
    finally:
        time.sleep(0.5)
        end_heading = robot.get_heading()
        rotation = (end_heading - start_heading) * 180.0 / 3.14159
        print(f"\nRotation: {rotation:.1f}°")


def list_excel_files():
    """
    List available Excel files in data/excel directory.
    Returns: List of Excel file paths
    """
    package_dir = os.path.join(os.path.dirname(__file__), '..')
    excel_dir = os.path.join(package_dir, 'data', 'excel')

    if not os.path.exists(excel_dir):
        return []

    # Find all Excel files
    excel_files = []
    for ext in ['*.xlsx', '*.xls']:
        excel_files.extend(glob.glob(os.path.join(excel_dir, ext)))

    return sorted(excel_files)


def select_excel_file():
    """
    Interactive Excel file selection.
    Returns: Selected Excel file path or None
    """
    excel_files = list_excel_files()

    if not excel_files:
        print("\n[ERROR] No Excel files found in data/excel/ directory!")
        print("Please place your Excel files in: data/excel/\n")
        return None

    print("\nAvailable Excel files:")
    print("-" * 50)
    for i, filepath in enumerate(excel_files, 1):
        filename = os.path.basename(filepath)
        print(f"{i}: {filename}")
    print("-" * 50)

    while True:
        try:
            choice = input(f"\nSelect file (1-{len(excel_files)}): ").strip()
            idx = int(choice) - 1
            if 0 <= idx < len(excel_files):
                return excel_files[idx]
            print(f"Please enter a number between 1 and {len(excel_files)}")
        except (ValueError, KeyboardInterrupt, EOFError):
            return None


def select_mode():
    """
    Interactive mode selection.
    Returns: (mode, target_tag/excel_path)
    """
    print("=" * 50)
    print("AprilTag Navigation System")
    print("=" * 50)
    print("\n1: Task 1 (Zone B + C)")
    print("2: Task 2 (Zone D + E)")
    print("3: Direct Navigation (Go to specific tag)")
    print("4: Scan Mode (from Excel)\n")

    while True:
        try:
            choice = input("Select mode (1/2/3/4): ").strip()
            if choice == '1':
                return 1, None
            elif choice == '2':
                return 2, None
            elif choice == '3':
                target_tag = input("Enter target tag ID: ").strip()
                if target_tag.isdigit():
                    return 3, int(target_tag)
                print("Please enter a valid tag ID (number)!")
            elif choice == '4':
                excel_path = select_excel_file()
                if excel_path:
                    return 4, excel_path
                print("Excel file selection cancelled!")
        except (KeyboardInterrupt, EOFError):
            return None, None


def main():
    """
    Main entry point.
    Ultra-clean, reads like pseudocode.
    """
    # Parse command-line arguments
    parser = argparse.ArgumentParser(
        description='AprilTag Navigation System',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Interactive mode selection
  rosrun apriltag_navigation main_node.py

  # Direct mode selection
  rosrun apriltag_navigation main_node.py --mode 1
  rosrun apriltag_navigation main_node.py --mode 3 --tag 5

  # Test/diagnostic modes
  rosrun apriltag_navigation main_node.py --test-vision
  rosrun apriltag_navigation main_node.py --test-motor --distance 2.0
        """
    )

    parser.add_argument('--mode', type=int, choices=[1, 2, 3, 4],
                        help='Navigation mode: 1=Task1, 2=Task2, 3=Direct, 4=Scan')
    parser.add_argument('--tag', type=int, help='Target tag ID for mode 3')
    parser.add_argument('--excel', type=str, help='Excel file path for mode 4')

    # Test/diagnostic options
    parser.add_argument('--test-vision', action='store_true', help='Test vision system')
    parser.add_argument('--test-motor', action='store_true', help='Test motor movement')
    parser.add_argument('--test-pivot', action='store_true', help='Test pivot rotation')
    parser.add_argument('--distance', type=float, default=1.0, help='Distance for motor test (meters)')
    parser.add_argument('--direction', type=str, default='forward',
                        choices=['forward', 'backward', 'cw', 'ccw'],
                        help='Direction for motor/pivot test')
    parser.add_argument('--duration', type=float, default=10.0, help='Duration for vision test (seconds)')

    args = parser.parse_args()

    # Initialize ROS
    rospy.init_node('apriltag_navigation', anonymous=True)

    # Check for test modes
    if args.test_vision or args.test_motor or args.test_pivot:
        run_diagnostics(args)
        return

    # Determine mode and parameter
    if args.mode is not None:
        # Command-line mode
        mode = args.mode
        if mode == 3:
            if args.tag is None:
                rospy.logerr("--tag required for mode 3")
                return
            mode_param = args.tag
        elif mode == 4:
            if args.excel:
                mode_param = args.excel
            else:
                mode_param = select_excel_file()
                if mode_param is None:
                    return
        else:
            mode_param = None
    else:
        # Interactive mode selection
        mode, mode_param = select_mode()
        if mode is None:
            return

    # Initialize robot
    robot = RobotInterface()

    # Wait for all systems ready
    if not robot.wait_until_ready():
        rospy.logerr("Failed to initialize robot")
        return

    # Create and load mission
    mission = NavigationMission(robot, robot.map_manager)

    if mode == 1:
        mission.load_task('task1')
    elif mode == 2:
        mission.load_task('task2')
    elif mode == 3:
        mission.load_direct_navigation(mode_param)
    elif mode == 4:
        mission.load_scan_task(mode_param)

    # Execute mission
    rospy.loginfo("Starting mission...")
    rate = rospy.Rate(30)  # 30 Hz

    while not rospy.is_shutdown():
        if mission.execute():
            break
        rate.sleep()

    # Mission complete
    robot.stop()
    rospy.loginfo("Mission complete!")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
