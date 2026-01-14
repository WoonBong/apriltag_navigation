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

# Add src to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from apriltag_navigation.robot_interface import RobotInterface, NavigationState
from apriltag_navigation.ui_helper import select_mode, select_excel_file
from apriltag_navigation.test_helper import run_diagnostics


class NavigationMission:
    """
    High-level navigation mission controller.
    Reads like English pseudocode.
    """

    def __init__(self, robot, map_manager, mode=None):
        """
        Initialize navigation mission.

        Args:
            robot: RobotInterface instance
            map_manager: MapManager instance
            mode: Navigation mode (1, 2, 3, 4) or None
        """
        self.robot = robot
        self.map = map_manager
        self.mode = mode

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

            # Mode 3: Continuous navigation - ask for next destination
            if self.mode == 3:
                return self._ask_next_destination()
            else:
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

    def _ask_next_destination(self):
        """
        Ask user for next destination (Mode 3 continuous navigation).
        Returns True to exit mission, False to continue.
        """
        rospy.loginfo("[MODE 3] Destination reached!")
        print("\n" + "="*50)
        print("  Destination reached!")
        print("  Enter next target tag ID (or 'q' to quit): ")
        print("="*50)

        try:
            user_input = input("Target tag ID: ").strip()

            if user_input.lower() == 'q':
                rospy.loginfo("[MODE 3] User quit. Returning to dock.")
                self.robot.compare_dock_return()
                return True

            target_tag = int(user_input)

            # Validate tag exists
            if not self.map.tag_db.get(target_tag):
                rospy.logerr(f"[MODE 3] Invalid tag ID: {target_tag}")
                return self._ask_next_destination()  # Ask again

            # Load new navigation task
            rospy.loginfo(f"[MODE 3] New destination: {target_tag}")
            self.load_direct_navigation(target_tag)
            return False  # Continue mission

        except ValueError:
            rospy.logerr("[MODE 3] Invalid input. Please enter a number.")
            return self._ask_next_destination()  # Ask again
        except KeyboardInterrupt:
            rospy.loginfo("[MODE 3] Interrupted. Returning to dock.")
            self.robot.compare_dock_return()
            return True


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

  # Quick test modes (for detailed diagnostics, use scripts/diagnostics.py)
  rosrun apriltag_navigation main_node.py --test-vision --duration 10
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
                # Interactive Excel file selection
                package_dir = os.path.join(os.path.dirname(__file__), '..')
                mode_param = select_excel_file(package_dir)
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
    mission = NavigationMission(robot, robot.map_manager, mode=mode)

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
