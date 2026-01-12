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


def select_mode():
    """
    Interactive mode selection.
    Returns: (mode, excel_path)
    """
    print("=" * 50)
    print("AprilTag Navigation System")
    print("=" * 50)
    print("\n1: Task 1 (Zone B + C)")
    print("2: Task 2 (Zone D + E)")
    print("4: Scan Mode (from Excel)\n")

    while True:
        try:
            choice = input("Select mode (1/2/4): ").strip()
            if choice == '1':
                return 1, None
            elif choice == '2':
                return 2, None
            elif choice == '4':
                excel_path = input("Excel file path: ").strip()
                if excel_path:
                    return 4, excel_path
                print("Excel path required for mode 4!")
        except (KeyboardInterrupt, EOFError):
            return None, None


def main():
    """
    Main entry point.
    Ultra-clean, reads like pseudocode.
    """
    # Initialize ROS
    rospy.init_node('apriltag_navigation', anonymous=True)

    # Select mission mode
    mode, excel_path = select_mode()
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
    elif mode == 4:
        mission.load_scan_task(excel_path)

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
