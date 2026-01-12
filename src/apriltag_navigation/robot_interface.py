#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Robot Interface - High-Level Robot API
Provides clean, pseudocode-like interface that hides all low-level details.
This is the primary interface used by the main navigation node.
"""

import rospy
import math
import json
from enum import Enum, auto
from std_msgs.msg import Bool, String

# Import subsystem modules
from apriltag_navigation.map.map_manager import MapManager
from apriltag_navigation.perception.vision_module import VisionModule
from apriltag_navigation.hardware.robot_controller import RobotController, RotationController
from apriltag_navigation.navigation.pure_pursuit import PurePursuitController, CoordinateTransformer


class NavigationState(Enum):
    """Navigation state machine states"""
    IDLE = auto()
    MOVING = auto()
    ROTATING = auto()
    ALIGNING = auto()
    STOPPING = auto()
    WAIT_SCAN = auto()
    COMPLETED = auto()


class RobotInterface:
    """
    High-level robot interface providing clean API for navigation.
    Hides all ROS communication, state management, and control details.
    """

    def __init__(self, config_path=None):
        """
        Initialize robot interface.

        Args:
            config_path: Optional path to map configuration YAML
        """
        rospy.loginfo("=" * 50)
        rospy.loginfo("Initializing Robot Interface")
        rospy.loginfo("=" * 50)

        # Initialize subsystems
        self.map_manager = MapManager(config_path)
        self.vision = VisionModule(
            tag_size=self.map_manager.get_param('tag_size', 0.06)
        )
        self.robot = RobotController(self.map_manager)
        self.rotation_ctrl = RotationController(self.robot)
        self.pursuit = PurePursuitController(self.map_manager)

        # Navigation state
        self.state = NavigationState.IDLE
        self.current_tag = 508  # Start at dock
        self.current_lateral = 0.0
        self.current_align_angle = 0.0

        # Load parameters
        self.image_center_y = self.map_manager.get_param('camera.height', 720) // 2
        self.center_y_tolerance = self.map_manager.get_param('thresholds.center_y_tolerance', 50)
        self.align_angle_threshold = self.map_manager.get_param('thresholds.align_angle', 0.5)
        self.lateral_threshold = self.map_manager.get_param('thresholds.lateral', 0.05)
        self.linear_speed = self.map_manager.get_param('speeds.linear', 0.3)
        self.slow_factor = self.map_manager.get_param('speeds.slow_factor', 0.3)

        # Stop control
        self.stop_duration = 0.8
        self.stop_start_time = None

        # Dock accuracy tracking
        self.dock_start_lateral = None
        self.dock_start_center_x = None
        self.dock_start_recorded = False

        # Debug status publisher
        self.debug_status_pub = rospy.Publisher('/nav_debug_status', String, queue_size=10)

        # Mode 4 (scan mode) support
        self.scan_mode_active = False
        self.scan_finished = False
        self.pose_pub = None
        self.scan_finished_sub = None

        rospy.loginfo("[ROBOT INTERFACE] Initialization complete")

    def wait_until_ready(self):
        """Wait until all subsystems are ready"""
        rospy.loginfo("Waiting for subsystems...")
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.vision.is_ready() and self.robot.is_ready() and self.vision.is_tag_visible(508):
                rospy.loginfo("All subsystems ready!")
                self.record_dock_start()
                return True
            rate.sleep()

        return False

    def record_dock_start(self):
        """Record starting position at dock for return accuracy tracking"""
        if not self.dock_start_recorded:
            tag_data = self.vision.get_tag_data(508)
            if tag_data:
                self.dock_start_lateral = tag_data['x']
                self.dock_start_center_x = tag_data['center_x']
                self.dock_start_recorded = True
                rospy.loginfo(f"[DOCK START] lateral:{self.dock_start_lateral:.4f}m center_x:{self.dock_start_center_x:.1f}px")

    def compare_dock_return(self):
        """Compare return position to start position at dock"""
        if self.dock_start_recorded:
            tag_data = self.vision.get_tag_data(508)
            if tag_data:
                end_lateral = tag_data['x']
                end_center_x = tag_data['center_x']

                lateral_diff = end_lateral - self.dock_start_lateral
                pixel_diff = end_center_x - self.dock_start_center_x

                rospy.loginfo("=" * 50)
                rospy.loginfo("[DOCK RETURN ACCURACY]")
                rospy.loginfo(f"  Start: lateral={self.dock_start_lateral:.4f}m, cx={self.dock_start_center_x:.1f}px")
                rospy.loginfo(f"  End:   lateral={end_lateral:.4f}m, cx={end_center_x:.1f}px")
                rospy.loginfo(f"  Diff:  lateral={lateral_diff:+.4f}m ({lateral_diff*100:+.2f}cm), pixel={pixel_diff:+.1f}px")
                rospy.loginfo("=" * 50)

    def enable_scan_mode(self):
        """Enable Mode 4 scan mode with pose publishing"""
        self.scan_mode_active = True

        # Import custom message
        try:
            from robot_msgs.msg import Pose2DWithFlag
            self.pose_pub = rospy.Publisher('/robot_pose', Pose2DWithFlag, queue_size=10)
            self.scan_finished_sub = rospy.Subscriber('/scan_finished', Bool, self._scan_finished_callback)
            rospy.loginfo("[ROBOT INTERFACE] Scan mode enabled")
        except ImportError:
            rospy.logwarn("[ROBOT INTERFACE] robot_msgs not found, scan mode disabled")
            self.scan_mode_active = False

    def _scan_finished_callback(self, msg):
        """Callback for scan finished signal"""
        if msg.data:
            rospy.loginfo("[SCAN MODE] Scan finished signal received")
            self.scan_finished = True

    def publish_robot_pose(self, tag_id, robot_x, robot_y, theta):
        """
        Publish robot pose for external systems (Mode 4).

        Args:
            tag_id: Current tag ID
            robot_x: Robot X position (Isaac Sim coordinates)
            robot_y: Robot Y position (Isaac Sim coordinates)
            theta: Robot heading in degrees
        """
        if not self.pose_pub:
            return

        # Convert coordinates
        cam_manip_x, cam_manip_y = CoordinateTransformer.isaac_to_manipulator(robot_x, robot_y)

        # Get zone for offset calculation
        zone = self.map_manager.tag_db.get_zone(tag_id)

        # Apply offset to get robot CENTER position
        pub_x, pub_y = CoordinateTransformer.apply_robot_center_offset(cam_manip_x, cam_manip_y, zone)

        # Publish message
        from robot_msgs.msg import Pose2DWithFlag

        pm = Pose2DWithFlag()
        pm.header.stamp = rospy.Time.now()
        pm.header.frame_id = 'world'
        pm.x = pub_x
        pm.y = pub_y
        pm.theta = theta
        pm.theta_web = 0.0
        pm.flag = True
        pm.id = tag_id

        self.pose_pub.publish(pm)

        rospy.loginfo(f"[POSE] tag:{tag_id} | isaac:({robot_x:.3f},{robot_y:.3f}) | manip_center:({pub_x:.3f},{pub_y:.3f}) | theta:{theta:.2f}°")

    def publish_debug_status(self, current_wp=None, next_wp=None, total_wps=None, wp_idx=None):
        """Publish real-time navigation debug status"""
        zone = self.map_manager.tag_db.get_zone(self.current_tag)

        # Calculate wall distance
        wall_dist = self.pursuit.calculate_wall_distance(
            self.current_lateral,
            self.current_align_angle,
            zone
        )

        # Determine drift direction
        if zone in ['B', 'C', 'D', 'E']:
            drift_toward_wall = self.current_lateral > 0
        else:
            drift_toward_wall = self.current_lateral < 0

        drift_direction = "TOWARD_WALL" if drift_toward_wall else "AWAY_FROM_WALL"
        min_clearance = self.map_manager.get_param('wall_distances.min_clearance', 0.10)
        wall_status = "OK" if wall_dist >= min_clearance else "WARNING"

        status = {
            'timestamp': rospy.Time.now().to_sec(),
            'state': self.state.name,
            'zone': zone,
            'current_tag': self.current_tag,
            'target_tag': current_wp,
            'next_tag': next_wp,
            'waypoint_progress': f"{wp_idx}/{total_wps}" if wp_idx is not None else "N/A",
            'lateral_offset_m': round(self.current_lateral, 4),
            'drift_direction': drift_direction,
            'wall_distance_m': round(wall_dist, 4),
            'wall_status': wall_status,
            'align_angle_deg': round(self.current_align_angle, 2),
        }

        msg = String()
        msg.data = json.dumps(status)
        self.debug_status_pub.publish(msg)

    def move_to_tag(self, target_tag, edge_info):
        """
        High-level command: Move to a specific tag.

        Args:
            target_tag: Target tag ID
            edge_info: Edge information dict with 'direction' and 'type'

        Returns:
            True if arrived, False if still moving
        """
        direction = edge_info['direction']
        move_dir = 'backward' if 'backward' in direction else 'forward'

        # Check if tag is visible
        if not self.vision.is_tag_visible(target_tag):
            # Tag not visible - move straight
            linear = self.linear_speed if move_dir == 'forward' else -self.linear_speed
            self.robot.move(linear, 0)
            rospy.loginfo_throttle(1.0, f"[MOVE] -> {target_tag} | no tag | straight | {move_dir}")
            return False

        # Get tag data
        tag_data = self.vision.get_tag_data(target_tag)
        lateral = tag_data['x']
        center_y = tag_data['center_y']

        self.current_lateral = lateral

        # Check if arrived (tag centered vertically)
        if abs(center_y - self.image_center_y) < self.center_y_tolerance:
            rospy.loginfo(f">>> ARRIVED at tag {target_tag} (cy:{center_y:.0f} lat:{lateral:.3f}m)")
            return True

        # Calculate Pure Pursuit control
        robot_x, robot_y, robot_heading = self.robot.get_estimated_pose()
        target_x, target_y = self.map_manager.tag_db.get_position(target_tag)

        angular = self.pursuit.calculate_absolute(
            robot_x, robot_y, robot_heading,
            target_x, target_y,
            lateral, move_dir
        )

        # Calculate wall distance
        zone = self.map_manager.tag_db.get_zone(self.current_tag)
        wall_dist = self.pursuit.calculate_wall_distance(lateral, self.current_align_angle, zone)
        min_clearance = self.map_manager.get_param('wall_distances.min_clearance', 0.10)
        wall_status = "OK" if wall_dist >= min_clearance else "WARN!"

        rospy.loginfo_throttle(0.5, f"[MOVE] -> {target_tag} | lat:{lateral:+.3f}m | ang:{angular:+.2f} | wall:{wall_dist:.3f}m {wall_status} | {move_dir}")

        # Send velocity command
        linear = self.linear_speed
        if move_dir == 'backward':
            linear = -linear
        if target_tag == 508:  # Slow down approaching dock
            linear *= self.slow_factor

        self.robot.move(linear, angular)
        return False

    def align_to_tag(self, tag_id):
        """
        High-level command: Align precisely to a tag.

        Args:
            tag_id: Tag ID to align to

        Returns:
            True if aligned, False if still aligning
        """
        if not self.vision.is_tag_visible(tag_id):
            rospy.loginfo(f"[ALIGN] Tag {tag_id} not visible, skip")
            return True

        # Get alignment angle
        align_angle_deg = self.vision.get_alignment_angle(tag_id)
        if align_angle_deg is None:
            rospy.loginfo(f"[ALIGN] No corners for tag {tag_id}, skip")
            return True

        lateral = self.vision.get_tag_lateral(tag_id)

        # Check if aligned
        if abs(align_angle_deg) < self.align_angle_threshold:
            # Save aligned state
            self.current_lateral = lateral
            self.current_align_angle = align_angle_deg

            # Update reference point
            robot_x, robot_y, robot_heading = self.pursuit.get_robot_pose_from_tag(
                tag_id, lateral, align_angle_deg
            )
            if robot_x is not None:
                self.robot.set_reference_point(robot_x, robot_y, robot_heading)

            # Calculate wall distance
            zone = self.map_manager.tag_db.get_zone(tag_id)
            wall_dist = self.pursuit.calculate_wall_distance(lateral, align_angle_deg, zone)
            min_clearance = self.map_manager.get_param('wall_distances.min_clearance', 0.10)
            wall_status = "OK" if wall_dist >= min_clearance else "WARN!"

            rospy.loginfo(f"[ALIGN] Done! angle:{align_angle_deg:.2f}° lateral:{lateral:.3f}m wall:{wall_dist:.3f}m {wall_status}")
            return True

        # Rotate to align
        # Positive angle = need to rotate CW (negative angular)
        angular = -align_angle_deg * 0.8

        # Minimum speed to overcome friction
        if abs(angular) < 0.08 and abs(align_angle_deg) > 0.3:
            angular = 0.08 if angular > 0 else -0.08

        angular = max(-0.2, min(0.2, angular))
        self.robot.move(0, angular)
        rospy.loginfo_throttle(0.3, f"[ALIGN] angle:{align_angle_deg:.2f}° lat:{lateral:.3f}m ang:{angular:.2f}")

        return False

    def rotate_90(self, direction):
        """
        High-level command: Rotate 90 degrees (for pivots).

        Args:
            direction: 'cw' or 'ccw'

        Returns:
            True if rotation complete, False otherwise
        """
        if not self.rotation_ctrl.is_rotating:
            self.rotation_ctrl.start_rotation(90, direction)

        return self.rotation_ctrl.update()

    def stop_and_wait(self, duration=None):
        """
        High-level command: Stop and wait for a duration.

        Args:
            duration: Wait duration in seconds (default: from config)

        Returns:
            True if wait complete, False otherwise
        """
        if duration is None:
            duration = self.stop_duration

        if self.stop_start_time is None:
            self.stop_start_time = rospy.Time.now()
            self.robot.stop()

        elapsed = (rospy.Time.now() - self.stop_start_time).to_sec()
        if elapsed >= duration:
            self.stop_start_time = None
            return True

        return False

    def wait_for_scan(self, tag_id):
        """
        High-level command: Wait for external scan to complete (Mode 4).

        Args:
            tag_id: Current tag ID

        Returns:
            True if scan complete, False if waiting
        """
        self.robot.stop()

        if self.scan_finished:
            rospy.loginfo(f"[SCAN MODE] Scan completed at tag {tag_id}")
            self.scan_finished = False
            return True

        # Keep publishing robot pose periodically
        robot_x, robot_y, robot_heading = self.robot.get_estimated_pose()
        self.publish_robot_pose(tag_id, robot_x, robot_y, math.degrees(robot_heading))
        rospy.loginfo_throttle(2.0, f"[SCAN MODE] Waiting for /scan_finished at tag {tag_id}...")

        return False

    def stop(self):
        """Stop the robot"""
        self.robot.stop()

    def get_current_position(self):
        """Get current tag position"""
        return self.current_tag

    def set_current_position(self, tag_id):
        """Update current tag position"""
        self.current_tag = tag_id
