#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Pure Pursuit Controller
Implements Pure Pursuit path following algorithms and coordinate transformations.
IMPORTANT: Core mathematics preserved from original implementation.
"""

import math
import numpy as np
import rospy


class PurePursuitController:
    """
    Pure Pursuit controller for path following navigation.
    Supports forward, backward, and absolute coordinate-based control.
    """

    def __init__(self, map_manager):
        """
        Initialize Pure Pursuit controller.

        Args:
            map_manager: MapManager instance for accessing configuration
        """
        self.map_manager = map_manager
        self.tag_db = map_manager.tag_db

        # Load parameters from configuration
        self.look_ahead_base = map_manager.get_param('pure_pursuit.look_ahead_base', 0.4)
        self.pp_gain_forward = map_manager.get_param('pure_pursuit.gain_forward', 1.0)
        self.pp_gain_backward = map_manager.get_param('pure_pursuit.gain_backward', 0.8)
        self.pp_deadband_cte = map_manager.get_param('pure_pursuit.deadband_cte', 0.005)
        self.pp_deadband_head = map_manager.get_param('pure_pursuit.deadband_heading', 1.0)

        # Robot parameters
        self.robot_length = map_manager.get_param('length', 0.80)
        self.robot_width = map_manager.get_param('width', 0.50)
        self.linear_speed = map_manager.get_param('speeds.linear', 0.3)

        # Wall distance parameters
        self.wall_dist_work_zone = map_manager.get_param('wall_distances.work_zone', 0.35)
        self.wall_dist_zone_a = map_manager.get_param('wall_distances.zone_a', 0.6275)
        self.min_wall_clearance = map_manager.get_param('wall_distances.min_clearance', 0.10)

    def calculate_forward(self, lateral, distance_to_tag):
        """
        Pure Pursuit for forward movement.
        PRESERVED: Original algorithm from navigation_purepursuit.py

        Args:
            lateral: Cross-track error (+ = tag is right = need to turn right)
            distance_to_tag: Distance to target tag

        Returns:
            Angular velocity command
        """
        # Effective look-ahead
        L_eff = max(distance_to_tag, self.look_ahead_base)

        # Alpha: angle to target
        # If lateral > 0, tag is on right, alpha < 0
        alpha = math.atan2(-lateral, L_eff)

        # Curvature
        curvature = (2.0 * math.sin(alpha)) / L_eff

        # Angular velocity
        w = self.linear_speed * curvature * self.pp_gain_forward

        # Deadband for stability
        if abs(lateral) < self.pp_deadband_cte:
            w = 0.0

        return np.clip(w, -0.3, 0.3)

    def calculate_backward(self, lateral, distance_to_tag):
        """
        Pure Pursuit for backward movement.
        PRESERVED: Original algorithm from navigation_purepursuit.py

        When backing up:
        - lateral > 0: tag is on RIGHT in camera = robot drifted LEFT of path
        - To get back to center, robot needs to move RIGHT
        - When backing up, to move RIGHT, we turn RIGHT (angular < 0)

        Same angular direction as forward! Only linear velocity is negative.

        Args:
            lateral: Cross-track error (+ = tag is right in camera view)
            distance_to_tag: Distance to target tag

        Returns:
            Angular velocity command
        """
        # Effective look-ahead
        L_eff = max(distance_to_tag, self.look_ahead_base)

        # Same alpha as forward: lateral > 0 → alpha < 0 → turn right
        alpha = math.atan2(-lateral, L_eff)

        # Curvature
        curvature = (2.0 * math.sin(alpha)) / L_eff

        # Angular velocity - use abs(speed) to keep curvature direction
        w = abs(self.linear_speed) * curvature * self.pp_gain_backward

        # Deadband for stability
        if abs(lateral) < self.pp_deadband_cte:
            w = 0.0

        return np.clip(w, -0.3, 0.3)

    def calculate_absolute(self, robot_x, robot_y, robot_heading, target_x, target_y,
                          current_lateral, move_dir):
        """
        Pure Pursuit using absolute coordinates with real-time tracking.
        PRESERVED: Original algorithm from navigation_purepursuit.py

        Args:
            robot_x: Robot X position in world frame
            robot_y: Robot Y position in world frame
            robot_heading: Robot heading in radians
            target_x: Target X position in world frame
            target_y: Target Y position in world frame
            current_lateral: Current lateral offset
            move_dir: 'forward' or 'backward'

        Returns:
            Angular velocity command
        """
        dx = target_x - robot_x
        dy = target_y - robot_y
        distance = math.sqrt(dx*dx + dy*dy)

        if distance < 0.01:
            return 0.0

        angle_to_target = math.atan2(dy, dx)

        # Calculate alpha (angle error)
        if move_dir == 'backward':
            alpha = angle_to_target - (robot_heading + math.pi)
        else:
            alpha = angle_to_target - robot_heading

        # Normalize to [-pi, pi]
        while alpha > math.pi:
            alpha -= 2 * math.pi
        while alpha < -math.pi:
            alpha += 2 * math.pi

        L_eff = max(distance, self.look_ahead_base)
        curvature = (2.0 * math.sin(alpha)) / L_eff

        # Angular velocity
        if move_dir == 'backward':
            w = -abs(self.linear_speed) * curvature * self.pp_gain_backward
        else:
            w = self.linear_speed * curvature * self.pp_gain_forward

        # Reduce correction when close to target (prevents oversteer near arrival)
        if distance < 0.3:
            w *= (distance / 0.3)  # Scale down as we get closer

        if abs(current_lateral) < self.pp_deadband_cte and abs(math.degrees(alpha)) < self.pp_deadband_head:
            w = 0.0

        return np.clip(w, -0.3, 0.3)

    def calculate_wall_distance(self, lateral, align_angle_deg, zone):
        """
        Calculate distance from robot's right side to wall.
        PRESERVED: Original algorithm from navigation_purepursuit.py

        Wall is always on the right side of the robot.

        Args:
            lateral: pose_t[0] from AprilTag detection (+ = tag is right of camera center)
            align_angle_deg: Robot rotation angle in degrees
            zone: Current zone ('A', 'B', 'C', 'D', 'E', 'DOCK')

        Returns:
            Distance from robot's right side to wall in meters
        """
        # Base distance from tag center line to wall
        if zone in ['B', 'C', 'D', 'E']:
            base_dist = self.wall_dist_work_zone  # 35cm
        else:
            base_dist = self.wall_dist_zone_a  # 63cm

        # Camera looks DOWN at floor tags.
        # lateral > 0 means tag appears on RIGHT side of camera image.
        #
        # Zone A: Robot faces +X, wall at -Y (robot's right)
        #   Camera right = world +Y direction
        #   lateral > 0 = tag shifted +Y = robot is at -Y = CLOSER to wall
        #   → center_to_wall = base - lateral
        #
        # Zone B/D: Robot faces +Y, wall at -X (robot's right)
        #   Camera right = world -X direction
        #   lateral > 0 = robot shifted -X = CLOSER to wall (-X)
        #   → center_to_wall = base + lateral
        #
        # Zone C/E: Robot faces -Y, wall at +X (robot's right)
        #   Camera right = world +X direction
        #   lateral > 0 = robot shifted +X = CLOSER to wall (+X)
        #   → center_to_wall = base + lateral

        if zone in ['B', 'D']:
            center_to_wall = base_dist + lateral
        elif zone in ['C', 'E']:
            center_to_wall = base_dist + lateral
        else:  # Zone A
            center_to_wall = base_dist - lateral

        # Account for robot rotation
        angle_rad = math.radians(align_angle_deg)
        half_length = self.robot_length / 2
        half_width = self.robot_width / 2

        # Right side distance considering rotation
        right_side_dist = center_to_wall - half_width * math.cos(angle_rad) - half_length * abs(math.sin(angle_rad))

        return right_side_dist

    def get_robot_pose_from_tag(self, tag_id, lateral, align_angle_deg):
        """
        Calculate robot position in world coordinates from tag detection.
        PRESERVED: Original coordinate transformation logic.

        Args:
            tag_id: Detected tag ID
            lateral: Lateral offset from tag (+ = tag is right of camera center)
            align_angle_deg: Robot rotation relative to tag in degrees

        Returns:
            Tuple of (robot_x, robot_y, robot_heading_rad) or (None, None, None)
        """
        tag_info = self.tag_db.get(tag_id)
        if not tag_info:
            return None, None, None

        tag_x = tag_info['x']
        tag_y = tag_info['y']
        zone = tag_info.get('zone', 'A')

        if zone == 'A' or zone == 'DOCK':
            # Robot heading +X
            robot_x = tag_x
            robot_y = tag_y + lateral
            robot_heading = math.radians(align_angle_deg)
        elif zone == 'B':
            # Robot heading +Y, backward goes -Y
            # Wall is at -X side
            # lateral > 0 = tag on camera right = robot is shifted toward -X (toward wall)
            robot_x = tag_x - lateral
            robot_y = tag_y
            robot_heading = math.pi/2 + math.radians(align_angle_deg)
        elif zone == 'C':
            # Robot heading -Y, forward goes -Y
            # Wall is at +X side
            # lateral > 0 = tag on camera right = robot is shifted toward +X (toward wall)
            robot_x = tag_x + lateral
            robot_y = tag_y
            robot_heading = -math.pi/2 + math.radians(align_angle_deg)
        elif zone == 'D':
            # Robot heading +Y, backward goes -Y (same as B)
            robot_x = tag_x - lateral
            robot_y = tag_y
            robot_heading = math.pi/2 + math.radians(align_angle_deg)
        elif zone == 'E':
            # Robot heading -Y, forward goes -Y (same as C)
            # Wall is at +X side
            # lateral > 0 = tag on camera right = robot is shifted toward +X (toward wall)
            robot_x = tag_x + lateral
            robot_y = tag_y
            robot_heading = -math.pi/2 + math.radians(align_angle_deg)
        else:
            robot_x = tag_x
            robot_y = tag_y
            robot_heading = 0

        return robot_x, robot_y, robot_heading


class CoordinateTransformer:
    """
    Coordinate system transformations for different subsystems.
    Used for Mode 4 pose publishing to manipulator.
    """

    @staticmethod
    def isaac_to_manipulator(isaac_x, isaac_y):
        """
        Convert Isaac Sim coordinates to manipulator coordinate system.

        Isaac Sim: tag 100 at (-0.3975, 0.45)
        Manipulator: tag 100 at (-0.45, 0.3975)

        Args:
            isaac_x: X coordinate in Isaac Sim frame
            isaac_y: Y coordinate in Isaac Sim frame

        Returns:
            Tuple of (pub_x, pub_y) in manipulator frame
        """
        pub_x = -isaac_y
        pub_y = -isaac_x
        return pub_x, pub_y

    @staticmethod
    def apply_robot_center_offset(manip_x, manip_y, zone, camera_offset=0.45):
        """
        Apply camera-to-robot-center offset.
        Camera is 0.45m in front of robot center.

        Args:
            manip_x: X coordinate in manipulator frame (camera position)
            manip_y: Y coordinate in manipulator frame (camera position)
            zone: Current zone
            camera_offset: Camera offset from robot center (default: 0.45m)

        Returns:
            Robot CENTER position (not camera position) as (robot_x, robot_y)
        """
        if zone == 'A' or zone == 'DOCK':
            # Robot heading +X (Isaac) = +Y (Manip)
            robot_x = manip_x
            robot_y = manip_y + camera_offset
        elif zone in ['B', 'D']:
            # Robot heading +Y (Isaac) = +X (Manip)
            robot_x = manip_x + camera_offset
            robot_y = manip_y
        elif zone in ['C', 'E']:
            # Robot heading -Y (Isaac) = -X (Manip)
            robot_x = manip_x - camera_offset
            robot_y = manip_y
        else:
            robot_x = manip_x
            robot_y = manip_y

        return robot_x, robot_y
