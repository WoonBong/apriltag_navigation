#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Robot Controller - Hardware Interface and Movement Primitives
Encapsulates all ROS communication (pub/sub) and basic movement commands.
Provides clean interface for higher-level control.
"""

import rospy
import math
import tf.transformations as tft
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class RobotController:
    """
    Low-level robot controller for movement and odometry tracking.
    Hides all ROS publishers/subscribers from higher-level code.
    """

    def __init__(self, map_manager):
        """
        Initialize robot controller.

        Args:
            map_manager: MapManager instance for accessing configuration
        """
        self.map_manager = map_manager

        # Load speed parameters
        self.linear_speed = map_manager.get_param('speeds.linear', 0.3)
        self.angular_speed = map_manager.get_param('speeds.angular', 0.25)
        self.slow_factor = map_manager.get_param('speeds.slow_factor', 0.3)

        # Odometry state
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.current_theta = 0.0
        self.odom_received = False

        # Reference point for odom-based tracking
        # Set when aligned at a tag, used to calculate delta during movement
        self.ref_odom_x = 0.0
        self.ref_odom_y = 0.0
        self.ref_odom_theta = 0.0
        self.ref_robot_x = 0.0  # World coordinates
        self.ref_robot_y = 0.0
        self.ref_robot_heading = 0.0

        # ROS publishers/subscribers
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self._odom_callback)

        rospy.loginfo("[ROBOT CONTROLLER] Initialized")

    def _odom_callback(self, msg):
        """Callback for odometry data"""
        # Position
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y

        # Orientation (convert quaternion to euler)
        q = msg.pose.pose.orientation
        euler = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_theta = euler[2]  # yaw angle
        self.odom_received = True

    def is_ready(self):
        """Check if robot controller is ready (odometry received)"""
        return self.odom_received

    def get_position(self):
        """
        Get current robot position from odometry.

        Returns:
            Tuple of (x, y) in meters
        """
        return self.odom_x, self.odom_y

    def get_heading(self):
        """
        Get current robot heading from odometry.

        Returns:
            Heading in radians
        """
        return self.current_theta

    def set_reference_point(self, robot_x, robot_y, robot_heading):
        """
        Set reference point for odometry-based tracking.
        Called when robot is aligned at a known tag position.

        Args:
            robot_x: Robot X position in world frame
            robot_y: Robot Y position in world frame
            robot_heading: Robot heading in radians
        """
        self.ref_robot_x = robot_x
        self.ref_robot_y = robot_y
        self.ref_robot_heading = robot_heading
        self.ref_odom_x = self.odom_x
        self.ref_odom_y = self.odom_y
        self.ref_odom_theta = self.current_theta

        rospy.loginfo(f"[ROBOT CONTROLLER] Reference set: robot=({robot_x:.3f},{robot_y:.3f}) odom=({self.odom_x:.3f},{self.odom_y:.3f})")

    def get_estimated_pose(self):
        """
        Get estimated robot pose using odometry delta from reference point.
        Uses odometry to track position when tags are not visible.

        Returns:
            Tuple of (robot_x, robot_y, robot_heading)
        """
        # Calculate odometry delta from reference
        odom_dx = self.odom_x - self.ref_odom_x
        odom_dy = self.odom_y - self.ref_odom_y

        # Odom is global and aligned with Isaac Sim at start
        # So isaac_delta = odom_delta (no transformation needed!)
        robot_x = self.ref_robot_x + odom_dx
        robot_y = self.ref_robot_y + odom_dy

        # Heading: use current odom theta relative to reference
        theta_delta = self.current_theta - self.ref_odom_theta
        robot_heading = self.ref_robot_heading + theta_delta

        return robot_x, robot_y, robot_heading

    def move(self, linear=0.0, angular=0.0):
        """
        Send velocity command to robot.

        Args:
            linear: Linear velocity in m/s
            angular: Angular velocity in rad/s
        """
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_pub.publish(twist)

    def stop(self):
        """Stop the robot (all velocities to zero)"""
        self.move(0, 0)

    def move_forward(self, speed=None):
        """
        Move forward at specified or default speed.

        Args:
            speed: Linear speed (default: configured linear_speed)
        """
        if speed is None:
            speed = self.linear_speed
        self.move(linear=speed, angular=0)

    def move_backward(self, speed=None):
        """
        Move backward at specified or default speed.

        Args:
            speed: Linear speed magnitude (default: configured linear_speed)
        """
        if speed is None:
            speed = self.linear_speed
        self.move(linear=-speed, angular=0)

    def rotate_cw(self, speed=None):
        """
        Rotate clockwise at specified or default speed.

        Args:
            speed: Angular speed magnitude (default: configured angular_speed)
        """
        if speed is None:
            speed = self.angular_speed
        self.move(linear=0, angular=-speed)

    def rotate_ccw(self, speed=None):
        """
        Rotate counter-clockwise at specified or default speed.

        Args:
            speed: Angular speed magnitude (default: configured angular_speed)
        """
        if speed is None:
            speed = self.angular_speed
        self.move(linear=0, angular=speed)


class RotationController:
    """
    Helper class for executing precise rotations.
    Uses odometry feedback to achieve target heading.
    """

    def __init__(self, robot_controller):
        """
        Initialize rotation controller.

        Args:
            robot_controller: RobotController instance
        """
        self.robot = robot_controller
        self.target_theta = None
        self.is_rotating = False

    def start_rotation(self, degrees, direction):
        """
        Start rotation to target angle.

        Args:
            degrees: Angle to rotate in degrees
            direction: 'cw' for clockwise, 'ccw' for counter-clockwise
        """
        angle_rad = math.radians(abs(degrees))
        current = self.robot.get_heading()

        if direction == 'cw':
            self.target_theta = current - angle_rad
        else:
            self.target_theta = current + angle_rad

        # Normalize to [-pi, pi]
        while self.target_theta > math.pi:
            self.target_theta -= 2 * math.pi
        while self.target_theta < -math.pi:
            self.target_theta += 2 * math.pi

        self.is_rotating = True

        rospy.loginfo(f"[ROTATION] {math.degrees(current):.1f}° -> {math.degrees(self.target_theta):.1f}° ({direction})")

    def get_rotation_error(self):
        """
        Get current rotation error.

        Returns:
            Error in radians (signed)
        """
        if self.target_theta is None:
            return 0.0

        current = self.robot.get_heading()
        error = self.target_theta - current

        # Normalize to [-pi, pi]
        while error > math.pi:
            error -= 2 * math.pi
        while error < -math.pi:
            error += 2 * math.pi

        return error

    def is_complete(self, tolerance_deg=1.0):
        """
        Check if rotation is complete.

        Args:
            tolerance_deg: Tolerance in degrees (default: 1.0)

        Returns:
            True if within tolerance
        """
        if not self.is_rotating:
            return True

        error = self.get_rotation_error()
        return abs(math.degrees(error)) < tolerance_deg

    def update(self):
        """
        Update rotation control.
        Call this periodically during rotation.

        Returns:
            True if rotation complete, False otherwise
        """
        if not self.is_rotating:
            return True

        if self.is_complete():
            self.robot.stop()
            self.is_rotating = False
            rospy.loginfo("[ROTATION] Complete")
            return True

        # Calculate control
        error = self.get_rotation_error()
        angular = 0.2 if error > 0 else -0.2

        # Slow down near target
        if abs(math.degrees(error)) < 10:
            angular *= 0.5

        self.robot.move(0, angular)
        rospy.loginfo_throttle(0.3, f"[ROTATION] Error: {math.degrees(error):.1f}°")

        return False

    def cancel(self):
        """Cancel ongoing rotation"""
        self.is_rotating = False
        self.robot.stop()
