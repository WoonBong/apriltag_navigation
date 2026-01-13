#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Test Helper Module
Provides lightweight testing functions for integration with main_node.py.
For detailed diagnostics, use scripts/diagnostics.py instead.
"""

import rospy
import sys
import time


def run_diagnostics(args):
    """
    Run diagnostic tests based on command-line arguments.

    Args:
        args: Parsed command-line arguments
    """
    from apriltag_navigation.map.map_manager import MapManager
    from apriltag_navigation.perception.vision_module import VisionModule
    from apriltag_navigation.hardware.robot_controller import RobotController, RotationController

    print("=" * 60)
    print("AprilTag Navigation - Quick Test Mode")
    print("=" * 60)
    print("\nNote: For detailed diagnostics, use scripts/diagnostics.py\n")

    # Initialize subsystems
    print("Initializing subsystems...")
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

    print("\nTests complete!")


def test_vision(vision, duration):
    """
    Quick vision system test.

    Args:
        vision: VisionModule instance
        duration: Test duration in seconds
    """
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
    """
    Quick motor movement test.

    Args:
        robot: RobotController instance
        distance: Distance to travel in meters
        direction: 'forward' or 'backward'
    """
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
    """
    Quick pivot rotation test.

    Args:
        robot: RobotController instance
        rotation_ctrl: RotationController instance
        direction: 'cw' or 'ccw'
    """
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
