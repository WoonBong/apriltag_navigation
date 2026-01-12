#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Field Diagnostics Tool
CLI tool for testing individual robot subsystems in the field.
Allows verification of hardware and perception without running full navigation.

Usage:
    python diagnostics.py --test-motor --distance 2.0
    python diagnostics.py --test-vision
    python diagnostics.py --test-pivot --direction ccw
    python diagnostics.py --test-all
"""

import rospy
import argparse
import sys
import os
import time

# Add src to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from apriltag_navigation.map.map_manager import MapManager
from apriltag_navigation.perception.vision_module import VisionModule
from apriltag_navigation.hardware.robot_controller import RobotController, RotationController


class DiagnosticsTool:
    """
    Diagnostic tool for field testing robot subsystems.
    """

    def __init__(self):
        """Initialize diagnostics tool"""
        rospy.init_node('diagnostics', anonymous=True)

        print("=" * 60)
        print("AprilTag Navigation - Field Diagnostics Tool")
        print("=" * 60)

        # Initialize subsystems
        print("\nInitializing subsystems...")
        self.map_manager = MapManager()
        self.vision = VisionModule()
        self.robot = RobotController(self.map_manager)
        self.rotation_ctrl = RotationController(self.robot)

        print("[OK] All subsystems initialized")

    def wait_for_ready(self):
        """Wait for subsystems to be ready"""
        print("\nWaiting for camera and odometry...")
        rate = rospy.Rate(10)

        timeout = rospy.Time.now() + rospy.Duration(10.0)
        while not rospy.is_shutdown() and rospy.Time.now() < timeout:
            if self.vision.is_ready() and self.robot.is_ready():
                print("[OK] All systems ready\n")
                return True
            rate.sleep()

        print("[ERROR] Timeout waiting for systems")
        return False

    def test_motor(self, distance=1.0, direction='forward'):
        """
        Test motor movement.

        Args:
            distance: Distance to move in meters
            direction: 'forward' or 'backward'
        """
        print("=" * 60)
        print(f"MOTOR TEST: Move {direction} {distance}m")
        print("=" * 60)

        # Get starting position
        start_x, start_y = self.robot.get_position()
        print(f"Starting position: ({start_x:.3f}, {start_y:.3f})")

        # Calculate target distance
        target_distance = distance
        speed = self.map_manager.get_param('speeds.linear', 0.3)

        if direction == 'backward':
            speed = -speed

        print(f"Moving at {abs(speed):.2f} m/s...")
        print("Press Ctrl+C to stop early\n")

        rate = rospy.Rate(30)
        start_time = rospy.Time.now()

        try:
            while not rospy.is_shutdown():
                # Move
                self.robot.move(speed, 0)

                # Check distance traveled
                current_x, current_y = self.robot.get_position()
                dx = current_x - start_x
                dy = current_y - start_y
                traveled = (dx**2 + dy**2)**0.5

                # Print status
                remaining = target_distance - traveled
                if traveled % 0.1 < 0.05:  # Print every ~10cm
                    print(f"  Traveled: {traveled:.2f}m / {target_distance:.2f}m (remaining: {remaining:.2f}m)")

                # Check if reached target
                if traveled >= target_distance:
                    break

                rate.sleep()

        except KeyboardInterrupt:
            print("\n[INTERRUPTED] Stopping...")

        finally:
            self.robot.stop()
            time.sleep(0.5)

            # Final position
            end_x, end_y = self.robot.get_position()
            dx = end_x - start_x
            dy = end_y - start_y
            actual_distance = (dx**2 + dy**2)**0.5

            print("\n" + "=" * 60)
            print("MOTOR TEST RESULTS")
            print("=" * 60)
            print(f"Starting position: ({start_x:.3f}, {start_y:.3f})")
            print(f"Ending position:   ({end_x:.3f}, {end_y:.3f})")
            print(f"Target distance:   {target_distance:.3f}m")
            print(f"Actual distance:   {actual_distance:.3f}m")
            print(f"Error:             {abs(actual_distance - target_distance):.3f}m ({abs(actual_distance - target_distance)/target_distance*100:.1f}%)")
            print("=" * 60)

    def test_vision(self, duration=10.0):
        """
        Test AprilTag vision system.

        Args:
            duration: How long to run test in seconds
        """
        print("=" * 60)
        print(f"VISION TEST: Detecting AprilTags for {duration}s")
        print("=" * 60)

        if not self.vision.is_ready():
            print("[ERROR] Vision system not ready!")
            return

        print("Detecting tags...\n")

        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        detected_tags_history = set()

        try:
            while not rospy.is_shutdown():
                elapsed = (rospy.Time.now() - start_time).to_sec()
                if elapsed >= duration:
                    break

                # Get detected tags
                detected_tags = self.vision.get_detected_tags()

                # Update history
                detected_tags_history.update(detected_tags.keys())

                # Print current detections
                print(f"\r[{elapsed:.1f}s] ", end='')
                if detected_tags:
                    tag_strs = []
                    for tag_id, data in detected_tags.items():
                        tag_strs.append(f"Tag {tag_id}: dist={data['z']:.2f}m lat={data['x']:+.3f}m")
                    print(" | ".join(tag_strs), end='')
                else:
                    print("No tags detected", end='')

                sys.stdout.flush()
                rate.sleep()

        except KeyboardInterrupt:
            print("\n[INTERRUPTED] Stopping...")

        print("\n\n" + "=" * 60)
        print("VISION TEST RESULTS")
        print("=" * 60)
        print(f"Test duration: {duration}s")
        print(f"Unique tags detected: {len(detected_tags_history)}")
        print(f"Tag IDs: {sorted(detected_tags_history)}")
        print("=" * 60)

    def test_pivot(self, direction='cw', angle=90):
        """
        Test pivot rotation.

        Args:
            direction: 'cw' or 'ccw'
            angle: Angle to rotate in degrees (default: 90)
        """
        print("=" * 60)
        print(f"PIVOT TEST: Rotate {angle}° {direction.upper()}")
        print("=" * 60)

        # Get starting heading
        start_heading = self.robot.get_heading()
        start_deg = start_heading * 180.0 / 3.14159

        print(f"Starting heading: {start_deg:.1f}°")
        print(f"Rotating {angle}° {direction}...\n")

        # Start rotation
        self.rotation_ctrl.start_rotation(angle, direction)

        rate = rospy.Rate(30)

        try:
            while not rospy.is_shutdown():
                # Update rotation controller
                if self.rotation_ctrl.update():
                    break
                rate.sleep()

        except KeyboardInterrupt:
            print("\n[INTERRUPTED] Stopping...")
            self.rotation_ctrl.cancel()

        finally:
            time.sleep(0.5)

            # Final heading
            end_heading = self.robot.get_heading()
            end_deg = end_heading * 180.0 / 3.14159

            # Calculate actual rotation
            rotation = end_heading - start_heading
            # Normalize to [-pi, pi]
            while rotation > 3.14159:
                rotation -= 2 * 3.14159
            while rotation < -3.14159:
                rotation += 2 * 3.14159
            rotation_deg = rotation * 180.0 / 3.14159

            # Expected rotation
            expected = angle if direction == 'ccw' else -angle

            print("\n" + "=" * 60)
            print("PIVOT TEST RESULTS")
            print("=" * 60)
            print(f"Starting heading: {start_deg:.1f}°")
            print(f"Ending heading:   {end_deg:.1f}°")
            print(f"Target rotation:  {expected:+.1f}°")
            print(f"Actual rotation:  {rotation_deg:+.1f}°")
            print(f"Error:            {abs(rotation_deg - expected):.1f}°")
            print("=" * 60)

    def test_all(self):
        """Run all diagnostic tests"""
        print("\n" + "=" * 60)
        print("RUNNING ALL DIAGNOSTIC TESTS")
        print("=" * 60)

        # Test 1: Vision
        print("\n[1/3] Testing vision system...")
        self.test_vision(duration=5.0)

        input("\nPress Enter to continue to motor test...")

        # Test 2: Motor
        print("\n[2/3] Testing motor (forward 1m)...")
        self.test_motor(distance=1.0, direction='forward')

        input("\nPress Enter to continue to pivot test...")

        # Test 3: Pivot
        print("\n[3/3] Testing pivot rotation...")
        self.test_pivot(direction='cw', angle=90)

        print("\n" + "=" * 60)
        print("ALL TESTS COMPLETE")
        print("=" * 60)


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(
        description='Field diagnostics tool for AprilTag navigation robot',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Test motor moving forward 2 meters
  python diagnostics.py --test-motor --distance 2.0 --direction forward

  # Test vision system for 10 seconds
  python diagnostics.py --test-vision --duration 10

  # Test pivot rotation 90 degrees counter-clockwise
  python diagnostics.py --test-pivot --direction ccw --angle 90

  # Run all tests
  python diagnostics.py --test-all
        """
    )

    parser.add_argument('--test-motor', action='store_true', help='Test motor movement')
    parser.add_argument('--test-vision', action='store_true', help='Test AprilTag vision')
    parser.add_argument('--test-pivot', action='store_true', help='Test pivot rotation')
    parser.add_argument('--test-all', action='store_true', help='Run all tests')

    parser.add_argument('--distance', type=float, default=1.0, help='Distance to move (meters) for motor test')
    parser.add_argument('--direction', type=str, default='forward', choices=['forward', 'backward', 'cw', 'ccw'],
                        help='Direction for motor or pivot test')
    parser.add_argument('--duration', type=float, default=10.0, help='Duration (seconds) for vision test')
    parser.add_argument('--angle', type=float, default=90.0, help='Angle (degrees) for pivot test')

    args = parser.parse_args()

    # Check if any test selected
    if not (args.test_motor or args.test_vision or args.test_pivot or args.test_all):
        parser.print_help()
        return

    try:
        # Initialize diagnostics tool
        diag = DiagnosticsTool()

        # Wait for systems ready
        if not diag.wait_for_ready():
            return

        # Run selected tests
        if args.test_all:
            diag.test_all()

        else:
            if args.test_motor:
                diag.test_motor(distance=args.distance, direction=args.direction)

            if args.test_vision:
                diag.test_vision(duration=args.duration)

            if args.test_pivot:
                diag.test_pivot(direction=args.direction, angle=args.angle)

        print("\nDiagnostics complete!")

    except KeyboardInterrupt:
        print("\n\nDiagnostics interrupted by user")
    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
