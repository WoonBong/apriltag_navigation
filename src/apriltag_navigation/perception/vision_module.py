#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Vision Module - AprilTag Detection and Pose Estimation
Encapsulates all camera and AprilTag detection logic
"""

import rospy
import cv2
import math
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

try:
    from dt_apriltags import Detector
except ImportError:
    import subprocess
    subprocess.check_call(['pip', 'install', 'dt-apriltags'])
    from dt_apriltags import Detector


class VisionModule:
    """
    Vision module for AprilTag detection and pose estimation.
    Handles camera calibration and provides detected tag information.
    """

    def __init__(self, tag_size=0.06, image_topic='/rgb', camera_info_topic='/camera_info'):
        """
        Initialize vision module.

        Args:
            tag_size: Physical size of AprilTags in meters (default: 6cm)
            image_topic: ROS topic for camera images
            camera_info_topic: ROS topic for camera calibration info
        """
        self.tag_size = tag_size
        self.bridge = CvBridge()

        # Camera parameters
        self.camera_params = None  # [fx, fy, cx, cy]
        self.camera_info_received = False
        self.image_width = 1280
        self.image_height = 720
        self.image_center_x = self.image_width // 2
        self.image_center_y = self.image_height // 2

        # Detected tags {id: tag_data}
        self.detected_tags = {}

        # AprilTag detector
        self.detector = Detector(
            families='tag36h11',
            nthreads=4,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

        # ROS subscribers
        self.image_sub = rospy.Subscriber(image_topic, Image, self._image_callback)
        self.camera_info_sub = rospy.Subscriber(camera_info_topic, CameraInfo, self._camera_info_callback)

        rospy.loginfo("[VISION] Vision module initialized")

    def _camera_info_callback(self, msg):
        """Callback for camera calibration info"""
        if not self.camera_info_received:
            K = msg.K
            fx, fy = K[0], K[4]
            cx, cy = K[2], K[5]
            self.camera_params = [fx, fy, cx, cy]
            self.camera_info_received = True
            rospy.loginfo(f"[VISION] Camera params: fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}")

    def _image_callback(self, msg):
        """Callback for camera images - performs tag detection"""
        if not self.camera_info_received:
            return

        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Detect tags with pose estimation
            detections = self.detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=self.camera_params,
                tag_size=self.tag_size
            )

            # Clear previous detections
            self.detected_tags.clear()

            # Process detections
            for det in detections:
                if det.pose_t is not None:
                    # pose_t: [x, y, z] in camera frame
                    # x: right+, y: down+, z: forward+
                    pose_t = det.pose_t.flatten()
                    pose_R = det.pose_R

                    # Calculate yaw angle of tag relative to camera
                    yaw = math.atan2(pose_R[1, 0], pose_R[0, 0])

                    self.detected_tags[det.tag_id] = {
                        'x': pose_t[0],      # lateral offset (+ = tag is right)
                        'y': pose_t[1],      # vertical offset
                        'z': pose_t[2],      # distance to tag
                        'yaw': yaw,          # tag rotation
                        'pose_R': pose_R,
                        'center_x': det.center[0],  # pixel x
                        'center_y': det.center[1],  # pixel y
                        'corners': det.corners,     # 4 corners for alignment
                    }

        except Exception as e:
            rospy.logerr(f"[VISION] Image processing error: {e}")

    def is_ready(self):
        """Check if vision module is ready (camera calibrated)"""
        return self.camera_info_received

    def get_detected_tags(self):
        """Get dictionary of currently detected tags"""
        return self.detected_tags

    def is_tag_visible(self, tag_id):
        """Check if a specific tag is currently visible"""
        return tag_id in self.detected_tags

    def get_tag_data(self, tag_id):
        """
        Get detection data for a specific tag.

        Returns:
            Dictionary with tag data or None if not detected
        """
        return self.detected_tags.get(tag_id)

    def get_tag_lateral(self, tag_id):
        """
        Get lateral offset of tag from camera center.

        Returns:
            Lateral offset in meters (+ = right), or None if not detected
        """
        tag_data = self.get_tag_data(tag_id)
        return tag_data['x'] if tag_data else None

    def get_tag_distance(self, tag_id):
        """
        Get distance to tag.

        Returns:
            Distance in meters, or None if not detected
        """
        tag_data = self.get_tag_data(tag_id)
        return tag_data['z'] if tag_data else None

    def get_alignment_angle(self, tag_id):
        """
        Calculate alignment angle from tag's top edge.
        Uses tag corners to determine if robot is aligned with tag.

        Args:
            tag_id: Tag ID to check alignment

        Returns:
            Alignment angle in degrees, or None if tag not detected or no corners
        """
        tag_data = self.get_tag_data(tag_id)
        if not tag_data:
            return None

        corners = tag_data.get('corners')
        if corners is None:
            return None

        # Calculate alignment angle from tag's TOP EDGE
        # corners: [top-left, top-right, bottom-right, bottom-left]
        top_left = corners[0]
        top_right = corners[1]

        dx = top_right[0] - top_left[0]
        dy = top_right[1] - top_left[1]

        # Angle from horizontal (should be 0 when aligned)
        align_angle = math.atan2(dy, dx)
        align_angle_deg = math.degrees(align_angle)

        # In Zone C/E, tags appear "upside down" (rotated 180°)
        # Normalize to [-90, 90] range
        if align_angle_deg > 90:
            align_angle_deg -= 180
        elif align_angle_deg < -90:
            align_angle_deg += 180

        return align_angle_deg

    def get_tag_center_pixel(self, tag_id):
        """
        Get pixel coordinates of tag center.

        Returns:
            Tuple of (center_x, center_y) or None if not detected
        """
        tag_data = self.get_tag_data(tag_id)
        if tag_data:
            return tag_data['center_x'], tag_data['center_y']
        return None

    def is_tag_centered(self, tag_id, tolerance=50):
        """
        Check if tag is centered horizontally in image.

        Args:
            tag_id: Tag ID to check
            tolerance: Pixel tolerance (default: 50 pixels)

        Returns:
            True if tag center is within tolerance of image center
        """
        pixel_coords = self.get_tag_center_pixel(tag_id)
        if pixel_coords is None:
            return False

        center_x, center_y = pixel_coords
        return abs(center_y - self.image_center_y) < tolerance

    def get_detection_summary(self):
        """
        Get summary string of currently detected tags.

        Returns:
            Human-readable string describing detected tags
        """
        tag_info = []
        for tid, data in self.detected_tags.items():
            tag_info.append(f"{tid}(lat:{data['x']:.2f}m z:{data['z']:.2f}m)")
        return f"Tags: {tag_info}" if tag_info else "No tags detected"
