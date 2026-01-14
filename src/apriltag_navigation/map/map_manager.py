#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Map Manager - Data-Driven Map Configuration
Loads all map topology from YAML configuration file
"""

import yaml
import rospy
import rospkg
from enum import Enum, auto
from collections import deque


class TagType(Enum):
    """Tag type enumeration"""
    DOCK = auto()
    PIVOT = auto()
    WORK = auto()
    MOVE = auto()


class TagDatabase:
    """
    Tag database loaded from YAML configuration.
    Provides query interface for tag information.
    """

    def __init__(self, config_dict):
        """
        Initialize from configuration dictionary.

        Args:
            config_dict: Dictionary loaded from YAML
        """
        self.tags = {}
        self._load_tags(config_dict['tags'])

    def _load_tags(self, tags_dict):
        """Load tags from configuration dictionary"""
        for tag_id, tag_data in tags_dict.items():
            # Convert tag_id to int
            tag_id = int(tag_id)

            # Convert type string to enum
            type_str = tag_data['type']
            tag_type = TagType[type_str]

            # Store tag info
            self.tags[tag_id] = {
                'x': float(tag_data['x']),
                'y': float(tag_data['y']),
                'type': tag_type,
                'zone': tag_data['zone']
            }

            # Optional name field
            if 'name' in tag_data:
                self.tags[tag_id]['name'] = tag_data['name']

    def get(self, tag_id):
        """Get tag information by ID"""
        return self.tags.get(tag_id)

    def exists(self, tag_id):
        """Check if tag exists in database"""
        return tag_id in self.tags

    def get_zone(self, tag_id):
        """Get zone for a tag"""
        tag_info = self.get(tag_id)
        return tag_info.get('zone', 'A') if tag_info else 'A'

    def get_position(self, tag_id):
        """Get (x, y) position of tag"""
        tag_info = self.get(tag_id)
        if tag_info:
            return tag_info['x'], tag_info['y']
        return None, None


class NavigationGraph:
    """
    Navigation graph loaded from YAML configuration.
    Provides pathfinding and edge query interface.
    """

    def __init__(self, config_dict):
        """
        Initialize from configuration dictionary.

        Args:
            config_dict: Dictionary loaded from YAML
        """
        self.edges = {}
        self._load_edges(config_dict['edges'])

    def _load_edges(self, edges_list):
        """Load edges from configuration list"""
        for edge_data in edges_list:
            from_tag = int(edge_data['from'])
            to_tag = int(edge_data['to'])
            direction = edge_data['direction']
            edge_type = edge_data['type']

            if from_tag not in self.edges:
                self.edges[from_tag] = []

            self.edges[from_tag].append({
                'to': to_tag,
                'direction': direction,
                'type': edge_type
            })

    def get_edge(self, from_tag, to_tag):
        """Get edge information between two tags"""
        if from_tag in self.edges:
            for edge in self.edges[from_tag]:
                if edge['to'] == to_tag:
                    return edge
        return None

    def find_path(self, start, goal):
        """
        Find shortest path from start to goal using BFS.

        Args:
            start: Starting tag ID
            goal: Goal tag ID

        Returns:
            List of tag IDs representing path, or None if no path exists
        """
        if start == goal:
            return [start]

        if start not in self.edges:
            return None

        visited = {start}
        queue = deque([(start, [start])])

        while queue:
            current, path = queue.popleft()

            if current not in self.edges:
                continue

            for edge in self.edges[current]:
                next_tag = edge['to']
                if next_tag not in visited:
                    new_path = path + [next_tag]
                    if next_tag == goal:
                        return new_path
                    visited.add(next_tag)
                    queue.append((next_tag, new_path))

        return None


class TaskManager:
    """
    Task manager for generating waypoint sequences.
    Loads predefined tasks from YAML and supports Excel-based scan tasks.
    """

    def __init__(self, config_dict, tag_db, nav_graph):
        """
        Initialize task manager.

        Args:
            config_dict: Configuration dictionary from YAML
            tag_db: TagDatabase instance
            nav_graph: NavigationGraph instance
        """
        self.tag_db = tag_db
        self.nav_graph = nav_graph
        self.tasks = config_dict.get('tasks', {})

    def get_task_waypoints(self, task_name):
        """
        Get waypoints for a named task.

        Args:
            task_name: Task name (e.g., 'task1', 'task2')

        Returns:
            List of waypoint tag IDs
        """
        if task_name in self.tasks:
            return self.tasks[task_name]['waypoints']
        return []

    def get_excel_scan_waypoints(self, excel_path):
        """
        Generate waypoints from Excel file for scan mode.
        Excel has group_id which maps to tag_id = 100 + group_id.
        Allows duplicate group_ids (revisiting same tag).

        Args:
            excel_path: Path to Excel file

        Returns:
            Tuple of (waypoints, scan_tags)
        """
        try:
            import pandas as pd

            df = pd.read_excel(excel_path, usecols=['group_id'])

            # Extract group_ids in order, keeping duplicates based on when they change
            # e.g., [4,4,4,5,5,6,6,7,7,6,6] -> [4,5,6,7,6]
            group_order = []
            prev_gid = None
            for gid in df['group_id']:
                if gid != prev_gid:
                    group_order.append(gid)
                    prev_gid = gid

            # Convert to tag IDs (group_id 0 -> tag 100, etc.)
            scan_tags = [100 + int(gid) for gid in group_order]

            rospy.loginfo(f"[TASK MANAGER] Scan tags from Excel: {scan_tags}")

            # Build waypoints: 508 -> first_tag -> ... -> last_tag -> 508
            waypoints = [508]

            for i, tag in enumerate(scan_tags):
                # Get path from current position to next scan tag
                if i == 0:
                    path = self.nav_graph.find_path(508, tag)
                else:
                    path = self.nav_graph.find_path(scan_tags[i-1], tag)

                if path:
                    # Skip first element to avoid duplicates
                    waypoints.extend(path[1:])

            # Return to dock
            if scan_tags:
                return_path = self.nav_graph.find_path(scan_tags[-1], 508)
                if return_path:
                    waypoints.extend(return_path[1:])

            return waypoints, scan_tags

        except Exception as e:
            rospy.logerr(f"[TASK MANAGER] Failed to read Excel: {e}")
            return [508], []

    def get_path_to_tag(self, current, target):
        """Get path from current tag to target tag"""
        return self.nav_graph.find_path(current, target)


class MapManager:
    """
    Main map manager that loads configuration and provides access to all map data.
    This is the primary interface for accessing map information.
    """

    def __init__(self, config_path=None):
        """
        Initialize map manager from YAML configuration.

        Args:
            config_path: Path to YAML config file. If None, uses default package location.
        """
        if config_path is None:
            # Get default path from package
            rospack = rospkg.RosPack()
            pkg_path = rospack.get_path('apriltag_navigation')
            config_path = f"{pkg_path}/config/map.yaml"

        # Load configuration
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)

        # Initialize components
        self.tag_db = TagDatabase(self.config)
        self.nav_graph = NavigationGraph(self.config)
        self.task_manager = TaskManager(self.config, self.tag_db, self.nav_graph)

        # Store robot parameters
        self.robot_params = self.config.get('robot', {})

        rospy.loginfo(f"[MAP MANAGER] Loaded {len(self.tag_db.tags)} tags from {config_path}")

    def get_param(self, param_path, default=None):
        """
        Get robot parameter from configuration.

        Args:
            param_path: Dot-separated path (e.g., 'robot.length', 'speeds.linear')
            default: Default value if parameter not found

        Returns:
            Parameter value or default
        """
        keys = param_path.split('.')
        value = self.robot_params

        try:
            for key in keys:
                value = value[key]
            return value
        except (KeyError, TypeError):
            return default

    def validate_waypoints(self, waypoints):
        """
        Validate waypoint path for connectivity and tag existence.

        Args:
            waypoints: List of tag IDs representing the path

        Returns:
            Tuple of (is_valid, error_message)
        """
        if not waypoints:
            return False, "Empty waypoint list"

        if len(waypoints) < 2:
            return False, "Path must have at least 2 waypoints"

        # Check all tags exist
        for tag_id in waypoints:
            if not self.tag_db.get(tag_id):
                return False, f"Tag {tag_id} does not exist in map"

        # Check connectivity (edges exist between consecutive waypoints)
        for i in range(len(waypoints) - 1):
            current = waypoints[i]
            next_wp = waypoints[i + 1]

            edge = self.nav_graph.get_edge(current, next_wp)
            if not edge:
                return False, f"No edge from tag {current} to tag {next_wp}"

        return True, "Path is valid"
