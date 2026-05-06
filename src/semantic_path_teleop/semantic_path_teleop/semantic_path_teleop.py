#!/usr/bin/env python3
"""Follow semantic_path.json with Gazebo odometry feedback and cmd_vel teleop."""

import json
import math
from pathlib import Path
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

Pose2D = Tuple[float, float, float]


def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Return yaw from a quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class SemanticPathTeleop(Node):
    """Odometry-based lookahead controller for a holonomic quadruped cmd_vel interface."""

    def __init__(self) -> None:
        super().__init__('semantic_path_teleop')

        self.declare_parameter('path_file', 'outputs/go2_ros_tracking_minimal/semantic_path.json')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom/ground_truth')
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('linear_speed', 0.28)
        self.declare_parameter('max_lateral_speed', 0.22)
        self.declare_parameter('max_angular_speed', 1.4)
        self.declare_parameter('position_gain', 0.9)
        self.declare_parameter('heading_gain', 2.8)
        self.declare_parameter('lookahead_points', 4)
        self.declare_parameter('closest_search_window', 25)
        self.declare_parameter('waypoint_tolerance', 0.16)
        self.declare_parameter('goal_tolerance', 0.25)
        self.declare_parameter('use_relative_start', False)
        self.declare_parameter('dry_run', False)
        self.declare_parameter('loop', False)

        path_file = self.get_parameter('path_file').get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.rate_hz = self.get_parameter('rate_hz').get_parameter_value().double_value
        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        self.max_lateral_speed = self.get_parameter('max_lateral_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        self.position_gain = self.get_parameter('position_gain').get_parameter_value().double_value
        self.heading_gain = self.get_parameter('heading_gain').get_parameter_value().double_value
        self.lookahead_points = self.get_parameter('lookahead_points').get_parameter_value().integer_value
        self.closest_search_window = self.get_parameter('closest_search_window').get_parameter_value().integer_value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').get_parameter_value().double_value
        self.goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value
        self.use_relative_start = self.get_parameter('use_relative_start').get_parameter_value().bool_value
        self.dry_run = self.get_parameter('dry_run').get_parameter_value().bool_value
        self.loop = self.get_parameter('loop').get_parameter_value().bool_value

        self.poses = self._load_path(path_file)
        self.current_index = 0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.odom_origin = None
        self.path_origin = self.poses[0]
        self.last_odom = None

        self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.odom_subscription = self.create_subscription(Odometry, self.odom_topic, self._on_odom, 10)
        self.timer = self.create_timer(1.0 / self.rate_hz, self._on_timer)

        mode = 'DRY-RUN' if self.dry_run else 'PUBLISH'
        frame_mode = 'relative start' if self.use_relative_start else 'absolute odom/world coordinates'
        self.get_logger().info(
            f'{mode}: {len(self.poses)} poses loaded from {path_file}; publishing to '
            f'{self.cmd_vel_topic}; odom={self.odom_topic}; tracking={frame_mode}'
        )

    def _load_path(self, path_file: str) -> List[Pose2D]:
        path = Path(path_file).expanduser()
        if not path.is_absolute():
            path = Path.cwd() / path
        if not path.exists():
            raise FileNotFoundError(f'Path JSON not found: {path}')

        with path.open('r', encoding='utf-8') as stream:
            data = json.load(stream)

        poses = []
        for item in data.get('poses', []):
            pose = item.get('pose', {})
            position = pose.get('position', {})
            orientation = pose.get('orientation', {})
            poses.append((float(position.get('x', 0.0)), float(position.get('y', 0.0)), float(orientation.get('theta', 0.0))))

        if not poses:
            raise ValueError(f'No poses found in JSON file: {path}')
        return poses

    def _on_odom(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        odom_pose = (
            position.x,
            position.y,
            yaw_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w),
        )

        if self.odom_origin is None:
            self.odom_origin = odom_pose
            self.get_logger().info('Odometry locked.')

        self.last_odom = odom_pose
        if self.use_relative_start:
            self.robot_x, self.robot_y, self.robot_yaw = self._odom_to_path_frame(odom_pose)
        else:
            self.robot_x, self.robot_y, self.robot_yaw = odom_pose

    def _odom_to_path_frame(self, odom_pose: Pose2D) -> Pose2D:
        odom_x, odom_y, odom_yaw = odom_pose
        odom_origin_x, odom_origin_y, odom_origin_yaw = self.odom_origin
        path_origin_x, path_origin_y, path_origin_yaw = self.path_origin

        dx = odom_x - odom_origin_x
        dy = odom_y - odom_origin_y
        rotation = path_origin_yaw - odom_origin_yaw
        cos_r = math.cos(rotation)
        sin_r = math.sin(rotation)
        path_x = path_origin_x + cos_r * dx - sin_r * dy
        path_y = path_origin_y + sin_r * dx + cos_r * dy
        path_yaw = normalize_angle(path_origin_yaw + normalize_angle(odom_yaw - odom_origin_yaw))
        return path_x, path_y, path_yaw

    def _on_timer(self) -> None:
        if self.last_odom is None:
            self.get_logger().info('Waiting for odometry before publishing velocity commands...', throttle_duration_sec=2.0)
            self._publish_stop()
            return

        if self._goal_reached():
            if self.loop:
                self.current_index = 0
            else:
                self._publish_stop()
                self.get_logger().info('Semantic path goal reached; robot stopped.')
                self.timer.cancel()
                return

        self._refresh_current_index()
        target_index = min(self.current_index + self.lookahead_points, len(self.poses) - 1)
        target_x, target_y, target_yaw = self.poses[target_index]

        error_x_world = target_x - self.robot_x
        error_y_world = target_y - self.robot_y
        cos_yaw = math.cos(self.robot_yaw)
        sin_yaw = math.sin(self.robot_yaw)

        # Convert world-frame position error into robot body-frame cmd_vel.
        error_x_body = cos_yaw * error_x_world + sin_yaw * error_y_world
        error_y_body = -sin_yaw * error_x_world + cos_yaw * error_y_world

        desired_heading = math.atan2(error_y_world, error_x_world)
        path_heading_error = normalize_angle(desired_heading - self.robot_yaw)
        final_heading_error = normalize_angle(target_yaw - self.robot_yaw)
        heading_error = path_heading_error if abs(path_heading_error) > 0.25 else final_heading_error

        linear_x = self._clamp(self.position_gain * error_x_body, -self.linear_speed, self.linear_speed)
        linear_y = self._clamp(self.position_gain * error_y_body, -self.max_lateral_speed, self.max_lateral_speed)
        angular_z = self._clamp(self.heading_gain * heading_error, -self.max_angular_speed, self.max_angular_speed)

        if abs(path_heading_error) > 1.35:
            linear_x *= 0.25
            linear_y *= 0.25

        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.angular.z = angular_z

        if not self.dry_run:
            self.publisher.publish(msg)

    def _refresh_current_index(self) -> None:
        start = max(0, self.current_index - 3)
        stop = min(len(self.poses), self.current_index + self.closest_search_window)
        closest_index = min(
            range(start, stop),
            key=lambda i: math.hypot(self.poses[i][0] - self.robot_x, self.poses[i][1] - self.robot_y),
        )
        self.current_index = max(self.current_index, closest_index)

        while self.current_index < len(self.poses) - 1:
            x, y, _ = self.poses[self.current_index]
            if math.hypot(x - self.robot_x, y - self.robot_y) > self.waypoint_tolerance:
                break
            self.current_index += 1

    def _goal_reached(self) -> bool:
        goal_x, goal_y, _ = self.poses[-1]
        return math.hypot(goal_x - self.robot_x, goal_y - self.robot_y) <= self.goal_tolerance

    def _publish_stop(self) -> None:
        if not self.dry_run:
            self.publisher.publish(Twist())

    @staticmethod
    def _clamp(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(maximum, value))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SemanticPathTeleop()
    try:
        rclpy.spin(node)
    finally:
        node._publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
