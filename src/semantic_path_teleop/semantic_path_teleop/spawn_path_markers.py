#!/usr/bin/env python3
"""Spawn semantic_path.json as visual markers directly in Gazebo."""

import json
from pathlib import Path
from typing import Dict, Iterable, Tuple

import rclpy
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from rclpy.node import Node

Pose2D = Tuple[float, float, float]


MATERIALS: Dict[str, str] = {
    'path': 'Gazebo/Yellow',
    'start': 'Gazebo/Green',
    'goal': 'Gazebo/Red',
}


def cylinder_sdf(name: str, radius: float, length: float, material: str) -> str:
    return f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{name}'>
    <static>true</static>
    <link name='link'>
      <visual name='visual'>
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{length}</length>
          </cylinder>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>{material}</name>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


class GazeboPathMarkerSpawner(Node):
    """Spawn colored cylinders in Gazebo at path waypoints."""

    def __init__(self) -> None:
        super().__init__('spawn_path_markers')
        self.declare_parameter('path_file', 'outputs/go2_ros_tracking_minimal/semantic_path.json')
        self.declare_parameter('stride', 3)
        self.declare_parameter('z', 0.025)
        self.declare_parameter('radius', 0.055)
        self.declare_parameter('height', 0.05)
        self.declare_parameter('namespace', 'semantic_path')
        self.declare_parameter('clear_existing', True)

        self.path_file = self.get_parameter('path_file').get_parameter_value().string_value
        self.stride = max(1, self.get_parameter('stride').get_parameter_value().integer_value)
        self.z = self.get_parameter('z').get_parameter_value().double_value
        self.radius = self.get_parameter('radius').get_parameter_value().double_value
        self.height = self.get_parameter('height').get_parameter_value().double_value
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value
        self.clear_existing = self.get_parameter('clear_existing').get_parameter_value().bool_value

        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')

    def run(self) -> None:
        poses = self._load_path(self.path_file)
        sampled = list(poses[::self.stride])
        if poses[-1] != sampled[-1]:
            sampled.append(poses[-1])

        self.get_logger().info('Waiting for Gazebo /spawn_entity service...')
        self.spawn_client.wait_for_service()
        if self.clear_existing:
            self.delete_client.wait_for_service(timeout_sec=2.0)

        if self.clear_existing:
            self._delete_previous_markers(len(poses) + 10)

        for index, pose in enumerate(sampled):
            marker_type = 'path'
            if index == 0:
                marker_type = 'start'
            elif index == len(sampled) - 1:
                marker_type = 'goal'

            self._spawn_marker(index, pose, marker_type)

        self.get_logger().info(
            f'Spawned {len(sampled)} Gazebo path markers from {self.path_file} '
            f'(stride={self.stride}).'
        )

    def _load_path(self, path_file: str) -> Iterable[Pose2D]:
        path = Path(path_file).expanduser()
        if not path.is_absolute():
            path = Path.cwd() / path

        data = json.loads(path.read_text())
        poses = []
        for item in data['poses']:
            pose = item['pose']
            position = pose['position']
            orientation = pose['orientation']
            poses.append((float(position['x']), float(position['y']), float(orientation['theta'])))
        return poses

    def _delete_previous_markers(self, max_count: int) -> None:
        for index in range(max_count):
            request = DeleteEntity.Request()
            request.name = f'{self.namespace}_{index:04d}'
            future = self.delete_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.1)

    def _spawn_marker(self, index: int, pose: Pose2D, marker_type: str) -> None:
        x, y, _ = pose
        name = f'{self.namespace}_{index:04d}'
        request = SpawnEntity.Request()
        request.name = name
        request.xml = cylinder_sdf(name, self.radius, self.height, MATERIALS[marker_type])
        request.robot_namespace = ''
        request.initial_pose.position.x = x
        request.initial_pose.position.y = y
        request.initial_pose.position.z = self.z

        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done() or not future.result().success:
            message = 'timeout' if not future.done() else future.result().status_message
            self.get_logger().warn(f'Failed to spawn marker {name}: {message}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GazeboPathMarkerSpawner()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
