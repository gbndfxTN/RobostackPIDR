import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("pathfinding_octomap")
    default_params = os.path.join(package_share, "config", "octomap_server.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    cloud_in_topic = LaunchConfiguration("cloud_in_topic")
    frame_id = LaunchConfiguration("frame_id")
    base_frame_id = LaunchConfiguration("base_frame_id")
    resolution = LaunchConfiguration("resolution")
    max_range = LaunchConfiguration("max_range")
    params_file = LaunchConfiguration("params_file")

    octomap_server = Node(
        package="octomap_server",
        executable="octomap_server_node",
        name="octomap_server",
        output="screen",
        remappings=[("cloud_in", cloud_in_topic)],
        parameters=[
            params_file,
            {
                "use_sim_time": use_sim_time,
                "frame_id": frame_id,
                "base_frame_id": base_frame_id,
                "resolution": resolution,
                "sensor_model.max_range": max_range,
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation time",
            ),
            DeclareLaunchArgument(
                "cloud_in_topic",
                default_value="/camera/depth/color/points",
                description="PointCloud2 topic from RGBD camera",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="base_link",
                description="Frame in which OctoMap is published",
            ),
            DeclareLaunchArgument(
                "base_frame_id",
                default_value="base_link",
                description="Robot base frame used by OctoMap",
            ),
            DeclareLaunchArgument(
                "resolution",
                default_value="0.05",
                description="OctoMap voxel resolution in meters",
            ),
            DeclareLaunchArgument(
                "max_range",
                default_value="6.0",
                description="Max sensor ray length in meters",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Path to OctoMap parameter file",
            ),
            octomap_server,
        ]
    )
