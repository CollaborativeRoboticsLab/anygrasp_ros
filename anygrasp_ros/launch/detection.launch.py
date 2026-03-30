from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # Get the package directory
    pkg_share = FindPackageShare('anygrasp_ros')
    config_path = PathJoinSubstitution([pkg_share, 'config', 'config.yaml'])

    # RGBD node - handles RGB/depth to pointcloud conversion
    rgbd_node = Node(
        package='anygrasp_ros',
        executable='rgbd_to_pointcloud_node',
        name='rgbd_to_pointcloud_node',
        output='screen',
        parameters=[config_path],
    )

    # Detection node - consumes pointcloud from RGBD node
    detection_node = Node(
        package='anygrasp_ros',
        executable='anygrasp_detection_node',
        name='anygrasp_detection_node',
        output='screen',
        parameters=[config_path],
    )

    return LaunchDescription(
        [
            rgbd_node,
            detection_node,
        ]
    )
