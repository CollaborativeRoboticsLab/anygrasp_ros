from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # Get the package directory
    pkg_share = FindPackageShare('anygrasp_ros')
    config_path = PathJoinSubstitution([pkg_share, 'config', 'config.yaml'])

    # Declare launch arguments
    input_pointcloud_topic = LaunchConfiguration('input_pointcloud_topic')

    # RGBD node - handles RGB/depth to pointcloud conversion
    rgbd_node = Node(
        package='anygrasp_ros',
        executable='rgbd_to_pointcloud_node',
        name='rgbd_to_pointcloud_node',
        output='screen',
        parameters=[config_path],
    )

    # Tracking node - consumes pointcloud from RGBD node
    tracking_node = Node(
        package='anygrasp_ros',
        executable='anygrasp_tracking_node',
        name='anygrasp_tracking_node',
        output='screen',
        parameters=[config_path],
    )

    return LaunchDescription(
        [
            rgbd_node,
            tracking_node,
        ]
    )
