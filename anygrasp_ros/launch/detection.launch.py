from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # Get the package directory
    pkg_share = FindPackageShare('anygrasp_ros')
    config_path = PathJoinSubstitution([pkg_share, 'config', 'config.yaml'])
    rviz_config_path = PathJoinSubstitution([pkg_share, 'config', 'anygrasp.rviz'])

    use_rviz = LaunchConfiguration('use_rviz')

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

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='anygrasp_rviz',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('use_rviz', default_value='true'),
            rgbd_node,
            detection_node,
            rviz_node,
        ]
    )
