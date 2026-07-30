from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description() -> LaunchDescription:
    # Get the package directory
    pkg_share = FindPackageShare('anygrasp_ros')
    config_path = PathJoinSubstitution([pkg_share, 'config', 'config.yaml'])
    rviz_config_path = PathJoinSubstitution([pkg_share, 'config', 'anygrasp.rviz'])

    use_rviz = LaunchConfiguration('use_rviz')
    rgb_image_topic = LaunchConfiguration('rgb_image_topic')
    depth_image_topic = LaunchConfiguration('depth_image_topic')
    color_camera_info_topic_name = LaunchConfiguration('color_camera_info_topic_name')
    depth_camera_info_topic_name = LaunchConfiguration('depth_camera_info_topic_name')

    # RGBD node - handles RGB/depth to pointcloud conversion
    rgbd_node = Node(
        package='anygrasp_ros',
        executable='rgbd_to_pointcloud_node',
        name='rgbd_to_pointcloud_node',
        output='screen',
        parameters=[
            config_path,
            {
                'rgb_image_topic': rgb_image_topic,
                'depth_image_topic': depth_image_topic,
                'color_camera_info_topic_name': color_camera_info_topic_name,
                'depth_camera_info_topic_name': depth_camera_info_topic_name,
            },
        ],
    )

    # Tracking node - consumes pointcloud from RGBD node
    tracking_node = Node(
        package='anygrasp_ros',
        executable='anygrasp_tracking_node',
        name='anygrasp_tracking_node',
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
            DeclareLaunchArgument('rgb_image_topic', default_value='/camera/color/image_raw'),
            DeclareLaunchArgument('depth_image_topic', default_value='/camera/depth/image_rect_raw'),
            DeclareLaunchArgument(
                'color_camera_info_topic_name', default_value='/camera/color/camera_info'
            ),
            DeclareLaunchArgument(
                'depth_camera_info_topic_name', default_value='/camera/depth/camera_info'
            ),
            rgbd_node,
            tracking_node,
            rviz_node,
        ]
    )
