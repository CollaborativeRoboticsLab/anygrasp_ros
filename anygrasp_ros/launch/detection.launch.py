from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    rgb_topic = LaunchConfiguration('rgb_topic')
    depth_topic = LaunchConfiguration('depth_topic')

    detection_checkpoint = LaunchConfiguration('detection_checkpoint')

    fx = LaunchConfiguration('fx')
    fy = LaunchConfiguration('fy')
    cx = LaunchConfiguration('cx')
    cy = LaunchConfiguration('cy')
    depth_scale = LaunchConfiguration('depth_scale')

    detection_node = Node(
        package='anygrasp_ros',
        executable='anygrasp_detection_node',
        name='anygrasp_detection_node',
        output='screen',
        parameters=[
            {
                'checkpoint_path': detection_checkpoint,
                'fx': fx,
                'fy': fy,
                'cx': cx,
                'cy': cy,
                'depth_scale': depth_scale,
            }
        ],
        remappings=[
            ('rgb_image', rgb_topic),
            ('depth_image', depth_topic),
            ('detection', '/anygrasp/detection'),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('rgb_topic', default_value='/rgb_image'),
            DeclareLaunchArgument('depth_topic', default_value='/depth_image'),
            DeclareLaunchArgument('detection_checkpoint', default_value=''),
            DeclareLaunchArgument('fx', default_value='927.17'),
            DeclareLaunchArgument('fy', default_value='927.37'),
            DeclareLaunchArgument('cx', default_value='651.32'),
            DeclareLaunchArgument('cy', default_value='349.62'),
            DeclareLaunchArgument('depth_scale', default_value='1000.0'),
            detection_node,
        ]
    )
