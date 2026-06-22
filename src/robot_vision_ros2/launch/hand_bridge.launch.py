from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('udp_ip', default_value='0.0.0.0'),
        DeclareLaunchArgument('udp_port', default_value='9090'),
        DeclareLaunchArgument('frame_id', default_value='camera_link'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('publish_markers', default_value='true'),
        Node(
            package='robot_vision_ros2',
            executable='hand_bridge',
            name='robot_vision_bridge',
            output='screen',
            parameters=[{
                'udp_ip': LaunchConfiguration('udp_ip'),
                'udp_port': LaunchConfiguration('udp_port'),
                'frame_id': LaunchConfiguration('frame_id'),
                'publish_tf': LaunchConfiguration('publish_tf'),
                'publish_markers': LaunchConfiguration('publish_markers'),
            }],
        ),
    ])
