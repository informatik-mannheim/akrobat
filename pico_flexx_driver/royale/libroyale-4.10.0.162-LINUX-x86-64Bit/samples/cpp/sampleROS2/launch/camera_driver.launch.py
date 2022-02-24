from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            namespace='',
            executable='static_transform_publisher',
            arguments=['0','0', '0', '1.57','0','-1.57','100', 'RoyaleInRos_link', 'RoyaleInRos_optical_frame']
        ),
        Node(
            package='royale_in_ros2',
            namespace='',
            executable='royale_in_ros2',
            name='RoyaleInRos',
            output='screen'
        )
    ])
