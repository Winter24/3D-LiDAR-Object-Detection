from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_detection',
            executable='ransac',
            name='ransac'
        ),
        Node(
            package='lidar_detection',
            executable='object_detection.py',  # Corrected the typo here
            name='object_detection',
            output='screen'
        )
    ])
