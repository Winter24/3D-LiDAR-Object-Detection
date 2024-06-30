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
            executable='DBSCAN.py',  
            name='object_detection',
            output='screen'
        )
    ])
