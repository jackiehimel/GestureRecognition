from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('surgical_gesture_control')
    rviz_config = os.path.join(pkg_dir, 'config', 'rviz', 'robot_visualization.rviz')
    
    return LaunchDescription([
        Node(
            package='surgical_gesture_control',
            executable='vision_node',
            name='vision_node',
            output='screen'
        ),
        Node(
            package='surgical_gesture_control',
            executable='gesture_classifier_node',
            name='gesture_node',
            output='screen'
        ),
        Node(
            package='surgical_gesture_control',
            executable='robot_node',
            name='robot_node',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])
