from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='surgical_gesture_control',
            executable='vision_node',
            name='vision_node',
            output='screen'
        ),
        Node(
            package='surgical_gesture_control',
            executable='gesture_control_node',
            name='gesture_control_node',
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
            arguments=['-d', 'install/surgical_gesture_control/share/surgical_gesture_control/config/rviz/default.rviz']
        )
    ])
