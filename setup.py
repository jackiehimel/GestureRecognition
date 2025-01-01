from setuptools import setup, find_packages

package_name = 'surgical_gesture_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/gesture_config.yaml']),
        ('share/' + package_name + '/config/rviz', ['config/rviz/default.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='ROS 2 package for surgical robot control using hand gestures',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = surgical_gesture_control.nodes.vision_node:main',
            'gesture_control_node = surgical_gesture_control.nodes.gesture_control_node:main',
            'robot_node = surgical_gesture_control.nodes.robot_node:main',
        ],
    },
)
