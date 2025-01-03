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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='himel',
    maintainer_email='your.email@example.com',
    description='Surgical robot control using hand gestures',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gesture_classifier_node = surgical_gesture_control.nodes.gesture_classifier_node:main',
            'ros_bridge_node = surgical_gesture_control.nodes.ros_bridge_node:main',
        ],
    },
)
