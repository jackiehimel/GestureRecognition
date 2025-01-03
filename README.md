# Surgical Gesture Control

A ROS2-based system for sterile surgical robot control using hand gestures.

## Overview

This project provides real-time gesture recognition for surgical robot control using MediaPipe Hands. The system demonstrates real-time gesture recognition and robot control visualization using RViz2.

## Prerequisites

- ROS 2 Humble or later
- Python 3.10 or later
- OpenCV and MediaPipe
- A webcam

## Installation

1. Install ROS 2 dependencies:
```bash
sudo apt-get update
sudo apt-get install ros-humble-desktop
```

2. Install Python dependencies:
```bash
pip install opencv-python mediapipe numpy
```

3. Build the package:
```bash
cd ~/GestureRecognition
colcon build
source install/setup.bash
```

## Running the System

Launch the complete system using:
```bash
ros2 launch surgical_gesture_control gesture_control.launch.py
```

This will start:
- Vision Node (camera input and hand tracking)
- Gesture Classifier Node (gesture recognition)
- Robot Node (visualization in RViz2)
- RViz2 with our custom configuration

## System Components

### Vision Node
Handles camera input and processes the video feed using MediaPipe Hands for real-time 3D hand landmark detection.

### Gesture Classifier Node
Analyzes hand landmark configurations to identify control gestures:
- NEUTRAL: Default pose
- GRASP: Pinching motion
- RELEASE: Opening hand
- MOVE_TO: Pointing gesture
- EMERGENCY_STOP: Open palm

### Robot Node
Provides visualization of the robot state in RViz2 and handles gesture-to-control mapping.

## Development

The system uses a modular ROS2 architecture for easy extension and modification. Key features:
- Kalman filtering for smooth motion
- Temporal gesture filtering for stability
- Real-time visualization
- Modular node structure

## Contributing

1. Fork the repository
2. Create your feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

**Aim**: leveraging readily available hardware and established frameworks such as MediaPipe Hands, this project explores an accessible approach to sterile surgical robot control. This system will demonstrate real-time gesture recognition and robot control visualization. We plan to initially validate the approach using a virtual robot in Rviz2

**Functionalities** (encapsulated by specific ROS nodes):
• imaging: Reports the raw camera input from the Mac’s built-in webcam. For simplicity, we assume a stationary camera at the origin in the world frame.
• hand tracking: Processes the raw camera feed using MediaPipe Hands for real-time 3D hand landmark detection and tracking. Reports the positions of 21 hand landmarks in the camera frame.
• gesture classifier: Analyzes hand landmark configurations and movements to identify specific control gestures. Implements confidence thresholds for reliable detection.
• motion filter: Applies Kalman filtering to smooth hand motion trajectories and reduce jitter in the tracking data.
• control mapping: Translates recognized gestures into robot control commands based on predefined mappings.
• robot control: Manages robot state and processes control commands. For the initial implementation, controls a virtual robot in RViz2.
• visualization: Renders the virtual robot, gesture recognition status, and system state in RViz2.
• system monitor: Tracks and reports system performance metrics including latency, recognition accuracy, and control precision.
• calibration (optional): Handles initial system calibration and coordinate frame alignments.

**Development**:
We will test this project with a progressive development approach, starting with basic gesture recognition and gradually increasing complexity. The development will proceed in phases: 1) All nodes downstream of hand tracking can be developed with pre-recorded gesture sequences, allowing rapid iteration of the gesture recognition and control mapping systems, and 2) The motion filter and control mapping nodes will require real-time testing to properly tune their parameters. This phase will focus on achieving stable, responsive control using the virtual robot.

**System validation** will focus on three key metrics:
1. Gesture recognition accuracy, targeting >95% for a core set of control gestures.
2. End-to-end system latency, aiming for <100ms response time.
3. Control precision, measured through virtual robot positioning error.
We will evaluate the system through a series of standardized tasks, measuring both quantitative performance metrics and gathering qualitative user feedback on the interface’s intuitiveness and reliability.

**Implementation**: The system implementation requires careful consideration of both hardware and software components to
achieve reliable real-time performance. For hardware, we will utilize a Mac computer with its built-in webcam as the primary input device, supplemented by a development platform with full ROS2 support. We may optionally add an external camera to improve the field of view or provide additional viewpoints for more robust tracking. The software stack builds on established frameworks, with ROS2 Humble serving as the core robotics middleware. We integrate this with MediaPipe Hands for hand tracking, OpenCV
2for auxiliary image processing tasks, and RViz2 for visualization. All components will be implemented in Python, leveraging its extensive library ecosystem and rapid prototyping capabilities. The development timeline is structured into four primary phases. The first phase focuses on foundational setup, including literature review completion, ROS2 environment configuration, and basic webcam integration testing. We will also implement and validate the MediaPipe Hands framework during this period. The second phase transitions to core functionality development, implementing the hand tracking node and basic gesture recognition systems, along with initial RViz2 visualization setup. The third phase emphasizes integration, developing the gesture-to-command mapping and implementing motion filtering, followed by comprehensive integration testing. The final phase is dedicated to system refinement, performance optimization, and preparation of documentation and demonstrations.

**Alternatives**:
There are certain technical challenges to consider; the primary concern is tracking reliability under varying conditions. Hand detection may become inconsistent under different lighting conditions or with complex
backgrounds. To address this, we will implement robust confidence thresholds and temporal filtering techniques. If these measures prove insufficient, we can fall back to a simplified gesture set that focuses on more distinctive and easily recognized movements.
System latency presents another challenge, as processing delays could severely impact real-time control performance. Our mitigation strategy includes frame rate optimization and implementation of predictive elements in the motion filter. If necessary, we can reduce processing resolution or simplify the gesture recognition pipeline to maintain responsive control. The integration of multiple ROS2 nodes also poses
potential challenges, particularly regarding communication delays. We plan to optimize QOS settings and message passing protocols to minimize latency. As a fallback option, we can consolidate critical nodes to reduce communication overhead. The modular architecture of our system provides significant flexibility in addressing these challenges. By maintaining clear interfaces between components, we can modify or replace
individual elements without disrupting the entire system. This adaptability ensures that we can rapidly iterate and optimize the system based on performance metrics and user feedback during development

**Requirements**:
1. Your project needs to be related to the content of the course. It needs to have a clinical value, but it
doesn’t need to be directly ready to implement in a clinical scenario. For example, you can use real-time
data to control even the Turtlebot, if you think you can explain its translation to clinical scenarios well.
2. It needs to work in real-time and all the data streams and controls should be done using ROS2. The
project must have a ROS2 component.
3. You should be able to physically demonstrate the end product, i.e. it cannot be just code/neural network
etc. resulting in graphs and numerical results. It can have a physical result, a real-time video s


Simplified Pipeline:
surgical_gesture_control/
├── package.xml
├── setup.py
├── setup.cfg
├── surgical_gesture_control/
│   ├── __init__.py
│   ├── nodes/
│   │   ├── vision_node.py          # Combined imaging + hand tracking
│   │   ├── gesture_control_node.py # Combined gesture recognition + control mapping
│   │   └── robot_node.py          # Combined robot control + visualization
│   ├── utils/
│   │   ├── simple_gestures.py     # Basic gesture definitions
│   │   └── smoothing.py           # Simple moving average filter
│   └── config/
│       ├── gesture_config.yaml    # Basic gesture parameters
│       └── rviz/
│           └── default.rviz       # Basic visualization config
├── launch/
│   └── basic_system.launch.py     # Single launch file
└── README.md