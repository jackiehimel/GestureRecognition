#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Point
from surgical_gesture_control.msg import GestureDetection
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
import tf2_ros
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from surgical_gesture_control.utils.motion_filter import KalmanFilter
import time

class RobotVisualizerNode(Node):
    def __init__(self):
        super().__init__('robot_visualizer')
        
        # QoS profile for better real-time performance
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        
        # Initialize motion filter
        self.motion_filter = KalmanFilter(process_variance=0.1, measurement_variance=0.1)
        
        # Subscribers
        self.gesture_sub = self.create_subscription(
            GestureDetection,
            'gesture_detection',
            self.gesture_callback,
            qos_profile
        )
        
        # Publishers for visualization
        self.marker_pub = self.create_publisher(
            MarkerArray,
            'robot_visualization',
            10
        )
        
        # TF broadcaster for coordinate frames
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Initialize robot state
        self.robot_position = Point()
        self.robot_position.z = 0.5  # Start 0.5m above ground
        self.gripper_state = False  # False = open, True = closed
        self.target_position = Point()
        
        # Performance metrics
        self.last_command_time = time.time()
        self.command_latency = []
        self.position_error = []
        self.gesture_confidence = []
        
        # Timer for visualization updates
        self.create_timer(0.1, self.publish_visualization)  # 10Hz updates
        
        self.get_logger().info('Robot visualizer node initialized')
    
    def gesture_callback(self, msg):
        """Handle incoming gesture commands and update robot state"""
        # Update performance metrics
        current_time = time.time()
        self.command_latency.append(current_time - self.last_command_time)
        self.last_command_time = current_time
        self.gesture_confidence.append(msg.confidence)
        
        # Process gesture command
        gesture = msg.gesture
        if gesture == "grasp" and msg.confidence > 0.7:
            self.gripper_state = True
            self.get_logger().info(f'Gripper closed (confidence: {msg.confidence:.2f})')
        elif gesture == "release" and msg.confidence > 0.7:
            self.gripper_state = False
            self.get_logger().info(f'Gripper opened (confidence: {msg.confidence:.2f})')
        elif gesture == "move_to" and msg.confidence > 0.7:
            # Update target position from hand landmarks
            if msg.landmarks:
                # Use index fingertip (landmark 8) for pointing
                self.target_position.x = msg.landmarks[8].x
                self.target_position.y = msg.landmarks[8].y
                self.target_position.z = msg.landmarks[8].z
                
                # Calculate position error
                error = np.sqrt(
                    (self.target_position.x - self.robot_position.x)**2 +
                    (self.target_position.y - self.robot_position.y)**2 +
                    (self.target_position.z - self.robot_position.z)**2
                )
                self.position_error.append(error)
                
                # Apply motion filter and update robot position
                filtered_position = self.motion_filter.update(self.target_position)
                self.robot_position.x = filtered_position.x
                self.robot_position.y = filtered_position.y
                self.robot_position.z = filtered_position.z
                
                self.get_logger().info(
                    f'Moving to target (x:{self.target_position.x:.2f}, '
                    f'y:{self.target_position.y:.2f}, z:{self.target_position.z:.2f}) '
                    f'with confidence {msg.confidence:.2f}'
                )
        elif gesture == "stop" and msg.confidence > 0.8:
            # Emergency stop - freeze current position
            self.get_logger().warn('Emergency stop activated!')
            
        # Log performance metrics
        if len(self.command_latency) > 100:
            avg_latency = np.mean(self.command_latency[-100:]) * 1000  # Convert to ms
            avg_confidence = np.mean(self.gesture_confidence[-100:])
            avg_error = np.mean(self.position_error[-100:]) if self.position_error else 0
            
            self.get_logger().debug(
                f'Performance Metrics:\n'
                f'  Command Latency: {avg_latency:.1f}ms\n'
                f'  Gesture Confidence: {avg_confidence:.2f}\n'
                f'  Position Error: {avg_error:.3f}m'
            )
    
    def publish_visualization(self):
        """Publish visualization markers for RViz"""
        marker_array = MarkerArray()
        
        # Robot base marker
        base_marker = Marker()
        base_marker.header.frame_id = "camera_frame"
        base_marker.header.stamp = self.get_clock().now().to_msg()
        base_marker.id = 0
        base_marker.type = Marker.CYLINDER
        base_marker.action = Marker.ADD
        base_marker.pose.position = self.robot_position
        base_marker.scale.x = 0.1
        base_marker.scale.y = 0.1
        base_marker.scale.z = 0.2
        base_marker.color.r = 0.2
        base_marker.color.g = 0.2
        base_marker.color.b = 0.8
        base_marker.color.a = 1.0
        marker_array.markers.append(base_marker)
        
        # Gripper marker
        gripper_marker = Marker()
        gripper_marker.header.frame_id = "camera_frame"
        gripper_marker.header.stamp = self.get_clock().now().to_msg()
        gripper_marker.id = 1
        gripper_marker.type = Marker.SPHERE
        gripper_marker.action = Marker.ADD
        gripper_marker.pose.position = self.robot_position
        gripper_marker.pose.position.z += 0.15  # Offset above base
        gripper_marker.scale.x = 0.05 if self.gripper_state else 0.08
        gripper_marker.scale.y = 0.05 if self.gripper_state else 0.08
        gripper_marker.scale.z = 0.05 if self.gripper_state else 0.08
        gripper_marker.color.r = 0.8
        gripper_marker.color.g = 0.2
        gripper_marker.color.b = 0.2
        gripper_marker.color.a = 1.0
        marker_array.markers.append(gripper_marker)
        
        # Target position marker (when moving)
        if self.target_position != self.robot_position:
            target_marker = Marker()
            target_marker.header.frame_id = "camera_frame"
            target_marker.header.stamp = self.get_clock().now().to_msg()
            target_marker.id = 2
            target_marker.type = Marker.SPHERE
            target_marker.action = Marker.ADD
            target_marker.pose.position = self.target_position
            target_marker.scale.x = 0.05
            target_marker.scale.y = 0.05
            target_marker.scale.z = 0.05
            target_marker.color.r = 0.2
            target_marker.color.g = 0.8
            target_marker.color.b = 0.2
            target_marker.color.a = 0.6
            marker_array.markers.append(target_marker)
        
        # Publish markers
        self.marker_pub.publish(marker_array)
        
        # Broadcast transform
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'camera_frame'
        transform.child_frame_id = 'robot_base'
        transform.transform.translation.x = self.robot_position.x
        transform.transform.translation.y = self.robot_position.y
        transform.transform.translation.z = self.robot_position.z
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = RobotVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
