#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import tf2_ros
import numpy as np

class RobotNode(Node):
    def __init__(self):
        super().__init__('robot_node')
        
        # Initialize subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Initialize publishers for visualization
        self.marker_pub = self.create_publisher(
            MarkerArray,
            'visualization_marker_array',
            10
        )
        
        # Initialize robot state
        self.position = np.zeros(3)
        self.orientation = 0.0
        
        # Create timer for visualization updates
        self.create_timer(0.1, self.visualization_callback)
        
        self.get_logger().info('Robot node initialized')

    def cmd_vel_callback(self, msg):
        # Update robot state based on velocity commands
        dt = 0.1  # Assume 10Hz update rate
        
        # Update position
        self.position[0] += msg.linear.x * dt
        self.position[1] += msg.linear.y * dt
        self.position[2] += msg.linear.z * dt
        
        # Update orientation
        self.orientation += msg.angular.z * dt

    def visualization_callback(self):
        marker_array = MarkerArray()
        
        # Create robot body marker
        body_marker = Marker()
        body_marker.header.frame_id = "map"
        body_marker.header.stamp = self.get_clock().now().to_msg()
        body_marker.ns = "robot"
        body_marker.id = 0
        body_marker.type = Marker.CUBE
        body_marker.action = Marker.ADD
        
        # Set position and orientation
        body_marker.pose.position.x = self.position[0]
        body_marker.pose.position.y = self.position[1]
        body_marker.pose.position.z = self.position[2]
        
        # Set orientation (simplified to just yaw)
        body_marker.pose.orientation.w = np.cos(self.orientation/2)
        body_marker.pose.orientation.z = np.sin(self.orientation/2)
        
        # Set scale and color
        body_marker.scale.x = 0.2
        body_marker.scale.y = 0.2
        body_marker.scale.z = 0.2
        body_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        
        marker_array.markers.append(body_marker)
        
        # Publish markers
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = RobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
