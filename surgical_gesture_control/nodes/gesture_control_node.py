#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Twist
import numpy as np

class GestureControlNode(Node):
    def __init__(self):
        super().__init__('gesture_control_node')
        
        # Subscribe to hand landmarks
        self.subscription = self.create_subscription(
            PoseArray,
            'hand_landmarks',
            self.landmarks_callback,
            10
        )
        
        # Publisher for robot commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        
        self.get_logger().info('Gesture control node initialized')

    def landmarks_callback(self, msg):
        if not msg.poses:
            return
            
        # Convert poses to numpy array for easier processing
        landmarks = np.array([[pose.position.x, pose.position.y, pose.position.z] 
                            for pose in msg.poses])
        
        # Detect gestures
        if self._is_open_palm(landmarks):
            self.get_logger().info('Open Palm detected - Moving Up')
            self._publish_command(0.0, 0.0, 0.2)  # Move up
            
        elif self._is_closed_fist(landmarks):
            self.get_logger().info('Closed Fist detected - Moving Down')
            self._publish_command(0.0, 0.0, -0.2)  # Move down
            
        elif self._is_pointing(landmarks):
            # Get pointing direction
            direction = self._get_pointing_direction(landmarks)
            self.get_logger().info(f'Pointing detected - Moving {direction}')
            if direction == 'right':
                self._publish_command(0.2, 0.0, 0.0)
            elif direction == 'left':
                self._publish_command(-0.2, 0.0, 0.0)
            elif direction == 'forward':
                self._publish_command(0.0, 0.2, 0.0)
            elif direction == 'backward':
                self._publish_command(0.0, -0.2, 0.0)

    def _is_open_palm(self, landmarks):
        # Check if fingers are extended
        finger_tips = [4, 8, 12, 16, 20]  # Thumb to pinky tips
        finger_bases = [2, 5, 9, 13, 17]  # Corresponding bases
        
        extended_fingers = 0
        for tip, base in zip(finger_tips, finger_bases):
            if landmarks[tip][1] < landmarks[base][1]:  # Y coordinate comparison
                extended_fingers += 1
        
        return extended_fingers >= 4

    def _is_closed_fist(self, landmarks):
        # Check if fingers are curled
        finger_tips = [4, 8, 12, 16, 20]
        palm_center = landmarks[0]  # Wrist landmark
        
        curled_fingers = 0
        for tip in finger_tips:
            dist = np.linalg.norm(landmarks[tip] - palm_center)
            if dist < 0.1:  # Distance threshold
                curled_fingers += 1
        
        return curled_fingers >= 4

    def _is_pointing(self, landmarks):
        # Check if index finger is extended while others are curled
        index_extended = landmarks[8][1] < landmarks[5][1]  # Index tip above base
        
        other_fingers_curled = True
        for tip in [12, 16, 20]:  # Middle, ring, pinky tips
            if landmarks[tip][1] < landmarks[tip-3][1]:  # Compare with base
                other_fingers_curled = False
                break
        
        return index_extended and other_fingers_curled

    def _get_pointing_direction(self, landmarks):
        # Get vector from index base to tip
        index_base = landmarks[5]
        index_tip = landmarks[8]
        direction = index_tip - index_base
        
        # Get angle in XY plane
        angle = np.arctan2(direction[1], direction[0])
        angle_deg = np.degrees(angle)
        
        # Classify direction based on angle
        if -45 <= angle_deg <= 45:
            return 'right'
        elif 45 < angle_deg <= 135:
            return 'backward'
        elif -135 <= angle_deg < -45:
            return 'forward'
        else:
            return 'left'

    def _publish_command(self, x, y, z):
        cmd = Twist()
        cmd.linear.x = float(x)
        cmd.linear.y = float(y)
        cmd.linear.z = float(z)
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = GestureControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
