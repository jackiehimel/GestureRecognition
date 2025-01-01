#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import mediapipe as mp
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
import sys

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        self.get_logger().info('Starting vision node initialization...')
        
        # Initialize publishers
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 10)
        self.hand_landmarks_pub = self.create_publisher(PoseArray, 'hand_landmarks', 10)
        
        self.use_test_pattern = False
        try:
            # Try to initialize camera
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                raise Exception("Camera failed to open")
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            # Try to read one frame to verify camera works
            ret, _ = self.cap.read()
            if not ret:
                raise Exception("Failed to read initial frame")
        except Exception as e:
            self.get_logger().warn(f'Camera initialization failed: {str(e)}. Using test pattern')
            self.use_test_pattern = True
            if hasattr(self, 'cap') and self.cap is not None:
                self.cap.release()
        
        self.bridge = CvBridge()
        
        # Initialize MediaPipe Hands
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        
        # Create timer for camera capture (30 FPS)
        self.create_timer(1/30, self.camera_callback)
        self.frame_count = 0
        self.get_logger().info('Vision node initialized successfully')

    def generate_test_pattern(self):
        # Create a moving test pattern
        height, width = 480, 640
        # Create a gray background instead of black
        image = np.ones((height, width, 3), dtype=np.uint8) * 128
        
        # Create a moving circle
        center_x = int(width/2 + width/4 * np.sin(self.frame_count * 0.05))
        center_y = int(height/2 + height/4 * np.cos(self.frame_count * 0.05))
        
        # Draw circle and cross with thicker lines
        cv2.circle(image, (center_x, center_y), 40, (0, 255, 0), -1)  # Bigger green circle
        cv2.line(image, (center_x-50, center_y), (center_x+50, center_y), (255, 0, 0), 4)  # Thicker blue lines
        cv2.line(image, (center_x, center_y-50), (center_x, center_y+50), (255, 0, 0), 4)
        
        # Add more visible text
        cv2.putText(image, f"Test Pattern - Frame {self.frame_count}", (50, 50), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)  # Red text, thicker
        
        # Add a rectangle border
        cv2.rectangle(image, (0, 0), (width-1, height-1), (255, 255, 255), 2)
        
        self.frame_count += 1
        return image

    def camera_callback(self):
        try:
            if self.use_test_pattern:
                frame = self.generate_test_pattern()
                self.get_logger().debug('Generated test pattern frame')
            else:
                try:
                    ret, frame = self.cap.read()
                    if not ret:
                        self.get_logger().error('Failed to capture frame, switching to test pattern')
                        self.use_test_pattern = True
                        frame = self.generate_test_pattern()
                except Exception as e:
                    self.get_logger().error(f'Camera read error: {str(e)}, switching to test pattern')
                    self.use_test_pattern = True
                    frame = self.generate_test_pattern()

            # Display the frame directly using cv2
            cv2.imshow('Test Pattern', frame)
            cv2.waitKey(1)  # Required for cv2.imshow to work

            # Publish camera image
            try:
                img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.image_pub.publish(img_msg)
                if self.frame_count % 30 == 0:  # Log every 30 frames
                    self.get_logger().debug('Published image frame')
            except Exception as e:
                self.get_logger().error(f'Failed to publish image: {str(e)}')

            # Only process MediaPipe if we have a valid frame
            if frame is not None:
                # Convert BGR to RGB for MediaPipe
                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                
                # Process frame with MediaPipe
                results = self.hands.process(rgb_frame)
                
                # Draw hand landmarks on frame
                if results.multi_hand_landmarks:
                    self.get_logger().info('Hand detected!')
                    for hand_landmarks in results.multi_hand_landmarks:
                        mp.solutions.drawing_utils.draw_landmarks(
                            frame,
                            hand_landmarks,
                            self.mp_hands.HAND_CONNECTIONS
                        )
                        
                        # Create and publish hand landmarks
                        pose_array = PoseArray()
                        pose_array.header.frame_id = "camera_frame"
                        pose_array.header.stamp = self.get_clock().now().to_msg()
                        
                        for landmark in hand_landmarks.landmark:
                            pose = Pose()
                            pose.position.x = landmark.x
                            pose.position.y = landmark.y
                            pose.position.z = landmark.z
                            pose_array.poses.append(pose)
                        
                        self.hand_landmarks_pub.publish(pose_array)

        except Exception as e:
            self.get_logger().error(f'Error in camera callback: {str(e)}')
            # Ensure we switch to test pattern on any error
            self.use_test_pattern = True

    def __del__(self):
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = VisionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {str(e)}')
    finally:
        cv2.destroyAllWindows()
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
