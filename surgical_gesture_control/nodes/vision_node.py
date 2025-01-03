#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Point
from surgical_gesture_control.msg import GestureDetection
from builtin_interfaces.msg import Time
import cv2
import mediapipe as mp
import numpy as np
import time
import platform

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        # Publishers
        self.landmarks_pub = self.create_publisher(GestureDetection, 'hand_tracking', 10)
        
        # Initialize camera
        self.get_logger().info('Initializing camera...')
        self.setup_camera()
        
        # Initialize MediaPipe
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(
            model_complexity=1,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7
        )
        
        # Performance metrics
        self.frame_times = []
        self.detection_times = []
        self.max_metrics_samples = 100
        
        self.get_logger().info('Vision node initialized successfully')

    def setup_camera(self):
        camera_configs = [
            (0, cv2.CAP_DSHOW) if platform.system() == 'Windows' else (0, cv2.CAP_V4L2),
            (0, cv2.CAP_ANY)
        ]
        
        for idx, backend in camera_configs:
            self.cap = cv2.VideoCapture(idx, backend)
            if self.cap.isOpened():
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                self.cap.set(cv2.CAP_PROP_FPS, 30)
                return
                
        raise RuntimeError("Could not initialize camera")

    def process_frame(self, frame):
        start_time = time.time()
        
        # Convert to RGB for MediaPipe
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(frame_rgb)
        
        msg = GestureDetection()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.landmarks = []
        
        if results.multi_hand_landmarks:
            landmarks = results.multi_hand_landmarks[0]  # We only track one hand
            
            # Convert landmarks to ROS message
            for landmark in landmarks.landmark:
                point = Point()
                point.x = landmark.x
                point.y = landmark.y
                point.z = landmark.z
                msg.landmarks.append(point)
            
            # Draw landmarks for visualization
            self.mp_drawing.draw_landmarks(
                frame,
                landmarks,
                self.mp_hands.HAND_CONNECTIONS,
                self.mp_drawing.DrawingSpec(color=(0,255,0), thickness=2, circle_radius=2),
                self.mp_drawing.DrawingSpec(color=(0,0,255), thickness=2)
            )
            
            # Update performance metrics
            detection_time = time.time() - start_time
            self.detection_times.append(detection_time)
            if len(self.detection_times) > self.max_metrics_samples:
                self.detection_times.pop(0)
            
            msg.latency = np.mean(self.detection_times) * 1000  # Convert to ms
            
        return frame, msg

    def run(self):
        cv2.namedWindow('Hand Tracking', cv2.WINDOW_NORMAL)
        
        try:
            while rclpy.ok():
                frame_start = time.time()
                
                ret, frame = self.cap.read()
                if not ret:
                    self.get_logger().error('Failed to read frame')
                    break
                
                # Process frame and get landmarks
                frame, msg = self.process_frame(frame)
                
                # Publish landmarks
                self.landmarks_pub.publish(msg)
                
                # Update FPS calculation
                frame_time = time.time() - frame_start
                self.frame_times.append(frame_time)
                if len(self.frame_times) > self.max_metrics_samples:
                    self.frame_times.pop(0)
                
                # Display FPS
                fps = 1.0 / np.mean(self.frame_times)
                cv2.putText(frame, f'FPS: {fps:.1f}', (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                # Show frame
                cv2.imshow('Hand Tracking', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                
        except Exception as e:
            self.get_logger().error(f'Error in run loop: {str(e)}')
        finally:
            self.cleanup()

    def cleanup(self):
        self.get_logger().info('Cleaning up...')
        if hasattr(self, 'cap'):
            self.cap.release()
        cv2.destroyAllWindows()
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
