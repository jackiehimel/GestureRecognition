#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Point, Pose
from surgical_gesture_control.msg import GestureDetection
import numpy as np
from enum import Enum
from collections import deque
import time

class SurgicalGesture(Enum):
    """Supported surgical control gestures"""
    NEUTRAL = "neutral"           # Default pose, palm facing forward
    GRASP = "grasp"              # Pinching motion for grasping
    RELEASE = "release"          # Opening hand from grasp
    MOVE_TO = "move_to"          # Point with index finger
    EMERGENCY_STOP = "stop"      # Open palm, fingers spread

class GestureClassifierNode(Node):
    def __init__(self):
        super().__init__('gesture_classifier')
        
        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('confidence_threshold', 0.7),
                ('temporal_window', 5),
                ('min_gesture_duration', 0.1),
                ('gesture_smoothing', True)
            ]
        )
        
        # Landmark indices for key points
        self.WRIST = 0
        self.THUMB_TIP = 4
        self.INDEX_TIP = 8
        self.INDEX_BASE = 5
        
        # Subscribers and Publishers
        self.landmarks_sub = self.create_subscription(
            GestureDetection,
            'hand_tracking',
            self.landmarks_callback,
            10
        )
        
        self.gesture_pub = self.create_publisher(
            GestureDetection,
            'gesture_detection',
            10
        )
        
        # Performance tracking
        self.gesture_history = deque(maxlen=self.get_parameter('temporal_window').value)
        self.last_gesture = SurgicalGesture.NEUTRAL
        self.gesture_start_time = time.time()
        self.gesture_counts = {gesture: 0 for gesture in SurgicalGesture}
        self.total_detections = 0
        
        # Metrics
        self.processing_times = deque(maxlen=100)
        self.confidence_scores = deque(maxlen=100)
        
        self.get_logger().info('Gesture classifier initialized')
    
    def landmarks_callback(self, msg):
        """Process incoming hand landmarks and detect gestures"""
        start_time = time.time()
        
        if not msg.landmarks:
            return
            
        # Convert landmarks to numpy array
        landmarks = np.array([[p.x, p.y, p.z] for p in msg.landmarks])
        
        # Detect gesture and calculate confidence
        gesture, confidence = self._classify_gesture(landmarks)
        
        # Update metrics
        processing_time = time.time() - start_time
        self.processing_times.append(processing_time)
        self.confidence_scores.append(confidence)
        
        # Apply temporal smoothing if enabled
        if self.get_parameter('gesture_smoothing').value:
            self.gesture_history.append(gesture)
            gesture = self._get_majority_gesture()
        
        # Update gesture statistics
        self.total_detections += 1
        self.gesture_counts[gesture] += 1
        
        # Create and publish gesture detection message
        detection_msg = GestureDetection()
        detection_msg.gesture = gesture.value
        detection_msg.confidence = confidence
        detection_msg.timestamp = self.get_clock().now().to_msg()
        detection_msg.landmarks = msg.landmarks
        
        # Add performance metrics
        detection_msg.latency = np.mean(self.processing_times) * 1000  # Convert to ms
        detection_msg.recognition_accuracy = self.gesture_counts[gesture] / self.total_detections
        
        # Calculate control precision based on landmark stability
        if len(msg.landmarks) >= 21:
            stability = self._calculate_stability(landmarks)
            detection_msg.control_precision = stability
        
        self.gesture_pub.publish(detection_msg)
        self.last_gesture = gesture
    
    def _calculate_stability(self, landmarks):
        """Calculate stability metric based on hand movement"""
        if len(self.gesture_history) < 2:
            return 1.0
            
        # Calculate movement of key points between frames
        wrist_movement = np.linalg.norm(landmarks[self.WRIST] - landmarks[self.INDEX_BASE])
        index_movement = np.linalg.norm(landmarks[self.INDEX_TIP] - landmarks[self.INDEX_BASE])
        
        # Lower values indicate more stable hand position
        stability = 1.0 - min(1.0, (wrist_movement + index_movement) / 2.0)
        return stability
    
    def _classify_gesture(self, landmarks):
        """Classify gesture based on hand landmark positions and calculate confidence"""
        confidence_threshold = self.get_parameter('confidence_threshold').value
        
        # Check each gesture with confidence calculation
        if self._is_emergency_stop(landmarks):
            confidence = self._calculate_emergency_confidence(landmarks)
            return (SurgicalGesture.EMERGENCY_STOP, confidence) if confidence > confidence_threshold else (SurgicalGesture.NEUTRAL, 0.0)
            
        elif self._is_grasp(landmarks):
            confidence = self._calculate_grasp_confidence(landmarks)
            return (SurgicalGesture.GRASP, confidence) if confidence > confidence_threshold else (SurgicalGesture.NEUTRAL, 0.0)
            
        elif self._is_release(landmarks):
            confidence = self._calculate_release_confidence(landmarks)
            return (SurgicalGesture.RELEASE, confidence) if confidence > confidence_threshold else (SurgicalGesture.NEUTRAL, 0.0)
            
        elif self._is_move_to(landmarks):
            confidence = self._calculate_move_confidence(landmarks)
            return (SurgicalGesture.MOVE_TO, confidence) if confidence > confidence_threshold else (SurgicalGesture.NEUTRAL, 0.0)
            
        return (SurgicalGesture.NEUTRAL, 1.0)
    
    def _is_emergency_stop(self, landmarks):
        """Check for open palm with spread fingers"""
        finger_tips = [4, 8, 12, 16, 20]  # Thumb to pinky tips
        finger_bases = [2, 5, 9, 13, 17]  # Corresponding bases
        
        extended_fingers = 0
        for tip, base in zip(finger_tips, finger_bases):
            if landmarks[tip][1] < landmarks[base][1]:  # Y coordinate comparison
                extended_fingers += 1
        
        # Check finger spread
        spread = np.mean([
            np.linalg.norm(landmarks[8] - landmarks[4]),   # thumb to index
            np.linalg.norm(landmarks[8] - landmarks[12]),  # index to middle
            np.linalg.norm(landmarks[12] - landmarks[16]), # middle to ring
            np.linalg.norm(landmarks[16] - landmarks[20])  # ring to pinky
        ])
        
        return extended_fingers >= 4 and spread > 0.15
    
    def _calculate_emergency_confidence(self, landmarks):
        """Calculate confidence for emergency stop gesture"""
        finger_tips = [4, 8, 12, 16, 20]
        finger_bases = [2, 5, 9, 13, 17]
        
        # Calculate finger extension ratios
        extension_ratios = []
        for tip, base in zip(finger_tips, finger_bases):
            extension = landmarks[base][1] - landmarks[tip][1]
            total_length = np.linalg.norm(landmarks[tip] - landmarks[base])
            if total_length > 0:
                extension_ratios.append(extension / total_length)
        
        # Calculate spread confidence
        spread_distances = [
            np.linalg.norm(landmarks[8] - landmarks[4]),
            np.linalg.norm(landmarks[8] - landmarks[12]),
            np.linalg.norm(landmarks[12] - landmarks[16]),
            np.linalg.norm(landmarks[16] - landmarks[20])
        ]
        spread_confidence = min(1.0, np.mean(spread_distances) / 0.2)
        
        return min(1.0, (np.mean(extension_ratios) + spread_confidence) / 2)
    
    def _is_grasp(self, landmarks):
        """Check for pinching gesture"""
        thumb_tip = landmarks[self.THUMB_TIP]
        index_tip = landmarks[self.INDEX_TIP]
        
        distance = np.linalg.norm(thumb_tip - index_tip)
        return distance < 0.05
    
    def _calculate_grasp_confidence(self, landmarks):
        """Calculate confidence for grasp gesture"""
        thumb_tip = landmarks[self.THUMB_TIP]
        index_tip = landmarks[self.INDEX_TIP]
        
        # Distance-based confidence
        distance = np.linalg.norm(thumb_tip - index_tip)
        distance_confidence = max(0.0, 1.0 - (distance / 0.05))
        
        # Check if other fingers are curled
        other_tips = [12, 16, 20]  # middle, ring, pinky
        other_bases = [9, 13, 17]
        curl_ratios = []
        
        for tip, base in zip(other_tips, other_bases):
            curl = landmarks[tip][1] - landmarks[base][1]
            length = np.linalg.norm(landmarks[tip] - landmarks[base])
            if length > 0:
                curl_ratios.append(min(1.0, curl / length))
        
        curl_confidence = np.mean(curl_ratios) if curl_ratios else 0.0
        
        return min(1.0, (distance_confidence + curl_confidence) / 2)
    
    def _is_release(self, landmarks):
        """Check for release gesture"""
        thumb_tip = landmarks[self.THUMB_TIP]
        index_tip = landmarks[self.INDEX_TIP]
        
        distance = np.linalg.norm(thumb_tip - index_tip)
        return distance > 0.1
    
    def _calculate_release_confidence(self, landmarks):
        """Calculate confidence for release gesture"""
        thumb_tip = landmarks[self.THUMB_TIP]
        index_tip = landmarks[self.INDEX_TIP]
        
        distance = np.linalg.norm(thumb_tip - index_tip)
        base_confidence = min(1.0, (distance - 0.1) / 0.1)
        
        # Check finger spread
        spread = np.mean([
            np.linalg.norm(landmarks[8] - landmarks[12]),  # index to middle
            np.linalg.norm(landmarks[12] - landmarks[16])  # middle to ring
        ])
        spread_confidence = min(1.0, spread / 0.15)
        
        return min(1.0, (base_confidence + spread_confidence) / 2)
    
    def _is_move_to(self, landmarks):
        """Check for pointing gesture"""
        # Check if index finger is extended
        index_extended = landmarks[self.INDEX_TIP][1] < landmarks[self.INDEX_BASE][1]
        
        # Check if other fingers are curled
        other_tips = [4, 12, 16, 20]  # thumb, middle, ring, pinky
        other_bases = [2, 9, 13, 17]
        
        other_curled = True
        for tip, base in zip(other_tips, other_bases):
            if landmarks[tip][1] < landmarks[base][1]:
                other_curled = False
                break
        
        return index_extended and other_curled
    
    def _calculate_move_confidence(self, landmarks):
        """Calculate confidence for move_to gesture"""
        # Calculate index finger extension
        index_extension = landmarks[self.INDEX_BASE][1] - landmarks[self.INDEX_TIP][1]
        index_length = np.linalg.norm(landmarks[self.INDEX_TIP] - landmarks[self.INDEX_BASE])
        extension_confidence = min(1.0, index_extension / index_length) if index_length > 0 else 0.0
        
        # Calculate curl of other fingers
        other_tips = [4, 12, 16, 20]  # thumb, middle, ring, pinky
        other_bases = [2, 9, 13, 17]
        curl_ratios = []
        
        for tip, base in zip(other_tips, other_bases):
            curl = landmarks[tip][1] - landmarks[base][1]
            length = np.linalg.norm(landmarks[tip] - landmarks[base])
            if length > 0:
                curl_ratios.append(min(1.0, -curl / length))
        
        curl_confidence = np.mean(curl_ratios) if curl_ratios else 0.0
        
        return min(1.0, (extension_confidence + curl_confidence) / 2)
    
    def _get_majority_gesture(self):
        """Get most common gesture in history window"""
        if not self.gesture_history:
            return self.last_gesture
            
        gesture_counts = {}
        for gesture in self.gesture_history:
            gesture_counts[gesture] = gesture_counts.get(gesture, 0) + 1
            
        return max(gesture_counts.items(), key=lambda x: x[1])[0]

def main(args=None):
    rclpy.init(args=args)
    node = GestureClassifierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
