#!/usr/bin/env python3

import numpy as np
from geometry_msgs.msg import Point
from collections import deque

class KalmanFilter:
    """Simple Kalman filter for 3D point tracking"""
    def __init__(self, process_variance=0.1, measurement_variance=0.1):
        # State: [x, y, z, dx, dy, dz]
        self.state = np.zeros(6)
        self.covariance = np.eye(6)
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        
        # State transition matrix (constant velocity model)
        self.F = np.array([
            [1, 0, 0, 1, 0, 0],
            [0, 1, 0, 0, 1, 0],
            [0, 0, 1, 0, 0, 1],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])
        
        # Measurement matrix (we only measure position)
        self.H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0]
        ])
    
    def predict(self):
        """Predict next state"""
        self.state = self.F @ self.state
        self.covariance = self.F @ self.covariance @ self.F.T + self.process_variance * np.eye(6)
        return self._get_position()
    
    def update(self, measurement):
        """Update state with new measurement"""
        z = np.array([measurement.x, measurement.y, measurement.z])
        
        # Kalman gain
        S = self.H @ self.covariance @ self.H.T + self.measurement_variance * np.eye(3)
        K = self.covariance @ self.H.T @ np.linalg.inv(S)
        
        # Update state
        y = z - (self.H @ self.state)
        self.state = self.state + K @ y
        self.covariance = (np.eye(6) - K @ self.H) @ self.covariance
        
        return self._get_position()
    
    def _get_position(self):
        """Convert state to Point message"""
        point = Point()
        point.x = self.state[0]
        point.y = self.state[1]
        point.z = self.state[2]
        return point

class MovingAverageFilter:
    """Simple moving average filter for smoothing"""
    def __init__(self, window_size=5):
        self.window_size = window_size
        self.points = deque(maxlen=window_size)
    
    def update(self, point):
        """Add new point and return filtered result"""
        self.points.append(point)
        
        # Calculate average
        filtered = Point()
        n = len(self.points)
        if n == 0:
            return point
            
        filtered.x = sum(p.x for p in self.points) / n
        filtered.y = sum(p.y for p in self.points) / n
        filtered.z = sum(p.z for p in self.points) / n
        
        return filtered
