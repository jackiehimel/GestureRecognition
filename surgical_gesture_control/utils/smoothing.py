import numpy as np
from geometry_msgs.msg import Twist
from collections import deque

class MovingAverageFilter:
    def __init__(self, window_size=5):
        self.window_size = window_size
        self.linear_x = deque(maxlen=window_size)
        self.linear_y = deque(maxlen=window_size)
        self.linear_z = deque(maxlen=window_size)
        self.angular_z = deque(maxlen=window_size)
    
    def update(self, cmd):
        """
        Apply moving average filter to command velocities
        Args:
            cmd: Twist message containing current command
        Returns:
            Twist: Filtered command
        """
        # Add new values to queues
        self.linear_x.append(cmd.linear.x)
        self.linear_y.append(cmd.linear.y)
        self.linear_z.append(cmd.linear.z)
        self.angular_z.append(cmd.angular.z)
        
        # Create filtered command
        filtered_cmd = Twist()
        filtered_cmd.linear.x = np.mean(self.linear_x)
        filtered_cmd.linear.y = np.mean(self.linear_y)
        filtered_cmd.linear.z = np.mean(self.linear_z)
        filtered_cmd.angular.z = np.mean(self.angular_z)
        
        return filtered_cmd
