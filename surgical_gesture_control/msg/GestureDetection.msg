string gesture                    # The detected gesture type (NEUTRAL, GRASP, RELEASE, MOVE_TO, EMERGENCY_STOP)
float32 confidence                # Confidence score (0-1)
builtin_interfaces/Time timestamp # When the gesture was detected

# Hand landmark positions (21 landmarks, each with x,y,z coordinates)
geometry_msgs/Point[] landmarks   # Array of 3D points for each hand landmark

# Filtered motion data
geometry_msgs/Pose target_pose    # Target pose for robot end-effector
float32[] joint_angles           # Target joint angles if applicable

# System metrics
float32 latency                  # Processing latency in milliseconds
float32 recognition_accuracy     # Gesture recognition accuracy score
float32 control_precision        # Control precision metric
