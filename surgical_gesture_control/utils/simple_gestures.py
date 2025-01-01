import numpy as np

class GestureRecognizer:
    def __init__(self):
        # Landmark indices for key points
        self.WRIST = 0
        self.THUMB_TIP = 4
        self.INDEX_TIP = 8
        self.INDEX_BASE = 5
        
    def recognize(self, landmarks):
        """
        Recognize gestures from hand landmarks
        Args:
            landmarks: numpy array of shape (21, 3) containing hand landmarks
        Returns:
            str: recognized gesture name
        """
        if self._is_open_palm(landmarks):
            return 'OPEN_PALM'
        elif self._is_closed_fist(landmarks):
            return 'CLOSED_FIST'
        elif self._is_pointing(landmarks):
            return 'POINTING'
        else:
            return 'UNKNOWN'
    
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
