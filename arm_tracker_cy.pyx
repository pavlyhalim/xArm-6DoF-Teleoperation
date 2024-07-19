# arm_tracker_cy.pyx

import cython
import numpy as np
cimport numpy as np

@cython.boundscheck(False)
@cython.wraparound(False)
def track_arm_cy(pose_landmarks, depth_frame, mp_pose, filters):
    cdef dict keypoints = {}
    cdef list landmarks = [
        mp_pose.PoseLandmark.LEFT_SHOULDER,
        mp_pose.PoseLandmark.LEFT_ELBOW,
        mp_pose.PoseLandmark.LEFT_WRIST
    ]
    cdef list names = ['shoulder', 'elbow', 'wrist']
    
    for name, landmark_index in zip(names, landmarks):
        landmark = pose_landmarks.landmark[landmark_index]
        
        if landmark.visibility < 0.5:
            continue

        x = int(landmark.x * depth_frame.width)
        y = int(landmark.y * depth_frame.height)
        
        x = max(0, min(x, depth_frame.width - 1))
        y = max(0, min(y, depth_frame.height - 1))
        
        z = depth_frame.get_distance(x, y)
        
        point = np.array([x, y, z], dtype=np.float64)
        
        filtered_point = filters[name].update(point)
        keypoints[name] = filtered_point

    return keypoints