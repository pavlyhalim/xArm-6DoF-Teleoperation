# arm_tracker_cy.pyx
cimport numpy as np
import numpy as np
from filters import ExtendedKalmanFilter, LowPassFilter
import logging
import cv2

cdef class ArmTracker:
    cdef:
        object mp_pose
        object pose
        dict kalman_filters
        object low_pass_filter

    def __init__(self):
        import mediapipe as mp
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(static_image_mode=False, model_complexity=2, min_detection_confidence=0.6, min_tracking_confidence=0.6)
        
        # Initialize ExtendedKalmanFilter for each joint
        initial_state = np.zeros(6, dtype=np.float64)  # 3 position + 3 velocity
        initial_P = np.eye(6, dtype=np.float64) * 100
        Q = np.eye(6, dtype=np.float64)
        Q[:3, :3] *= 2000  
        Q[3:, 3:] *= 200   
        R = np.eye(3, dtype=np.float64) * 0.5  
        
        self.kalman_filters = {
            'shoulder': ExtendedKalmanFilter(initial_state, initial_P, Q, R),
            'elbow': ExtendedKalmanFilter(initial_state, initial_P, Q, R),
            'wrist': ExtendedKalmanFilter(initial_state, initial_P, Q, R)
        }
        
        # Initialize LowPassFilter with a higher alpha for more responsiveness
        self.low_pass_filter = LowPassFilter(alpha=0.9) 

    cpdef dict track_arm(self, np.ndarray[unsigned char, ndim=3] color_image, object depth_frame):
        cdef dict keypoints = {}
        cdef list landmarks = [
            self.mp_pose.PoseLandmark.LEFT_SHOULDER,
            self.mp_pose.PoseLandmark.LEFT_ELBOW,
            self.mp_pose.PoseLandmark.LEFT_WRIST
        ]
        cdef list names = ['shoulder', 'elbow', 'wrist']
        
        logging.debug("Processing frame with MediaPipe Pose")
        results = self.pose.process(cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB))
        
        if not results.pose_landmarks:
            logging.warning("No pose landmarks detected")
            return None

        logging.debug("Pose landmarks detected, processing arm keypoints")
        for name, landmark_index in zip(names, landmarks):
            landmark = results.pose_landmarks.landmark[landmark_index]
            
            if landmark.visibility < 0.5:
                logging.warning(f"{name} landmark visibility too low: {landmark.visibility}")
                continue

            x = float(landmark.x * depth_frame.width)
            y = float(landmark.y * depth_frame.height)
            
            x = max(0.0, min(x, float(depth_frame.width - 1)))
            y = max(0.0, min(y, float(depth_frame.height - 1)))
            
            z = float(depth_frame.get_distance(int(x), int(y)))
            
            point = np.array([x, y, z], dtype=np.float64)
            
            logging.debug(f"{name} keypoint detected at ({x:.2f}, {y:.2f}, {z:.2f})")
            
            # Apply Extended Kalman Filter
            filtered_state = self.kalman_filters[name].update(point)
            filtered_point = filtered_state[:3] 
            
            keypoints[name] = filtered_point

        # Apply Low-Pass Filter to all keypoints
        if len(keypoints) == 3:  
            all_points = np.concatenate([keypoints['shoulder'], keypoints['elbow'], keypoints['wrist']])
            smoothed_points = self.low_pass_filter.update(all_points)
            
            keypoints['shoulder'] = smoothed_points[:3]
            keypoints['elbow'] = smoothed_points[3:6]
            keypoints['wrist'] = smoothed_points[6:]
            
            logging.debug(f"Filtered keypoints: {keypoints}")
        else:
            logging.warning(f"Not all keypoints detected. Found {len(keypoints)} keypoints.")

        return keypoints

    cpdef visualize_tracking(self, np.ndarray[unsigned char, ndim=3] color_image, dict keypoints):
        if keypoints is None:
            return color_image

        for name, point in keypoints.items():
            cv2.circle(color_image, (int(point[0]), int(point[1])), 5, (0, 255, 0), -1)
            cv2.putText(color_image, name, (int(point[0]), int(point[1]) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        if len(keypoints) == 3:
            cv2.line(color_image, 
                     (int(keypoints['shoulder'][0]), int(keypoints['shoulder'][1])),
                     (int(keypoints['elbow'][0]), int(keypoints['elbow'][1])),
                     (255, 0, 0), 2)
            cv2.line(color_image, 
                     (int(keypoints['elbow'][0]), int(keypoints['elbow'][1])),
                     (int(keypoints['wrist'][0]), int(keypoints['wrist'][1])),
                     (255, 0, 0), 2)

        return color_image