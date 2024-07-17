# cython: language_level=3
import cython
import numpy as np
cimport numpy as np
import mediapipe as mp
import pyrealsense2 as rs

np.import_array()

cdef class ArmTracker:
    cdef:
        object mp_pose
        object pose

    def __cinit__(self):
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(static_image_mode=False, model_complexity=2, min_detection_confidence=0.7, min_tracking_confidence=0.7)

    @cython.boundscheck(False)
    @cython.wraparound(False)
    def track_arm(self, unsigned char[:, :, ::1] color_image, object depth_frame):
        cdef np.ndarray[np.uint8_t, ndim=3] np_color_image = np.asarray(color_image)
        results = self.pose.process(np_color_image)
        if results.pose_landmarks:
            return self._extract_keypoints(results.pose_landmarks, depth_frame)
        return None

    cdef _extract_keypoints(self, object pose_landmarks, object depth_frame):
        cdef dict keypoints = {}
        cdef list landmarks = [
            self.mp_pose.PoseLandmark.LEFT_SHOULDER,
            self.mp_pose.PoseLandmark.LEFT_ELBOW,
            self.mp_pose.PoseLandmark.LEFT_WRIST
        ]
        cdef str name
        cdef int landmark_index
        cdef int x, y
        cdef float z
        cdef np.ndarray[np.float64_t, ndim=1] point
        cdef int width = depth_frame.get_width()
        cdef int height = depth_frame.get_height()
        
        for name, landmark_index in zip(['shoulder', 'elbow', 'wrist'], landmarks):
            landmark = pose_landmarks.landmark[landmark_index]
            x = int(landmark.x * width)
            y = int(landmark.y * height)
            z = depth_frame.get_distance(x, y)
            point = np.array([x, y, z], dtype=np.float64)
            keypoints[name] = point
        
        return keypoints