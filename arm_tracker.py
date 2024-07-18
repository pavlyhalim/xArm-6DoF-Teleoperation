import numpy as np
import mediapipe as mp
import cv2

class LowPassFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.value = None

    def update(self, value):
        if self.value is None:
            self.value = value
        else:
            self.value = self.alpha * value + (1 - self.alpha) * self.value
        return self.value

class SimpleKalmanFilter:
    def __init__(self, initial_state, initial_estimate_error, measurement_noise, process_noise):
        self.state = initial_state
        self.estimate_error = initial_estimate_error
        self.measurement_noise = measurement_noise
        self.process_noise = process_noise

    def update(self, measurement):
        prediction = self.state
        prediction_error = self.estimate_error + self.process_noise

        kalman_gain = prediction_error / (prediction_error + self.measurement_noise)
        self.state = prediction + kalman_gain * (measurement - prediction)
        self.estimate_error = (1 - kalman_gain) * prediction_error

        return self.state

class ArmTracker:
    def __init__(self):
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(static_image_mode=False, model_complexity=2, min_detection_confidence=0.7, min_tracking_confidence=0.7)
        self.filters = {
            'shoulder': SimpleKalmanFilter(np.zeros(3), 1, 0.1, 0.01),
            'elbow': SimpleKalmanFilter(np.zeros(3), 1, 0.1, 0.01),
            'wrist': SimpleKalmanFilter(np.zeros(3), 1, 0.1, 0.01)
        }
        self.low_pass_filters = {
            'shoulder': LowPassFilter(0.5),
            'elbow': LowPassFilter(0.5),
            'wrist': LowPassFilter(0.5)
        }

    def track_arm(self, color_image, depth_frame):
        results = self.pose.process(cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB))
        if results.pose_landmarks:
            return self._extract_keypoints(results.pose_landmarks, depth_frame)
        return None

    def _extract_keypoints(self, pose_landmarks, depth_frame):
        keypoints = {}
        landmarks = [
            self.mp_pose.PoseLandmark.LEFT_SHOULDER,
            self.mp_pose.PoseLandmark.LEFT_ELBOW,
            self.mp_pose.PoseLandmark.LEFT_WRIST
        ]
        for name, landmark_index in zip(['shoulder', 'elbow', 'wrist'], landmarks):
            landmark = pose_landmarks.landmark[landmark_index]
            
            # Add confidence threshold
            if landmark.visibility < 0.5:
                continue

            x = int(landmark.x * depth_frame.width)
            y = int(landmark.y * depth_frame.height)
            
            # Ensure x and y are within the frame boundaries
            x = max(0, min(x, depth_frame.width - 1))
            y = max(0, min(y, depth_frame.height - 1))
            
            try:
                z = depth_frame.get_distance(x, y)
            except RuntimeError:
                z = 0.0
                print(f"Warning: Could not get depth for {name} at ({x}, {y})")
            
            point = np.array([x, y, z])
            
            # Apply low-pass filter
            filtered_point = self.low_pass_filters[name].update(point)
            
            # Apply Kalman filter
            keypoints[name] = self.filters[name].update(filtered_point)

        return keypoints

def main():
    # Initialize RealSense camera
    import pyrealsense2 as rs
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    align = rs.align(rs.stream.color)

    # Initialize ArmTracker
    tracker = ArmTracker()

    while True:
        # Get frames from RealSense camera
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Track arm and get keypoints
        tracked_keypoints = tracker.track_arm(color_image, depth_frame)

        if tracked_keypoints is not None:
            # Draw keypoints on color image
            for point_name, point in tracked_keypoints.items():
                cv2.circle(color_image, (int(point[0]), int(point[1])), 5, (0, 255, 0), -1)
                cv2.putText(color_image, point_name, (int(point[0])+10, int(point[1])+10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

            # Here you can call your Cython functions for further processing
            # For example:
            #arm_angles = calculate_arm_angles(tracked_keypoints['shoulder'], tracked_keypoints['elbow'], tracked_keypoints['wrist'])
            # processed_cloud = process_point_cloud(depth_frame, tracked_keypoints)

        # Display the image with pose landmarks
        cv2.imshow('Arm Tracking', color_image)

        if cv2.waitKey(5) & 0xFF == 27:
            break

    pipeline.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
