import numpy as np
import pyrealsense2 as rs
import cv2
from arm_tracker_cy import ArmTracker
from xarm_kinematics import calculate_arm_angles, convert_angles_to_xarm
from realtime_control import PIDController, generate_smooth_target
from trajectory import generate_minimum_jerk_trajectory, limit_joint_angles, limit_joint_velocities, limit_joint_accelerations
from xarm.wrapper import XArmAPI
import time
import logging
import os
import warnings
import traceback 
import threading

# Set up logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

class TimeoutError(Exception):
    pass

def timeout(seconds):
    def decorator(func):
        def wrapper(*args, **kwargs):
            result = [TimeoutError('Function call timed out')]
            def target():
                try:
                    result[0] = func(*args, **kwargs)
                except Exception as e:
                    result[0] = e
            thread = threading.Thread(target=target)
            thread.daemon = True
            thread.start()
            thread.join(seconds)
            if isinstance(result[0], Exception):
                raise result[0]
            return result[0]
        return wrapper
    return decorator

def initialize_xarm():
    logging.info("Initializing xArm...")
    try:
        arm = XArmAPI('192.168.1.220')
        arm.connect()
        arm.motion_enable(enable=True)
        arm.set_mode(1)  # Set to position control mode
        arm.set_state(state=0)
        logging.info("xArm initialized successfully")
        return arm
    except Exception as e:
        logging.error(f"Error initializing xArm: {str(e)}")
        raise

def initialize_realsense():
    logging.info("Initializing RealSense camera...")
    try:
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        pipeline.start(config)
        align = rs.align(rs.stream.color)
        logging.info("RealSense camera initialized successfully")
        return pipeline, align
    except Exception as e:
        logging.error(f"Error initializing RealSense camera: {str(e)}")
        raise

@timeout(5)
def get_frames(pipeline, align):
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    return depth_frame, color_frame

def main():
    logging.info("Starting main function")
    try:
        arm = initialize_xarm()
        logging.info("Initializing ArmTracker...")
        tracker = ArmTracker()
        logging.info("ArmTracker initialized")
        pipeline, align = initialize_realsense()

        current_angles = np.zeros(6, dtype=np.float64)
        prev_angles = np.zeros(6, dtype=np.float64)
        current_velocity = np.zeros(6, dtype=np.float64)
        max_velocity = 60.0  # degrees per second
        max_acceleration = 180.0  # degrees per second^2
        min_angles = np.array([-360, -117, -225, -360, -97, -360])
        max_angles = np.array([360, 120, 11, 360, 180, 360])
        
        logging.info("Initializing PID controller...")
        pid_controller = PIDController(kp=5.5, ki=0.1, kd=0.2, num_joints=6)
        logging.info("PID controller initialized")
        
        target_dt = 1.0 / 60.0  # Target control loop frequency (60 Hz)
        trajectory_points = 60  # Number of points in the minimum jerk trajectory

        logging.info("Entering main loop")
        while True:
            loop_start_time = time.time()

            logging.debug("Waiting for frames...")
            try:
                depth_frame, color_frame = get_frames(pipeline, align)
            except TimeoutError:
                logging.error("Timeout while waiting for frames")
                continue
            except Exception as e:
                logging.error(f"Error getting frames: {str(e)}")
                continue

            if not depth_frame or not color_frame:
                logging.warning("Missing depth or color frame")
                continue

            logging.debug("Processing color image")
            color_image = np.asanyarray(color_frame.get_data())
            
            logging.debug("Tracking arm")
            tracked_keypoints = tracker.track_arm(color_image, depth_frame)

            # Visualize tracking results
            visualized_image = tracker.visualize_tracking(color_image.copy(), tracked_keypoints)
            cv2.imshow("Arm Tracking", visualized_image)
            cv2.waitKey(1)

            if tracked_keypoints is not None:
                try:
                    logging.debug("Calculating arm angles")
                    arm_angles = calculate_arm_angles(tracked_keypoints['shoulder'], tracked_keypoints['elbow'], tracked_keypoints['wrist'])
                    target_angles = convert_angles_to_xarm(arm_angles)

                    logging.debug("Generating minimum jerk trajectory")
                    trajectory = generate_minimum_jerk_trajectory(current_angles, target_angles, trajectory_points)

                    for point in trajectory:
                        logging.debug("Applying PID control")
                        control_output = pid_controller.update(point - current_angles, target_dt)
                        
                        logging.debug("Updating current angles and velocity")
                        new_angles = limit_joint_angles(current_angles + control_output * target_dt, min_angles, max_angles)
                        new_velocity = (new_angles - current_angles) / target_dt
                        limited_velocity = limit_joint_velocities(new_velocity, max_velocity)
                        limited_velocity = limit_joint_accelerations(limited_velocity, current_velocity, max_acceleration, target_dt)

                        current_angles = current_angles + limited_velocity * target_dt
                        current_velocity = limited_velocity

                        logging.debug("Sending command to the arm")
                        speed = np.max(np.abs(current_velocity)) * 2
                        speed = min(max(speed, 10), 500)  # Limit speed between 10 and 500 deg/s

                        arm.set_servo_angle_j(angles=current_angles.tolist(), speed=speed, is_radian=False, wait=False)
                        logging.info(f"Angles: {current_angles}, Speed: {speed}")

                        elapsed_time = time.time() - loop_start_time
                        if elapsed_time < target_dt:
                            time.sleep(target_dt - elapsed_time)
                        loop_start_time = time.time()

                except Exception as e:
                    logging.error(f"Error in main loop: {str(e)}")
                    logging.error(traceback.format_exc())
            else:
                logging.warning("No keypoints detected")

            elapsed_time = time.time() - loop_start_time
            if elapsed_time < target_dt:
                time.sleep(target_dt - elapsed_time)

    except KeyboardInterrupt:
        logging.info("Program interrupted by user")
    except Exception as e:
        logging.error(f"Unexpected error: {str(e)}")
        logging.error(traceback.format_exc())
    finally:
        logging.info("Cleaning up...")
        pipeline.stop()
        arm.disconnect()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()