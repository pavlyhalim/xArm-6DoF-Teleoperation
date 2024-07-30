import numpy as np
import pyrealsense2 as rs
import cv2
from arm_tracker_cy import ArmTracker
from xarm_kinematics import calculate_arm_angles, convert_angles_to_xarm , limit_joint_angles
from xarm.wrapper import XArmAPI
import time
import logging
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
        arm.set_mode(6) 
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

        # Enable the arm
        code = arm.motion_enable(enable=True)
        if code != 0:
            logging.error(f"Failed to enable the arm: {code}")
            return

        current_angles = np.zeros(6, dtype=np.float64)
        min_angles = np.array([-360, -117, -225, -360, -97, -360])
        max_angles = np.array([360, 120, 11, 360, 180, 360])
        
        target_dt = 1.0 / 30.0  

        while True:
            loop_start_time = time.time()

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

            color_image = np.asanyarray(color_frame.get_data())
            
            tracked_keypoints = tracker.track_arm(color_image, depth_frame)

            visualized_image = tracker.visualize_tracking(color_image.copy(), tracked_keypoints)
            cv2.imshow("Arm Tracking", visualized_image)
            cv2.waitKey(1)

            if tracked_keypoints is not None:
                try:
                    arm_angles = calculate_arm_angles(tracked_keypoints['shoulder'], tracked_keypoints['elbow'], tracked_keypoints['wrist'])
                    target_angles = convert_angles_to_xarm(arm_angles)

                    # Apply joint limits
                    limited_angles = limit_joint_angles(target_angles, min_angles, max_angles)
                    
                    # Set joint angles using set_servo_angle
                    code = arm.set_servo_angle(angle=limited_angles.tolist(), speed=180, mvacc=1800,wait=False, radius=-1)
                    if code != 0:
                        logging.error(f"Failed to set joint angles: {code}")
                    else:
                        current_angles = limited_angles

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