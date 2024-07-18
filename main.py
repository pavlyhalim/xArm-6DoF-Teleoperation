import numpy as np
import pyrealsense2 as rs
import cv2
from arm_tracker import ArmTracker
from xarm_kinematics import calculate_arm_angles, convert_angles_to_xarm, limit_angular_movement
from xarm.wrapper import XArmAPI
import time
import logging
import traceback
from filters import MovingAverageFilter, HysteresisFilter

logging.basicConfig(level=logging.INFO)

def initialize_xarm():
    arm = XArmAPI('192.168.1.220')  # Replace with your xArm's IP address
    arm.connect()
    arm.motion_enable(enable=True)
    arm.set_mode(1)  # Set to position control mode
    arm.set_state(state=0)  # Set to ready state
    return arm

def initialize_realsense():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    align = rs.align(rs.stream.color)
    return pipeline, align

def visualize_results(color_image, tracked_keypoints):
    if tracked_keypoints is not None:
        for point_name, point in tracked_keypoints.items():
            cv2.circle(color_image, (int(point[0]), int(point[1])), 5, (0, 255, 0), -1)
            cv2.putText(color_image, f"{point_name}: ({point[0]:.2f}, {point[1]:.2f}, {point[2]:.2f})", 
                        (int(point[0])+10, int(point[1])+10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    else:
        cv2.putText(color_image, "No keypoints detected", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

    cv2.imshow('Arm Tracking', color_image)

def main():
    arm = initialize_xarm()
    tracker = ArmTracker()
    pipeline, align = initialize_realsense()

    moving_avg_filter = MovingAverageFilter(window_size=15, vector_size=6)
    hysteresis_filters = [HysteresisFilter(max_val=0.1, min_val=-0.1) for _ in range(6)]

    prev_angles = np.zeros(6)
    prev_velocities = np.zeros(6)
    max_angular_change = 5.0  # degrees per iteration
    max_velocity = 15.0  # degrees per second
    max_acceleration = 30.0  # degrees per second^2

    prev_time = time.time()

    try:
        while True:
            loop_start_time = time.time()

            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                continue

            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            tracked_keypoints = tracker.track_arm(color_image, depth_frame)

            if tracked_keypoints is not None:
                current_time = time.time()
                dt = current_time - prev_time

                arm_angles = calculate_arm_angles(tracked_keypoints['shoulder'], tracked_keypoints['elbow'], tracked_keypoints['wrist'])
                xarm_angles = convert_angles_to_xarm(arm_angles)

                # Calculate velocities
                velocities = (xarm_angles - prev_angles) / dt

                # Apply moving average filter
                moving_avg_filter.add_new_values(xarm_angles, velocities)
                filtered_angles = moving_avg_filter.get_filtered_values()
                filtered_velocities = np.array(velocities)  # We're not filtering velocities in this version

                # Apply angular movement limitation
                limited_angles = np.array([limit_angular_movement(angle, prev, max_angular_change) 
                                           for angle, prev in zip(filtered_angles, prev_angles)])

                # Apply velocity limitation
                limited_velocities = np.clip(filtered_velocities, -max_velocity, max_velocity)

                # Apply acceleration limitation
                accelerations = (limited_velocities - prev_velocities) / dt
                limited_accelerations = np.clip(accelerations, -max_acceleration, max_acceleration)
                limited_velocities = prev_velocities + limited_accelerations * dt

                # Recalculate angles based on limited velocities
                limited_angles = prev_angles + limited_velocities * dt

                # Apply hysteresis filter
                for i in range(6):
                    if hysteresis_filters[i].filter(limited_angles[i] - prev_angles[i]):
                        limited_angles[i] = prev_angles[i]

                # Calculate speed based on the maximum velocity
                speed = np.max(np.abs(limited_velocities)) * 5  # Adjust this multiplier as needed
                speed = min(max(speed, 10), 100)  # Limit speed between 10 and 100

                # Only move if the change is significant
                if np.max(np.abs(limited_angles - prev_angles)) > 0.5:  # 0.5 degree threshold
                    arm.set_servo_angle_j(angles=limited_angles.tolist(), speed=speed, is_radian=False, wait=False)
                    logging.info(f"Angles: {limited_angles}, Speed: {speed}")
                else:
                    logging.info("Movement within threshold, not sending command")

                prev_angles = limited_angles
                prev_velocities = limited_velocities
                prev_time = current_time

            visualize_results(color_image, tracked_keypoints)

            if cv2.waitKey(5) & 0xFF == 27:
                break

            # Ensure consistent loop time
            time.sleep(max(0, 1/30 - (time.time() - loop_start_time)))

    except KeyboardInterrupt:
        logging.info("Program interrupted by user")
    except Exception as e:
        logging.error(f"Unexpected error: {str(e)}")
        logging.error(traceback.format_exc())
    finally:
        pipeline.stop()
        arm.disconnect()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()