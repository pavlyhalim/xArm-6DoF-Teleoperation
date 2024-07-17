import numpy as np
import pyrealsense2 as rs
import cv2
from arm_tracker import ArmTracker
from xarm_kinematics import calculate_arm_angles, convert_angles_to_xarm, limit_angular_movement, limit_velocity, calculate_velocity
from point_cloud_processor import process_point_cloud, cuboid_filter_custom
from xarm.wrapper import XArmAPI
import time

def main():
    # Initialize xArm
    arm = XArmAPI('192.168.1.220')  # Replace with your xArm's IP address
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(state=0)

    # Initialize ArmTracker
    tracker = ArmTracker()

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    align = rs.align(rs.stream.color)

    # Initialize variables for smoothing and velocity calculation
    prev_angles = np.zeros(6)
    prev_time = time.time()

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
            # Calculate arm angles using Cython function
            shoulder = tracked_keypoints['shoulder']
            elbow = tracked_keypoints['elbow']
            hand = tracked_keypoints['wrist']
            arm_angles = calculate_arm_angles(shoulder, elbow, hand)

            # Process point cloud using Cython function
            processed_cloud = process_point_cloud(depth_frame, tracked_keypoints)

            # Convert angles to xArm joint angles
            xarm_angles = convert_angles_to_xarm(arm_angles)

            # Limit angular movement
            xarm_angles = np.array([limit_angular_movement(angle, prev, 5) for angle, prev in zip(xarm_angles, prev_angles)])

            # Calculate and limit velocity
            current_time = time.time()
            time_step = current_time - prev_time
            velocities = calculate_velocity(xarm_angles, prev_angles, time_step)
            limited_velocities = limit_velocity(velocities, max_velocity=180.0)  # 180 deg/s max velocity

            # Send angles to xArm
            arm.set_servo_angle_j(angles=xarm_angles.tolist(), speed=limited_velocities.tolist(), is_radian=False)

            # Update previous angles and time for next iteration
            prev_angles = xarm_angles
            prev_time = current_time

            # Visualize results (you can implement this part as needed)
            visualize_results(color_image, tracked_keypoints, processed_cloud)

        if cv2.waitKey(5) & 0xFF == 27:
            break

    pipeline.stop()
    arm.disconnect()
    cv2.destroyAllWindows()

def visualize_results(color_image, tracked_keypoints, processed_cloud, ax1, ax2):
    # Draw keypoints on color image
    for point_name, point in tracked_keypoints.items():
        cv2.circle(color_image, (int(point[0]), int(point[1])), 5, (0, 255, 0), -1)
        cv2.putText(color_image, point_name, (int(point[0])+10, int(point[1])+10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

    # Display the image with pose landmarks
    ax1.clear()
    ax1.imshow(cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB))
    ax1.set_title('Arm Tracking')
    ax1.axis('off')

    # Visualize processed point cloud
    if processed_cloud is not None:
        ax2.clear()
        points = np.asarray(processed_cloud.points)
        ax2.scatter(points[:, 0], points[:, 1], points[:, 2], s=1, c=points[:, 2], cmap='viridis')
        ax2.set_title('Processed Point Cloud')
        ax2.set_xlabel('X')
        ax2.set_ylabel('Y')
        ax2.set_zlabel('Z')
        ax2.set_xlim([-0.5, 0.5])
        ax2.set_ylim([-0.5, 0.5])
        ax2.set_zlim([0, 1])

    plt.draw()
    plt.pause(0.001)

if __name__ == "__main__":
    main()