# xArm Real-Time Arm Tracking with Minimum Jerk Trajectory Control

## Table of Contents
1. [Project Overview](#project-overview)
2. [Implementation Details](#implementation-details)
   - [Real-Time Arm Tracking](#1-real-time-arm-tracking)
   - [Inverse Kinematics](#2-inverse-kinematics)
   - [Minimum Jerk Trajectory Control](#3-minimum-jerk-trajectory-control)
   - [Filtering and Smoothing](#4-filtering-and-smoothing)
   - [xArm Control](#5-xarm-control)
3. [Building and Running the Project](#building-and-running-the-project)
   - [Prerequisites](#prerequisites)
   - [Building the Project](#building-the-project)
   - [Running the Project](#running-the-project)
4. [Safety Considerations](#safety-considerations)
5. [Troubleshooting](#troubleshooting)

## Project Overview

This repository hosts a sophisticated Python-based framework for real-time arm tracking and control of an xArm robotic manipulator using a RealSense depth camera. Leveraging computer vision, inverse kinematics, and advanced control techniques, the system enables the xArm to mimic human arm movements with high fidelity and smoothness.

## Implementation Details

### 1. Real-Time Arm Tracking

The system uses a highly optimized arm tracking module (`arm_tracker_cy`) implemented in Cython for enhanced performance. This module leverages the RealSense depth camera to capture depth and color data streams. The depth information is crucial for accurately estimating the 3D position of the user's arm, while the color stream facilitates robust arm segmentation and tracking against complex backgrounds.

### 2. Inverse Kinematics

The tracked arm joint positions are fed into an inverse kinematics solver (`xarm_kinematics`), which calculates the corresponding joint angles required for the xArm to achieve the same pose. The system applies joint angle limits to ensure safe operation within the physical constraints of the xArm.

### 3. Minimum Jerk Trajectory Control

The system implements minimum jerk trajectory control (`trajectory.pyx`) to ensure smooth and natural-looking movements of the robotic arm. This approach minimizes the jerk (rate of change of acceleration) along the path, resulting in more human-like motion. The trajectory module includes functions for limiting joint angles within safe ranges, which is crucial for preventing damage to the robotic arm and ensuring safe operation.

### 4. Filtering and Smoothing

The system employs both Extended Kalman Filters and Low-Pass Filters (`filters`) to reduce noise and provide smooth, stable tracking results. This helps to mitigate jitter and sudden movements that could be problematic for the robotic arm.

### 5. xArm Control

The main control loop (`main.py`) integrates all components, handling the real-time processing of camera data, arm tracking, inverse kinematics calculations, trajectory generation, and sending control commands to the xArm.

## Building and Running the Project

### Prerequisites

- Python 3.7 or higher
- Cython
- NumPy
- OpenCV
- MediaPipe
- pyrealsense2
- xArm Python SDK

### Building the Project

1. Clone the repository:
   ```
   git clone https://github.com/pavlyhalim/Xdof_teleoperation.git
   cd Xdof_teleoperation
   ```

2. Install the required dependencies:
   ```
   pip install -r requirements.txt
   ```

3. Build the Cython extensions:
   ```
   python setup.py build_ext --inplace
   ```

### Running the Project

1. Ensure your RealSense camera is connected and recognized by your system.

2. Make sure your xArm is connected to the same network as your computer and note its IP address.

3. Update the xArm IP address in `main.py`:
   ```python
   arm = XArmAPI('192.168.1.220')  # Replace with your xArm's IP address
   ```

4. Run the main script:
   ```
   python main.py
   ```

5. The system will initialize the xArm and the RealSense camera. You should see a window displaying the arm tracking visualization.

6. Stand in front of the camera, ensuring your left arm is visible. The xArm should begin mimicking your arm movements.

7. To stop the program, press Ctrl+C in the terminal.

## Safety Considerations

- Always maintain a safe distance from the xArm while it's in operation.
- Be prepared to use the emergency stop if the arm behaves unexpectedly.
- Ensure the workspace around the xArm is clear of obstacles.
- Regularly check and maintain the xArm and its surrounding area to prevent potential hazards.

## Troubleshooting

- If you encounter issues with arm tracking, try adjusting lighting conditions or your distance from the camera.
- If the xArm movements seem jerky or unstable, you may need to adjust the PID controller parameters or filtering settings.
- Ensure all cable connections are secure and the xArm is properly powered.
- Check the system logs for any error messages or warnings that might indicate the source of the problem.

For any issues or suggestions, please open an issue in the GitHub repository.