import numpy as np
import pyrealsense2 as rs

def process_point_cloud(depth_frame, tracked_keypoints):
    pc = rs.pointcloud()
    points = pc.calculate(depth_frame)
    vtx = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)

    shoulder = tracked_keypoints['shoulder']
    wrist = tracked_keypoints['wrist']
    arm_direction = wrist - shoulder
    arm_direction /= np.linalg.norm(arm_direction)

    cuboid_center = (shoulder + wrist) / 2
    cuboid_length = np.linalg.norm(wrist - shoulder) * 1.2
    cuboid_width = 0.2
    cuboid_height = 0.2

    filtered_points = cuboid_filter_custom(
        cuboid_center, arm_direction,
        np.array([-cuboid_length/2, -cuboid_width/2, -cuboid_height/2]),
        np.array([cuboid_length/2, cuboid_width/2, cuboid_height/2]),
        vtx
    )

    return filtered_points

def cuboid_filter_custom(origin, orientation, min_pt, max_pt, cloud_points):
    rot_mat = np.zeros((3, 3))
    orientation /= np.linalg.norm(orientation)
    rot_mat[:, 2] = orientation

    normal = np.cross(np.array([0.0, 0.0, 1.0]), orientation)
    normal /= np.linalg.norm(normal)
    rot_mat[:, 0] = normal

    normal2 = np.cross(orientation, normal)
    rot_mat[:, 1] = normal2

    transform = np.eye(4)
    transform[:3, :3] = rot_mat
    transform[:3, 3] = origin

    inv_transform = np.linalg.inv(transform)
    transformed_points = np.dot(inv_transform, np.vstack((cloud_points.T, np.ones(cloud_points.shape[0]))))

    mask = np.all((transformed_points[:3].T >= min_pt[:3]) & (transformed_points[:3].T <= max_pt[:3]), axis=1)

    return cloud_points[mask]