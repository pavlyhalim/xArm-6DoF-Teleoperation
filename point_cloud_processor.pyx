# cython: language_level=3
import cython
import numpy as np
cimport numpy as np
import pyrealsense2 as rs

np.import_array()

cdef extern from "pyrealsense2.h" namespace "rs2":
    cdef cppclass pointcloud:
        pointcloud()
        points calculate(rs2_frame* frame)

    cdef cppclass points:
        const float* get_vertices()
        int size()

    cdef cppclass rs2_frame:
        pass

@cython.boundscheck(False)
@cython.wraparound(False)
def process_point_cloud(object depth_frame, dict tracked_keypoints):
    cdef pointcloud pc
    cdef points points = pc.calculate(<rs2_frame*>depth_frame.ptr())
    cdef const float* vertices = points.get_vertices()
    cdef int num_points = points.size()
    cdef np.ndarray[float, ndim=2] vtx = np.asarray(<float[:num_points, :3]> vertices)

    cdef np.ndarray[double, ndim=1] shoulder = tracked_keypoints['shoulder']
    cdef np.ndarray[double, ndim=1] wrist = tracked_keypoints['wrist']
    cdef np.ndarray[double, ndim=1] arm_direction = wrist - shoulder
    arm_direction /= np.linalg.norm(arm_direction)

    cdef np.ndarray[double, ndim=1] cuboid_center = (shoulder + wrist) / 2
    cdef double cuboid_length = np.linalg.norm(wrist - shoulder) * 1.2
    cdef double cuboid_width = 0.2
    cdef double cuboid_height = 0.2

    filtered_points = cuboid_filter_custom(
        cuboid_center, arm_direction,
        np.array([-cuboid_length/2, -cuboid_width/2, -cuboid_height/2]),
        np.array([cuboid_length/2, cuboid_width/2, cuboid_height/2]),
        vtx
    )

    return filtered_points

@cython.boundscheck(False)
@cython.wraparound(False)
def cuboid_filter_custom(np.ndarray[double, ndim=1] origin, 
                         np.ndarray[double, ndim=1] orientation, 
                         np.ndarray[double, ndim=1] min_pt, 
                         np.ndarray[double, ndim=1] max_pt, 
                         np.ndarray[float, ndim=2] cloud_points):
    cdef np.ndarray[double, ndim=2] rot_mat = np.zeros((3, 3), dtype=np.float64)
    cdef np.ndarray[double, ndim=1] normal, normal2
    cdef int i, j, n_points = cloud_points.shape[0]
    cdef np.ndarray[double, ndim=2] transform, inv_transform
    cdef np.ndarray[double, ndim=2] transformed_points
    cdef np.ndarray[np.uint8_t, ndim=1] mask

    orientation /= np.linalg.norm(orientation)
    rot_mat[:, 2] = orientation

    normal = np.cross(np.array([0.0, 0.0, 1.0], dtype=np.float64), orientation)
    normal /= np.linalg.norm(normal)
    rot_mat[:, 0] = normal

    normal2 = np.cross(orientation, normal)
    rot_mat[:, 1] = normal2

    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rot_mat
    transform[:3, 3] = origin

    inv_transform = np.linalg.inv(transform)
    transformed_points = np.dot(inv_transform, np.vstack((cloud_points.T, np.ones(n_points, dtype=np.float64))))

    mask = np.all((transformed_points[:3].T >= min_pt[:3]) & (transformed_points[:3].T <= max_pt[:3]), axis=1)

    return cloud_points[mask]