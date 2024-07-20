# trajectory.pyx
cimport numpy as np
import numpy as np
from libc.math cimport pow

cpdef np.ndarray[double, ndim=2] generate_minimum_jerk_trajectory(np.ndarray[double, ndim=1] start_angles, np.ndarray[double, ndim=1] end_angles, int num_points):
    cdef int num_joints = start_angles.shape[0]
    cdef np.ndarray[double, ndim=2] trajectory = np.zeros((num_points, num_joints), dtype=np.float64)
    cdef double t
    cdef int i, j
    cdef double tau, tau_3, tau_4, tau_5

    for i in range(num_points):
        t = i / (num_points - 1.0)
        tau = t
        tau_3 = pow(tau, 3)
        tau_4 = pow(tau, 4)
        tau_5 = pow(tau, 5)
        
        for j in range(num_joints):
            trajectory[i, j] = start_angles[j] + \
                               (end_angles[j] - start_angles[j]) * \
                               (10 * tau_3 - 15 * tau_4 + 6 * tau_5)
    
    return trajectory

cpdef np.ndarray[double, ndim=1] limit_joint_angles(np.ndarray angles, np.ndarray min_angles, np.ndarray max_angles):
    cdef np.ndarray[double, ndim=1] angles_f64 = np.asarray(angles, dtype=np.float64)
    cdef np.ndarray[double, ndim=1] min_angles_f64 = np.asarray(min_angles, dtype=np.float64)
    cdef np.ndarray[double, ndim=1] max_angles_f64 = np.asarray(max_angles, dtype=np.float64)
    return np.clip(angles_f64, min_angles_f64, max_angles_f64)

cpdef np.ndarray[double, ndim=1] limit_joint_velocities(np.ndarray velocities, double max_velocity):
    cdef np.ndarray[double, ndim=1] velocities_f64 = np.asarray(velocities, dtype=np.float64)
    return np.clip(velocities_f64, -max_velocity, max_velocity)

cpdef np.ndarray[double, ndim=1] limit_joint_accelerations(np.ndarray velocities, np.ndarray prev_velocities, double max_acceleration, double dt):
    cdef np.ndarray[double, ndim=1] velocities_f64 = np.asarray(velocities, dtype=np.float64)
    cdef np.ndarray[double, ndim=1] prev_velocities_f64 = np.asarray(prev_velocities, dtype=np.float64)
    cdef np.ndarray[double, ndim=1] accelerations = (velocities_f64 - prev_velocities_f64) / dt
    cdef np.ndarray[double, ndim=1] limited_accelerations = np.clip(accelerations, -max_acceleration, max_acceleration)
    return prev_velocities_f64 + limited_accelerations * dt