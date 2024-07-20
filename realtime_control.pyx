# realtime_control.pyx
cimport numpy as np
import numpy as np

cdef class PIDController:
    cdef double kp, ki, kd
    cdef np.ndarray integral, previous_error

    def __init__(self, double kp, double ki, double kd, int num_joints):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = np.zeros(num_joints, dtype=np.float64)
        self.previous_error = np.zeros(num_joints, dtype=np.float64)

    cpdef np.ndarray[double, ndim=1] update(self, np.ndarray[double, ndim=1] error, double dt):
        cdef np.ndarray[double, ndim=1] derivative = (error - self.previous_error) / dt
        self.integral += error * dt
        self.previous_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

cdef np.ndarray[double, ndim=1] interpolate_position(np.ndarray[double, ndim=1] current, np.ndarray[double, ndim=1] target, double alpha):
    return current + alpha * (target - current)

cpdef tuple generate_smooth_target(np.ndarray[double, ndim=1] current_position, np.ndarray[double, ndim=1] target_position, 
                                   np.ndarray[double, ndim=1] current_velocity, double max_velocity, double dt):
    cdef np.ndarray[double, ndim=1] position_error = target_position - current_position
    cdef np.ndarray[double, ndim=1] desired_velocity = position_error / (dt * 2)  # More aggressive
    cdef np.ndarray[double, ndim=1] limited_velocity = np.clip(desired_velocity, -max_velocity, max_velocity)
    cdef np.ndarray[double, ndim=1] smooth_target = current_position + limited_velocity * dt * 2  # More aggressive
    return smooth_target, limited_velocity