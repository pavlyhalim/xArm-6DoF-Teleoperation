# xarm_kinematics.pyx
import cython
import numpy as np
cimport numpy as np
from libc.math cimport M_PI, sin, cos, atan2, acos, sqrt, fmod

cdef class CalculateArmAngles:
    cdef public np.ndarray shoulderPoint, elbowPoint, handPoint, upperarm, forearm

    def __cinit__(self):
        self.shoulderPoint = np.zeros(3)
        self.elbowPoint = np.zeros(3)
        self.handPoint = np.zeros(3)
        self.upperarm = np.zeros(3)
        self.forearm = np.zeros(3)

    @cython.boundscheck(False)
    @cython.wraparound(False)
    cpdef void setArmData(self, np.ndarray[double, ndim=1] shoulder, np.ndarray[double, ndim=1] elbow, np.ndarray[double, ndim=1] hand):
        self.shoulderPoint = shoulder
        self.elbowPoint = elbow
        self.handPoint = hand
        self.upperarm = -self.shoulderPoint + self.elbowPoint
        self.forearm = -self.elbowPoint + self.handPoint

    @cython.boundscheck(False)
    @cython.wraparound(False)
    cpdef double getAlpha(self):
        return atan2(self.upperarm[1], self.upperarm[0]) * (180 / M_PI)

    @cython.boundscheck(False)
    @cython.wraparound(False)
    cpdef double getBeta(self):
        cdef np.ndarray[double, ndim=1] shouldCS_Z = np.array([self.upperarm[0], self.upperarm[1], 0.0])
        shouldCS_Z /= np.linalg.norm(shouldCS_Z)
        cdef np.ndarray[double, ndim=1] shouldCS_Y = np.array([0.0, 0.0, 1.0])
        cdef np.ndarray[double, ndim=1] shouldCS_X = np.cross(shouldCS_Z, shouldCS_Y)
        shouldCS_X /= np.linalg.norm(shouldCS_X)

        cdef np.ndarray[double, ndim=2] shouldCS = np.array([
            [shouldCS_Y[0], shouldCS_X[0], shouldCS_Z[0], self.shoulderPoint[0]],
            [shouldCS_Y[1], shouldCS_X[1], shouldCS_Z[1], self.shoulderPoint[1]],
            [shouldCS_Y[2], shouldCS_X[2], shouldCS_Z[2], self.shoulderPoint[2]],
            [0,             0,             0,             1]
        ])

        cdef np.ndarray[double, ndim=2] B = np.linalg.inv(shouldCS)
        cdef np.ndarray[double, ndim=1] elbowPoint_homogen = np.append(self.elbowPoint, 1)
        cdef np.ndarray[double, ndim=1] elbowPoint_projected_temp = np.dot(B, elbowPoint_homogen)
        cdef np.ndarray[double, ndim=1] elbowPoint_projected = elbowPoint_projected_temp[[0, 2]]

        return atan2(elbowPoint_projected[1], elbowPoint_projected[0]) * (180 / M_PI)

    @cython.boundscheck(False)
    @cython.wraparound(False)
    cpdef double getGamma(self):
        return acos(-(np.dot(self.upperarm, self.forearm)) / (np.linalg.norm(self.upperarm) * np.linalg.norm(self.forearm))) * (180 / M_PI)

    @cython.boundscheck(False)
    @cython.wraparound(False)
    cpdef double getDelta(self):
        cdef np.ndarray[double, ndim=1] elbCS_Z = self.upperarm / np.linalg.norm(self.upperarm)
        cdef np.ndarray[double, ndim=1] elbCS_Y = np.cross(self.upperarm, self.forearm)
        elbCS_Y /= np.linalg.norm(elbCS_Y)
        cdef np.ndarray[double, ndim=1] elbCS_X = np.cross(elbCS_Z, elbCS_Y)

        cdef np.ndarray[double, ndim=2] elbCS = np.array([
            [elbCS_Y[0], elbCS_X[0], elbCS_Z[0], self.elbowPoint[0]],
            [elbCS_Y[1], elbCS_X[1], elbCS_Z[1], self.elbowPoint[1]],
            [elbCS_Y[2], elbCS_X[2], elbCS_Z[2], self.elbowPoint[2]],
            [0,          0,          0,          1]
        ])

        cdef np.ndarray[double, ndim=2] A = np.linalg.inv(elbCS)
        cdef np.ndarray[double, ndim=1] handPoint_homogen = np.append(self.handPoint, 1)
        cdef np.ndarray[double, ndim=1] handPoint_projected_temp = np.dot(A, handPoint_homogen)
        cdef np.ndarray[double, ndim=1] handPoint_projected = handPoint_projected_temp[:2]

        cdef np.ndarray[double, ndim=1] Z = np.array([self.elbowPoint[0], self.elbowPoint[1], self.elbowPoint[2] + 1.0, 1.0])
        cdef np.ndarray[double, ndim=1] Z_projected_temp = np.dot(A, Z)
        cdef np.ndarray[double, ndim=1] Z_projected = Z_projected_temp[:2]

        return atan2(Z_projected[1], Z_projected[0]) * (180 / M_PI)

@cython.boundscheck(False)
@cython.wraparound(False)
def calculate_arm_angles(np.ndarray[double, ndim=1] shoulder, np.ndarray[double, ndim=1] elbow, np.ndarray[double, ndim=1] hand):
    cdef CalculateArmAngles calculator = CalculateArmAngles()
    calculator.setArmData(shoulder, elbow, hand)
    
    cdef double alpha = calculator.getAlpha()
    cdef double beta = calculator.getBeta()
    cdef double gamma = calculator.getGamma()
    cdef double delta = calculator.getDelta()
    
    # Apply offsets and normalize to [-180, 180] range
    alpha = fmod((alpha - 180.0 + 180.0), 360.0) - 180.0
    beta = beta  # Negate beta
    gamma = fmod((gamma - 180.0 + 180.0), 360.0) - 180.0
    delta = fmod((-delta - 90.0 + 180.0), 360.0) - 180.0
    
    return np.array([alpha, beta, gamma, delta])

@cython.boundscheck(False)
@cython.wraparound(False)
def convert_angles_to_xarm(np.ndarray[double, ndim=1] angles):
    cdef np.ndarray[double, ndim=1] xarm_angles = np.zeros(6)
    
    # Convert tracked angles to xArm 6 joint angles
    xarm_angles[0] = angles[0]  # Base rotation (alpha)
    xarm_angles[1] = -angles[1]  # Shoulder (beta, inverted)
    xarm_angles[2] = -angles[2] + 180.0  # Elbow (gamma, negated and offset by 180 degrees)
    xarm_angles[3] = -angles[3] - 90.0  # Wrist 1 (delta, inverted and offset by 90 degrees)
    xarm_angles[4] = angles[4] if len(angles) > 4 else 0  # Wrist 2 (epsilon or 0 if not provided)
    xarm_angles[5] = angles[5] if len(angles) > 5 else 0  # Wrist 3 (zeta or 0 if not provided)

    # Normalize angles to [-180, 180] range
    for i in range(6):
        xarm_angles[i] = fmod((xarm_angles[i] + 180.0), 360.0) - 180.0

    return xarm_angles

@cython.boundscheck(False)
@cython.wraparound(False)
def limit_angular_movement(double angle, double angle_old, double max_angular_change):
    cdef double delta_angle = angle - angle_old
    if min(abs(delta_angle), 360.0 - abs(delta_angle)) > max_angular_change:
        if 0.0 < delta_angle <= 180.0:
            return (angle_old + max_angular_change + 180.0) % 360.0 - 180.0
        elif -180.0 < delta_angle <= 0.0:
            return (angle_old - max_angular_change + 180.0) % 360.0 - 180.0
        elif 180.0 < delta_angle <= 360.0:
            return (angle_old - max_angular_change + 180.0) % 360.0 - 180.0
        elif -360.0 < delta_angle <= -180.0:
            return (angle_old + max_angular_change + 180.0) % 360.0 - 180.0
    return angle

@cython.boundscheck(False)
@cython.wraparound(False)
def limit_angle(double angle, double min_val, double max_val):
    return max(min(angle, max_val), min_val)


@cython.boundscheck(False)
@cython.wraparound(False)
def limit_velocity(np.ndarray[double, ndim=1] velocities, double max_velocity):
    return np.clip(velocities, -max_velocity, max_velocity)

@cython.boundscheck(False)
@cython.wraparound(False)
def calculate_velocity(np.ndarray[double, ndim=1] current_angles, np.ndarray[double, ndim=1] prev_angles, double time_step):
    return (current_angles - prev_angles) / time_step



@cython.boundscheck(False)
@cython.wraparound(False)
def forward_kinematics(np.ndarray[double, ndim=1] joint_angles):
    cdef double theta1 = joint_angles[0] * M_PI / 180.0
    cdef double theta2 = joint_angles[1] * M_PI / 180.0
    cdef double theta3 = joint_angles[2] * M_PI / 180.0
    cdef double theta4 = joint_angles[3] * M_PI / 180.0
    cdef double theta5 = joint_angles[4] * M_PI / 180.0
    cdef double theta6 = joint_angles[5] * M_PI / 180.0

    # DH parameters for xArm 6 (in mm and radians)
    cdef double d1 = 267.0
    cdef double a2 = 289.48866
    cdef double d4 = 342.5
    cdef double d6 = 97.0

    cdef double c1 = cos(theta1)
    cdef double s1 = sin(theta1)
    cdef double c2 = cos(theta2)
    cdef double s2 = sin(theta2)
    cdef double c3 = cos(theta3)
    cdef double s3 = sin(theta3)
    cdef double c4 = cos(theta4)
    cdef double s4 = sin(theta4)
    cdef double c5 = cos(theta5)
    cdef double s5 = sin(theta5)
    cdef double c6 = cos(theta6)
    cdef double s6 = sin(theta6)

    cdef double s23 = sin(theta2 + theta3)
    cdef double c23 = cos(theta2 + theta3)

    cdef double nx = c1*(c5*s23*s4 + c23*c4*c5) - s1*(c6*s5 + c5*s6) + c1*(c23*s4 - c4*s23)*s6
    cdef double ny = s1*(c5*s23*s4 + c23*c4*c5) + c1*(c6*s5 + c5*s6) + s1*(c23*s4 - c4*s23)*s6
    cdef double nz = c5*s23*s4 - s23*s5*s6 + c23*c4*c5 + c23*s4*s6 + c4*s23*s6
    cdef double ox = -c1*(c6*(c5*s23*s4 + c23*c4*c5) - s6*(c23*s4 - c4*s23)) - s1*(c5*c6 - s5*s6)
    cdef double oy = -s1*(c6*(c5*s23*s4 + c23*c4*c5) - s6*(c23*s4 - c4*s23)) + c1*(c5*c6 - s5*s6)
    cdef double oz = -c6*(s23*s5*s6 - c23*c4*c5 - c23*s4*s6 - c4*s23*s6) + s6*(c5*s23*s4 - s23*s5*c6)
    cdef double ax = c1*(s6*(c5*s23*s4 + c23*c4*c5) + c6*(c23*s4 - c4*s23)) - s1*(c5*s6 + c6*s5)
    cdef double ay = s1*(s6*(c5*s23*s4 + c23*c4*c5) + c6*(c23*s4 - c4*s23)) + c1*(c5*s6 + c6*s5)
    cdef double az = s6*(s23*s5*s6 - c23*c4*c5 - c23*s4*s6 - c4*s23*s6) + c6*(c5*s23*s4 - s23*s5*c6)
    cdef double px = c1*(a2*c2 + d4*c23 - d6*(c5*s23*s4 - s23*s5*c6 + c23*c4*c5 + c23*s4*s6 + c4*s23*s6))
    cdef double py = s1*(a2*c2 + d4*c23 - d6*(c5*s23*s4 - s23*s5*c6 + c23*c4*c5 + c23*s4*s6 + c4*s23*s6))
    cdef double pz = d1 - a2*s2 - d4*s23 - d6*(c5*s23*s4 + c23*c4*c5)

    cdef np.ndarray[double, ndim=2] T = np.array([
        [nx, ox, ax, px],
        [ny, oy, ay, py],
        [nz, oz, az, pz],
        [0, 0, 0, 1]
    ])

    return T

@cython.boundscheck(False)
@cython.wraparound(False)
def calculate_tcp_velocity(np.ndarray[double, ndim=2] prev_pose, np.ndarray[double, ndim=2] current_pose, double dt):
    cdef np.ndarray[double, ndim=1] prev_pos = prev_pose[:3, 3]
    cdef np.ndarray[double, ndim=1] current_pos = current_pose[:3, 3]
    cdef np.ndarray[double, ndim=1] linear_velocity = (current_pos - prev_pos) / dt

    cdef np.ndarray[double, ndim=2] prev_rot = prev_pose[:3, :3]
    cdef np.ndarray[double, ndim=2] current_rot = current_pose[:3, :3]
    cdef np.ndarray[double, ndim=2] rot_diff = np.dot(current_rot, prev_rot.T)
    
    cdef double angle = acos((np.trace(rot_diff) - 1) / 2)
    cdef np.ndarray[double, ndim=1] axis = np.array([rot_diff[2, 1] - rot_diff[1, 2],
                                                     rot_diff[0, 2] - rot_diff[2, 0],
                                                     rot_diff[1, 0] - rot_diff[0, 1]])
    axis /= (2 * sin(angle) + 1e-10)  # Add small value to avoid division by zero
    cdef np.ndarray[double, ndim=1] angular_velocity = (angle / dt) * axis

    return np.concatenate([linear_velocity, angular_velocity])

@cython.boundscheck(False)
@cython.wraparound(False)
def limit_velocity(np.ndarray[double, ndim=1] velocities, double max_velocity):
    return np.clip(velocities, -max_velocity, max_velocity)

@cython.boundscheck(False)
@cython.wraparound(False)
def limit_acceleration(np.ndarray[double, ndim=1] current_velocities, np.ndarray[double, ndim=1] prev_velocities, double max_acceleration, double dt):
    cdef np.ndarray[double, ndim=1] accelerations = (current_velocities - prev_velocities) / dt
    cdef np.ndarray[double, ndim=1] limited_accelerations = np.clip(accelerations, -max_acceleration, max_acceleration)
    return prev_velocities + limited_accelerations * dt
    