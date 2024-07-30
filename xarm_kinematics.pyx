# xarm_kinematics.pyx
import cython
import numpy as np
cimport numpy as np
from libc.math cimport M_PI, sin, cos, atan2, acos, sqrt, fmod, pow


cpdef np.ndarray[double, ndim=1] limit_joint_angles(np.ndarray angles, np.ndarray min_angles, np.ndarray max_angles):
    cdef np.ndarray[double, ndim=1] angles_f64 = np.asarray(angles, dtype=np.float64)
    cdef np.ndarray[double, ndim=1] min_angles_f64 = np.asarray(min_angles, dtype=np.float64)
    cdef np.ndarray[double, ndim=1] max_angles_f64 = np.asarray(max_angles, dtype=np.float64)
    return np.clip(angles_f64, min_angles_f64, max_angles_f64)


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
    xarm_angles[4] = angles[4] if len(angles) > 4 else 0  # Wrist 2
    xarm_angles[5] = angles[5] if len(angles) > 5 else 0  # Wrist 3

    # Normalize angles to [-180, 180] range
    for i in range(6):
        xarm_angles[i] = fmod((xarm_angles[i] + 180.0), 360.0) - 180.0

    return xarm_angles

    