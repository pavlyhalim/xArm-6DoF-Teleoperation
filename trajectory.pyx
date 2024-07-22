# trajectory.pyx
cimport numpy as np
import numpy as np
from libc.math cimport pow


cpdef np.ndarray[double, ndim=1] limit_joint_angles(np.ndarray angles, np.ndarray min_angles, np.ndarray max_angles):
    cdef np.ndarray[double, ndim=1] angles_f64 = np.asarray(angles, dtype=np.float64)
    cdef np.ndarray[double, ndim=1] min_angles_f64 = np.asarray(min_angles, dtype=np.float64)
    cdef np.ndarray[double, ndim=1] max_angles_f64 = np.asarray(max_angles, dtype=np.float64)
    return np.clip(angles_f64, min_angles_f64, max_angles_f64)
