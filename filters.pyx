# filters.pyx
import cython
import numpy as np
cimport numpy as np
from libc.math cimport M_PI, sin, cos, atan2
from collections import deque

cdef class MovingAverageFilter:
    cdef:
        int window_size
        int vector_size
        object angles_deque  # This will be a Python deque object

    def __cinit__(self, int window_size, int vector_size):
        self.window_size = window_size
        self.vector_size = vector_size
        self.angles_deque = deque(maxlen=window_size)

    @cython.boundscheck(False)
    @cython.wraparound(False)
    cpdef void add_new_values(self, np.ndarray[double, ndim=1] angles):
        self.angles_deque.append(angles)

    @cython.boundscheck(False)
    @cython.wraparound(False)
    cpdef np.ndarray[double, ndim=1] get_filtered_values(self):
        cdef int i, j
        cdef np.ndarray[double, ndim=1] sum_cos = np.zeros(self.vector_size)
        cdef np.ndarray[double, ndim=1] sum_sin = np.zeros(self.vector_size)
        cdef np.ndarray[double, ndim=1] angles

        for angles in self.angles_deque:
            for j in range(self.vector_size):
                sum_cos[j] += cos(angles[j] * M_PI / 180.0)
                sum_sin[j] += sin(angles[j] * M_PI / 180.0)

        cdef np.ndarray[double, ndim=1] result = np.zeros(self.vector_size)
        for j in range(self.vector_size):
            result[j] = atan2(sum_sin[j], sum_cos[j]) * 180.0 / M_PI

        return result

cdef class HysteresisFilter:
    cdef:
        double max_val
        double min_val
        bint state

    def __cinit__(self, double max_val, double min_val):
        self.max_val = max_val
        self.min_val = min_val
        self.state = False

    @cython.boundscheck(False)
    @cython.wraparound(False)
    cpdef bint filter(self, double val):
        if val > self.max_val and not self.state:
            self.state = True
        elif val < self.min_val and self.state:
            self.state = False
        return self.state