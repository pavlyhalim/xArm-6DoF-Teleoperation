# filters.pyx

cimport numpy as np
import numpy as np
from libc.math cimport sin, cos, atan2, M_PI

np.import_array()


cdef class MovingAverageFilter:
    cdef int window_size
    cdef int vector_size
    cdef np.ndarray buffer
    cdef int current_index
    cdef bint is_filled

    def __init__(self, int window_size, int vector_size):
        self.window_size = window_size
        self.vector_size = vector_size
        self.buffer = np.zeros((window_size, vector_size), dtype=np.double)
        self.current_index = 0
        self.is_filled = False

    cpdef np.ndarray update(self, np.ndarray[double, ndim=1] new_value):
        self.buffer[self.current_index] = new_value
        self.current_index = (self.current_index + 1) % self.window_size
        
        if self.current_index == 0:
            self.is_filled = True
        
        if self.is_filled:
            return np.mean(self.buffer, axis=0)
        else:
            return np.mean(self.buffer[:self.current_index], axis=0)

cdef class LowPassFilter:
    cdef double alpha
    cdef np.ndarray state

    def __init__(self, double alpha, int size):
        self.alpha = alpha
        self.state = np.zeros(size, dtype=np.double)

    cpdef np.ndarray update(self, np.ndarray[double, ndim=1] new_value):
        self.state = self.alpha * new_value + (1 - self.alpha) * self.state
        return self.state

cdef class HysteresisFilter:
    cdef double max_val
    cdef double min_val
    cdef bint state

    def __init__(self, double max_val, double min_val):
        self.max_val = max_val
        self.min_val = min_val
        self.state = False

    cpdef bint update(self, double val):
        if val > self.max_val:
            self.state = True
        elif val < self.min_val:
            self.state = False
        return self.state