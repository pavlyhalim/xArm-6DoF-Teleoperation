# filters.pyx

cimport numpy as np
import numpy as np
from libc.math cimport sin, cos, atan2, M_PI

np.import_array()

cdef class MovingAverageFilter:
    cdef int window_size
    cdef int vector_size
    cdef list angles_deque
    cdef list velocities_deque

    def __init__(self, int window_size, int vector_size):
        self.window_size = window_size
        self.vector_size = vector_size
        self.angles_deque = []
        self.velocities_deque = []

    def add_new_values(self, np.ndarray[double, ndim=1] angles, np.ndarray[double, ndim=1] velocities):
        self.angles_deque.append(angles)
        self.velocities_deque.append(velocities)
        if len(self.angles_deque) > self.window_size:
            self.angles_deque.pop(0)
            self.velocities_deque.pop(0)

    def get_filtered_values(self):
        if not self.angles_deque:
            return np.zeros(self.vector_size, dtype=np.double)
        
        cdef np.ndarray[double, ndim=1] sum_cos = np.zeros(self.vector_size, dtype=np.double)
        cdef np.ndarray[double, ndim=1] sum_sin = np.zeros(self.vector_size, dtype=np.double)
        
        for angles in self.angles_deque:
            sum_cos += np.cos(np.radians(angles))
            sum_sin += np.sin(np.radians(angles))
        
        return np.degrees(np.arctan2(sum_sin, sum_cos))

cdef class HysteresisFilter:
    cdef double max_val
    cdef double min_val
    cdef bint state

    def __init__(self, double max_val, double min_val):
        self.max_val = max_val
        self.min_val = min_val
        self.state = False

    def filter(self, double val):
        if val < self.max_val and not self.state:
            self.state = False
        elif val > self.max_val and not self.state:
            self.state = True
        elif val > self.min_val and self.state:
            self.state = True
        elif val < self.min_val and self.state:
            self.state = False
        return self.state