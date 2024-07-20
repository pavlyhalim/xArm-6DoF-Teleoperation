# filters.pyx
cimport numpy as np
import numpy as np
from libc.math cimport sqrt, pow

cdef class ExtendedKalmanFilter:
    cdef:
        double[:] state
        double[:,:] P
        double[:,:] Q
        double[:,:] R
        int state_dim
        int measurement_dim

    def __init__(self, np.ndarray[double, ndim=1] initial_state, np.ndarray[double, ndim=2] initial_P, np.ndarray[double, ndim=2] Q, np.ndarray[double, ndim=2] R):
        self.state = initial_state
        self.P = initial_P
        self.Q = Q
        self.R = R
        self.state_dim = initial_state.shape[0]
        self.measurement_dim = R.shape[0]

    cdef np.ndarray[double, ndim=1] state_transition(self, np.ndarray[double, ndim=1] state):
        # Implement your state transition function here
        # For simplicity, we'll use a constant velocity model
        cdef np.ndarray[double, ndim=1] new_state = np.zeros(self.state_dim, dtype=np.float64)
        cdef int i
        for i in range(0, self.state_dim, 2):
            new_state[i] = state[i] + state[i+1]
            new_state[i+1] = state[i+1]
        return new_state

    cdef np.ndarray[double, ndim=2] jacobian(self, np.ndarray[double, ndim=1] state):
        # Implement the Jacobian of the state transition function
        cdef np.ndarray[double, ndim=2] J = np.eye(self.state_dim, dtype=np.float64)
        cdef int i
        for i in range(0, self.state_dim, 2):
            J[i, i+1] = 1.0
        return J

    cpdef np.ndarray[double, ndim=1] update(self, np.ndarray[double, ndim=1] measurement):
        # Prediction
        cdef np.ndarray[double, ndim=1] predicted_state = self.state_transition(np.asarray(self.state))
        cdef np.ndarray[double, ndim=2] F = self.jacobian(np.asarray(self.state))
        cdef np.ndarray[double, ndim=2] predicted_P = np.dot(np.dot(F, self.P), F.T) + self.Q

        # Update
        cdef np.ndarray[double, ndim=1] y = measurement - predicted_state[:self.measurement_dim]
        cdef np.ndarray[double, ndim=2] H = np.zeros((self.measurement_dim, self.state_dim), dtype=np.float64)
        H[:, :self.measurement_dim] = np.eye(self.measurement_dim)
        
        cdef np.ndarray[double, ndim=2] S = np.dot(np.dot(H, predicted_P), H.T) + self.R
        cdef np.ndarray[double, ndim=2] K = np.dot(np.dot(predicted_P, H.T), np.linalg.inv(S))

        self.state = predicted_state + np.dot(K, y)
        self.P = predicted_P - np.dot(np.dot(K, H), predicted_P)

        return np.asarray(self.state)

cdef class LowPassFilter:
    cdef double alpha
    cdef list prev_output

    def __init__(self, double alpha):
        self.alpha = alpha
        self.prev_output = None

    cpdef np.ndarray[double, ndim=1] update(self, np.ndarray[double, ndim=1] input_data):
        if self.prev_output is None:
            self.prev_output = input_data.tolist()
        else:
            self.prev_output = (self.alpha * input_data + (1 - self.alpha) * np.array(self.prev_output)).tolist()
        return np.array(self.prev_output)