# distutils: language = c++

import numpy as np
cimport numpy as np
from kalman_jerk cimport KalmanJerk1D as _KalmanJerk1D
from libcpp cimport bool as bool_t

# Create a Cython extension type which holds a C++ instance
# as an attribute and create a bunch of forwarding methods
# Python extension type.

cdef class KalmanJerk1D:
    cdef _KalmanJerk1D* c_kalman  # Hold a C++ instance which we're wrapping
    cdef bint boolean_variable

    def __cinit__(self, double alpha, double x_resolution_error, double x_jerk_error, bint time_is_relative = False, double x_mod_limit =-1):
        self.boolean_variable = False

        if x_mod_limit == -1:
            self.c_kalman = new _KalmanJerk1D(alpha, x_resolution_error, x_jerk_error, time_is_relative)
        else:
            self.c_kalman = new _KalmanJerk1D(alpha, x_resolution_error, x_jerk_error, time_is_relative, x_mod_limit)

    def __dealloc__(self):
        del self.c_kalman

    def get_kalman_vector(self):
        cdef np.ndarray[double, ndim=1, mode='c'] X = np.empty([4])
        # cdef double[5] X = [0.0, 0.0, 0.0, 0.0]
        _X = self.c_kalman.get_kalman_vector()
        X[0] = _X[0]
        X[1] = _X[1]
        X[2] = _X[2]
        X[3] = _X[3]
        return X 

    def get_eular_vector(self):
        cdef np.ndarray[double, ndim=1, mode='c'] eular  = np.empty([5])
        _eular = self.c_kalman.get_eular_vector()
        eular[0] = _eular[0]
        eular[1] = _eular[1]
        eular[2] = _eular[2]
        eular[3] = _eular[3]
        eular[4] = _eular[4]
        return eular
    
    def get_covariance_matrix(self):
        cdef np.ndarray[double, ndim=2, mode='c'] P = np.empty([4,4])
        _P = self.c_kalman.get_covariance_matrix()
        P[0,0] = _P[0][0]
        P[0,1] = _P[0][1]
        P[0,2] = _P[0][2]
        P[0,3] = _P[0][3]

        P[1,0] = _P[1][0]
        P[1,0] = _P[1][1]
        P[1,0] = _P[1][2]
        P[1,0] = _P[1][3]

        P[2,0] = _P[2][0]
        P[2,0] = _P[2][1]
        P[2,0] = _P[2][2]
        P[2,0] = _P[2][3]

        P[3,0] = _P[3][0]
        P[3,0] = _P[3][1]
        P[3,0] = _P[3][2]
        P[3,0] = _P[3][3]
        return P

    def step(self, double time, double x):
        self.c_kalman.step(time, x)