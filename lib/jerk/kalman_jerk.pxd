cdef extern from "kalman_jerk.cpp":
    pass

ctypedef double * DoublePtr
ctypedef (double *)[4] Double4x4
ctypedef double ** Double4x4_2
ctypedef double[4][4] Double4x4_3

# Declare the class with cdef
cdef extern from "kalman_jerk.hpp" namespace "kaepek":
    cdef cppclass KalmanJerk1D:
        #Kalman1D() except +
        KalmanJerk1D(double,double,double,bint) except +
        KalmanJerk1D(double,double,double,bint,double) except +

        DoublePtr get_kalman_vector()
        DoublePtr get_eular_vector()
        Double4x4_3 get_covariance_matrix()
        void step(double time, double x)