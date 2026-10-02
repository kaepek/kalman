cdef extern from "python/kalman_jerk_python.hpp" namespace "kaepek::python":
    cdef cppclass KalmanJerk2DPython:
        KalmanJerk2DPython(double, double, double, bint, double, int) except +
        void step(double, const double *)
        void step_covariance(double, const double *, const double *)
        const double * kalman_vector()
        const double * covariance_matrix()
        const double * eular_vector()
        const double * innovation()
        const double * innovation_covariance()
        const double * innovation_covariance_inverse()
        double innovation_covariance_determinant()

    cdef cppclass KalmanJerk3DPython:
        KalmanJerk3DPython(double, double, double, bint, double, int) except +
        void step(double, const double *)
        void step_covariance(double, const double *, const double *)
        const double * kalman_vector()
        const double * covariance_matrix()
        const double * eular_vector()
        const double * innovation()
        const double * innovation_covariance()
        const double * innovation_covariance_inverse()
        double innovation_covariance_determinant()

    cdef cppclass KalmanJerk2DPolarPython:
        KalmanJerk2DPolarPython(double, double, double, double, bint, double, int) except +
        void step(double, double, double)
        const double * kalman_vector()
        const double * covariance_matrix()
        const double * innovation()
        const double * innovation_covariance()
        const double * innovation_covariance_inverse()
        double innovation_covariance_determinant()

    cdef cppclass KalmanJerk3DSphericalPython:
        KalmanJerk3DSphericalPython(double, double, double, double, double, bint, double, int) except +
        void step(double, double, double, double)
        const double * kalman_vector()
        const double * covariance_matrix()
        const double * innovation()
        const double * innovation_covariance()
        const double * innovation_covariance_inverse()
        double innovation_covariance_determinant()

    cdef cppclass KalmanJerk2DAzElPython:
        KalmanJerk2DAzElPython(double, double, double, bint, double, int, int, int) except +
        void step(double, double, double)
        void step_covariance(double, double, double, const double *)
        const double * kalman_vector()
        const double * covariance_matrix()
        const double * state_vector()
        const double * basis()
        const double * basis_covariance_matrix()
        bint azimuth_defined()
        const double * innovation()
        const double * innovation_covariance()
        const double * innovation_covariance_inverse()
        double innovation_covariance_determinant()

    cdef cppclass KalmanJerk1DBearingMovingSensorPython:
        KalmanJerk1DBearingMovingSensorPython(double, double, double, bint, double, double, double, double, double, double, int, int) except +
        void step(double, double, const double *)
        double range()
        const double * kalman_vector()
        const double * covariance_matrix()
        const double * innovation()
        const double * innovation_covariance()
        const double * innovation_covariance_inverse()
        double innovation_covariance_determinant()

    cdef cppclass KalmanJerk2DAzElMovingSensorPython:
        KalmanJerk2DAzElMovingSensorPython(double, double, double, bint, double, double, double, double, double, double, int, int) except +
        void step(double, double, double, const double *)
        void step_covariance(double, double, double, const double *, const double *)
        const double * state_vector()
        const double * basis()
        double range()
        const double * kalman_vector()
        const double * covariance_matrix()
        const double * innovation()
        const double * innovation_covariance()
        const double * innovation_covariance_inverse()
        double innovation_covariance_determinant()

FORMS = {'small_alpha_t': 0, 'exact': 1}
ORDERS = {'first': 0, 'second': 1, 'unscented': 2}

def _form_code(form):
    if form not in FORMS:
        raise ValueError("form must be one of " + ", ".join(FORMS))
    return FORMS[form]

def _order_code(order):
    if order not in ORDERS:
        raise ValueError("order must be one of " + ", ".join(ORDERS))
    return ORDERS[order]

cdef _vector(const double * data, int n):
    cdef np.ndarray[double, ndim=1, mode='c'] out = np.empty([n])
    cdef int i
    for i in range(n):
        out[i] = data[i]
    return out

cdef _matrix(const double * data, int n):
    cdef np.ndarray[double, ndim=2, mode='c'] out = np.empty([n, n])
    cdef int i, k
    for i in range(n):
        for k in range(n):
            out[i, k] = data[n * i + k]
    return out

cdef class KalmanJerk2D:
    cdef KalmanJerk2DPython* c_kalman

    def __cinit__(self, double alpha, double x_resolution_error, double x_jerk_error, bint time_is_relative = False, double x_acceleration_error = 0.0, form = 'small_alpha_t'):
        self.c_kalman = new KalmanJerk2DPython(alpha, x_resolution_error, x_jerk_error, time_is_relative, x_acceleration_error, _form_code(form))

    def __dealloc__(self):
        del self.c_kalman

    def step(self, double time, x, R = None):
        cdef np.ndarray[double, ndim=1, mode='c'] xv = np.ascontiguousarray(x, dtype=np.float64)
        cdef np.ndarray[double, ndim=2, mode='c'] Rv
        if R is None:
            self.c_kalman.step(time, &xv[0])
        else:
            Rv = np.ascontiguousarray(R, dtype=np.float64)
            self.c_kalman.step_covariance(time, &xv[0], &Rv[0, 0])

    def get_kalman_vector(self):
        return _vector(self.c_kalman.kalman_vector(), 8)

    def get_covariance_matrix(self):
        return _matrix(self.c_kalman.covariance_matrix(), 8)

    def get_eular_vector(self):
        return _vector(self.c_kalman.eular_vector(), 9)

    def get_innovation(self):
        return _vector(self.c_kalman.innovation(), 2)

    def get_innovation_covariance(self):
        return _matrix(self.c_kalman.innovation_covariance(), 2)

    def get_innovation_covariance_inverse(self):
        return _matrix(self.c_kalman.innovation_covariance_inverse(), 2)

    def get_innovation_covariance_determinant(self):
        return self.c_kalman.innovation_covariance_determinant()

cdef class KalmanJerk3D:
    cdef KalmanJerk3DPython* c_kalman

    def __cinit__(self, double alpha, double x_resolution_error, double x_jerk_error, bint time_is_relative = False, double x_acceleration_error = 0.0, form = 'small_alpha_t'):
        self.c_kalman = new KalmanJerk3DPython(alpha, x_resolution_error, x_jerk_error, time_is_relative, x_acceleration_error, _form_code(form))

    def __dealloc__(self):
        del self.c_kalman

    def step(self, double time, x, R = None):
        cdef np.ndarray[double, ndim=1, mode='c'] xv = np.ascontiguousarray(x, dtype=np.float64)
        cdef np.ndarray[double, ndim=2, mode='c'] Rv
        if R is None:
            self.c_kalman.step(time, &xv[0])
        else:
            Rv = np.ascontiguousarray(R, dtype=np.float64)
            self.c_kalman.step_covariance(time, &xv[0], &Rv[0, 0])

    def get_kalman_vector(self):
        return _vector(self.c_kalman.kalman_vector(), 12)

    def get_covariance_matrix(self):
        return _matrix(self.c_kalman.covariance_matrix(), 12)

    def get_eular_vector(self):
        return _vector(self.c_kalman.eular_vector(), 13)

    def get_innovation(self):
        return _vector(self.c_kalman.innovation(), 3)

    def get_innovation_covariance(self):
        return _matrix(self.c_kalman.innovation_covariance(), 3)

    def get_innovation_covariance_inverse(self):
        return _matrix(self.c_kalman.innovation_covariance_inverse(), 3)

    def get_innovation_covariance_determinant(self):
        return self.c_kalman.innovation_covariance_determinant()

cdef class KalmanJerk2DPolar:
    cdef KalmanJerk2DPolarPython* c_kalman

    def __cinit__(self, double alpha, double range_error, double angle_error, double jerk_error, bint time_is_relative = False, double acceleration_error = 0.0, form = 'small_alpha_t'):
        self.c_kalman = new KalmanJerk2DPolarPython(alpha, range_error, angle_error, jerk_error, time_is_relative, acceleration_error, _form_code(form))

    def __dealloc__(self):
        del self.c_kalman

    def step(self, double time, double range, double angle):
        self.c_kalman.step(time, range, angle)

    def get_kalman_vector(self):
        return _vector(self.c_kalman.kalman_vector(), 8)

    def get_covariance_matrix(self):
        return _matrix(self.c_kalman.covariance_matrix(), 8)

    def get_innovation(self):
        return _vector(self.c_kalman.innovation(), 2)

    def get_innovation_covariance(self):
        return _matrix(self.c_kalman.innovation_covariance(), 2)

    def get_innovation_covariance_inverse(self):
        return _matrix(self.c_kalman.innovation_covariance_inverse(), 2)

    def get_innovation_covariance_determinant(self):
        return self.c_kalman.innovation_covariance_determinant()

cdef class KalmanJerk3DSpherical:
    cdef KalmanJerk3DSphericalPython* c_kalman

    def __cinit__(self, double alpha, double range_error, double azimuth_error, double elevation_error, double jerk_error, bint time_is_relative = False, double acceleration_error = 0.0, form = 'small_alpha_t'):
        self.c_kalman = new KalmanJerk3DSphericalPython(alpha, range_error, azimuth_error, elevation_error, jerk_error, time_is_relative, acceleration_error, _form_code(form))

    def __dealloc__(self):
        del self.c_kalman

    def step(self, double time, double range, double azimuth, double elevation):
        self.c_kalman.step(time, range, azimuth, elevation)

    def get_kalman_vector(self):
        return _vector(self.c_kalman.kalman_vector(), 12)

    def get_covariance_matrix(self):
        return _matrix(self.c_kalman.covariance_matrix(), 12)

    def get_innovation(self):
        return _vector(self.c_kalman.innovation(), 3)

    def get_innovation_covariance(self):
        return _matrix(self.c_kalman.innovation_covariance(), 3)

    def get_innovation_covariance_inverse(self):
        return _matrix(self.c_kalman.innovation_covariance_inverse(), 3)

    def get_innovation_covariance_determinant(self):
        return self.c_kalman.innovation_covariance_determinant()

cdef class KalmanJerk2DAzEl:
    cdef KalmanJerk2DAzElPython* c_kalman

    def __cinit__(self, double alpha, double direction_error, double jerk_error, bint time_is_relative = False, double acceleration_error = 0.0, int substeps = 4, form = 'small_alpha_t', order = 'first'):
        self.c_kalman = new KalmanJerk2DAzElPython(alpha, direction_error, jerk_error, time_is_relative, acceleration_error, substeps, _form_code(form), _order_code(order))

    def __dealloc__(self):
        del self.c_kalman

    def step(self, double time, double azimuth, double elevation, R = None):
        cdef np.ndarray[double, ndim=2, mode='c'] Rv
        if R is None:
            self.c_kalman.step(time, azimuth, elevation)
        else:
            Rv = np.ascontiguousarray(R, dtype=np.float64)
            self.c_kalman.step_covariance(time, azimuth, elevation, &Rv[0, 0])

    @staticmethod
    def azel_noise_to_tangent(double elevation, double azimuth_error, double elevation_error):
        c = np.cos(elevation)
        return np.array([[azimuth_error * azimuth_error * c * c, 0.0], [0.0, elevation_error * elevation_error]])

    def azimuth_defined(self):
        return self.c_kalman.azimuth_defined()

    def get_kalman_vector(self):
        return _vector(self.c_kalman.kalman_vector(), 8)

    def get_covariance_matrix(self):
        return _matrix(self.c_kalman.covariance_matrix(), 8)

    def get_state_vector(self):
        return _vector(self.c_kalman.state_vector(), 12)

    def get_basis(self):
        return _vector(self.c_kalman.basis(), 6)

    def get_basis_covariance_matrix(self):
        return _matrix(self.c_kalman.basis_covariance_matrix(), 8)

    def get_innovation(self):
        return _vector(self.c_kalman.innovation(), 2)

    def get_innovation_covariance(self):
        return _matrix(self.c_kalman.innovation_covariance(), 2)

    def get_innovation_covariance_inverse(self):
        return _matrix(self.c_kalman.innovation_covariance_inverse(), 2)

    def get_innovation_covariance_determinant(self):
        return self.c_kalman.innovation_covariance_determinant()

cdef class KalmanJerk1DBearingMovingSensor:
    cdef KalmanJerk1DBearingMovingSensorPython* c_kalman

    def __cinit__(self, double alpha, double bearing_error, double jerk_error, bint time_is_relative, double range_min, double range_max,
                  double log_range_rate_error, double log_range_acceleration_error, double log_range_jerk_error, double acceleration_error = 0.0,
                  form = 'small_alpha_t', order = 'first'):
        self.c_kalman = new KalmanJerk1DBearingMovingSensorPython(alpha, bearing_error, jerk_error, time_is_relative, range_min, range_max,
                                                                  log_range_rate_error, log_range_acceleration_error, log_range_jerk_error,
                                                                  acceleration_error, _form_code(form), _order_code(order))

    def __dealloc__(self):
        del self.c_kalman

    def step(self, double time, double bearing, sensor):
        cdef np.ndarray[double, ndim=1, mode='c'] s = np.ascontiguousarray(sensor, dtype=np.float64)
        self.c_kalman.step(time, bearing, &s[0])

    def get_range(self):
        return self.c_kalman.range()

    def get_kalman_vector(self):
        return _vector(self.c_kalman.kalman_vector(), 8)

    def get_covariance_matrix(self):
        return _matrix(self.c_kalman.covariance_matrix(), 8)

    def get_innovation(self):
        return _vector(self.c_kalman.innovation(), 1)

    def get_innovation_covariance(self):
        return _matrix(self.c_kalman.innovation_covariance(), 1)

    def get_innovation_covariance_inverse(self):
        return _matrix(self.c_kalman.innovation_covariance_inverse(), 1)

    def get_innovation_covariance_determinant(self):
        return self.c_kalman.innovation_covariance_determinant()

cdef class KalmanJerk2DAzElMovingSensor:
    cdef KalmanJerk2DAzElMovingSensorPython* c_kalman

    def __cinit__(self, double alpha, double direction_error, double jerk_error, bint time_is_relative, double range_min, double range_max,
                  double log_range_rate_error, double log_range_acceleration_error, double log_range_jerk_error, double acceleration_error = 0.0,
                  form = 'small_alpha_t', order = 'first'):
        self.c_kalman = new KalmanJerk2DAzElMovingSensorPython(alpha, direction_error, jerk_error, time_is_relative, range_min, range_max,
                                                               log_range_rate_error, log_range_acceleration_error, log_range_jerk_error,
                                                               acceleration_error, _form_code(form), _order_code(order))

    def __dealloc__(self):
        del self.c_kalman

    def step(self, double time, double azimuth, double elevation, sensor, R = None):
        cdef np.ndarray[double, ndim=1, mode='c'] s = np.ascontiguousarray(sensor, dtype=np.float64)
        cdef np.ndarray[double, ndim=2, mode='c'] Rv
        if R is None:
            self.c_kalman.step(time, azimuth, elevation, &s[0])
        else:
            Rv = np.ascontiguousarray(R, dtype=np.float64)
            self.c_kalman.step_covariance(time, azimuth, elevation, &s[0], &Rv[0, 0])

    def get_range(self):
        return self.c_kalman.range()

    def get_kalman_vector(self):
        return _vector(self.c_kalman.kalman_vector(), 12)

    def get_covariance_matrix(self):
        return _matrix(self.c_kalman.covariance_matrix(), 12)

    def get_state_vector(self):
        return _vector(self.c_kalman.state_vector(), 12)

    def get_basis(self):
        return _vector(self.c_kalman.basis(), 6)

    def get_innovation(self):
        return _vector(self.c_kalman.innovation(), 2)

    def get_innovation_covariance(self):
        return _matrix(self.c_kalman.innovation_covariance(), 2)

    def get_innovation_covariance_inverse(self):
        return _matrix(self.c_kalman.innovation_covariance_inverse(), 2)

    def get_innovation_covariance_determinant(self):
        return self.c_kalman.innovation_covariance_determinant()
