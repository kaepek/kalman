"""
Jerk filter for N Cartesian axes, Section V of [Ref1], and the range and angle filters built on it with
(44) and (45) of [Ref1].

State per axis [position, velocity, acceleration, jerk], stacked by axis.
F_N = I_N (x) F, Q_N = I_N (x) Q, H selects the positions, R is a full N by N measurement covariance.
"""

import torch
from .jerk import check_form, transition, init_coefficients, init_estimate, init_process_covariance, kalman_update, StepTimer, Diagnostics
from .generated import polar_spherical

class KalmanJerkCartesian(Diagnostics, torch.nn.Module):
    """
    @param N number of axes
    @param alpha alpha used to calculate q factor with q = 2 * alpha * x_jerk_error^2
    @param x_resolution_error The standard deviation of each measured coordinate, used when no measurement covariance is given
    @param x_jerk_error The standard deviation of the jerk of each coordinate
    @param time_is_relative Indicates whether time values parsed to the step function are absolute temporal values or temporal differences between consecutive steps.
    @param x_acceleration_error The standard deviation of the acceleration of each coordinate, used by the exact initialisation (28)
    @param form 'small_alpha_t' or 'exact'
    """

    def __init__(self, N, alpha, x_resolution_error, x_jerk_error, time_is_relative=False, x_acceleration_error=0.0,
                 form="small_alpha_t", dtype=torch.float64, device=None):
        super().__init__()
        self.N = N
        self.alpha = float(alpha)
        self.x_variance = float(x_resolution_error) ** 2
        self.jerk_variance = float(x_jerk_error) ** 2
        self.acceleration_variance = float(x_acceleration_error) ** 2
        self.q_scale = 2.0 * self.alpha * self.jerk_variance
        self.form = check_form(form)
        self.timer = StepTimer(time_is_relative)
        self.current_idx = -1
        self.dts = [0.0, 0.0]
        self.measurements = []
        self.measurement_covariances = []
        self.register_buffer("X", torch.zeros(4 * N, dtype=dtype, device=device))
        self.register_buffer("P", torch.zeros(4 * N, 4 * N, dtype=dtype, device=device))
        self.register_buffer("eular_state", torch.zeros(1 + 4 * N, dtype=dtype, device=device))
        self.register_diagnostics(N, dtype, device)

    def tensor(self, value):
        return torch.as_tensor(value, dtype=self.X.dtype, device=self.X.device)

    def initialise(self):
        N, dt1, dt2 = self.N, self.dts[0], self.dts[1]
        m = torch.stack(self.measurements)
        self.X = init_estimate(m[0], m[1], m[2], dt1, dt2).reshape(-1)
        c = init_coefficients(dt1, dt2, self.X)
        P_proc = init_process_covariance(dt1, dt2, self.alpha, self.q_scale, self.acceleration_variance, self.jerk_variance, self.form, self.X)
        R = torch.stack(self.measurement_covariances)
        blocks = torch.einsum("nik,nr,ns->irks", R, c, c)
        blocks = blocks + torch.einsum("ik,rs->irks", torch.eye(N, dtype=self.X.dtype, device=self.X.device), P_proc)
        self.P = blocks.reshape(4 * N, 4 * N)

    def predict(self, dt):
        F, Q = transition(self.tensor(dt), self.alpha, self.form)
        I = torch.eye(self.N, dtype=self.X.dtype, device=self.X.device)
        F_N = torch.kron(I, F)
        self.X = F_N @ self.X
        self.P = F_N @ self.P @ F_N.T + torch.kron(I, self.q_scale * Q)

    def update(self, x, R):
        idx = list(range(0, 4 * self.N, 4))
        y = x - self.X[idx]
        ok, correction, self.P, S, S_inv, det_S = kalman_update(self.P, idx, y, R)
        self.X = self.X + correction
        self.store_diagnostics(ok, y, S, S_inv, det_S)

    def update_eular(self, dt, x):
        e = self.eular_state[1:].reshape(self.N, 4).clone()
        v = (x - e[:, 0]) / dt
        a = (v - e[:, 1]) / dt
        j = (a - e[:, 2]) / dt
        e[:, 0] = x
        e[:, 1] = v
        e[:, 2] = a if self.current_idx >= 1 else 0.0
        e[:, 3] = j if self.current_idx >= 2 else 0.0
        self.eular_state = torch.cat([self.tensor([self.timer.time]), e.reshape(-1)])

    def step(self, time_or_dt, x, R=None):
        """
        @param time_or_dt absolute time or relative time since last step (depending on how time_is_relative is set)
        @param x measured coordinates
        @param R measurement covariance of x, x_resolution_error^2 I when not given
        """
        x = self.tensor(x)
        R = self.x_variance * torch.eye(self.N, dtype=self.X.dtype, device=self.X.device) if R is None else self.tensor(R)
        dt = self.timer.advance(time_or_dt, self.current_idx == -1)
        if dt is None:
            self.eular_state = torch.cat([self.tensor([self.timer.time]), torch.stack([x] + [torch.zeros_like(x)] * 3, -1).reshape(-1)])
        else:
            self.update_eular(dt, x)
            if self.current_idx < 2:
                self.dts[self.current_idx] = dt
            else:
                self.predict(self.tensor(dt))
                self.update(x, R)
        if self.current_idx < 2:
            self.measurements.append(x)
            self.measurement_covariances.append(R)
            if self.current_idx + 1 == 2:
                self.initialise()
            self.current_idx += 1

    def forward(self, time_or_dt, x, R=None):
        """Steps the filter and returns the Kalman state estimate."""
        self.step(time_or_dt, x, R)
        return self.X

    def get_kalman_vector(self):
        """Kalman state estimate, per axis [position, velocity, acceleration, jerk]."""
        return self.X

    def get_covariance_matrix(self):
        return self.P

    def get_eular_vector(self):
        """Eular state estimate [time, then per axis position, velocity, acceleration, jerk]."""
        return self.eular_state

class KalmanJerk2D(KalmanJerkCartesian):
    def __init__(self, alpha, x_resolution_error, x_jerk_error, time_is_relative=False, x_acceleration_error=0.0,
                 form="small_alpha_t", dtype=torch.float64, device=None):
        super().__init__(2, alpha, x_resolution_error, x_jerk_error, time_is_relative, x_acceleration_error, form, dtype, device)

class KalmanJerk3D(KalmanJerkCartesian):
    def __init__(self, alpha, x_resolution_error, x_jerk_error, time_is_relative=False, x_acceleration_error=0.0,
                 form="small_alpha_t", dtype=torch.float64, device=None):
        super().__init__(3, alpha, x_resolution_error, x_jerk_error, time_is_relative, x_acceleration_error, form, dtype, device)

class KalmanJerk2DPolar(KalmanJerk2D):
    """
    Point in a plane measured as range and angle. The measurement and its covariance are converted to
    Cartesian coordinates. State [x, vx, ax, jx, y, vy, ay, jy].
    """

    def __init__(self, alpha, range_error, angle_error, jerk_error, time_is_relative=False, acceleration_error=0.0,
                 form="small_alpha_t", dtype=torch.float64, device=None):
        super().__init__(alpha, 0.0, jerk_error, time_is_relative, acceleration_error, form, dtype, device)
        self.range_variance = float(range_error) ** 2
        self.angle_variance = float(angle_error) ** 2

    def step(self, time_or_dt, range, angle):
        r, az = self.tensor(range), self.tensor(angle)
        M = polar_spherical.polar_position(r, az)
        R = polar_spherical.polar_covariance(r, az, self.tensor(self.range_variance), self.tensor(self.angle_variance))
        super().step(time_or_dt, M, R)

    def forward(self, time_or_dt, range, angle):
        self.step(time_or_dt, range, angle)
        return self.X

class KalmanJerk3DSpherical(KalmanJerk3D):
    """
    Point in space measured as range, azimuth and elevation. The measurement and its covariance are converted
    to Cartesian coordinates. State [x, vx, ax, jx, y, vy, ay, jy, z, vz, az, jz].
    """

    def __init__(self, alpha, range_error, azimuth_error, elevation_error, jerk_error, time_is_relative=False, acceleration_error=0.0,
                 form="small_alpha_t", dtype=torch.float64, device=None):
        super().__init__(alpha, 0.0, jerk_error, time_is_relative, acceleration_error, form, dtype, device)
        self.variances = [float(range_error) ** 2, float(azimuth_error) ** 2, float(elevation_error) ** 2]

    def step(self, time_or_dt, range, azimuth, elevation):
        r, az, el = self.tensor(range), self.tensor(azimuth), self.tensor(elevation)
        M = polar_spherical.spherical_position(r, az, el)
        R = polar_spherical.spherical_covariance(r, az, el, *[self.tensor(v) for v in self.variances])
        super().step(time_or_dt, M, R)

    def forward(self, time_or_dt, range, azimuth, elevation):
        self.step(time_or_dt, range, azimuth, elevation)
        return self.X
