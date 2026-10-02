"""
Jerk filters for a target measured as bearings, or as azimuth and elevation, from a sensor with known kinematic
state. The target follows the per axis jerk model of [Ref1].

KalmanJerk1DBearingMovingSensor: modified polar coordinates of jerk order,
Y = [beta, d beta, d2 beta, d3 beta, d lambda, d2 lambda, d3 lambda, 1/r], lambda = ln r.
KalmanJerk2DAzElMovingSensor: modified spherical coordinates of jerk order, the direction state of KalmanJerk2DAzEl,
[d lambda, d2 lambda, d3 lambda] and 1/r, with error [8 direction components, d lambda, d2 lambda, d3 lambda, 1/r].
See model/derivation.md.
"""

import math
import torch
from torch.func import vmap
from .jerk import check_form, check_order, transition, init_coefficients, init_estimate, init_process_covariance, kalman_update, StepTimer, Diagnostics
from .sphere import dot3, unit_from_azel, azel_from_unit, azel_frame, direction_retract, direction_inverse_retract, transport_basis, direction_normalise
from .direction import direction_initialise, direction_innovation
from .propagation import map_propagate
from .generated import mpc, msc

TWO_PI = 2.0 * math.pi

def wrapped_angle_difference(a, b):
    """Angle difference a - b mapped into (-pi, pi]."""
    raw = a.detach() - b
    turns = torch.floor(raw / TWO_PI + 0.5)
    turns = torch.where(raw - turns * TWO_PI <= -0.5 * TWO_PI, turns - 1.0, turns)
    return a - b - turns * TWO_PI

class MovingSensorFilter(Diagnostics, torch.nn.Module):
    """Parameters, time handling and diagnostics shared by the moving sensor filters."""

    def __init__(self, alpha, angle_error, jerk_error, time_is_relative, range_min, range_max, log_range_rate_error,
                 log_range_acceleration_error, log_range_jerk_error, acceleration_error, form, order, axes, error_size, measurement_size, dtype, device):
        super().__init__()
        self.alpha = float(alpha)
        self.q_scale = 2.0 * self.alpha * float(jerk_error) ** 2
        self.angle_variance = float(angle_error) ** 2
        self.jerk_error = float(jerk_error)
        self.acceleration_error = float(acceleration_error)
        self.range_min = float(range_min)
        self.range_max = float(range_max)
        self.log_range_variances = [float(log_range_rate_error) ** 2, float(log_range_acceleration_error) ** 2, float(log_range_jerk_error) ** 2]
        self.form = check_form(form)
        self.order = check_order(order)
        self.axes = axes
        self.timer = StepTimer(time_is_relative)
        self.current_idx = -1
        self.dts = [0.0, 0.0]
        self.measurements = []
        self.register_buffer("sensor_state", torch.zeros(4 * axes, dtype=dtype, device=device))
        self.register_buffer("P", torch.zeros(error_size, error_size, dtype=dtype, device=device))
        self.register_diagnostics(measurement_size, dtype, device)

    def tensor(self, value):
        return torch.as_tensor(value, dtype=self.P.dtype, device=self.P.device)

    def inverse_range_bounds(self):
        return 1.0 / self.range_max, 1.0 / self.range_min

    def prediction_terms(self, dt, sensor):
        """F of one axis, the sensor term F s(k) - s(k + 1) per axis and the process noise covariance of the relative state."""
        F, Q = transition(self.tensor(dt), self.alpha, self.form)
        I = torch.eye(self.axes, dtype=F.dtype, device=F.device)
        sensor_term = torch.kron(I, F) @ self.sensor_state - sensor
        return F, sensor_term, torch.kron(I, self.q_scale * Q)

    def get_covariance_matrix(self):
        return self.P

class KalmanJerk1DBearingMovingSensor(MovingSensorFilter):
    """
    @param alpha alpha used to calculate q factor with q = 2 * alpha * jerk_error^2
    @param bearing_error The standard deviation of a bearing measurement in radians
    @param jerk_error The standard deviation of the jerk of each Cartesian coordinate of the target
    @param time_is_relative Indicates whether time values parsed to the step function are absolute temporal values or temporal differences between consecutive steps.
    @param range_min Lower bound of the initial range
    @param range_max Upper bound of the initial range
    @param log_range_rate_error Initial standard deviation of d lambda, lambda = ln r
    @param log_range_acceleration_error Initial standard deviation of d2 lambda
    @param log_range_jerk_error Initial standard deviation of d3 lambda
    @param acceleration_error The standard deviation of the acceleration of each Cartesian coordinate, used by the exact initialisation (28)
    @param form 'small_alpha_t' or 'exact'
    @param order 'first', 'second' or 'unscented'
    """

    def __init__(self, alpha, bearing_error, jerk_error, time_is_relative, range_min, range_max, log_range_rate_error,
                 log_range_acceleration_error, log_range_jerk_error, acceleration_error=0.0, form="small_alpha_t", order="first",
                 dtype=torch.float64, device=None):
        super().__init__(alpha, bearing_error, jerk_error, time_is_relative, range_min, range_max, log_range_rate_error,
                         log_range_acceleration_error, log_range_jerk_error, acceleration_error, form, order, 2, 8, 1, dtype, device)
        self.register_buffer("Y", torch.zeros(8, dtype=dtype, device=device))

    def initialise(self):
        b = self.measurements
        m2 = b[0] + wrapped_angle_difference(b[1], b[0])
        m3 = m2 + wrapped_angle_difference(b[2], b[1])
        est = init_estimate(b[0], m2, m3, self.dts[0], self.dts[1])
        q_min, q_max = self.inverse_range_bounds()
        zero = torch.zeros_like(est[0])
        self.Y = torch.stack([wrapped_angle_difference(est[0], 0.0), est[1], est[2], zero, zero, zero, zero, zero + 0.5 * (q_min + q_max)])
        c = init_coefficients(self.dts[0], self.dts[1], self.Y)
        angular_jerk = self.jerk_error * q_max
        angular_acceleration = self.acceleration_error * q_max
        P_angle = self.angle_variance * c.T @ c + init_process_covariance(self.dts[0], self.dts[1], self.alpha, 2.0 * self.alpha * angular_jerk ** 2,
                                                                         angular_acceleration ** 2, angular_jerk ** 2, self.form, self.Y)
        P_range = torch.diag(self.tensor(self.log_range_variances + [(q_max - q_min) ** 2 / 12.0]))
        self.P = torch.block_diag(P_angle, P_range)

    def predict(self, dt, sensor):
        F, sensor_term, Qv = self.prediction_terms(dt, sensor)
        F2 = torch.kron(torch.eye(2, dtype=F.dtype, device=F.device), F)
        Y = self.Y

        def propagate(zeta):
            Ys = Y + zeta[:8]
            q = Ys[7]
            y = mpc.mpc_from_xi(F2 @ mpc.mpc_to_xi(Ys[:7]) + q * (sensor_term + zeta[8:]))
            return torch.cat([y[:7], (q / y[7])[None]])

        Y_pred = propagate(torch.zeros(16, dtype=Y.dtype, device=Y.device))

        def psi(zeta):
            raw = propagate(zeta)
            return torch.cat([wrapped_angle_difference(raw[0], Y_pred[0])[None], raw[1:] - Y_pred[1:]])

        mean, self.P = map_propagate(psi, self.P, Qv, self.order, lambda mean, xi: xi - mean)
        Y = Y_pred + mean
        self.Y = torch.cat([wrapped_angle_difference(Y[0], 0.0)[None], Y[1:]])

    def update(self, bearing):
        y = wrapped_angle_difference(bearing, self.Y[0])[None]
        ok, correction, self.P, S, S_inv, det_S = kalman_update(self.P, [0], y, self.tensor([[self.angle_variance]]))
        Y = self.Y + correction
        self.Y = torch.cat([wrapped_angle_difference(Y[0], 0.0)[None], Y[1:]])
        self.store_diagnostics(ok, y, S, S_inv, det_S)

    def step(self, time_or_dt, bearing, sensor):
        """
        @param time_or_dt absolute time or relative time since last step (depending on how time_is_relative is set)
        @param bearing measured bearing in radians
        @param sensor sensor state per axis [x, vx, ax, jx, y, vy, ay, jy] at the measurement time
        """
        bearing, sensor = self.tensor(bearing), self.tensor(sensor)
        dt = self.timer.advance(time_or_dt, self.current_idx == -1)
        if dt is not None:
            if self.current_idx < 2:
                self.dts[self.current_idx] = dt
            else:
                self.predict(self.tensor(dt), sensor)
                self.update(bearing)
        if self.current_idx < 2:
            self.measurements.append(bearing)
            if self.current_idx + 1 == 2:
                self.initialise()
            self.current_idx += 1
        self.sensor_state = sensor

    def forward(self, time_or_dt, bearing, sensor):
        """Steps the filter and returns the Kalman state estimate."""
        self.step(time_or_dt, bearing, sensor)
        return self.Y

    def get_kalman_vector(self):
        """Kalman state estimate [beta, d beta, d2 beta, d3 beta, d lambda, d2 lambda, d3 lambda, 1/r]."""
        return self.Y

    def get_range(self):
        """Range estimate 1 / (1/r)."""
        return 1.0 / self.Y[7]

class KalmanJerk2DAzElMovingSensor(MovingSensorFilter):
    """
    @param alpha alpha used to calculate q factor with q = 2 * alpha * jerk_error^2
    @param direction_error The standard deviation of the angular error of a measured direction in radians, isotropic on the sphere
    @param jerk_error The standard deviation of the jerk of each Cartesian coordinate of the target
    @param time_is_relative Indicates whether time values parsed to the step function are absolute temporal values or temporal differences between consecutive steps.
    @param range_min Lower bound of the initial range
    @param range_max Upper bound of the initial range
    @param log_range_rate_error Initial standard deviation of d lambda, lambda = ln r
    @param log_range_acceleration_error Initial standard deviation of d2 lambda
    @param log_range_jerk_error Initial standard deviation of d3 lambda
    @param acceleration_error The standard deviation of the acceleration of each Cartesian coordinate, used by the exact initialisation (28)
    @param form 'small_alpha_t' or 'exact'
    @param order 'first', 'second' or 'unscented'
    """

    def __init__(self, alpha, direction_error, jerk_error, time_is_relative, range_min, range_max, log_range_rate_error,
                 log_range_acceleration_error, log_range_jerk_error, acceleration_error=0.0, form="small_alpha_t", order="first",
                 dtype=torch.float64, device=None):
        super().__init__(alpha, direction_error, jerk_error, time_is_relative, range_min, range_max, log_range_rate_error,
                         log_range_acceleration_error, log_range_jerk_error, acceleration_error, form, order, 3, 12, 2, dtype, device)
        self.register_buffer("x", torch.zeros(12, dtype=dtype, device=device))
        self.register_buffer("b", torch.zeros(6, dtype=dtype, device=device))
        self.register_buffer("lam", torch.zeros(3, dtype=dtype, device=device))
        self.register_buffer("q", torch.zeros((), dtype=dtype, device=device))

    def initialise(self):
        q_min, q_max = self.inverse_range_bounds()
        angular_jerk = self.jerk_error * q_max
        angular_acceleration = self.acceleration_error * q_max
        m = torch.stack([s[0] for s in self.measurements])
        R = torch.stack([s[1] for s in self.measurements])
        isotropic = [s[2] for s in self.measurements]
        self.x, self.b, P8 = direction_initialise(m, R, isotropic, self.dts, self.alpha, 2.0 * self.alpha * angular_jerk ** 2,
                                                  angular_acceleration ** 2, angular_jerk ** 2, self.form)
        self.P = torch.block_diag(P8, torch.diag(self.tensor(self.log_range_variances + [(q_max - q_min) ** 2 / 12.0])))
        self.lam = torch.zeros_like(self.lam)
        self.q = self.tensor(0.5 * (q_min + q_max))

    def predict(self, dt, sensor):
        F, sensor_term, Qv = self.prediction_terms(dt, sensor)
        F3 = torch.kron(torch.eye(3, dtype=F.dtype, device=F.device), F)
        x, b, lam, q = self.x, self.b, self.lam, self.q

        def propagate(zeta):
            xs, _ = direction_retract(x, b, zeta[:8])
            qs = q + zeta[11]
            y = msc.msc_from_xi(F3 @ msc.msc_to_xi(xs, lam + zeta[8:11]) + qs * (sensor_term + zeta[12:]))
            return y[:12], y[12:15], qs / y[15]

        x_pred, lam_pred, q_pred = propagate(torch.zeros(24, dtype=x.dtype, device=x.device))
        b_pred = transport_basis(x[:3], x_pred[:3], b)

        def psi(zeta):
            xn, lamn, qn = propagate(zeta)
            return torch.cat([direction_inverse_retract(x_pred, b_pred, xn), lamn - lam_pred, (qn - q_pred)[None]])

        def reexpress_one(mean, xi):
            xs, _ = direction_retract(x_pred, b_pred, xi[:8])
            xm, bm = direction_retract(x_pred, b_pred, mean[:8])
            return torch.cat([direction_inverse_retract(xm, bm, xs), xi[8:] - mean[8:]])

        reexpress = lambda mean, xi: vmap(lambda v: reexpress_one(mean, v))(xi)
        mean, self.P = map_propagate(psi, self.P, Qv, self.order, reexpress)
        self.x, self.b = direction_normalise(*direction_retract(x_pred, b_pred, mean[:8]))
        self.lam = lam_pred + mean[8:11]
        self.q = q_pred + mean[11]

    def update(self, m, R, isotropic):
        y, R_b = direction_innovation(self.x[:3], self.b, m, R, isotropic)
        ok, correction, self.P, S, S_inv, det_S = kalman_update(self.P, [0, 4], y, R_b)
        x, b = direction_normalise(*direction_retract(self.x, self.b, correction[:8]))
        self.x = torch.where(ok, x, self.x)
        self.b = torch.where(ok, b, self.b)
        self.lam = self.lam + correction[8:11]
        self.q = self.q + correction[11]
        self.store_diagnostics(ok, y, S, S_inv, det_S)

    def step(self, time_or_dt, azimuth, elevation, sensor, R=None):
        """
        @param time_or_dt absolute time or relative time since last step (depending on how time_is_relative is set)
        @param azimuth measured azimuth in radians
        @param elevation measured elevation in radians
        @param sensor sensor state per axis [x, vx, ax, jx, y, vy, ay, jy, z, vz, az, jz] at the measurement time
        @param R measurement covariance in radians squared in the tangent plane of the measured direction, in the
            frame of increasing azimuth and increasing elevation; direction_error^2 I when not given
        """
        isotropic = R is None
        R = self.angle_variance * torch.eye(2, dtype=self.P.dtype, device=self.P.device) if isotropic else self.tensor(R)
        sensor = self.tensor(sensor)
        m = unit_from_azel(self.tensor(azimuth), self.tensor(elevation))
        dt = self.timer.advance(time_or_dt, self.current_idx == -1)
        if dt is not None:
            if self.current_idx < 2:
                self.dts[self.current_idx] = dt
            else:
                self.predict(self.tensor(dt), sensor)
                self.update(m, R, isotropic)
        if self.current_idx < 2:
            self.measurements.append((m, R, isotropic))
            if self.current_idx + 1 == 2:
                self.initialise()
            self.current_idx += 1
        self.sensor_state = sensor

    def forward(self, time_or_dt, azimuth, elevation, sensor, R=None):
        """Steps the filter and returns the Kalman state estimate."""
        self.step(time_or_dt, azimuth, elevation, sensor, R)
        return self.get_kalman_vector()

    def get_kalman_vector(self):
        """
        Kalman state estimate [azimuth, w_a, a_a, j_a, elevation, w_e, a_e, j_e, d lambda, d2 lambda, d3 lambda, 1/r]
        with tangent components along increasing azimuth and increasing elevation, given in the internal basis at
        the zenith and nadir.
        """
        frame = azel_frame(self.x[:3])
        e_a, e_e = frame if frame is not None else (self.b[:3], self.b[3:])
        az, el = azel_from_unit(self.x[:3])
        v = [self.x[3 * k:3 * k + 3] for k in range(1, 4)]
        return torch.cat([torch.stack([az] + [dot3(t, e_a) for t in v] + [el] + [dot3(t, e_e) for t in v]), self.lam, self.q[None]])

    def get_state_vector(self):
        """State [u, w, a, j]: unit direction, angular velocity, angular acceleration and angular jerk vectors."""
        return self.x

    def get_basis(self):
        """Tangent basis [b1, b2] of the internal covariance."""
        return self.b

    def get_range(self):
        """Range estimate 1 / (1/r)."""
        return 1.0 / self.q
