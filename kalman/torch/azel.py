"""
Jerk filter for a direction measured as azimuth and elevation, the range unknown.

The direction moves on the unit sphere with the jerk model of [Ref1] posed intrinsically:
Dw/dt = a, Da/dt = j, Dj/dt = -alpha j + w_noise, with the noise isotropic in the tangent plane.
The state is held as a unit vector with tangent vectors, see model/derivation.md.
"""

import math
import torch
from torch.func import vmap
from .jerk import check_form, check_order, drift_alpha, kalman_update, symmetrise, StepTimer, Diagnostics
from .sphere import (dot3, unit_from_azel, azel_from_unit, azel_frame, direction_retract, direction_inverse_retract,
                     direction_normalise)
from .direction import direction_initialise, direction_innovation
from .propagation import second_order_terms, sigma_points, weights, weighted_covariance
from .generated import azel

def azel_flow(x, alpha, dt, n):
    """Integrates the noise free direction dynamics over dt with n Runge Kutta steps."""
    h = dt / n
    for _ in range(n):
        k1 = azel.azel_dynamics(x, alpha)
        k2 = azel.azel_dynamics(x + 0.5 * h * k1, alpha)
        k3 = azel.azel_dynamics(x + 0.5 * h * k2, alpha)
        k4 = azel.azel_dynamics(x + h * k3, alpha)
        x = x + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
    return x

def azel_psi(x0, b0, x1, b1, alpha, dt, n):
    """
    Map from the error d at the estimate (x0, b0) to the error at the end of the interval relative to the
    propagated estimate (x1, b1).
    """
    def psi(d):
        xs, _ = direction_retract(x0, b0, d)
        return direction_inverse_retract(x1, b1, azel_flow(xs, alpha, dt, n))
    return psi

class KalmanJerk2DAzEl(Diagnostics, torch.nn.Module):
    """
    @param alpha alpha used to calculate q factor with q = 2 * alpha * jerk_error^2
    @param direction_error The standard deviation of the angular error of a measured direction in radians, isotropic on the sphere
    @param jerk_error The standard deviation of the angular jerk in radians per second cubed
    @param time_is_relative Indicates whether time values parsed to the step function are absolute temporal values or temporal differences between consecutive steps.
    @param acceleration_error The standard deviation of the angular acceleration, used by the exact initialisation (28)
    @param substeps Number of Runge Kutta steps per sampling interval
    @param form 'small_alpha_t' or 'exact'
    @param order 'first', 'second' or 'unscented'
    """

    def __init__(self, alpha, direction_error, jerk_error, time_is_relative=False, acceleration_error=0.0, substeps=4,
                 form="small_alpha_t", order="first", dtype=torch.float64, device=None):
        super().__init__()
        self.alpha = float(alpha)
        self.q_scale = 2.0 * self.alpha * float(jerk_error) ** 2
        self.direction_variance = float(direction_error) ** 2
        self.jerk_variance = float(jerk_error) ** 2
        self.acceleration_variance = float(acceleration_error) ** 2
        self.substeps = int(substeps)
        self.form = check_form(form)
        self.order = check_order(order)
        self.timer = StepTimer(time_is_relative)
        self.current_idx = -1
        self.dts = [0.0, 0.0]
        self.measurements = []
        self.register_buffer("x", torch.zeros(12, dtype=dtype, device=device))
        self.register_buffer("b", torch.zeros(6, dtype=dtype, device=device))
        self.register_buffer("P", torch.zeros(8, 8, dtype=dtype, device=device))
        self.register_diagnostics(2, dtype, device)

    def tensor(self, value):
        return torch.as_tensor(value, dtype=self.P.dtype, device=self.P.device)

    def initialise(self):
        m = torch.stack([s[0] for s in self.measurements])
        R = torch.stack([s[1] for s in self.measurements])
        isotropic = [s[2] for s in self.measurements]
        self.x, self.b, self.P = direction_initialise(m, R, isotropic, self.dts, self.alpha, self.q_scale,
                                                      self.acceleration_variance, self.jerk_variance, self.form)

    def integrate(self, dt):
        """Integrates the state, the basis, the transition matrix and the process noise covariance over dt."""
        a_d = drift_alpha(self.alpha, self.form)
        G = torch.zeros(8, 8, dtype=self.P.dtype, device=self.P.device)
        G[3, 3] = G[7, 7] = self.q_scale

        def rate(Z):
            x, b, Phi, Q = Z
            A = azel.azel_error_dynamics(x, b, a_d)
            return azel.azel_dynamics(x, a_d), azel.azel_basis_rate(x, b), A @ Phi, A @ Q + Q @ A.T + G

        Z = (self.x, self.b, torch.eye(8, dtype=self.P.dtype, device=self.P.device), torch.zeros_like(self.P))
        h = dt / self.substeps
        for _ in range(self.substeps):
            k1 = rate(Z)
            k2 = rate(tuple(z + 0.5 * h * k for z, k in zip(Z, k1)))
            k3 = rate(tuple(z + 0.5 * h * k for z, k in zip(Z, k2)))
            k4 = rate(tuple(z + h * k for z, k in zip(Z, k3)))
            Z = tuple(z + (h / 6.0) * (a + 2.0 * b + 2.0 * c + d) for z, a, b, c, d in zip(Z, k1, k2, k3, k4))
        x1, b1 = direction_normalise(Z[0], Z[1])
        return x1, b1, Z[2], symmetrise(Z[3])

    def predict(self, dt):
        a_d = drift_alpha(self.alpha, self.form)
        x1, b1, F, Qd = self.integrate(dt)
        P_first = F @ self.P @ F.T + Qd
        if self.order == "first":
            self.x, self.b, self.P = x1, b1, P_first
            return
        if self.order == "second":
            mean, C = second_order_terms(azel_psi(self.x, self.b, x1, b1, a_d, dt, self.substeps), self.P)
            self.P = symmetrise(P_first + C)
            self.x, self.b = direction_normalise(*direction_retract(x1, b1, mean))
            return
        points, ok = sigma_points(self.P)
        w_m, w_c = weights(8, self.P)
        chi = vmap(lambda d: azel_flow(direction_retract(self.x, self.b, d)[0], a_d, dt, self.substeps))(points)
        mean = w_m @ vmap(lambda c: direction_inverse_retract(x1, b1, c))(chi)
        x, b = direction_normalise(*direction_retract(x1, b1, mean))
        xi = vmap(lambda c: direction_inverse_retract(x, b, c))(chi)
        self.x = torch.where(ok, x, x1)
        self.b = torch.where(ok, b, b1)
        self.P = torch.where(ok, weighted_covariance(xi, w_m, w_c) + Qd, P_first)

    def update(self, m, R, isotropic):
        y, R_b = direction_innovation(self.x[:3], self.b, m, R, isotropic)
        ok, correction, self.P, S, S_inv, det_S = kalman_update(self.P, [0, 4], y, R_b)
        x, b = direction_normalise(*direction_retract(self.x, self.b, correction))
        self.x = torch.where(ok, x, self.x)
        self.b = torch.where(ok, b, self.b)
        self.store_diagnostics(ok, y, S, S_inv, det_S)

    def step(self, time_or_dt, azimuth, elevation, R=None):
        """
        @param time_or_dt absolute time or relative time since last step (depending on how time_is_relative is set)
        @param azimuth measured azimuth in radians
        @param elevation measured elevation in radians
        @param R measurement covariance in radians squared in the tangent plane of the measured direction, in the
            frame of increasing azimuth and increasing elevation; direction_error^2 I when not given
        """
        isotropic = R is None
        R = self.direction_variance * torch.eye(2, dtype=self.P.dtype, device=self.P.device) if isotropic else self.tensor(R)
        m = unit_from_azel(self.tensor(azimuth), self.tensor(elevation))
        dt = self.timer.advance(time_or_dt, self.current_idx == -1)
        if dt is not None:
            if self.current_idx < 2:
                self.dts[self.current_idx] = dt
            else:
                self.predict(self.tensor(dt))
                self.update(m, R, isotropic)
        if self.current_idx < 2:
            self.measurements.append((m, R, isotropic))
            if self.current_idx + 1 == 2:
                self.initialise()
            self.current_idx += 1

    def forward(self, time_or_dt, azimuth, elevation, R=None):
        """Steps the filter and returns the Kalman state estimate."""
        self.step(time_or_dt, azimuth, elevation, R)
        return self.get_kalman_vector()

    @staticmethod
    def azel_noise_to_tangent(elevation, azimuth_error, elevation_error):
        """Tangent plane covariance of a measurement described by azimuth and elevation standard deviations."""
        c = math.cos(elevation)
        return torch.tensor([[azimuth_error * azimuth_error * c * c, 0.0], [0.0, elevation_error * elevation_error]], dtype=torch.float64)

    def azimuth_defined(self):
        """True when the estimated direction is away from the zenith and nadir, where the azimuth is defined."""
        return azel_frame(self.x[:3]) is not None

    def frame(self):
        frame = azel_frame(self.x[:3])
        return frame if frame is not None else (self.b[:3], self.b[3:])

    def get_kalman_vector(self):
        """
        Kalman state estimate [azimuth, w_a, a_a, j_a, elevation, w_e, a_e, j_e] with the angular rate, acceleration
        and jerk components along increasing azimuth and increasing elevation. At the zenith and nadir the
        components are given in the internal basis.
        """
        e_a, e_e = self.frame()
        az, el = azel_from_unit(self.x[:3])
        v = [self.x[3 * k:3 * k + 3] for k in range(1, 4)]
        return torch.stack([az] + [dot3(t, e_a) for t in v] + [el] + [dot3(t, e_e) for t in v])

    def get_covariance_matrix(self):
        """Covariance matrix in the frame of get_kalman_vector."""
        if azel_frame(self.x[:3]) is None:
            return self.P
        e_a, e_e = self.frame()
        M = torch.stack([torch.stack([dot3(self.b[:3], e_a), dot3(self.b[3:], e_a)]), torch.stack([dot3(self.b[:3], e_e), dot3(self.b[3:], e_e)])])
        T = torch.kron(M, torch.eye(4, dtype=self.P.dtype, device=self.P.device))
        return T @ self.P @ T.T

    def get_state_vector(self):
        """State [u, w, a, j]: unit direction, angular velocity, angular acceleration and angular jerk vectors."""
        return self.x

    def get_basis(self):
        """Tangent basis [b1, b2] of the internal covariance."""
        return self.b

    def get_basis_covariance_matrix(self):
        """Covariance matrix in the internal tangent basis."""
        return self.P
