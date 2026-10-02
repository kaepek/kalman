"""
Per axis jerk model of [Ref1]: transition and process noise matrices, the initialisation (22) to (29)
and the Kalman update, shared by the filters.
"""

import torch
from .generated import jerk_block

FORMS = ("small_alpha_t", "exact")
ORDERS = ("first", "second", "unscented")

def check_form(form):
    if form not in FORMS:
        raise ValueError("form must be one of " + ", ".join(FORMS))
    return form

def check_order(order):
    if order not in ORDERS:
        raise ValueError("order must be one of " + ", ".join(ORDERS))
    return order

def transition(dt, alpha, form):
    """F and Q of one axis: (16) and (21) of [Ref1] for small_alpha_t, (14), (15) and (20) for exact."""
    if form == "exact":
        return jerk_block.jerk_transition_exact(dt, alpha), jerk_block.jerk_process_noise_exact(dt, alpha)
    return jerk_block.jerk_transition_small(dt), jerk_block.jerk_process_noise_small(dt)

def drift_alpha(alpha, form):
    """Value of alpha used in the drift of a continuous model, zero in the small alpha T limit."""
    return alpha if form == "exact" else 0.0

def init_coefficients(dt1, dt2, like):
    """Coefficients of M(n) in the estimates (22), rows c[n - 1] over [x, v, a, j]."""
    c = torch.zeros(3, 4, dtype=like.dtype, device=like.device)
    c[0, 2] = 1.0 / (dt1 * dt2)
    c[1, 1] = -1.0 / dt2
    c[1, 2] = -1.0 / (dt2 * dt2) - 1.0 / (dt1 * dt2)
    c[2, 0] = 1.0
    c[2, 1] = 1.0 / dt2
    c[2, 2] = 1.0 / (dt2 * dt2)
    return c

def init_estimate(m1, m2, m3, dt1, dt2):
    """Estimates (22) of one axis from three measurements, [..., 4]."""
    v2 = (m3 - m2) / dt2
    v1 = (m2 - m1) / dt1
    return torch.stack([m3, v2, (v2 - v1) / dt2, torch.zeros_like(m3)], -1)

def init_process_covariance(dt1, dt2, alpha, q_scale, var_m, var_j, form, like):
    """
    Terms of the initial covariance of one axis arising from the target acceleration, jerk and process noise:
    (28) for exact and (29) for small_alpha_t. q_scale is 2 alpha sigma_j^2, var_m and var_j the variances
    of the target acceleration and jerk.
    """
    as_tensor = lambda v: torch.as_tensor(v, dtype=like.dtype, device=like.device)
    if form != "exact":
        return jerk_block.jerk_initial_covariance_small(as_tensor(dt2), as_tensor(0.0), as_tensor(var_j))
    # true states X(2) = F1 X(1) + u(1), X(3) = F2 X(2) + u(2); error e = sum_n c_n M(n) - X(3)
    F1, Q1 = transition(as_tensor(dt1), alpha, form)
    F2, Q2 = transition(as_tensor(dt2), alpha, form)
    c = init_coefficients(dt1, dt2, like)
    e1 = torch.zeros(4, dtype=like.dtype, device=like.device)
    e1[0] = 1.0
    F21 = F2 @ F1
    A = torch.outer(c[0], e1) + torch.outer(c[1], F1[0]) + torch.outer(c[2], F21[0]) - F21
    G1 = torch.outer(c[1], e1) + torch.outer(c[2], F2[0]) - F2
    G2 = torch.outer(c[2], e1) - torch.eye(4, dtype=like.dtype, device=like.device)
    S1 = torch.diag(torch.stack([as_tensor(0.0), as_tensor(0.0), as_tensor(var_m), as_tensor(var_j)]))
    return A @ S1 @ A.T + q_scale * (G1 @ Q1 @ G1.T + G2 @ Q2 @ G2.T)

def symmetrise(P):
    return 0.5 * (P + P.transpose(-1, -2))

def kalman_update(P, idx, y, R):
    """
    Kalman update of covariance P with a measurement of state components idx, innovation y and measurement
    covariance R. Returns (ok, correction K y, updated P, S, S inverse, det S); when S is not positive definite
    ok is false, the correction is zero and P is unchanged.
    """
    S = P[idx][:, idx] + R
    L, info = torch.linalg.cholesky_ex(S)
    ok = info == 0
    S_inv = torch.cholesky_inverse(L)
    det_S = torch.prod(torch.diagonal(L)) ** 2
    K = P[:, idx] @ S_inv
    correction = torch.where(ok, K @ y, torch.zeros_like(P[:, 0]))
    return ok, correction, torch.where(ok, symmetrise(P - K @ P[idx, :]), P), S, S_inv, det_S

class Diagnostics:
    """Innovation quantities of the last update."""

    def register_diagnostics(self, M, dtype, device):
        self.register_buffer("innovation", torch.zeros(M, dtype=dtype, device=device))
        self.register_buffer("innovation_covariance", torch.zeros(M, M, dtype=dtype, device=device))
        self.register_buffer("innovation_covariance_inverse", torch.zeros(M, M, dtype=dtype, device=device))
        self.register_buffer("innovation_covariance_determinant", torch.zeros((), dtype=dtype, device=device))

    def store_diagnostics(self, ok, y, S, S_inv, det_S):
        self.innovation = torch.where(ok, y, self.innovation)
        self.innovation_covariance = torch.where(ok, S, self.innovation_covariance)
        self.innovation_covariance_inverse = torch.where(ok, S_inv, self.innovation_covariance_inverse)
        self.innovation_covariance_determinant = torch.where(ok, det_S, self.innovation_covariance_determinant)

    def compile(self, **kwargs):
        """Compiles the prediction and the update with torch.compile, kwargs are passed to torch.compile."""
        self.predict = torch.compile(self.predict, **kwargs)
        self.update = torch.compile(self.update, **kwargs)
        return self

    def get_innovation(self):
        return self.innovation

    def get_innovation_covariance(self):
        return self.innovation_covariance

    def get_innovation_covariance_inverse(self):
        return self.innovation_covariance_inverse

    def get_innovation_covariance_determinant(self):
        return self.innovation_covariance_determinant

class StepTimer:
    """Time handling of the step functions: absolute times or differences between consecutive steps."""

    def __init__(self, time_is_relative):
        self.time_is_relative = time_is_relative
        self.time = 0.0

    def advance(self, time_or_dt, first):
        if first:
            self.time = float(time_or_dt)
            return None
        dt = float(time_or_dt) if self.time_is_relative else float(time_or_dt) - self.time
        self.time = self.time + float(time_or_dt) if self.time_is_relative else float(time_or_dt)
        return dt
