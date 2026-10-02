"""
Initialisation and measurement of a direction state.

The estimates (22) of [Ref1] are formed in Riemannian normal coordinates of the sphere centred on the third
measured direction, and the initial covariance is the error analysis (23) to (28) carried through these coordinates.
"""

import torch
from torch.func import jacfwd
from .jerk import init_estimate, init_process_covariance
from .sphere import (dot3, rotate3, log_map, tangent_basis, tangent_basis_axis, azel_frame_masked, basis_vector,
                     direction_inverse_retract, transport_basis)

def direction_tangent_covariance(m, bn, R, isotropic):
    """
    Measurement covariance at the direction m in a basis bn of its tangent plane. R is isotropic, or given in
    the frame of increasing azimuth and increasing elevation at m; at the zenith and nadir the mean of its two
    variances is used.
    """
    I = torch.eye(2, dtype=R.dtype, device=R.device)
    if isotropic:
        return R[0, 0] * I
    e_a, e_e, defined = azel_frame_masked(m)
    M = torch.stack([torch.stack([dot3(bn[:3], e_a), dot3(bn[:3], e_e)]), torch.stack([dot3(bn[3:], e_a), dot3(bn[3:], e_e)])])
    return torch.where(defined, M @ R @ M.T, 0.5 * (R[0, 0] + R[1, 1]) * I)

def direction_measurement_basis(m, n):
    """
    Tangent basis at measurement n used to express its error: the basis at the third measurement carried along
    the great circle to measurement n.
    """
    b0 = tangent_basis(m[2])
    return b0 if n == 2 else transport_basis(m[2], m[n], b0)

def direction_estimate(m, dts, noise, k):
    """Estimate (22) of the direction state from three measured directions displaced by tangent errors."""
    mp = []
    for n in range(3):
        bn = direction_measurement_basis(m, n)
        v = basis_vector(bn, noise[2 * n], noise[2 * n + 1])
        mp.append(rotate3(torch.linalg.cross(m[n], v), m[n]))
    b = tangent_basis(mp[2], k)
    p = []
    for n in range(3):
        v = log_map(mp[2], mp[n])
        p.append(torch.stack([dot3(v, b[:3]), dot3(v, b[3:])]))
    axis = [init_estimate(p[0][i], p[1][i], torch.zeros_like(p[0][i]), dts[0], dts[1]) for i in range(2)]
    w = axis[0][1] * b[:3] + axis[1][1] * b[3:]
    a = axis[0][2] * b[:3] + axis[1][2] * b[3:]
    return torch.cat([mp[2], w, a, torch.zeros_like(w)]), b

def direction_initialise(m, R, isotropic, dts, alpha, q_scale, var_m, var_j, form):
    """Initial direction state, basis and 8 by 8 covariance. q_scale, var_m and var_j are in angular units and enter through (28) or (29)."""
    zero = torch.zeros(6, dtype=m.dtype, device=m.device)
    k = tangent_basis_axis(m[2])
    x, b = direction_estimate(m, dts, zero, k)
    G = jacfwd(lambda noise: direction_inverse_retract(x, b, direction_estimate(m, dts, noise, k)[0]))(zero)
    R_block = torch.block_diag(*[direction_tangent_covariance(m[n], direction_measurement_basis(m, n), R[n], isotropic[n]) for n in range(3)])
    P_proc = init_process_covariance(dts[0], dts[1], alpha, q_scale, var_m, var_j, form, m)
    return x, b, G @ R_block @ G.T + torch.block_diag(P_proc, P_proc)

def direction_innovation(u, b, m, R, isotropic):
    """Innovation and measurement covariance in the basis b at the predicted direction u."""
    v = log_map(u, m)
    y = torch.stack([dot3(v, b[:3]), dot3(v, b[3:])])
    if isotropic:
        return y, torch.diag(torch.stack([R[0, 0], R[1, 1]]))
    return y, direction_tangent_covariance(m, transport_basis(u, m, b), R, False)
