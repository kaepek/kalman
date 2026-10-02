"""
Propagation of a mean and covariance through a map psi(zeta), zeta = [error (NE), process noise (NV)],
giving the error at the prediction relative to psi(0). The error has covariance P and the noise Qv.
Derivatives are taken with torch.func.
"""

import torch
from torch.func import jacfwd, hessian, vmap
from .jerk import symmetrise

UNSCENTED_ALPHA = 1.0
UNSCENTED_BETA = 2.0
UNSCENTED_KAPPA = 0.0

def augmented_covariance(P, Qv):
    return torch.block_diag(P, Qv)

def first_order(psi, Pz):
    J = jacfwd(psi)(torch.zeros(Pz.shape[0], dtype=Pz.dtype, device=Pz.device))
    return torch.zeros(J.shape[0], dtype=Pz.dtype, device=Pz.device), symmetrise(J @ Pz @ J.T)

def second_order_terms(psi, Pz):
    """Mean 0.5 tr(H_i Pz) and covariance 0.5 tr(H_i Pz H_k Pz) from the Hessians H_i of psi at zero."""
    H = hessian(psi)(torch.zeros(Pz.shape[0], dtype=Pz.dtype, device=Pz.device))
    HP = H @ Pz
    return 0.5 * torch.diagonal(HP, dim1=-2, dim2=-1).sum(-1), 0.5 * torch.einsum("irc,kcr->ik", HP, HP)

def second_order(psi, Pz):
    J = jacfwd(psi)(torch.zeros(Pz.shape[0], dtype=Pz.dtype, device=Pz.device))
    mean, C = second_order_terms(psi, Pz)
    return mean, symmetrise(J @ Pz @ J.T + C)

def unscented_weights(n):
    lam = UNSCENTED_ALPHA ** 2 * (n + UNSCENTED_KAPPA) - n
    w_m0 = lam / (n + lam)
    w_c0 = w_m0 + 1.0 - UNSCENTED_ALPHA ** 2 + UNSCENTED_BETA
    w_i = 1.0 / (2.0 * (n + lam))
    return lam, w_m0, w_c0, w_i

def sigma_points(Pz):
    """
    Sigma points [0, columns of L, minus columns of L] with L L^T = (n + lambda) Pz, and whether Pz is positive
    definite.
    """
    n = Pz.shape[0]
    lam = unscented_weights(n)[0]
    L, info = torch.linalg.cholesky_ex((n + lam) * Pz)
    return torch.cat([torch.zeros(1, n, dtype=Pz.dtype, device=Pz.device), L.T, -L.T]), info == 0

def weights(n, like):
    _, w_m0, w_c0, w_i = unscented_weights(n)
    w_m = torch.full((2 * n + 1,), w_i, dtype=like.dtype, device=like.device)
    w_c = w_m.clone()
    w_m[0] = w_m0
    w_c[0] = w_c0
    return w_m, w_c

def weighted_covariance(xi, w_m, w_c):
    mean = w_m @ xi
    D = xi - mean
    return symmetrise((w_c[:, None] * D).T @ D)

def map_propagate(psi, P, Qv, order, reexpress):
    """
    Mean of the error and covariance at the prediction. reexpress(mean, xi) expresses the errors xi relative to
    psi(0) as errors relative to the prediction moved by mean. The unscented transform falls back to first order
    when the augmented covariance is not positive definite.
    """
    Pz = augmented_covariance(P, Qv)
    if order == "first":
        return first_order(psi, Pz)
    if order == "second":
        return second_order(psi, Pz)
    points, ok = sigma_points(Pz)
    w_m, w_c = weights(Pz.shape[0], Pz)
    xi = vmap(psi)(points)
    mean = w_m @ xi
    P_unscented = weighted_covariance(reexpress(mean, xi), w_m, w_c)
    mean_first, P_first = first_order(psi, Pz)
    return torch.where(ok, mean, mean_first), torch.where(ok, P_unscented, P_first)
