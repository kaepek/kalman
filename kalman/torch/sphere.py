"""
Unit sphere geometry for the direction filters.

A direction state is the unit vector u with tangent vectors w, a, j stored as x [12] = [u, w, a, j],
and an orthonormal tangent basis b [6] = [b1, b2]. Errors are ordered by axis
[v_1, dw_1, da_1, dj_1, v_2, dw_2, da_2, dj_2] with components taken in the basis.
The functions are written without data dependent control flow so that torch.func can transform them.
"""

import torch

def dot3(a, b):
    return (a * b).sum(-1)

def rotate3(r, x):
    """Rotates x by the rotation vector r (angle |r| about r), smooth at r = 0."""
    t2 = dot3(r, r)
    small = t2 < 1e-8
    t2_safe = torch.where(small, torch.ones_like(t2), t2)
    t = torch.sqrt(t2_safe)
    s = torch.where(small, 1.0 - t2 / 6.0 + t2 * t2 / 120.0, torch.sin(t) / t)
    c = torch.where(small, 0.5 - t2 / 24.0 + t2 * t2 / 720.0, (1.0 - torch.cos(t)) / t2_safe)
    rx = torch.linalg.cross(r, x)
    return x + s[..., None] * rx + c[..., None] * torch.linalg.cross(r, rx)

def log_map(u, m):
    """Tangent vector at u pointing along the great circle to m with length the angle between them."""
    cd = dot3(u, m)
    tau = m - cd[..., None] * u
    s2 = dot3(tau, tau)
    small = (s2 < 1e-12) & (cd > 0.0)
    s = torch.sqrt(torch.where(small, torch.ones_like(s2), s2))
    factor = torch.where(small, 1.0 + s2 / 6.0 + 3.0 * s2 * s2 / 40.0, torch.atan2(s, cd) / s)
    return factor[..., None] * tau

def tangent_basis_axis(c):
    """Index of the coordinate axis least aligned with c."""
    return int(torch.argmin(torch.abs(c.detach())))

def tangent_basis(c, k=None):
    """Orthonormal tangent basis [b1, b2] at c built from coordinate axis k, by default the axis least aligned with c."""
    k = tangent_basis_axis(c) if k is None else k
    b1 = -c[k] * c
    b1 = b1 + torch.nn.functional.one_hot(torch.tensor(k), 3).to(c)
    b1 = b1 / torch.sqrt(dot3(b1, b1))
    return torch.cat([b1, torch.linalg.cross(c, b1)])

def unit_from_azel(az, el):
    return torch.stack([torch.cos(el) * torch.cos(az), torch.cos(el) * torch.sin(az), torch.sin(el)], -1)

def azel_from_unit(u):
    return torch.atan2(u[1], u[0]), torch.atan2(u[2], torch.sqrt(u[0] * u[0] + u[1] * u[1]))

def azel_frame(u):
    """Tangent vectors along increasing azimuth and elevation at u, None at the zenith and nadir."""
    rho = float(torch.sqrt(u[0] * u[0] + u[1] * u[1]))
    if rho < 1e-12:
        return None
    zero = torch.zeros_like(u[0])
    e_a = torch.stack([-u[1] / rho, u[0] / rho, zero])
    e_e = torch.stack([-u[2] * u[0] / rho, -u[2] * u[1] / rho, zero + rho])
    return e_a, e_e

def azel_frame_masked(u):
    """Tangent vectors along increasing azimuth and elevation at u, and whether u is away from the zenith and nadir."""
    rho = torch.sqrt(u[0] * u[0] + u[1] * u[1])
    defined = rho >= 1e-12
    rho = torch.where(defined, rho, torch.ones_like(rho))
    zero = torch.zeros_like(u[0])
    e_a = torch.stack([-u[1] / rho, u[0] / rho, zero])
    e_e = torch.stack([-u[2] * u[0] / rho, -u[2] * u[1] / rho, rho])
    return e_a, e_e, defined

def basis_vector(b, c1, c2):
    """Tangent vector with components c1, c2 in the basis b."""
    return c1[..., None] * b[:3] + c2[..., None] * b[3:]

def direction_retract(x, b, d):
    """Direction state obtained from the estimate (x, b) and the error d, with the basis carried along."""
    u = x[:3]
    r = torch.linalg.cross(u, basis_vector(b, d[0], d[4]))
    parts = [rotate3(r, u)]
    for k in range(1, 4):
        parts.append(rotate3(r, x[3 * k:3 * k + 3] + basis_vector(b, d[k], d[4 + k])))
    return torch.cat(parts), torch.cat([rotate3(r, b[:3]), rotate3(r, b[3:])])

def direction_inverse_retract(x, b, x_s):
    """Error d of the direction state x_s relative to the estimate (x, b)."""
    u = x[:3]
    v = log_map(u, x_s[:3])
    neg_r = -torch.linalg.cross(u, v)
    comp = [v] + [rotate3(neg_r, x_s[3 * k:3 * k + 3]) - x[3 * k:3 * k + 3] for k in range(1, 4)]
    first = torch.stack([dot3(c, b[:3]) for c in comp])
    second = torch.stack([dot3(c, b[3:]) for c in comp])
    return torch.cat([first, second])

def transport_basis(u0, u1, b):
    """Basis carried from u0 to u1 by the parallel transport along the great circle between them."""
    r = torch.linalg.cross(u0, log_map(u0, u1))
    return torch.cat([rotate3(r, b[:3]), rotate3(r, b[3:])])

def direction_normalise(x, b):
    """Normalises u and removes the normal parts of w, a, j and of the basis."""
    u = x[:3] / torch.sqrt(dot3(x[:3], x[:3]))
    parts = [u] + [x[3 * k:3 * k + 3] - dot3(u, x[3 * k:3 * k + 3]) * u for k in range(1, 4)]
    b1 = b[:3] - dot3(u, b[:3]) * u
    b1 = b1 / torch.sqrt(dot3(b1, b1))
    return torch.cat(parts), torch.cat([b1, torch.linalg.cross(u, b1)])
