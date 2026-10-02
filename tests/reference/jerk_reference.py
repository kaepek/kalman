import math
import numpy as np

"""
Numpy reference implementations of the jerk filters in lib/jerk, used by the tests.
Derivatives are taken by finite differences.
"""

TWO_PI = 2.0 * math.pi

def wrap(a):
    r = a - TWO_PI * math.floor(a / TWO_PI + 0.5)
    return r + TWO_PI if r <= -math.pi else r

def expm(A):
    n = max(0, int(math.ceil(math.log2(max(np.abs(A).sum(axis=1).max(), 1e-300)))) + 1)
    S = A / (2.0 ** n)
    E = np.eye(A.shape[0])
    term = np.eye(A.shape[0])
    for k in range(1, 25):
        term = term @ S / k
        E = E + term
    for _ in range(n):
        E = E @ E
    return E

def jerk_matrices(dt, alpha, exact):
    a = alpha if exact else 0.0
    A = np.array([[0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1], [0, 0, 0, -a]], float)
    B = np.array([[0], [0], [0], [1.0]])
    M = np.block([[-A, B @ B.T], [np.zeros((4, 4)), A.T]])
    E = expm(M * dt)
    F = E[4:, 4:].T
    Q = F @ E[:4, 4:]
    return F, 0.5 * (Q + Q.T)

def init_coefficients(dt1, dt2):
    return np.array([
        [0.0, 0.0, 1.0 / (dt1 * dt2), 0.0],
        [0.0, -1.0 / dt2, -1.0 / dt2**2 - 1.0 / (dt1 * dt2), 0.0],
        [1.0, 1.0 / dt2, 1.0 / dt2**2, 0.0],
    ])

def init_estimate(m1, m2, m3, dt1, dt2):
    v2 = (m3 - m2) / dt2
    v1 = (m2 - m1) / dt1
    return np.array([m3, v2, (v2 - v1) / dt2, 0.0])

def init_process_covariance(dt1, dt2, alpha, q_scale, var_m, var_j, exact):
    if not exact:
        T = dt2
        P = np.zeros((4, 4))
        P[1, 3] = P[3, 1] = 5.0 / 6.0 * var_j * T**2
        P[2, 3] = P[3, 2] = var_j * T
        P[3, 3] = var_j
        return P
    F1, Q1 = jerk_matrices(dt1, alpha, True)
    F2, Q2 = jerk_matrices(dt2, alpha, True)
    Q1, Q2 = q_scale * Q1, q_scale * Q2
    c = init_coefficients(dt1, dt2)
    e1 = np.array([1.0, 0, 0, 0])
    A = np.outer(c[0], e1) + np.outer(c[1], F1[0]) + np.outer(c[2], (F2 @ F1)[0]) - F2 @ F1
    G1 = np.outer(c[1], e1) + np.outer(c[2], F2[0]) - F2
    G2 = np.outer(c[2], e1) - np.eye(4)
    return A @ np.diag([0, 0, var_m, var_j]) @ A.T + G1 @ Q1 @ G1.T + G2 @ Q2 @ G2.T

def kalman_update(P, idx, y, R):
    S = P[np.ix_(idx, idx)] + R
    S_inv = np.linalg.inv(S)
    K = P[:, idx] @ S_inv
    P_new = P - K @ P[idx, :]
    return K @ y, 0.5 * (P_new + P_new.T), S

class Steps:
    """Shared handling of time and of the first three measurements."""

    def __init__(self, time_is_relative):
        self.time_is_relative = time_is_relative
        self.idx = -1
        self.time = 0.0
        self.dts = [0.0, 0.0]

    def advance(self, t):
        if self.idx == -1:
            self.time = t
            return None
        dt = t if self.time_is_relative else t - self.time
        self.time = self.time + t if self.time_is_relative else t
        return dt

class Cartesian(Steps):
    def __init__(self, N, alpha, res, jerk, rel, acc=0.0, exact=False):
        super().__init__(rel)
        self.N, self.alpha, self.exact = N, alpha, exact
        self.var_x, self.var_j, self.var_m = res**2, jerk**2, acc**2
        self.q = 2 * alpha * jerk**2
        self.meas, self.R = [], []

    def step(self, t, x, R=None):
        x = np.asarray(x, float)
        R = self.var_x * np.eye(self.N) if R is None else np.asarray(R, float)
        dt = self.advance(t)
        if self.idx >= 0 and self.idx < 2:
            self.dts[self.idx] = dt
        elif self.idx >= 2:
            F, Q = jerk_matrices(dt, self.alpha, self.exact)
            FN = np.kron(np.eye(self.N), F)
            self.X = FN @ self.X
            self.P = FN @ self.P @ FN.T + np.kron(np.eye(self.N), self.q * Q)
            idx = [4 * i for i in range(self.N)]
            y = x - self.X[idx]
            corr, self.P, self.S = kalman_update(self.P, idx, y, R)
            self.X = self.X + corr
            self.y = y
        if self.idx < 2:
            self.meas.append(x)
            self.R.append(R)
            if self.idx + 1 == 2:
                self.initialise()
            self.idx += 1

    def initialise(self):
        dt1, dt2 = self.dts
        N = self.N
        self.X = np.concatenate([init_estimate(self.meas[0][i], self.meas[1][i], self.meas[2][i], dt1, dt2) for i in range(N)])
        c = init_coefficients(dt1, dt2)
        Pp = init_process_covariance(dt1, dt2, self.alpha, self.q, self.var_m, self.var_j, self.exact)
        P = np.zeros((4 * N, 4 * N))
        for i in range(N):
            for k in range(N):
                blk = sum(self.R[n][i, k] * np.outer(c[n], c[n]) for n in range(3))
                if i == k:
                    blk = blk + Pp
                P[4 * i:4 * i + 4, 4 * k:4 * k + 4] = blk
        self.P = P

# Sphere geometry

def rotate(r, x):
    t = np.linalg.norm(r)
    if t < 1e-300:
        return x.copy()
    k = r / t
    return x * math.cos(t) + np.cross(k, x) * math.sin(t) + k * (k @ x) * (1 - math.cos(t))

def log_map(u, m):
    tau = m - (u @ m) * u
    n = np.linalg.norm(tau)
    if n < 1e-300:
        return np.zeros(3)
    return math.atan2(n, u @ m) * tau / n

def tangent_basis(c):
    e = np.eye(3)[int(np.argmin(np.abs(c)))]
    b1 = e - (e @ c) * c
    b1 = b1 / np.linalg.norm(b1)
    return np.column_stack([b1, np.cross(c, b1)])

def unit(az, el):
    return np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])

def vec(B, d, k):
    return B @ np.array([d[k], d[4 + k]])

def retract(X, B, d):
    u = X[0:3]
    R = lambda x: rotate(np.cross(u, vec(B, d, 0)), x)
    Xn = np.concatenate([R(u), R(X[3:6] + vec(B, d, 1)), R(X[6:9] + vec(B, d, 2)), R(X[9:12] + vec(B, d, 3))])
    return Xn, np.column_stack([R(B[:, 0]), R(B[:, 1])])

def inverse_retract(Xh, B, X):
    uh = Xh[0:3]
    v = log_map(uh, X[0:3])
    r = np.cross(uh, v)
    d = np.zeros(8)
    comps = [v, rotate(-r, X[3:6]) - Xh[3:6], rotate(-r, X[6:9]) - Xh[6:9], rotate(-r, X[9:12]) - Xh[9:12]]
    for k, c in enumerate(comps):
        t = B.T @ c
        d[k], d[4 + k] = t
    return d

def transport_basis(u0, u1, B):
    r = np.cross(u0, log_map(u0, u1))
    return np.column_stack([rotate(r, B[:, 0]), rotate(r, B[:, 1])])

def normalise(X, B):
    X = X.copy()
    X[0:3] /= np.linalg.norm(X[0:3])
    u = X[0:3]
    for k in (3, 6, 9):
        X[k:k + 3] -= (u @ X[k:k + 3]) * u
    b1 = B[:, 0] - (u @ B[:, 0]) * u
    b1 /= np.linalg.norm(b1)
    return X, np.column_stack([b1, np.cross(u, b1)])

def azel_dynamics(X, alpha):
    u, w, a, j = X[0:3], X[3:6], X[6:9], X[9:12]
    return np.concatenate([w, a - (w @ w) * u, j - (w @ a) * u, -alpha * j - (w @ j) * u])

def azel_error_dynamics(X, B, alpha):
    w, a, j = X[3:6], X[6:9], X[9:12]
    wt, at, jt = B.T @ w, B.T @ a, B.T @ j
    I = np.eye(2)
    K = [(w @ w) * I - np.outer(wt, wt), (w @ a) * I - np.outer(wt, at), (w @ j) * I - np.outer(wt, jt)]
    A = np.zeros((8, 8))
    for i in range(2):
        A[4 * i, 4 * i + 1] = A[4 * i + 1, 4 * i + 2] = A[4 * i + 2, 4 * i + 3] = 1.0
        A[4 * i + 3, 4 * i + 3] = -alpha
        for k in range(2):
            for r in range(3):
                A[4 * i + 1 + r, 4 * k] = -K[r][i, k]
    return A

def azel_flow(X, alpha, dt, n):
    h = dt / n
    for _ in range(n):
        k1 = azel_dynamics(X, alpha)
        k2 = azel_dynamics(X + 0.5 * h * k1, alpha)
        k3 = azel_dynamics(X + 0.5 * h * k2, alpha)
        k4 = azel_dynamics(X + h * k3, alpha)
        X = X + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    return X

def finite_jacobian(f, n, h=1e-6):
    cols = [(8 * (f(h * e) - f(-h * e)) - f(2 * h * e) + f(-2 * h * e)) / (12 * h) for e in np.eye(n)]
    return np.array(cols).T

def finite_hessians(f, n, h=1e-2):
    f0 = f(np.zeros(n))
    E = np.eye(n) * h
    H = np.zeros((len(f0), n, n))
    for k in range(n):
        for l in range(k, n):
            if k == l:
                v = (f(2 * E[k]) - 2 * f0 + f(-2 * E[k])) / (4 * h * h)
            else:
                v = (f(E[k] + E[l]) - f(E[k] - E[l]) - f(-E[k] + E[l]) + f(-E[k] - E[l])) / (4 * h * h)
            H[:, k, l] = H[:, l, k] = v
    return H

def second_order_terms(H, P):
    HP = [H[i] @ P for i in range(H.shape[0])]
    m = 0.5 * np.array([np.trace(x) for x in HP])
    C = 0.5 * np.array([[np.trace(HP[i] @ HP[k]) for k in range(len(HP))] for i in range(len(HP))])
    return m, C

def unscented_points(P):
    n = P.shape[0]
    L = np.linalg.cholesky(n * P)
    pts = [np.zeros(n)] + [L[:, i] for i in range(n)] + [-L[:, i] for i in range(n)]
    wm = np.array([0.0] + [1.0 / (2 * n)] * (2 * n))
    wc = wm.copy()
    wc[0] = 2.0
    return pts, wm, wc

def direction_measurement_basis(m, n):
    b0 = tangent_basis(m[2])
    return b0 if n == 2 else transport_basis(m[2], m[n], b0)

def direction_estimate(m, dts, noise):
    mp = []
    for n in range(3):
        bn = direction_measurement_basis(m, n)
        v = bn @ noise[2 * n:2 * n + 2]
        mp.append(rotate(np.cross(m[n], v), m[n]))
    b = tangent_basis(mp[2])
    p = [b.T @ log_map(mp[2], mp[n]) for n in range(3)]
    axis = [init_estimate(p[0][i], p[1][i], 0.0, dts[0], dts[1]) for i in range(2)]
    X = np.concatenate([mp[2], b @ np.array([axis[0][1], axis[1][1]]), b @ np.array([axis[0][2], axis[1][2]]), np.zeros(3)])
    return X, b

def direction_initialise(m, R_iso, dts, alpha, q_scale, var_m, var_j, exact):
    X, B = direction_estimate(m, dts, np.zeros(6))
    G = finite_jacobian(lambda z: inverse_retract(X, B, direction_estimate(m, dts, z)[0]), 6, 1e-7)
    P = G @ (R_iso * np.eye(6)) @ G.T
    Pp = init_process_covariance(dts[0], dts[1], alpha, q_scale, var_m, var_j, exact)
    for i in range(2):
        P[4 * i:4 * i + 4, 4 * i:4 * i + 4] += Pp
    return X, B, P

class AzEl(Steps):
    def __init__(self, alpha, direction_error, jerk_error, rel, acc=0.0, substeps=4, exact=False, order='first'):
        super().__init__(rel)
        self.alpha, self.exact, self.order, self.n = alpha, exact, order, substeps
        self.var_d, self.var_j, self.var_m = direction_error**2, jerk_error**2, acc**2
        self.q = 2 * alpha * jerk_error**2
        self.meas = []

    def step(self, t, az, el):
        m = unit(az, el)
        dt = self.advance(t)
        if self.idx >= 0 and self.idx < 2:
            self.dts[self.idx] = dt
        elif self.idx >= 2:
            self.predict(dt)
            self.update(m)
        if self.idx < 2:
            self.meas.append(m)
            if self.idx + 1 == 2:
                self.X, self.B, self.P = direction_initialise(self.meas, self.var_d, self.dts, self.alpha, self.q, self.var_m, self.var_j, self.exact)
            self.idx += 1

    def integrate(self, dt):
        ad = self.alpha if self.exact else 0.0
        G = np.zeros((8, 2))
        G[3, 0] = G[7, 1] = 1.0
        def rate(Z):
            X, B, Ph, Q = Z
            A = azel_error_dynamics(X, B, ad)
            dB = np.column_stack([-(X[3:6] @ B[:, 0]) * X[0:3], -(X[3:6] @ B[:, 1]) * X[0:3]])
            return (azel_dynamics(X, ad), dB, A @ Ph, A @ Q + Q @ A.T + self.q * G @ G.T)
        Z = (self.X.copy(), self.B.copy(), np.eye(8), np.zeros((8, 8)))
        h = dt / self.n
        add = lambda Z, K, s: tuple(z + s * k for z, k in zip(Z, K))
        for _ in range(self.n):
            k1 = rate(Z)
            k2 = rate(add(Z, k1, 0.5 * h))
            k3 = rate(add(Z, k2, 0.5 * h))
            k4 = rate(add(Z, k3, h))
            Z = tuple(z + h / 6 * (a + 2 * b + 2 * c + d) for z, a, b, c, d in zip(Z, k1, k2, k3, k4))
        X1, B1 = normalise(Z[0], Z[1])
        return X1, B1, Z[2], 0.5 * (Z[3] + Z[3].T)

    def predict(self, dt):
        ad = self.alpha if self.exact else 0.0
        X1, B1, F, Qd = self.integrate(dt)
        psi = lambda d: inverse_retract(X1, B1, azel_flow(retract(self.X, self.B, d)[0], ad, dt, self.n))
        if self.order == 'first':
            self.X, self.B, self.P = X1, B1, F @ self.P @ F.T + Qd
        elif self.order == 'second':
            m, C = second_order_terms(finite_hessians(psi, 8), self.P)
            self.X, self.B = normalise(*retract(X1, B1, m))
            self.P = F @ self.P @ F.T + C + Qd
        else:
            pts, wm, wc = unscented_points(self.P)
            chi = [azel_flow(retract(self.X, self.B, p)[0], ad, dt, self.n) for p in pts]
            mean = wm @ np.array([inverse_retract(X1, B1, c) for c in chi])
            self.X, self.B = normalise(*retract(X1, B1, mean))
            xi = np.array([inverse_retract(self.X, self.B, c) for c in chi])
            D = xi - wm @ xi
            self.P = (wc[:, None] * D).T @ D + Qd

    def update(self, m):
        v = log_map(self.X[0:3], m)
        y = self.B.T @ v
        corr, self.P, self.S = kalman_update(self.P, [0, 4], y, self.var_d * np.eye(2))
        self.X, self.B = normalise(*retract(self.X, self.B, corr))
        self.y = y

    def kalman_vector(self):
        u = self.X[0:3]
        rho = math.hypot(u[0], u[1])
        e_a = np.array([-u[1] / rho, u[0] / rho, 0.0])
        e_e = np.array([-u[2] * u[0] / rho, -u[2] * u[1] / rho, rho])
        out = np.zeros(8)
        out[0], out[4] = math.atan2(u[1], u[0]), math.atan2(u[2], rho)
        for k in range(1, 4):
            out[k], out[4 + k] = self.X[3 * k:3 * k + 3] @ e_a, self.X[3 * k:3 * k + 3] @ e_e
        return out

# Moving sensor filters

def mpc_to_xi(Y):
    b, bd, bdd, bddd, l1, l2, l3 = Y
    z1, z2, z3 = l1 + 1j * bd, l2 + 1j * bdd, l3 + 1j * bddd
    c0 = complex(math.cos(b), math.sin(b))
    c = [c0, c0 * z1, c0 * (z1**2 + z2), c0 * (z1**3 + 3 * z1 * z2 + z3)]
    return np.array([x.real for x in c] + [x.imag for x in c])

def mpc_from_xi(xi):
    c = [complex(xi[n], xi[4 + n]) for n in range(4)]
    w = [x / c[0] for x in c]
    z1 = w[1]
    z2 = w[2] - z1**2
    z3 = w[3] - 3 * z1 * z2 - z1**3
    return np.array([math.atan2(xi[4], xi[0]), z1.imag, z2.imag, z3.imag, z1.real, z2.real, z3.real, abs(c[0])])

def cartesian_to_mpc(rel):
    y = mpc_from_xi(rel)
    out = y.copy()
    out[7] = 1.0 / y[7]
    return out

class Moving(Steps):
    def __init__(self, alpha, angle_error, jerk_error, rel, range_min, range_max, l1, l2, l3, acc=0.0, exact=False, order='first'):
        super().__init__(rel)
        self.alpha, self.exact, self.order = alpha, exact, order
        self.var_a, self.jerk, self.acc = angle_error**2, jerk_error, acc
        self.q_scale = 2 * alpha * jerk_error**2
        self.rmin, self.rmax = range_min, range_max
        self.lvar = np.array([l1, l2, l3])**2
        self.sensor = None

    def propagate_terms(self, dt, sensor, axes):
        F, Q = jerk_matrices(dt, self.alpha, self.exact)
        FN = np.kron(np.eye(axes), F)
        return FN, FN @ self.sensor - sensor, np.kron(np.eye(axes), self.q_scale * Q)

    def order_propagate(self, psi, reexpress, n_e, Qv):
        n = n_e + Qv.shape[0]
        Pz = np.zeros((n, n))
        Pz[:n_e, :n_e] = self.P
        Pz[n_e:, n_e:] = Qv
        if self.order == 'unscented':
            pts, wm, wc = unscented_points(Pz)
            xi = np.array([psi(p) for p in pts])
            mean = wm @ xi
            xi = np.array([reexpress(mean, x) for x in xi])
            D = xi - wm @ xi
            return mean, (wc[:, None] * D).T @ D
        vals, vecs = np.linalg.eigh(Pz)
        L = vecs * np.sqrt(np.maximum(vals, 0.0))
        white = lambda z: psi(L @ z)
        J = finite_jacobian(white, n, 1e-2)
        if self.order == 'first':
            return np.zeros(n_e), J @ J.T
        m, C = second_order_terms(finite_hessians(white, n), np.eye(n))
        return m, J @ J.T + C

class BearingMovingSensor(Moving):
    def step(self, t, bearing, sensor):
        sensor = np.asarray(sensor, float)
        dt = self.advance(t)
        if self.idx >= 0 and self.idx < 2:
            self.dts[self.idx] = dt
        elif self.idx >= 2:
            self.predict(dt, sensor)
            self.update(bearing)
        if self.idx < 2:
            if self.idx == -1:
                self.bearings = []
            self.bearings.append(bearing)
            if self.idx + 1 == 2:
                self.initialise()
            self.idx += 1
        self.sensor = sensor

    def initialise(self):
        b1 = self.bearings[0]
        b2 = b1 + wrap(self.bearings[1] - b1)
        b3 = b2 + wrap(self.bearings[2] - self.bearings[1])
        est = init_estimate(b1, b2, b3, *self.dts)
        qmin, qmax = 1 / self.rmax, 1 / self.rmin
        self.Y = np.array([wrap(est[0]), est[1], est[2], 0, 0, 0, 0, 0.5 * (qmin + qmax)])
        c = init_coefficients(*self.dts)
        P = np.zeros((8, 8))
        P[:4, :4] = self.var_a * sum(np.outer(c[n], c[n]) for n in range(3))
        sja, sma = self.jerk * qmax, self.acc * qmax
        P[:4, :4] += init_process_covariance(*self.dts, self.alpha, 2 * self.alpha * sja**2, sma**2, sja**2, self.exact)
        P[4:7, 4:7] = np.diag(self.lvar)
        P[7, 7] = (qmax - qmin)**2 / 12
        self.P = P

    def predict(self, dt, sensor):
        F2, term, Qv = self.propagate_terms(dt, sensor, 2)
        def raw(z):
            Y = self.Y + z[:8]
            q = Y[7]
            xi = F2 @ mpc_to_xi(Y[:7]) + q * (term + z[8:])
            y = mpc_from_xi(xi)
            return np.concatenate([y[:7], [q / y[7]]])
        Yp = raw(np.zeros(16))
        def psi(z):
            e = raw(z) - Yp
            e[0] = wrap(e[0])
            return e
        mean, P = self.order_propagate(psi, lambda m, x: x - m, 8, Qv)
        self.Y = Yp + mean
        self.Y[0] = wrap(self.Y[0])
        self.P = 0.5 * (P + P.T)

    def update(self, bearing):
        y = np.array([wrap(bearing - self.Y[0])])
        corr, self.P, self.S = kalman_update(self.P, [0], y, np.array([[self.var_a]]))
        self.Y = self.Y + corr
        self.Y[0] = wrap(self.Y[0])
        self.y = y

def msc_to_xi(X, lam):
    u, w, a, j = X[0:3], X[3:6], X[6:9], X[9:12]
    l1, l2, l3 = lam
    ww, wa = w @ w, w @ a
    e = [u, l1 * u + w, (l2 + l1**2 - ww) * u + 2 * l1 * w + a,
         (l3 + 3 * l1 * l2 + l1**3 - 3 * l1 * ww - 3 * wa) * u + 3 * (l2 + l1**2) * w + 3 * l1 * a + j - ww * w]
    return np.array([e[n][i] for i in range(3) for n in range(4)])

def msc_from_xi(xi):
    E = [np.array([xi[4 * i + n] for i in range(3)]) for n in range(4)]
    s = np.linalg.norm(E[0])
    E = [e / s for e in E]
    u = E[0]
    T = lambda v: v - (u @ v) * u
    l1 = u @ E[1]
    w = E[1] - l1 * u
    ww = w @ w
    l2 = u @ E[2] - l1**2 + ww
    a = T(E[2]) - 2 * l1 * w
    l3 = u @ E[3] - 3 * l1 * l2 - l1**3 + 3 * l1 * ww + 3 * (w @ a)
    j = T(E[3]) - 3 * (l2 + l1**2) * w - 3 * l1 * a + ww * w
    return np.concatenate([u, w, a, j]), np.array([l1, l2, l3]), s

class AzElMovingSensor(Moving):
    def step(self, t, az, el, sensor):
        sensor = np.asarray(sensor, float)
        m = unit(az, el)
        dt = self.advance(t)
        if self.idx >= 0 and self.idx < 2:
            self.dts[self.idx] = dt
        elif self.idx >= 2:
            self.predict(dt, sensor)
            self.update(m)
        if self.idx < 2:
            if self.idx == -1:
                self.meas = []
            self.meas.append(m)
            if self.idx + 1 == 2:
                self.initialise()
            self.idx += 1
        self.sensor = sensor

    def initialise(self):
        qmin, qmax = 1 / self.rmax, 1 / self.rmin
        sja, sma = self.jerk * qmax, self.acc * qmax
        self.X, self.B, P8 = direction_initialise(self.meas, self.var_a, self.dts, self.alpha, 2 * self.alpha * sja**2, sma**2, sja**2, self.exact)
        self.P = np.zeros((12, 12))
        self.P[:8, :8] = P8
        self.P[8:11, 8:11] = np.diag(self.lvar)
        self.lam = np.zeros(3)
        self.q = 0.5 * (qmin + qmax)
        self.P[11, 11] = (qmax - qmin)**2 / 12

    def predict(self, dt, sensor):
        F3, term, Qv = self.propagate_terms(dt, sensor, 3)
        def raw(z):
            X, B = retract(self.X, self.B, z[:8])
            lam = self.lam + z[8:11]
            q = self.q + z[11]
            xi = F3 @ msc_to_xi(X, lam) + q * (term + z[12:])
            Xn, lamn, s = msc_from_xi(xi)
            return Xn, lamn, q / s
        Xp, lp, qp = raw(np.zeros(24))
        Bp = transport_basis(self.X[0:3], Xp[0:3], self.B)
        def psi(z):
            Xn, lamn, qn = raw(z)
            return np.concatenate([inverse_retract(Xp, Bp, Xn), lamn - lp, [qn - qp]])
        def reexpress(mean, xi):
            Xs, _ = retract(Xp, Bp, xi[:8])
            Xm, Bm = retract(Xp, Bp, mean[:8])
            return np.concatenate([inverse_retract(Xm, Bm, Xs), xi[8:] - mean[8:]])
        mean, P = self.order_propagate(psi, reexpress, 12, Qv)
        self.X, self.B = normalise(*retract(Xp, Bp, mean[:8]))
        self.lam = lp + mean[8:11]
        self.q = qp + mean[11]
        self.P = 0.5 * (P + P.T)

    def update(self, m):
        y = self.B.T @ log_map(self.X[0:3], m)
        corr, self.P, self.S = kalman_update(self.P, [0, 4], y, self.var_a * np.eye(2))
        self.X, self.B = normalise(*retract(self.X, self.B, corr[:8]))
        self.lam = self.lam + corr[8:11]
        self.q = self.q + corr[11]
        self.y = y
