import math
import numpy as np
import pytest
import kalman
from jerk_reference import Cartesian, jerk_matrices, init_coefficients, init_process_covariance

FORMS = [("small_alpha_t", False), ("exact", True)]

def trajectory(N, n, seed):
    rng = np.random.default_rng(seed)
    t = np.cumsum(rng.uniform(0.008, 0.012, n))
    X = np.stack([np.sin(0.7 * (i + 1) * t) * (i + 2) + 0.3 * t for i in range(N)], axis=1)
    return t, X + rng.normal(0, 0.01, X.shape)

@pytest.mark.parametrize("form,exact", FORMS)
@pytest.mark.parametrize("N", [2, 3])
def test_matches_reference(form, exact, N):
    cls = kalman.KalmanJerk2D if N == 2 else kalman.KalmanJerk3D
    f = cls(2.0, 0.01, 5.0, False, 0.5, form=form)
    ref = Cartesian(N, 2.0, 0.01, 5.0, False, 0.5, exact)
    t, X = trajectory(N, 200, N)
    for k in range(len(t)):
        f.step(t[k], X[k])
        ref.step(t[k], X[k])
        if k >= 2:
            np.testing.assert_allclose(f.get_kalman_vector(), ref.X, rtol=1e-8, atol=1e-8)
            np.testing.assert_allclose(f.get_covariance_matrix(), ref.P, rtol=1e-8, atol=1e-12)
    np.testing.assert_allclose(f.get_innovation(), ref.y, rtol=1e-8, atol=1e-10)
    np.testing.assert_allclose(f.get_innovation_covariance(), ref.S, rtol=1e-8, atol=1e-14)
    np.testing.assert_allclose(f.get_innovation_covariance_inverse(), np.linalg.inv(ref.S), rtol=1e-8)
    assert f.get_innovation_covariance_determinant() == pytest.approx(np.linalg.det(ref.S), rel=1e-8)

@pytest.mark.parametrize("form,exact", FORMS)
def test_full_measurement_covariance(form, exact):
    f = kalman.KalmanJerk2D(2.0, 0.01, 5.0, True, form=form)
    ref = Cartesian(2, 2.0, 0.01, 5.0, True, 0.0, exact)
    rng = np.random.default_rng(7)
    for k in range(100):
        A = rng.normal(0, 0.01, (2, 2))
        R = A @ A.T + 1e-5 * np.eye(2)
        x = [math.cos(0.01 * k), math.sin(0.02 * k)]
        f.step(0.01, x, R)
        ref.step(0.01, x, R)
    np.testing.assert_allclose(f.get_kalman_vector(), ref.X, rtol=1e-8, atol=1e-8)
    np.testing.assert_allclose(f.get_covariance_matrix(), ref.P, rtol=1e-8, atol=1e-12)

def test_eular_vector():
    f = kalman.KalmanJerk2D(2.0, 0.01, 5.0, False)
    t, X = trajectory(2, 20, 3)
    for k in range(len(t)):
        f.step(t[k], X[k])
    e = f.get_eular_vector()
    assert e.shape == (9,)
    assert e[0] == t[-1]

@pytest.mark.parametrize("dts", [(0.01, 0.01), (0.008, 0.013)])
def test_exact_initial_covariance_monte_carlo(dts):
    alpha, sj, sm = 2.0, 3.0, 0.5
    q = 2 * alpha * sj**2
    F1, Q1 = jerk_matrices(dts[0], alpha, True)
    F2, Q2 = jerk_matrices(dts[1], alpha, True)
    rng = np.random.default_rng(11)
    n = 200000
    x1 = np.zeros((n, 4))
    x1[:, 2] = rng.normal(0, sm, n)
    x1[:, 3] = rng.normal(0, sj, n)
    x2 = x1 @ F1.T + rng.multivariate_normal(np.zeros(4), q * Q1, n)
    x3 = x2 @ F2.T + rng.multivariate_normal(np.zeros(4), q * Q2, n)
    c = init_coefficients(*dts)
    est = np.outer(x1[:, 0], c[0]) + np.outer(x2[:, 0], c[1]) + np.outer(x3[:, 0], c[2])
    err = est - x3
    P_mc = err.T @ err / n
    P = init_process_covariance(dts[0], dts[1], alpha, q, sm**2, sj**2, True)
    assert abs(P[0]).max() < 1e-12
    scale = np.sqrt(np.outer(np.diag(P)[1:], np.diag(P)[1:]))
    np.testing.assert_allclose(P_mc[1:, 1:] / scale, P[1:, 1:] / scale, atol=0.02)

@pytest.mark.parametrize("form", ["small_alpha_t", "exact"])
def test_polar_matches_cartesian(form):
    p = kalman.KalmanJerk2DPolar(2.0, 0.05, 0.01, 5.0, False, form=form)
    ref = Cartesian(2, 2.0, 0.0, 5.0, False, 0.0, form == "exact")
    for k in range(60):
        t = 0.01 * k
        r, az = 10 + t, 0.3 + 0.2 * t
        p.step(t, r, az)
        c, s = math.cos(az), math.sin(az)
        J = np.array([[c, -r * s], [s, r * c]])
        ref.step(t, [r * c, r * s], J @ np.diag([0.05**2, 0.01**2]) @ J.T)
    np.testing.assert_allclose(p.get_kalman_vector(), ref.X, rtol=1e-8, atol=1e-8)
    np.testing.assert_allclose(p.get_covariance_matrix(), ref.P, rtol=1e-8, atol=1e-12)

@pytest.mark.parametrize("form", ["small_alpha_t", "exact"])
def test_spherical_matches_cartesian(form):
    p = kalman.KalmanJerk3DSpherical(2.0, 0.05, 0.01, 0.02, 5.0, False, form=form)
    ref = Cartesian(3, 2.0, 0.0, 5.0, False, 0.0, form == "exact")
    for k in range(60):
        t = 0.01 * k
        r, az, el = 10 + t, 0.3 + 0.2 * t, 0.4 - 0.1 * t
        p.step(t, r, az, el)
        ce, se, ca, sa = math.cos(el), math.sin(el), math.cos(az), math.sin(az)
        J = np.array([[ce * ca, -r * ce * sa, -r * se * ca], [ce * sa, r * ce * ca, -r * se * sa], [se, 0, r * ce]])
        ref.step(t, [r * ce * ca, r * ce * sa, r * se], J @ np.diag([0.05**2, 0.01**2, 0.02**2]) @ J.T)
    np.testing.assert_allclose(p.get_kalman_vector(), ref.X, rtol=1e-8, atol=1e-8)
    np.testing.assert_allclose(p.get_covariance_matrix(), ref.P, rtol=1e-8, atol=1e-12)

def test_bad_form():
    with pytest.raises(ValueError):
        kalman.KalmanJerk2D(2.0, 0.01, 5.0, False, form="other")
