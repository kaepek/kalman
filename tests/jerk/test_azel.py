import math
import numpy as np
import pytest
import kalman
from jerk_reference import AzEl

FORMS = [("small_alpha_t", False), ("exact", True)]
ORDERS = ["first", "second", "unscented"]

def track(n, dt, seed, overhead=False):
    rng = np.random.default_rng(seed)
    out = []
    for k in range(n):
        t = k * dt
        if overhead:
            p = np.array([-50 + 40 * t, 0.5, 20.0])
        else:
            p = np.array([30 * math.cos(0.4 * t), 30 * math.sin(0.4 * t), 10 + 2 * t])
        u = p / np.linalg.norm(p)
        out.append((t, math.atan2(u[1], u[0]) + rng.normal(0, 1e-3), math.asin(u[2]) + rng.normal(0, 1e-3)))
    return out

@pytest.mark.parametrize("form,exact", FORMS)
@pytest.mark.parametrize("order", ORDERS)
def test_matches_reference(form, exact, order):
    f = kalman.KalmanJerk2DAzEl(1.0, 1e-3, 0.5, False, 0.1, 4, form=form, order=order)
    ref = AzEl(1.0, 1e-3, 0.5, False, 0.1, 4, exact, order)
    for k, (t, az, el) in enumerate(track(80, 0.05, 1)):
        f.step(t, az, el)
        ref.step(t, az, el)
        if k >= 2:
            np.testing.assert_allclose(f.get_state_vector(), ref.X, rtol=1e-6, atol=1e-8)
            np.testing.assert_allclose(f.get_basis(), ref.B.T.reshape(-1), rtol=1e-6, atol=1e-8)
            np.testing.assert_allclose(f.get_basis_covariance_matrix(), ref.P, rtol=1e-5, atol=1e-11)
    np.testing.assert_allclose(f.get_kalman_vector(), ref.kalman_vector(), rtol=1e-6, atol=1e-8)
    np.testing.assert_allclose(f.get_innovation(), ref.y, rtol=1e-5, atol=1e-9)
    np.testing.assert_allclose(f.get_innovation_covariance(), ref.S, rtol=1e-5, atol=1e-14)

@pytest.mark.parametrize("order", ORDERS)
def test_overhead_pass(order):
    f = kalman.KalmanJerk2DAzEl(1.0, 1e-3, 2.0, False, 0.1, 4, order=order)
    for k, (t, az, el) in enumerate(track(250, 0.01, 2, overhead=True)):
        f.step(t, az, el)
        if k < 2:
            continue
        x = f.get_state_vector()
        assert np.all(np.isfinite(x))
        assert abs(np.linalg.norm(x[0:3]) - 1) < 1e-12
    p = np.array([-50 + 40 * 2.49, 0.5, 20.0])
    u = p / np.linalg.norm(p)
    assert np.linalg.norm(f.get_state_vector()[0:3] - u) < 5e-3

def test_full_measurement_covariance():
    R = np.array([[2e-6, 5e-7], [5e-7, 1e-6]])
    f = kalman.KalmanJerk2DAzEl(1.0, 1e-3, 0.5, False)
    g = kalman.KalmanJerk2DAzEl(1.0, 1e-3, 0.5, False)
    for t, az, el in track(30, 0.05, 3):
        f.step(t, az, el, R)
        g.step(t, az, el, 1e-6 * np.eye(2))
    assert not np.allclose(f.get_basis_covariance_matrix(), g.get_basis_covariance_matrix())
    h = kalman.KalmanJerk2DAzEl(1.0, 1e-3, 0.5, False)
    for t, az, el in track(30, 0.05, 3):
        h.step(t, az, el)
    np.testing.assert_allclose(g.get_basis_covariance_matrix(), h.get_basis_covariance_matrix(), rtol=1e-10, atol=1e-16)

def test_noise_to_tangent():
    R = kalman.KalmanJerk2DAzEl.azel_noise_to_tangent(0.5, 1e-3, 2e-3)
    np.testing.assert_allclose(R, np.diag([(math.cos(0.5) * 1e-3)**2, (2e-3)**2]))

def test_bad_order():
    with pytest.raises(ValueError):
        kalman.KalmanJerk2DAzEl(1.0, 1e-3, 0.5, False, order="third")
