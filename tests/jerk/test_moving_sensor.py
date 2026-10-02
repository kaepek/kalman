import math
import numpy as np
import pytest
import kalman
from helpers import assert_state_close, assert_covariance_close
from jerk_reference import BearingMovingSensor, AzElMovingSensor

FORMS = [("small_alpha_t", False), ("exact", True)]
ORDERS = ["first", "second", "unscented"]
TOLERANCE = {"first": 1e-8, "second": 1e-5, "unscented": 1e-8}

def planar_scenario(n, dt):
    out = []
    for k in range(n):
        t = k * dt
        s = np.array([5 * t, 5, 0, 0, 0.5 * math.sin(t), 0.5 * math.cos(t), -0.5 * math.sin(t), -0.5 * math.cos(t)])
        rel = np.array([100 + 2 * t, -10 * math.sin(0.1 * t)]) - s[[0, 4]]
        out.append((t, math.atan2(rel[1], rel[0]), s, np.linalg.norm(rel)))
    return out

def spatial_scenario(n, dt):
    out = []
    for k in range(n):
        t = k * dt
        s = np.array([5 * t, 5, 0, 0, 0.5 * math.sin(t), 0.5 * math.cos(t), -0.5 * math.sin(t), -0.5 * math.cos(t), 0, 0, 0, 0])
        rel = np.array([100 + 2 * t, -10 * math.sin(0.1 * t), 20 + t]) - s[[0, 4, 8]]
        u = rel / np.linalg.norm(rel)
        out.append((t, math.atan2(u[1], u[0]), math.asin(u[2]), s, np.linalg.norm(rel)))
    return out

@pytest.mark.parametrize("form,exact", FORMS)
@pytest.mark.parametrize("order", ORDERS)
def test_bearing_matches_reference(form, exact, order):
    args = (1.0, 1e-4, 0.5, False, 70.0, 140.0, 0.05, 0.01, 0.01, 0.1)
    f = kalman.KalmanJerk1DBearingMovingSensor(*args, form=form, order=order)
    ref = BearingMovingSensor(*args, exact=exact, order=order)
    tol = TOLERANCE[order]
    for k, (t, b, s, r) in enumerate(planar_scenario(60, 0.05)):
        f.step(t, b, s)
        ref.step(t, b, s)
        if k >= 2:
            assert_state_close(f.get_kalman_vector(), ref.Y, ref.P, tol)
            assert_covariance_close(f.get_covariance_matrix(), ref.P, tol)
    np.testing.assert_allclose(f.get_innovation() / np.sqrt(ref.S[0, 0]), ref.y / np.sqrt(ref.S[0, 0]), rtol=0, atol=tol)
    assert f.get_range() == pytest.approx(1.0 / ref.Y[7], rel=tol)

@pytest.mark.parametrize("form,exact", FORMS)
@pytest.mark.parametrize("order", ORDERS)
def test_azel_matches_reference(form, exact, order):
    args = (1.0, 1e-4, 0.5, False, 70.0, 140.0, 0.05, 0.01, 0.01, 0.1)
    f = kalman.KalmanJerk2DAzElMovingSensor(*args, form=form, order=order)
    ref = AzElMovingSensor(*args, exact=exact, order=order)
    tol = TOLERANCE[order]
    for k, (t, az, el, s, r) in enumerate(spatial_scenario(20 if order == "second" else 40, 0.05)):
        f.step(t, az, el, s)
        ref.step(t, az, el, s)
        if k >= 2:
            np.testing.assert_allclose(f.get_state_vector(), ref.X, rtol=0, atol=tol * 1e-2)
            np.testing.assert_allclose(f.get_basis(), ref.B.T.reshape(-1), rtol=0, atol=tol * 1e-2)
            assert_covariance_close(f.get_covariance_matrix(), ref.P, tol)
    assert f.get_range() == pytest.approx(1.0 / ref.q, rel=tol)

def test_bearing_first_order_range():
    f = kalman.KalmanJerk1DBearingMovingSensor(1.0, 1e-4, 0.5, False, 70.0, 140.0, 0.05, 0.01, 0.01, 0.1)
    for t, b, s, r in planar_scenario(400, 0.05):
        f.step(t, b, s)
    assert abs(f.get_range() - r) < 0.1 * r

def test_azel_first_order_range():
    f = kalman.KalmanJerk2DAzElMovingSensor(1.0, 1e-4, 0.5, False, 70.0, 140.0, 0.05, 0.01, 0.01, 0.1)
    for t, az, el, s, r in spatial_scenario(400, 0.05):
        f.step(t, az, el, s)
    assert abs(f.get_range() - r) < 0.1 * r
