import math
import numpy as np
import pytest
import kalman
from helpers import assert_state_close, assert_covariance_close
from test_azel import track
from test_moving_sensor import planar_scenario, spatial_scenario

torch = pytest.importorskip("torch")
import kalman.torch as kt

ORDERS = ["first", "second", "unscented"]
TOLERANCE = 1e-8
MOVING = (1.0, 1e-4, 0.5, False, 70.0, 140.0, 0.05, 0.01, 0.01, 0.1)

def numpy(t):
    return t.detach().cpu().numpy()

def compiled(f):
    return f.compile(fullgraph=True)

@pytest.mark.parametrize("form", ["small_alpha_t", "exact"])
def test_cartesian(form):
    f = compiled(kt.KalmanJerk3D(2.0, 0.01, 5.0, False, 0.5, form=form))
    c = kalman.KalmanJerk3D(2.0, 0.01, 5.0, False, 0.5, form=form)
    for k in range(60):
        x = [math.sin(0.01 * k), math.cos(0.02 * k), 0.1 * k]
        f.step(0.01 * k, x)
        c.step(0.01 * k, x)
        if k >= 2:
            assert_state_close(numpy(f.get_kalman_vector()), c.get_kalman_vector(), c.get_covariance_matrix(), TOLERANCE)
            assert_covariance_close(numpy(f.get_covariance_matrix()), c.get_covariance_matrix(), TOLERANCE)

@pytest.mark.parametrize("order", ORDERS)
def test_azel(order):
    f = compiled(kt.KalmanJerk2DAzEl(1.0, 1e-3, 0.5, False, 0.1, 4, order=order))
    c = kalman.KalmanJerk2DAzEl(1.0, 1e-3, 0.5, False, 0.1, 4, order=order)
    R = np.array([[2e-6, 5e-7], [5e-7, 1e-6]])
    for k, (t, az, el) in enumerate(track(40, 0.05, 1)):
        args = (t, az, el, R) if k % 2 else (t, az, el)
        f.step(*args)
        c.step(*args)
        if k >= 2:
            np.testing.assert_allclose(numpy(f.get_state_vector()), c.get_state_vector(), rtol=0, atol=1e-10)
            assert_covariance_close(numpy(f.get_basis_covariance_matrix()), c.get_basis_covariance_matrix(), TOLERANCE)

@pytest.mark.parametrize("order", ORDERS)
def test_bearing_moving_sensor(order):
    f = compiled(kt.KalmanJerk1DBearingMovingSensor(*MOVING, order=order))
    c = kalman.KalmanJerk1DBearingMovingSensor(*MOVING, order=order)
    for k, (t, b, s, r) in enumerate(planar_scenario(40, 0.05)):
        f.step(t, b, s)
        c.step(t, b, s)
        if k >= 2:
            assert_state_close(numpy(f.get_kalman_vector()), c.get_kalman_vector(), c.get_covariance_matrix(), TOLERANCE)
            assert_covariance_close(numpy(f.get_covariance_matrix()), c.get_covariance_matrix(), TOLERANCE)

@pytest.mark.parametrize("order", ORDERS)
def test_azel_moving_sensor(order):
    f = compiled(kt.KalmanJerk2DAzElMovingSensor(*MOVING, order=order))
    c = kalman.KalmanJerk2DAzElMovingSensor(*MOVING, order=order)
    for k, (t, az, el, s, r) in enumerate(spatial_scenario(30, 0.05)):
        f.step(t, az, el, s)
        c.step(t, az, el, s)
        if k >= 2:
            np.testing.assert_allclose(numpy(f.get_state_vector()), c.get_state_vector(), rtol=0, atol=1e-10)
            assert_covariance_close(numpy(f.get_covariance_matrix()), c.get_covariance_matrix(), TOLERANCE)
