import math
import numpy as np
import pytest
import kalman
from helpers import assert_state_close, assert_covariance_close
from test_azel import track
from test_moving_sensor import planar_scenario, spatial_scenario

torch = pytest.importorskip("torch")
import kalman.torch as kt

FORMS = ["small_alpha_t", "exact"]
ORDERS = ["first", "second", "unscented"]
TOLERANCE = 1e-8

def numpy(t):
    return t.detach().cpu().numpy()

@pytest.mark.parametrize("form", FORMS)
@pytest.mark.parametrize("N", [2, 3])
def test_cartesian(form, N):
    f = (kt.KalmanJerk2D if N == 2 else kt.KalmanJerk3D)(2.0, 0.01, 5.0, False, 0.5, form=form)
    c = (kalman.KalmanJerk2D if N == 2 else kalman.KalmanJerk3D)(2.0, 0.01, 5.0, False, 0.5, form=form)
    rng = np.random.default_rng(N)
    t = np.cumsum(rng.uniform(0.008, 0.012, 100))
    for k in range(len(t)):
        x = [math.sin(0.7 * (i + 1) * t[k]) * (i + 2) + rng.normal(0, 0.01) for i in range(N)]
        f.step(t[k], x)
        c.step(t[k], x)
        if k >= 2:
            assert_state_close(numpy(f.get_kalman_vector()), c.get_kalman_vector(), c.get_covariance_matrix(), TOLERANCE)
            assert_covariance_close(numpy(f.get_covariance_matrix()), c.get_covariance_matrix(), TOLERANCE)
    np.testing.assert_allclose(numpy(f.get_eular_vector()), c.get_eular_vector(), rtol=1e-12)
    np.testing.assert_allclose(numpy(f.get_innovation_covariance()), c.get_innovation_covariance(), rtol=1e-9)
    assert float(f.get_innovation_covariance_determinant()) == pytest.approx(c.get_innovation_covariance_determinant(), rel=1e-9)

def test_cartesian_full_measurement_covariance():
    f = kt.KalmanJerk2D(2.0, 0.01, 5.0, True)
    c = kalman.KalmanJerk2D(2.0, 0.01, 5.0, True)
    rng = np.random.default_rng(7)
    for k in range(50):
        A = rng.normal(0, 0.01, (2, 2))
        R = A @ A.T + 1e-5 * np.eye(2)
        x = [math.cos(0.01 * k), math.sin(0.02 * k)]
        f.step(0.01, x, R)
        c.step(0.01, x, R)
    assert_covariance_close(numpy(f.get_covariance_matrix()), c.get_covariance_matrix(), TOLERANCE)

@pytest.mark.parametrize("form", FORMS)
def test_polar_spherical(form):
    p, cp = kt.KalmanJerk2DPolar(2.0, 0.05, 0.01, 5.0, False, form=form), kalman.KalmanJerk2DPolar(2.0, 0.05, 0.01, 5.0, False, form=form)
    s = kt.KalmanJerk3DSpherical(2.0, 0.05, 0.01, 0.02, 5.0, False, form=form)
    cs = kalman.KalmanJerk3DSpherical(2.0, 0.05, 0.01, 0.02, 5.0, False, form=form)
    for k in range(40):
        t = 0.01 * k
        p.step(t, 10 + t, 0.3 + 0.2 * t)
        cp.step(t, 10 + t, 0.3 + 0.2 * t)
        s.step(t, 10 + t, 0.3 + 0.2 * t, 0.4 - 0.1 * t)
        cs.step(t, 10 + t, 0.3 + 0.2 * t, 0.4 - 0.1 * t)
    assert_covariance_close(numpy(p.get_covariance_matrix()), cp.get_covariance_matrix(), TOLERANCE)
    assert_covariance_close(numpy(s.get_covariance_matrix()), cs.get_covariance_matrix(), TOLERANCE)
    assert_state_close(numpy(s.get_kalman_vector()), cs.get_kalman_vector(), cs.get_covariance_matrix(), TOLERANCE)

@pytest.mark.parametrize("form", FORMS)
@pytest.mark.parametrize("order", ORDERS)
def test_azel(form, order):
    f = kt.KalmanJerk2DAzEl(1.0, 1e-3, 0.5, False, 0.1, 4, form=form, order=order)
    c = kalman.KalmanJerk2DAzEl(1.0, 1e-3, 0.5, False, 0.1, 4, form=form, order=order)
    R = np.array([[2e-6, 5e-7], [5e-7, 1e-6]])
    for k, (t, az, el) in enumerate(track(40, 0.05, 1)):
        if k % 2:
            f.step(t, az, el, R)
            c.step(t, az, el, R)
        else:
            f.step(t, az, el)
            c.step(t, az, el)
        if k >= 2:
            np.testing.assert_allclose(numpy(f.get_state_vector()), c.get_state_vector(), rtol=0, atol=1e-10)
            np.testing.assert_allclose(numpy(f.get_basis()), c.get_basis(), rtol=0, atol=1e-10)
            assert_covariance_close(numpy(f.get_basis_covariance_matrix()), c.get_basis_covariance_matrix(), TOLERANCE)
    np.testing.assert_allclose(numpy(f.get_kalman_vector()), c.get_kalman_vector(), rtol=1e-9, atol=1e-12)
    assert_covariance_close(numpy(f.get_covariance_matrix()), c.get_covariance_matrix(), TOLERANCE)
    np.testing.assert_allclose(numpy(f.get_innovation()), c.get_innovation(), rtol=1e-7, atol=1e-12)

@pytest.mark.parametrize("form", FORMS)
@pytest.mark.parametrize("order", ORDERS)
def test_bearing_moving_sensor(form, order):
    args = (1.0, 1e-4, 0.5, False, 70.0, 140.0, 0.05, 0.01, 0.01, 0.1)
    f = kt.KalmanJerk1DBearingMovingSensor(*args, form=form, order=order)
    c = kalman.KalmanJerk1DBearingMovingSensor(*args, form=form, order=order)
    for k, (t, b, s, r) in enumerate(planar_scenario(60, 0.05)):
        f.step(t, b, s)
        c.step(t, b, s)
        if k >= 2:
            assert_state_close(numpy(f.get_kalman_vector()), c.get_kalman_vector(), c.get_covariance_matrix(), TOLERANCE)
            assert_covariance_close(numpy(f.get_covariance_matrix()), c.get_covariance_matrix(), TOLERANCE)
    assert float(f.get_range()) == pytest.approx(c.get_range(), rel=1e-9)

@pytest.mark.parametrize("form", FORMS)
@pytest.mark.parametrize("order", ORDERS)
def test_azel_moving_sensor(form, order):
    args = (1.0, 1e-4, 0.5, False, 70.0, 140.0, 0.05, 0.01, 0.01, 0.1)
    f = kt.KalmanJerk2DAzElMovingSensor(*args, form=form, order=order)
    c = kalman.KalmanJerk2DAzElMovingSensor(*args, form=form, order=order)
    for k, (t, az, el, s, r) in enumerate(spatial_scenario(40, 0.05)):
        f.step(t, az, el, s)
        c.step(t, az, el, s)
        if k >= 2:
            np.testing.assert_allclose(numpy(f.get_state_vector()), c.get_state_vector(), rtol=0, atol=1e-10)
            assert_covariance_close(numpy(f.get_covariance_matrix()), c.get_covariance_matrix(), TOLERANCE)
    np.testing.assert_allclose(numpy(f.get_kalman_vector()), c.get_kalman_vector(), rtol=1e-9, atol=1e-12)
    assert float(f.get_range()) == pytest.approx(c.get_range(), rel=1e-9)

def test_forward_and_dtype():
    f = kt.KalmanJerk2D(2.0, 0.01, 5.0, True).to(torch.float32)
    for k in range(5):
        out = f(0.01, [0.01 * k, 0.0])
    assert out.dtype == torch.float32 and out.shape == (8,)

def test_bad_arguments():
    with pytest.raises(ValueError):
        kt.KalmanJerk2DAzEl(1.0, 1e-3, 0.5, order="third")
    with pytest.raises(ValueError):
        kt.KalmanJerk2D(1.0, 1e-3, 0.5, form="other")

def test_kalman_does_not_import_torch():
    import subprocess, sys
    code = "import sys, kalman; assert 'torch' not in sys.modules"
    subprocess.run([sys.executable, "-c", code], check=True, cwd=str(__import__("pathlib").Path(__file__).resolve().parent.parent.parent))

def test_generated_batches():
    from kalman.torch.generated import jerk_block, msc
    T = torch.tensor([0.05, 0.2, 1.0], dtype=torch.float64)
    alpha = torch.tensor([3.0, 1.0, 2.0], dtype=torch.float64)
    Q = jerk_block.jerk_process_noise_exact(T, alpha)
    assert Q.shape == (3, 4, 4)
    for i in range(3):
        torch.testing.assert_close(Q[i], jerk_block.jerk_process_noise_exact(T[i], alpha[i]))
    xi = torch.randn(5, 2, 12, dtype=torch.float64)
    y = msc.msc_from_xi(xi)
    assert y.shape == (5, 2, 16)
    torch.testing.assert_close(y[3, 1], msc.msc_from_xi(xi[3, 1]))
