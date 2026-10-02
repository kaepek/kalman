import ctypes
import math
import pathlib
import shutil
import subprocess
import sys
import numpy as np
import pytest
import jerk_reference as ref

root = pathlib.Path(__file__).resolve().parent.parent.parent
generated = root / "lib" / "jerk" / "generated"
torch_generated = root / "kalman" / "torch" / "generated"


SOURCE = """
#include "generated/jerk_block.hpp"
#include "generated/polar_spherical.hpp"
#include "generated/azel.hpp"
#include "generated/mpc.hpp"
#include "generated/msc.hpp"
using namespace kaepek;
extern "C" {
void c_jerk_transition_small(double T, double *F) { jerk_transition_small(T, (double (*)[4])F); }
void c_jerk_process_noise_small(double T, double *Q) { jerk_process_noise_small(T, (double (*)[4])Q); }
void c_jerk_transition_exact(double T, double alpha, double *F) { jerk_transition_exact(T, alpha, (double (*)[4])F); }
void c_jerk_process_noise_exact(double T, double alpha, double *Q) { jerk_process_noise_exact(T, alpha, (double (*)[4])Q); }
void c_jerk_initial_covariance_small(double T, double var_x, double var_j, double *P) { jerk_initial_covariance_small(T, var_x, var_j, (double (*)[4])P); }
void c_polar_position(double r, double az, double *M) { polar_position(r, az, M); }
void c_polar_covariance(double r, double az, double var_r, double var_az, double *R) { polar_covariance(r, az, var_r, var_az, (double (*)[2])R); }
void c_spherical_position(double r, double az, double el, double *M) { spherical_position(r, az, el, M); }
void c_spherical_covariance(double r, double az, double el, double var_r, double var_az, double var_el, double *R) { spherical_covariance(r, az, el, var_r, var_az, var_el, (double (*)[3])R); }
void c_azel_dynamics(const double *x, double alpha, double *dx) { azel_dynamics(x, alpha, dx); }
void c_azel_basis_rate(const double *x, const double *b, double *db) { azel_basis_rate(x, b, db); }
void c_azel_error_dynamics(const double *x, const double *b, double alpha, double *A) { azel_error_dynamics(x, b, alpha, (double (*)[8])A); }
void c_mpc_to_xi(const double *y, double *xi) { mpc_to_xi(y, xi); }
void c_mpc_from_xi(const double *xi, double *y) { mpc_from_xi(xi, y); }
void c_msc_to_xi(const double *x, const double *lam, double *xi) { msc_to_xi(x, lam, xi); }
void c_msc_from_xi(const double *xi, double *y) { msc_from_xi(xi, y); }
}
"""

@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    if shutil.which("g++") is None:
        pytest.skip("g++ is required")
    path = tmp_path_factory.mktemp("generated")
    (path / "generated.cpp").write_text(SOURCE)
    subprocess.run(["g++", "-std=c++11", "-O2", "-shared", "-fPIC", "-Wall", "-Wextra", "-Werror", "-I", str(root / "lib" / "jerk"),
                    str(path / "generated.cpp"), "-o", str(path / "generated.so")], check=True)
    return ctypes.CDLL(str(path / "generated.so"))

def call(fn, shape, *args):
    out = np.zeros(shape)
    converted = [ctypes.c_double(a) if np.isscalar(a) else np.ascontiguousarray(a, float).ctypes.data_as(ctypes.POINTER(ctypes.c_double)) for a in args]
    fn(*converted, out.ctypes.data_as(ctypes.POINTER(ctypes.c_double)))
    return out

def tangent_state(rng):
    u = rng.normal(size=3)
    u /= np.linalg.norm(u)
    tangent = lambda v: v - (u @ v) * u
    return u, np.concatenate([u] + [tangent(rng.normal(size=3)) for _ in range(3)])

def test_headers_up_to_date(tmp_path):
    pytest.importorskip("sympy")
    sys.path.insert(0, str(root / "model"))
    import codegen
    codegen.output_path = tmp_path / "cpp"
    codegen.torch_output_path = tmp_path / "torch"
    codegen.output_path.mkdir()
    codegen.torch_output_path.mkdir()
    for name, source, functions in codegen.modules:
        built = functions()
        codegen.header(name, source, built)
        codegen.torch_module(name, source, built)
        assert (tmp_path / "cpp" / (name + ".hpp")).read_text() == (generated / (name + ".hpp")).read_text(), name
        assert (tmp_path / "torch" / (name + ".py")).read_text() == (torch_generated / (name + ".py")).read_text(), name

@pytest.mark.parametrize("T", [0.001, 0.05, 1.0])
def test_jerk_block_small(lib, T):
    F, Q = ref.jerk_matrices(T, 0.0, False)
    np.testing.assert_allclose(call(lib.c_jerk_transition_small, (4, 4), T), F, rtol=1e-12, atol=1e-15)
    np.testing.assert_allclose(call(lib.c_jerk_process_noise_small, (4, 4), T), Q, rtol=1e-10, atol=1e-25)

@pytest.mark.parametrize("T,alpha", [(0.05, 3.0), (0.2, 1.0), (1.0, 2.0)])
def test_jerk_block_exact(lib, T, alpha):
    F, Q = ref.jerk_matrices(T, alpha, True)
    np.testing.assert_allclose(call(lib.c_jerk_transition_exact, (4, 4), T, alpha), F, rtol=1e-9, atol=1e-15)
    sd = np.sqrt(np.diag(Q))
    np.testing.assert_allclose(call(lib.c_jerk_process_noise_exact, (4, 4), T, alpha) / np.outer(sd, sd), Q / np.outer(sd, sd), rtol=0, atol=1e-6)

def test_initial_covariance_small(lib):
    P = call(lib.c_jerk_initial_covariance_small, (4, 4), 0.02, 0.0, 4.0)
    np.testing.assert_allclose(P, ref.init_process_covariance(0.02, 0.02, 1.0, 1.0, 0.0, 4.0, False), rtol=1e-12, atol=1e-15)

def test_polar_spherical(lib):
    r, az, el = 7.0, 0.4, -0.3
    np.testing.assert_allclose(call(lib.c_polar_position, 2, r, az), [r * math.cos(az), r * math.sin(az)])
    J = np.array([[math.cos(az), -r * math.sin(az)], [math.sin(az), r * math.cos(az)]])
    np.testing.assert_allclose(call(lib.c_polar_covariance, (2, 2), r, az, 0.01, 0.002), J @ np.diag([0.01, 0.002]) @ J.T)
    pos = lambda v: v[0] * ref.unit(v[1], v[2])
    np.testing.assert_allclose(call(lib.c_spherical_position, 3, r, az, el), pos([r, az, el]))
    v = np.array([r, az, el])
    J = np.array([(pos(v + h) - pos(v - h)) / 2e-6 for h in np.eye(3) * 1e-6]).T
    np.testing.assert_allclose(call(lib.c_spherical_covariance, (3, 3), r, az, el, 0.01, 0.002, 0.003), J @ np.diag([0.01, 0.002, 0.003]) @ J.T, rtol=1e-8)

def test_azel(lib):
    rng = np.random.default_rng(1)
    for _ in range(5):
        u, X = tangent_state(rng)
        B = ref.tangent_basis(u)
        b = B.T.reshape(-1)
        np.testing.assert_allclose(call(lib.c_azel_dynamics, 12, X, 0.7), ref.azel_dynamics(X, 0.7), atol=1e-14)
        db = np.concatenate([-(X[3:6] @ B[:, 0]) * u, -(X[3:6] @ B[:, 1]) * u])
        np.testing.assert_allclose(call(lib.c_azel_basis_rate, 6, X, b), db, atol=1e-14)
        np.testing.assert_allclose(call(lib.c_azel_error_dynamics, (8, 8), X, b, 0.7), ref.azel_error_dynamics(X, B, 0.7), atol=1e-14)

def test_mpc(lib):
    rng = np.random.default_rng(2)
    for _ in range(5):
        Y = rng.normal(size=7)
        xi = ref.mpc_to_xi(Y)
        np.testing.assert_allclose(call(lib.c_mpc_to_xi, 8, Y), xi, atol=1e-13)
        np.testing.assert_allclose(call(lib.c_mpc_from_xi, 8, 2.5 * xi), ref.mpc_from_xi(2.5 * xi), rtol=1e-12, atol=1e-12)

def test_msc(lib):
    rng = np.random.default_rng(3)
    for _ in range(5):
        u, X = tangent_state(rng)
        lam = rng.normal(size=3)
        xi = ref.msc_to_xi(X, lam)
        np.testing.assert_allclose(call(lib.c_msc_to_xi, 12, X, lam), xi, atol=1e-13)
        Xr, lr, s = ref.msc_from_xi(1.7 * xi)
        np.testing.assert_allclose(call(lib.c_msc_from_xi, 16, 1.7 * xi), np.concatenate([Xr, lr, [s]]), rtol=1e-12, atol=1e-12)
