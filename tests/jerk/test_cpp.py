import pathlib
import shutil
import subprocess
import pytest

root = pathlib.Path(__file__).resolve().parent.parent.parent
include = str(root / "lib" / "jerk")

pytestmark = pytest.mark.skipif(shutil.which("g++") is None or shutil.which("nm") is None, reason="g++ and nm are required")

HEADERS = """
#include "kalman_jerk_cartesian.hpp"
#include "kalman_jerk_2d_polar.hpp"
#include "kalman_jerk_3d_spherical.hpp"
#include "kalman_jerk_2d_azel.hpp"
#include "kalman_jerk_1d_bearing_moving_sensor.hpp"
#include "kalman_jerk_2d_azel_moving_sensor.hpp"
using namespace kaepek;
"""

def instantiations():
    lines = []
    for form in ["JerkSmallAlphaT", "JerkExact"]:
        for diag in ["NoDiagnostics", "WithDiagnostics"]:
            for n in [1, 2, 3, 4]:
                lines.append("template class kaepek::KalmanJerkCartesian<%d, %s, %s>;" % (n, form, diag))
            lines.append("template class kaepek::KalmanJerk2DPolar<%s, %s>;" % (form, diag))
            lines.append("template class kaepek::KalmanJerk3DSpherical<%s, %s>;" % (form, diag))
            for order in ["FirstOrder", "SecondOrder", "Unscented"]:
                for cls in ["KalmanJerk2DAzEl", "KalmanJerk1DBearingMovingSensor", "KalmanJerk2DAzElMovingSensor"]:
                    lines.append("template class kaepek::%s<%s, %s, %s>;" % (cls, form, order, diag))
    return "\n".join(lines)

def compile_object(path, source, *flags):
    (path / "unit.cpp").write_text(source)
    subprocess.run(["g++", "-std=c++11", "-Wall", "-Wextra", "-Wpedantic", "-Werror", "-I", include, *flags, "-c",
                    str(path / "unit.cpp"), "-o", str(path / "unit.o")], check=True)
    return subprocess.run(["nm", "-C", str(path / "unit.o")], check=True, capture_output=True, text=True).stdout

def test_all_policies_compile(tmp_path):
    compile_object(tmp_path, HEADERS + instantiations(), "-O0")

def test_unused_code_is_not_emitted(tmp_path):
    source = HEADERS + """
double run()
{
    KalmanJerk2D<> kalman(2.0, 0.01, 5.0, false);
    double x[2] = {0.0, 0.0};
    for (int i = 0; i < 10; i++)
    {
        x[0] += 1.0;
        kalman.step(0.1 * i, x);
    }
    return kalman.get_kalman_vector()[1];
}
"""
    symbols = compile_object(tmp_path, source, "-O0")
    assert "KalmanJerkCartesian<2, kaepek::JerkSmallAlphaT, kaepek::NoDiagnostics>" in symbols
    for unused in ["JerkExact", "WithDiagnostics", "AzEl", "MovingSensor", "Polar", "Spherical", "Dual", "HyperDual",
                   "azel_", "mpc_", "msc_", "polar_", "spherical_", "rotate3", "log_map", "jerk_transition_exact", "jerk_process_noise_exact"]:
        assert unused not in symbols, unused
