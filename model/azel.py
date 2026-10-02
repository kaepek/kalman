from sympy import *
import pathlib

"""
Jerk model on the unit sphere (KalmanJerk2DAzEl).

State: unit direction u and tangent vectors w (angular velocity), a (angular acceleration), j (angular jerk).
Covariant chain: Dw/dt = a, Da/dt = j, Dj/dt = -alpha j + noise, with Dv/dt = dv/dt + (du/dt . v) u.
Error state in a tangent basis b1, b2 transported in parallel: db/dt = -(w . b) u.
"""

output_images_path = str(pathlib.Path(__file__).parent / "kalman-cpp-math-azel") + "/"

def save_math_ent(name, ent):
    print(name, ent.shape)
    pathlib.Path(output_images_path).mkdir(parents=True, exist_ok=True)
    print(latex(ent))
    preview(ent, viewer='file', filename=(output_images_path+name+'.png'), dvioptions=['-D','1200'])

alpha = symbols('alpha')

def vector(name):
    return Matrix(symbols(name + '0:3', real=True))

u = vector('u')
w = vector('w')
a = vector('a')
j = vector('j')
b1 = vector('b1_')
b2 = vector('b2_')

def dynamics():
    """Ambient form of the covariant chain for the stacked state [u, w, a, j]."""
    return Matrix.vstack(
        w,
        a - w.dot(w) * u,
        j - w.dot(a) * u,
        -alpha * j - w.dot(j) * u
    )

def basis_rate():
    """Parallel transport of the tangent basis along the motion."""
    return Matrix.vstack(-w.dot(b1) * u, -w.dot(b2) * u)

def error_dynamics():
    """
    Linearised error dynamics in the transported basis, error ordered by axis
    [v_1, dw_1, da_1, dj_1, v_2, dw_2, da_2, dj_2].
    """
    B = Matrix.hstack(b1, b2)
    wt, at, jt = B.T * w, B.T * a, B.T * j
    I2 = eye(2)
    K_w = w.dot(w) * I2 - wt * wt.T
    K_a = w.dot(a) * I2 - wt * at.T
    K_j = w.dot(j) * I2 - wt * jt.T
    A = zeros(8, 8)
    for i in range(2):
        A[4 * i, 4 * i + 1] = 1
        A[4 * i + 1, 4 * i + 2] = 1
        A[4 * i + 2, 4 * i + 3] = 1
        A[4 * i + 3, 4 * i + 3] = -alpha
        for k in range(2):
            A[4 * i + 1, 4 * k] = -K_w[i, k]
            A[4 * i + 2, 4 * k] = -K_a[i, k]
            A[4 * i + 3, 4 * k] = -K_j[i, k]
    return A

def coordinate_geodesic():
    """Unforced motion written in azimuth theta and elevation phi."""
    t = symbols('t')
    theta, phi = Function('theta')(t), Function('phi')(t)
    U = Matrix([cos(phi) * cos(theta), cos(phi) * sin(theta), sin(phi)])
    dU, ddU = U.diff(t), U.diff(t, 2)
    # unforced: the tangential part of the acceleration vanishes
    e_a = Matrix([-sin(theta), cos(theta), 0])
    e_e = Matrix([-sin(phi) * cos(theta), -sin(phi) * sin(theta), cos(phi)])
    eqs = solve([simplify(ddU.dot(e_a)), simplify(ddU.dot(e_e))], [theta.diff(t, 2), phi.diff(t, 2)], dict=True)[0]
    return Matrix([simplify(eqs[theta.diff(t, 2)]), simplify(eqs[phi.diff(t, 2)])])

if __name__ == "__main__":
    save_math_ent("dynamics", dynamics())
    save_math_ent("basis_rate", basis_rate())
    save_math_ent("error_dynamics", error_dynamics())
    save_math_ent("coordinate_geodesic", coordinate_geodesic())
