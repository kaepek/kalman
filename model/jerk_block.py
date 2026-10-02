from sympy import *
import pathlib

"""
Per axis jerk model of [Ref1] (Mehrotra and Mahapatra 1997).

Continuous model (7): dX/dt = A X + B w, X = [x, dx/dt, d2x/dt2, d3x/dt3]
Transition matrix (11): F(T) = exp(A T)
Process noise (13): Q(T) = integral_0^T F(u) B B^T F(u)^T du, scaled by q = 2 alpha sigma_j^2
Small alpha T limit: (16) and (21)
Initial covariance in the small alpha T limit: (29)
"""

output_images_path = str(pathlib.Path(__file__).parent / "kalman-cpp-math-jerk-block") + "/"

def save_math_ent(name, ent):
    print(name, ent.shape)
    pathlib.Path(output_images_path).mkdir(parents=True, exist_ok=True)
    print(latex(ent))
    preview(ent, viewer='file', filename=(output_images_path+name+'.png'), dvioptions=['-D','1200'])

T, u, alpha = symbols('T u alpha', positive=True)
var_x, var_j = symbols('var_x var_j', positive=True)

A = Matrix([
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1],
    [0, 0, 0, -alpha]
])
B = Matrix([0, 0, 0, 1])

def transition(t, a):
    return simplify((A.subs(alpha, a) * t).exp())

def process_noise(a):
    g = transition(u, a) * B
    return simplify((g * g.T).applyfunc(lambda e: integrate(e, (u, 0, T))))

def initial_covariance_small():
    return Matrix([
        [var_x, var_x / T, var_x / T**2, 0],
        [var_x / T, 2 * var_x / T**2, 3 * var_x / T**3, Rational(5, 6) * var_j * T**2],
        [var_x / T**2, 3 * var_x / T**3, 6 * var_x / T**4, var_j * T],
        [0, Rational(5, 6) * var_j * T**2, var_j * T, var_j]
    ])

def F_exact():
    return transition(T, alpha)

def Q_exact():
    return process_noise(alpha)

def F_small():
    return transition(T, 0)

def Q_small():
    return process_noise(0)

if __name__ == "__main__":
    save_math_ent("A", A)
    save_math_ent("F_exact", F_exact())
    save_math_ent("Q_exact", Q_exact())
    save_math_ent("F_small", F_small())
    save_math_ent("Q_small", Q_small())
    save_math_ent("P0_small", initial_covariance_small())
