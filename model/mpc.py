from sympy import *
import pathlib

"""
Modified polar coordinates of jerk order (KalmanJerk1DBearingMovingSensor).

Relative position rho = r exp(i beta) = exp(L), L = lambda + i beta, lambda = ln r.
State Y = [beta, d beta, d2 beta, d3 beta, d lambda, d2 lambda, d3 lambda] and 1/r.
xi = relative Cartesian state divided by r, per axis [position, velocity, acceleration, jerk].
"""

output_images_path = str(pathlib.Path(__file__).parent / "kalman-cpp-math-mpc") + "/"

def save_math_ent(name, ent):
    print(name, ent.shape)
    pathlib.Path(output_images_path).mkdir(parents=True, exist_ok=True)
    print(latex(ent))
    preview(ent, viewer='file', filename=(output_images_path+name+'.png'), dvioptions=['-D','1200'])

beta, bd, bdd, bddd, l1, l2, l3 = symbols('beta bd bdd bddd l1 l2 l3', real=True)
xi = Matrix(symbols('xi0:8', real=True))

def to_xi():
    z1, z2, z3 = l1 + I * bd, l2 + I * bdd, l3 + I * bddd
    c0 = cos(beta) + I * sin(beta)
    c = [c0, c0 * z1, c0 * (z1**2 + z2), c0 * (z1**3 + 3 * z1 * z2 + z3)]
    c = [expand(expand_complex(x)) for x in c]
    return Matrix([re(x) for x in c] + [im(x) for x in c])

def from_xi():
    """Returns [beta, bd, bdd, bddd, l1, l2, l3, s] with s = |c0| the range ratio."""
    c = [xi[n] + I * xi[4 + n] for n in range(4)]
    s2 = xi[0]**2 + xi[4]**2
    w = [expand(expand_complex(c[n] * conjugate(c[0]))) / s2 for n in range(4)]
    z1 = w[1]
    z2 = expand(expand_complex(w[2] - z1**2))
    z3 = expand(expand_complex(w[3] - 3 * z1 * z2 - z1**3))
    return Matrix([atan2(xi[4], xi[0]), im(z1), im(z2), im(z3), re(z1), re(z2), re(z3), sqrt(s2)])

if __name__ == "__main__":
    save_math_ent("to_xi", to_xi())
    save_math_ent("from_xi", from_xi())
