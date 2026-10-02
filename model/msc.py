from sympy import *
import pathlib

"""
Modified spherical coordinates of jerk order (KalmanJerk2DAzElMovingSensor).

Relative position rho = r u with u the unit direction and lambda = ln r.
State: u, w, a, j as in KalmanJerk2DAzEl, [d lambda, d2 lambda, d3 lambda] and 1/r.
eta_n = d^n rho / dt^n / r; xi holds eta per axis [position, velocity, acceleration, jerk].
"""

output_images_path = str(pathlib.Path(__file__).parent / "kalman-cpp-math-msc") + "/"

def save_math_ent(name, ent):
    print(name, ent.shape)
    pathlib.Path(output_images_path).mkdir(parents=True, exist_ok=True)
    print(latex(ent))
    preview(ent, viewer='file', filename=(output_images_path+name+'.png'), dvioptions=['-D','1200'])

def vector(name):
    return Matrix(symbols(name + '0:3', real=True))

u, w, a, j = vector('u'), vector('w'), vector('a'), vector('j')
l1, l2, l3 = symbols('l1 l2 l3', real=True)
xi = Matrix(symbols('xi0:12', real=True))

def eta():
    ww, wa = w.dot(w), w.dot(a)
    e0 = u
    e1 = l1 * u + w
    e2 = (l2 + l1**2 - ww) * u + 2 * l1 * w + a
    e3 = (l3 + 3 * l1 * l2 + l1**3 - 3 * l1 * ww - 3 * wa) * u + 3 * (l2 + l1**2) * w + 3 * l1 * a + j - ww * w
    return [e0, e1, e2, e3]

def to_xi():
    e = eta()
    return Matrix([e[n][i] for i in range(3) for n in range(4)])

def from_xi():
    """Returns [u, w, a, j, l1, l2, l3, s] with s = |eta_0| the range ratio."""
    E = [Matrix([xi[4 * i + n] for i in range(3)]) for n in range(4)]
    s = sqrt(E[0].dot(E[0]))
    E = [e / s for e in E]
    U = E[0]
    tangential = lambda v: v - U.dot(v) * U
    L1 = U.dot(E[1])
    W = E[1] - L1 * U
    WW = W.dot(W)
    L2 = U.dot(E[2]) - L1**2 + WW
    Acc = tangential(E[2]) - 2 * L1 * W
    WA = W.dot(Acc)
    L3 = U.dot(E[3]) - 3 * L1 * L2 - L1**3 + 3 * L1 * WW + 3 * WA
    Jk = tangential(E[3]) - 3 * (L2 + L1**2) * W - 3 * L1 * Acc + WW * W
    return Matrix.vstack(U, W, Acc, Jk, Matrix([L1, L2, L3, s]))

def from_xi_steps():
    """The steps of from_xi with eta_n the n th derivative block of xi divided by s and T the tangential projection at u."""
    E0, E1, E2, E3 = [MatrixSymbol('eta_' + str(n), 3, 1) for n in range(4)]
    U, W, Acc = MatrixSymbol('u', 3, 1), MatrixSymbol('w', 3, 1), MatrixSymbol('a', 3, 1)
    L1, L2, L3 = symbols('dlambda d2lambda d3lambda')
    tangential = lambda v: v - U * (U.T * v)
    return Matrix([
        Eq(U, E0),
        Eq(L1, (U.T * E1)[0]),
        Eq(W, E1 - L1 * U),
        Eq(L2, (U.T * E2)[0] - L1**2 + (W.T * W)[0]),
        Eq(Acc, tangential(E2) - 2 * L1 * W),
        Eq(L3, (U.T * E3)[0] - 3 * L1 * L2 - L1**3 + 3 * L1 * (W.T * W)[0] + 3 * (W.T * Acc)[0]),
        Eq(MatrixSymbol('j', 3, 1), tangential(E3) - 3 * (L2 + L1**2) * W - 3 * L1 * Acc + (W.T * W)[0] * W),
    ])

if __name__ == "__main__":
    save_math_ent("to_xi", to_xi())
    save_math_ent("from_xi_steps", from_xi_steps())
