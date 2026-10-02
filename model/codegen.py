from sympy import *
from sympy.printing.c import C99CodePrinter
from sympy.printing.pycode import PythonCodePrinter
import pathlib

import jerk_block
import polar_spherical
import azel
import mpc
import msc

"""
Writes the generated inline headers in lib/jerk/generated and the generated torch modules in
kalman/torch/generated from the sympy models.
Run from any directory: python model/codegen.py
"""

output_path = pathlib.Path(__file__).parent.parent / "lib" / "jerk" / "generated"
torch_output_path = pathlib.Path(__file__).parent.parent / "kalman" / "torch" / "generated"

def print_power(printer, expr):
    """Integer powers as products and square roots by name, None for other powers."""
    base, exp = expr.base, expr.exp
    if exp.is_Integer and 0 < exp <= 8:
        return "(" + "*".join(["(" + printer._print(base) + ")"] * int(exp)) + ")"
    if exp.is_Integer and -8 <= exp < 0:
        return "(1.0/(" + printer._print(Pow(base, -exp)) + "))"
    if exp == Rational(1, 2):
        return printer.sqrt_name + "(" + printer._print(base) + ")"
    if exp == Rational(-1, 2):
        return "(1.0/" + printer.sqrt_name + "(" + printer._print(base) + "))"
    return None

class Printer(C99CodePrinter):
    sqrt_name = "sqrt"

    def _print_Pow(self, expr):
        text = print_power(self, expr)
        return text if text is not None else "pow(" + self._print(expr.base) + ", " + self._print(expr.exp) + ")"

class TorchPrinter(PythonCodePrinter):
    sqrt_name = "torch.sqrt"

    def _print_Pow(self, expr, rational=False):
        text = print_power(self, expr)
        return text if text is not None else "torch.pow(" + self._print(expr.base) + ", " + self._print(expr.exp) + ")"

    def _print_Rational(self, expr):
        return str(expr.p) + ".0/" + str(expr.q) + ".0"

    def _print_unary(self, name, expr):
        return "torch." + name + "(" + self._print(expr.args[0]) + ")"

    def _print_sin(self, expr):
        return self._print_unary("sin", expr)

    def _print_cos(self, expr):
        return self._print_unary("cos", expr)

    def _print_exp(self, expr):
        return self._print_unary("exp", expr)

    def _print_atan2(self, expr):
        return "torch.atan2(" + self._print(expr.args[0]) + ", " + self._print(expr.args[1]) + ")"

printer = Printer()
torch_printer = TorchPrinter()

class Generated:
    """
    One generated function.
    args: list of (name, C declaration, size) with size None for a scalar.
    inputs: list of (symbol, argument name, index) with index None for a scalar argument.
    output: (name, C declaration, sympy matrix).
    """

    def __init__(self, name, args, inputs, output, scalar="double", template=False):
        self.name, self.args, self.inputs, self.output = name, args, inputs, output
        self.scalar, self.template = scalar, template

    def expressions(self):
        M = self.output[2]
        return [M[i, k] for i in range(M.rows) for k in range(M.cols)]

    def c_lvalues(self):
        name, _, M = self.output
        if M.cols == 1:
            return [name + "[" + str(i) + "]" for i in range(M.rows)]
        return [name + "[" + str(i) + "][" + str(k) + "]" for i in range(M.rows) for k in range(M.cols)]

    def used_indexed_inputs(self, expressions):
        used = set().union(*[e.free_symbols for e in expressions])
        return [i for i in self.inputs if i[0] in used and i[2] is not None]

def reduce(expressions):
    return cse(expressions, symbols=numbered_symbols("t"), optimizations="basic")

def c_function(f):
    expressions = f.expressions()
    signature = "void " + f.name + "(" + ", ".join([a[1] for a in f.args] + [f.output[1]]) + ")"
    lines = ["        const " + f.scalar + " " + str(s) + " = " + arg + "[" + str(index) + "];" for s, arg, index in f.used_indexed_inputs(expressions)]
    temps, reduced = reduce(expressions)
    for t, e in temps:
        lines.append("        const " + f.scalar + " " + str(t) + " = " + printer.doprint(e) + ";")
    for lhs, e in zip(f.c_lvalues(), reduced):
        lines.append("        " + lhs + " = " + printer.doprint(e) + ";")
    head = ("    template <typename S>\n" if f.template else "") + "    inline " + signature + "\n    {\n"
    return head + "\n".join(lines) + "\n    }\n"

def torch_function(f):
    expressions = f.expressions()
    name, _, M = f.output
    shape = "(" + str(M.rows) + ",)" if M.cols == 1 else "(" + str(M.rows) + ", " + str(M.cols) + ")"
    first, _, size = f.args[0]
    lines = ["    z = torch.zeros_like(" + first + ("[..., 0]" if size is not None else "") + ")"]
    lines += ["    " + str(s) + " = " + arg + "[..., " + str(index) + "]" for s, arg, index in f.used_indexed_inputs(expressions)]
    temps, reduced = reduce(expressions)
    for t, e in temps:
        lines.append("    " + str(t) + " = " + torch_printer.doprint(e))
    lines.append("    " + name + " = torch.stack([")
    for e in reduced:
        lines.append("        z + " + torch_printer.doprint(e) + ",")
    lines.append("    ], -1)")
    lines.append("    return " + name + ".reshape(z.shape + " + shape + ")")
    described = [a[0] + (" [..., " + str(a[2]) + "]" if a[2] is not None else "") for a in f.args]
    doc = "    \"\"\"" + ", ".join(described) + " -> " + name + " [..., " + shape[1:-1].rstrip(",") + "]\"\"\"\n"
    return "def " + f.name + "(" + ", ".join(a[0] for a in f.args) + "):\n" + doc + "\n".join(lines) + "\n"

def header(name, source, functions):
    guard = "KAEPEK_GENERATED_" + name.upper() + "_H"
    text = "/*\n * Generated by model/codegen.py from model/" + source + ". Do not edit.\n */\n\n"
    text += "#ifndef " + guard + "\n#define " + guard + "\n\n#include <math.h>\n\nnamespace kaepek\n{\n"
    text += "\n".join(c_function(f) for f in functions)
    text += "}\n\n#endif\n"
    (output_path / (name + ".hpp")).write_text(text)

def torch_module(name, source, functions):
    text = "\"\"\"\nGenerated by model/codegen.py from model/" + source + ". Do not edit.\n"
    text += "Arguments are tensors with any leading batch dimensions.\n\"\"\"\n\nimport torch\n"
    text += "".join("\n\n" + torch_function(f) for f in functions)
    (torch_output_path / (name + ".py")).write_text(text)

def scalar(name):
    return (name, "double " + name, None)

def array(name, size, scalar="double"):
    return (name, "const " + scalar + " " + name + "[" + str(size) + "]", size)

def scalars(symbols_list):
    return [(s, str(s), None) for s in symbols_list]

def indexed(symbols_list, name):
    return [(s, name, i) for i, s in enumerate(symbols_list)]

def matrix_out(name, M):
    return (name, "double " + name + "[" + str(M.rows) + "][" + str(M.cols) + "]", M)

def vector_out(name, V, scalar="double"):
    return (name, scalar + " " + name + "[" + str(len(V)) + "]", Matrix(V))

def jerk_block_functions():
    T, alpha, var_x, var_j = jerk_block.T, jerk_block.alpha, jerk_block.var_x, jerk_block.var_j
    return [
        Generated("jerk_transition_small", [scalar("T")], scalars([T]), matrix_out("F", jerk_block.F_small())),
        Generated("jerk_process_noise_small", [scalar("T")], scalars([T]), matrix_out("Q", jerk_block.Q_small())),
        Generated("jerk_transition_exact", [scalar("T"), scalar("alpha")], scalars([T, alpha]), matrix_out("F", jerk_block.F_exact())),
        Generated("jerk_process_noise_exact", [scalar("T"), scalar("alpha")], scalars([T, alpha]), matrix_out("Q", jerk_block.Q_exact())),
        Generated("jerk_initial_covariance_small", [scalar("T"), scalar("var_x"), scalar("var_j")], scalars([T, var_x, var_j]),
                  matrix_out("P", jerk_block.initial_covariance_small())),
    ]

def polar_spherical_functions():
    r, az, el = polar_spherical.r, polar_spherical.az, polar_spherical.el
    var_r, var_az, var_el = polar_spherical.var_r, polar_spherical.var_az, polar_spherical.var_el
    return [
        Generated("polar_position", [scalar("r"), scalar("az")], scalars([r, az]), vector_out("M", polar_spherical.polar_position())),
        Generated("polar_covariance", [scalar("r"), scalar("az"), scalar("var_r"), scalar("var_az")], scalars([r, az, var_r, var_az]),
                  matrix_out("R", polar_spherical.polar_covariance())),
        Generated("spherical_position", [scalar("r"), scalar("az"), scalar("el")], scalars([r, az, el]),
                  vector_out("M", polar_spherical.spherical_position())),
        Generated("spherical_covariance", [scalar("r"), scalar("az"), scalar("el"), scalar("var_r"), scalar("var_az"), scalar("var_el")],
                  scalars([r, az, el, var_r, var_az, var_el]), matrix_out("R", polar_spherical.spherical_covariance())),
    ]

def azel_functions():
    state = list(azel.u) + list(azel.w) + list(azel.a) + list(azel.j)
    basis = list(azel.b1) + list(azel.b2)
    return [
        Generated("azel_dynamics", [array("x", 12, "S"), scalar("alpha")], indexed(state, "x") + scalars([azel.alpha]),
                  vector_out("dx", azel.dynamics(), "S"), "S", True),
        Generated("azel_basis_rate", [array("x", 12, "S"), array("b", 6, "S")], indexed(state, "x") + indexed(basis, "b"),
                  vector_out("db", azel.basis_rate(), "S"), "S", True),
        Generated("azel_error_dynamics", [array("x", 12), array("b", 6), scalar("alpha")], indexed(state, "x") + indexed(basis, "b") + scalars([azel.alpha]),
                  matrix_out("A", azel.error_dynamics())),
    ]

def mpc_functions():
    y = [mpc.beta, mpc.bd, mpc.bdd, mpc.bddd, mpc.l1, mpc.l2, mpc.l3]
    return [
        Generated("mpc_to_xi", [array("y", 7, "S")], indexed(y, "y"), vector_out("xi", mpc.to_xi(), "S"), "S", True),
        Generated("mpc_from_xi", [array("xi", 8, "S")], indexed(list(mpc.xi), "xi"), vector_out("y", mpc.from_xi(), "S"), "S", True),
    ]

def msc_functions():
    state = list(msc.u) + list(msc.w) + list(msc.a) + list(msc.j)
    return [
        Generated("msc_to_xi", [array("x", 12, "S"), array("lam", 3, "S")], indexed(state, "x") + indexed([msc.l1, msc.l2, msc.l3], "lam"),
                  vector_out("xi", msc.to_xi(), "S"), "S", True),
        Generated("msc_from_xi", [array("xi", 12, "S")], indexed(list(msc.xi), "xi"), vector_out("y", msc.from_xi(), "S"), "S", True),
    ]

modules = [
    ("jerk_block", "jerk_block.py", jerk_block_functions),
    ("polar_spherical", "polar_spherical.py", polar_spherical_functions),
    ("azel", "azel.py", azel_functions),
    ("mpc", "mpc.py", mpc_functions),
    ("msc", "msc.py", msc_functions),
]

if __name__ == "__main__":
    output_path.mkdir(parents=True, exist_ok=True)
    torch_output_path.mkdir(parents=True, exist_ok=True)
    for name, source, functions in modules:
        built = functions()
        header(name, source, built)
        torch_module(name, source, built)
