from sympy import *
import pathlib

"""
Measurement conversions of [Ref1] Section V.

Spherical (44): M = r [cos(el) cos(az), cos(el) sin(az), sin(el)]
Converted covariance (45): R = J diag(sigma_r^2, sigma_az^2, sigma_el^2) J^T, J the Jacobian of (44)
Polar: (44) and (45) at el = 0
"""

output_images_path = str(pathlib.Path(__file__).parent / "kalman-cpp-math-polar-spherical") + "/"

def save_math_ent(name, ent):
    print(name, ent.shape)
    pathlib.Path(output_images_path).mkdir(parents=True, exist_ok=True)
    print(latex(ent))
    preview(ent, viewer='file', filename=(output_images_path+name+'.png'), dvioptions=['-D','1200'])

r, az, el = symbols('r az el', real=True)
var_r, var_az, var_el = symbols('var_r var_az var_el', positive=True)

def polar_position():
    return Matrix([r * cos(az), r * sin(az)])

def polar_covariance():
    J = polar_position().jacobian([r, az])
    return simplify(J * diag(var_r, var_az) * J.T)

def spherical_position():
    return Matrix([r * cos(el) * cos(az), r * cos(el) * sin(az), r * sin(el)])

def spherical_covariance():
    J = spherical_position().jacobian([r, az, el])
    return simplify(J * diag(var_r, var_az, var_el) * J.T)

if __name__ == "__main__":
    save_math_ent("polar_position", polar_position())
    save_math_ent("polar_covariance", polar_covariance())
    save_math_ent("spherical_position", spherical_position())
    save_math_ent("spherical_covariance", spherical_covariance())
