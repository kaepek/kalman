import CyKalman
import PyKalman

def get_kalman_filter(name="py"):
    if name == "py":
        return PyKalman
    elif name == "cy":
        return CyKalman