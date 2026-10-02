"""
PyTorch implementations of the jerk filters of lib/jerk as torch.nn.Module classes. The state is held in
buffers so the filters move with .to(device, dtype); derivatives are taken with torch.func.
"""

from .cartesian import KalmanJerkCartesian, KalmanJerk2D, KalmanJerk3D, KalmanJerk2DPolar, KalmanJerk3DSpherical
from .azel import KalmanJerk2DAzEl
from .moving_sensor import KalmanJerk1DBearingMovingSensor, KalmanJerk2DAzElMovingSensor
