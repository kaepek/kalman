"""
Former name of the kalman package, re-exporting it.
"""

import warnings

warnings.warn("CyKalman is deprecated, import from kalman instead", DeprecationWarning, stacklevel=2)

from kalman import *
from kalman import __all__
