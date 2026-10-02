import sys
import pytest
import kalman

def test_former_name():
    sys.modules.pop("CyKalman", None)
    with pytest.warns(DeprecationWarning):
        import CyKalman
    assert CyKalman.KalmanJerk1D is kalman.KalmanJerk1D
    assert set(CyKalman.__all__) == set(kalman.__all__)
