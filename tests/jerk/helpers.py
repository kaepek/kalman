import numpy as np

def assert_state_close(x, x_ref, P_ref, tol):
    """Compares states in units of the reference standard deviations."""
    sd = np.sqrt(np.diag(P_ref))
    np.testing.assert_allclose(np.asarray(x) / sd, np.asarray(x_ref) / sd, rtol=0, atol=tol)

def assert_covariance_close(P, P_ref, tol):
    """Compares covariances after scaling by the reference standard deviations."""
    sd = np.sqrt(np.diag(P_ref))
    scale = np.outer(sd, sd)
    np.testing.assert_allclose(np.asarray(P) / scale, P_ref / scale, rtol=0, atol=tol)
