import numpy as np

def contouring_lag_error(position, ref_position, tangent, normal):
    """
    Computes contouring and lag errors.
    """
    error = position - ref_position

    e_c = normal.T @ error
    e_l = tangent.T @ error

    return e_c, e_l