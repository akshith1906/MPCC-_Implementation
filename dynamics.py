import numpy as np

def bicycle_model(state, control, dt, L=0.33):
    """
    Kinematic bicycle model.

    state = [x, y, psi, v]
    control = [a, delta]
    """
    x, y, psi, v = state
    a, delta = control

    x_next = x + dt * v * np.cos(psi)
    y_next = y + dt * v * np.sin(psi)
    psi_next = psi + dt * (v / L) * np.tan(delta)
    v_next = v + dt * a

    return np.array([x_next, y_next, psi_next, v_next])