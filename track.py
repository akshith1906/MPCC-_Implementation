import numpy as np
from scipy.interpolate import CubicSpline

class Track:
    def __init__(self):
        theta = np.linspace(0, 2 * np.pi, 100)
        x = 5 * np.cos(theta)
        y = 5 * np.sin(theta)

        self.theta = theta
        self.spline_x = CubicSpline(theta, x)
        self.spline_y = CubicSpline(theta, y)

    def get_position(self, t):
        x = self.spline_x(t)
        y = self.spline_y(t)
        return np.array([x, y])

    def get_tangent(self, t):
        dx = self.spline_x(t, 1)
        dy = self.spline_y(t, 1)
        norm = np.sqrt(dx**2 + dy**2)
        return np.array([dx, dy]) / norm

    def get_normal(self, t):
        tangent = self.get_tangent(t)
        return np.array([-tangent[1], tangent[0]])