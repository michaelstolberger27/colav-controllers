"""
Prescribed-time heading controller for ship navigation.

Guarantees heading convergence to line-of-sight (LOS) in prescribed time tp.
"""

import numpy as np


class PrescribedTimeController:
    """
    Prescribed-time heading controller for ship navigation.
    Guarantees heading convergence to LOS in prescribed time tp.
    """

    def __init__(self, a: float, v: float, eta: float, tp: float):
        """
        Args:
            a: System parameter (heading dynamics coefficient)
            v: Ship velocity (m/s)
            eta: Controller gain (eta > 1)
            tp: Prescribed time (seconds)
        """
        self.a = a
        self.v = v
        self.eta = eta
        self.tp = tp

    def compute_control(self, t: float, x: float, y: float, psi: float, xw: float, yw: float) -> float:
        """
        Prescribed-time control law.

        u = (1/a)dot{psi}_dg + psi - eta(psi - psi_dg)/(a(tp - t))  for t < tp
        u = (1/a)dot{psi}_dg + psi                                   for t >= tp

        Args:
            t: Current time
            x, y: Current ship position
            psi: Current heading
            xw, yw: Waypoint position

        Returns:
            Control input u
        """
        psi_dg = np.arctan2(yw - y, xw - x)

        dx = xw - x
        dy = yw - y
        d_squared = dx**2 + dy**2

        if d_squared < 1e-6:
            psi_dg_dot = 0.0
        else:
            psi_dg_dot = (-self.v * dx * np.sin(psi) + self.v * dy * np.cos(psi)) / d_squared

        e = np.arctan2(np.sin(psi - psi_dg), np.cos(psi - psi_dg))

        if t < self.tp:
            time_varying_term = self.eta * e / (self.a * (self.tp - t + 1e-6))
            u = (1/self.a) * psi_dg_dot + psi - time_varying_term
        else:
            u = (1/self.a) * psi_dg_dot + psi

        return u

    def compute_dynamics(self, t: float, x: float, y: float, psi: float, xw: float, yw: float) -> np.ndarray:
        """
        Ship dynamics with prescribed-time control.

        dot{x} = v cos(psi)
        dot{y} = v sin(psi)
        dot{psi} = -a*psi + a*u

        Args:
            t: Current time
            x, y: Current ship position
            psi: Current heading
            xw, yw: Waypoint position

        Returns:
            np.array([dx/dt, dy/dt, dpsi/dt])
        """
        u = self.compute_control(t, x, y, psi, xw, yw)

        dx_dt = self.v * np.cos(psi)
        dy_dt = self.v * np.sin(psi)
        dpsi_dt = -self.a * psi + self.a * u

        return np.array([dx_dt, dy_dt, dpsi_dt])
