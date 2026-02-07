"""Shared helper functions for colav_controllers."""

import numpy as np


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    return np.arctan2(np.sin(angle), np.cos(angle))


def compute_straight_line_dynamics(v: float, psi: float) -> np.ndarray:
    """
    Compute state derivatives for straight-line motion at constant heading.

    Args:
        v: Ship velocity (m/s)
        psi: Current heading (rad)

    Returns:
        np.array([dx/dt, dy/dt, dpsi/dt])
    """
    return np.array([v * np.cos(psi), v * np.sin(psi), 0.0])
