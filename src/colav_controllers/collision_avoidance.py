"""
COLREGs-compliant collision avoidance controller.

Coordinates unsafe set generation, virtual waypoint computation, and
prescribed-time navigation to avoid obstacles.
"""

from typing import List, Tuple, Optional, Callable
import numpy as np
from colav_controllers.prescribed_time import PrescribedTimeController
from colav_controllers.utils import (
    default_vertex_provider,
    compute_v1,
)


class CollisionAvoidanceController:
    """
    COLREGs-compliant collision avoidance controller.

    Orchestrates the collision avoidance process:
    1. Generates unsafe set vertices around obstacles
    2. Selects starboard-most vertex ahead (V1)
    3. Applies optional buffering to V1
    4. Uses prescribed-time control to navigate to V1

    The controller delegates geometric computations to specialized modules
    (geometry.py, virtual_waypoint.py) to keep this focused on control logic.
    """

    def __init__(
        self,
        a: float,
        v: float,
        eta: float,
        tp: float,
        Cs: float,
        vertex_provider: Optional[Callable[
            [float, float, List[Tuple[float, float, float, float]], float, float],
            Optional[List[Tuple[float, float]]]
        ]] = None,
        v1_buffer: float = 0.0
    ):
        """
        Initialize the collision avoidance controller.

        Args:
            a: System parameter (heading dynamics coefficient)
            v: Ship velocity (m/s)
            eta: Controller gain (eta > 1)
            tp: Prescribed time (seconds)
            Cs: Safe distance from obstacle (m)
            vertex_provider: Optional callable that returns unsafe set vertices.
                Signature: (pos_x, pos_y, obstacles_list, Cs, psi) -> List[(vx, vy)] or None
                If None, uses circular obstacle approximation (default_vertex_provider).
            v1_buffer: Buffer distance (m) to offset V1 outward for extra clearance.
                Default 0.0 means no buffer.
        """
        self.v = v
        self.Cs = Cs
        self.vertex_provider = vertex_provider or default_vertex_provider
        self.v1_buffer = v1_buffer
        self.virtual_waypoint = None
        self._pt_controller = PrescribedTimeController(a=a, v=v, eta=eta, tp=tp)

    def set_virtual_waypoint(
        self,
        pos_x: float,
        pos_y: float,
        psi: float,
        obstacles_list: List[Tuple[float, float, float, float]]
    ) -> None:
        """
        Compute and store virtual waypoint V1.

        Args:
            pos_x, pos_y: Current ship position
            psi: Current heading
            obstacles_list: List of (ox, oy, ov, o_psi) tuples
        """
        self.virtual_waypoint = compute_v1(
            pos_x, pos_y, psi, obstacles_list, self.Cs, 
            self.vertex_provider, self.v1_buffer
        )

    def compute_dynamics(self, t: float, x: float, y: float, psi: float) -> np.ndarray:
        """
        Compute state derivatives for collision avoidance mode.

        Delegates to the internal prescribed-time controller targeting V1.

        Args:
            t: Current time
            x, y: Current position
            psi: Current heading

        Returns:
            np.array([dx/dt, dy/dt, dpsi/dt])

        Raises:
            ValueError: If virtual waypoint not set via set_virtual_waypoint()
        """
        if self.virtual_waypoint is None:
            raise ValueError("Virtual waypoint not set. Call set_virtual_waypoint() first.")

        vx, vy = self.virtual_waypoint
        return self._pt_controller.compute_dynamics(t, x, y, psi, vx, vy)
