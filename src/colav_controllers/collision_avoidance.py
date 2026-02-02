"""
COLREGs-compliant collision avoidance controller.

Creates unsafe sets, computes virtual waypoints, and uses prescribed-time
control to navigate around obstacles.
"""

from typing import List, Tuple, Optional, Callable
import numpy as np


class CollisionAvoidanceController:
    """
    COLREGs-compliant collision avoidance controller.

    - Creates unsafe set via configurable vertex provider
    - Computes virtual waypoint V1 (starboard vertex of unsafe set)
    - Uses prescribed-time controller to navigate to V1
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
        Args:
            a: System parameter (heading dynamics coefficient)
            v: Ship velocity (m/s)
            eta: Controller gain (eta > 1)
            tp: Prescribed time (seconds)
            Cs: Safe distance from obstacle (m)
            vertex_provider: Optional callable that returns unsafe set vertices.
                Signature: (pos_x, pos_y, obstacles_list, Cs, psi) -> List[(vx, vy)] or None
                If None, uses simple circular obstacle approximation.
            v1_buffer: Buffer distance (m) to offset V1 to starboard for extra clearance.
                Default 0.0 means no buffer.
        """
        self.a = a
        self.v = v
        self.eta = eta
        self.tp = tp
        self.Cs = Cs
        self.vertex_provider = vertex_provider or self._default_vertex_provider
        self.v1_buffer = v1_buffer
        self.virtual_waypoint = None
        self.virtual_waypoint_history = []
        self.last_control = 0.0

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """Normalize angle to [-π, π]"""
        return np.arctan2(np.sin(angle), np.cos(angle))

    @staticmethod
    def _point_in_polygon(px: float, py: float, vertices: List[Tuple[float, float]]) -> bool:
        """
        Check if point (px, py) is inside a polygon using ray casting algorithm.

        Args:
            px, py: Point to check
            vertices: List of (x, y) polygon vertices

        Returns:
            True if point is inside polygon
        """
        if len(vertices) < 3:
            return False

        n = len(vertices)
        inside = False

        j = n - 1
        for i in range(n):
            xi, yi = vertices[i]
            xj, yj = vertices[j]

            if ((yi > py) != (yj > py)) and (px < (xj - xi) * (py - yi) / (yj - yi) + xi):
                inside = not inside
            j = i

        return inside

    def _apply_v1_buffer(
        self,
        pos_x: float,
        pos_y: float,
        vx: float,
        vy: float,
        obstacles_list: Optional[List[Tuple[float, float, float, float]]] = None,
        psi: float = 0.0
    ) -> Tuple[float, float]:
        """
        Apply buffer by moving V1 outward from polygon centroid.

        Args:
            pos_x, pos_y: Ship position
            vx, vy: Original V1 position (a vertex of the unsafe set polygon)
            obstacles_list: List of obstacles to get polygon vertices
            psi: Ship heading (for vertex provider)

        Returns:
            Tuple (buffered_vx, buffered_vy)
        """
        if self.v1_buffer <= 0:
            return (vx, vy)

        if not obstacles_list:
            return (vx, vy)

        # Get the polygon vertices
        vertices = self.vertex_provider(pos_x, pos_y, obstacles_list, self.Cs, psi)
        if not vertices or len(vertices) < 3:
            return (vx, vy)

        # Compute polygon centroid
        centroid_x = sum(v[0] for v in vertices) / len(vertices)
        centroid_y = sum(v[1] for v in vertices) / len(vertices)

        # Outward direction: from centroid to V1
        outward = np.array([vx - centroid_x, vy - centroid_y])
        outward_len = np.linalg.norm(outward)

        if outward_len < 1e-6:
            return (vx, vy)

        outward = outward / outward_len

        # Move outward by buffer amount
        buffered_vx = vx + self.v1_buffer * outward[0]
        buffered_vy = vy + self.v1_buffer * outward[1]

        # Safety check: don't use buffer if it would place V1 inside polygon
        if self._point_in_polygon(buffered_vx, buffered_vy, vertices):
            return (vx, vy)

        # Check if buffered point is closer to any obstacle than the original
        for ox, oy, _, _ in obstacles_list:
            orig_dist = np.sqrt((vx - ox)**2 + (vy - oy)**2)
            buffered_dist = np.sqrt((buffered_vx - ox)**2 + (buffered_vy - oy)**2)
            # Only reject if buffer moves V1 closer to obstacle
            if buffered_dist < orig_dist - 0.1:
                return (vx, vy)

        return (buffered_vx, buffered_vy)

    @staticmethod
    def _default_vertex_provider(
        pos_x: float,
        pos_y: float,
        obstacles_list: List[Tuple[float, float, float, float]],
        Cs: float,
        psi: float = 0.0
    ) -> Optional[List[Tuple[float, float]]]:
        """
        Default vertex provider using simple circular obstacle approximation.

        Creates 8 vertices around each obstacle at distance Cs.

        Args:
            pos_x, pos_y: Ship position
            obstacles_list: List of (ox, oy, ov, o_psi) tuples
            Cs: Safety radius

        Returns:
            List of (vx, vy) vertices or None
        """
        if not obstacles_list:
            return None

        vertices = []
        for ox, oy, _, _ in obstacles_list:
            # Create 8 vertices around obstacle
            for i in range(8):
                angle = i * np.pi / 4
                vx = ox + Cs * np.cos(angle)
                vy = oy + Cs * np.sin(angle)
                vertices.append((vx, vy))

        return vertices if vertices else None

    def compute_V1(
        self,
        pos_x: float,
        pos_y: float,
        psi: float,
        ox: float,
        oy: float
    ) -> Optional[Tuple[float, float]]:
        """
        Compute virtual waypoint V1 (starboard vertex of unsafe set).

        Selects from vertices of the unsafe set based on:
        - Must be ahead (within ±π/2 of heading)
        - Prefer starboard (smallest relative angle)

        Args:
            pos_x, pos_y: Current ship position
            psi: Current heading
            ox, oy: Obstacle position

        Returns:
            (v1_x, v1_y) or None
        """
        obstacles_list = [(ox, oy, 0.0, 0.0)]
        vertices = self.vertex_provider(pos_x, pos_y, obstacles_list, self.Cs, psi)

        if vertices is None or len(vertices) < 1:
            return None

        best_vertex = vertices[0]
        best_score = -np.inf

        for vertex in vertices:
            vx, vy = vertex
            angle_to_vertex = np.arctan2(vy - pos_y, vx - pos_x)
            relative_angle = self.normalize_angle(angle_to_vertex - psi)

            # Check if vertex is ahead (L2: within ±π/2)
            if -np.pi/2 < relative_angle < np.pi/2:
                dist = np.sqrt((vx - pos_x)**2 + (vy - pos_y)**2)
                score = -relative_angle - 0.01 * dist  # Prefer negative angles (starboard)

                if score > best_score:
                    best_score = score
                    best_vertex = (vx, vy)

        # Apply starboard buffer if configured
        if best_vertex is not None:
            best_vertex = self._apply_v1_buffer(pos_x, pos_y, best_vertex[0], best_vertex[1], obstacles_list, psi)

        return best_vertex

    def compute_V1_dynamic(
        self,
        pos_x: float,
        pos_y: float,
        psi: float,
        obstacles_list: List[Tuple[float, float, float, float]]
    ) -> Optional[Tuple[float, float]]:
        """
        Compute virtual waypoint V1 from dynamic unsafe set.

        Uses vertex_provider to get unsafe set vertices.
        Selects the starboard-most vertex that is ahead of the ship.

        Args:
            pos_x, pos_y: Current ship position
            psi: Current heading
            obstacles_list: List of (ox, oy, ov, o_psi) tuples

        Returns:
            Tuple (v1_x, v1_y) or None if no valid vertex
        """
        if not obstacles_list:
            return None

        vertices = self.vertex_provider(pos_x, pos_y, obstacles_list, self.Cs, psi)

        if not vertices or len(vertices) < 1:
            return None

        best_vertex = None
        best_angle = np.inf

        for vx, vy in vertices:
            angle_to_vertex = np.arctan2(vy - pos_y, vx - pos_x)
            relative_angle = self.normalize_angle(angle_to_vertex - psi)

            # Prefer starboard (negative angles, right side) and ahead (within ±π/2)
            if -np.pi/2 < relative_angle < np.pi/2:
                if relative_angle < best_angle:
                    best_angle = relative_angle
                    best_vertex = (vx, vy)

        # Apply starboard buffer if configured
        if best_vertex is not None:
            best_vertex = self._apply_v1_buffer(pos_x, pos_y, best_vertex[0], best_vertex[1], obstacles_list, psi)

        return best_vertex

    def set_virtual_waypoint(
        self,
        pos_x: float,
        pos_y: float,
        psi: float,
        ox: float,
        oy: float
    ):
        """Compute and store virtual waypoint V1."""
        self.virtual_waypoint = self.compute_V1(pos_x, pos_y, psi, ox, oy)
        if self.virtual_waypoint:
            self.virtual_waypoint_history.append(self.virtual_waypoint)

    def set_virtual_waypoint_dynamic(
        self,
        pos_x: float,
        pos_y: float,
        psi: float,
        obstacles_list: List[Tuple[float, float, float, float]]
    ):
        """
        Compute and store virtual waypoint V1 from dynamic unsafe set.

        Args:
            pos_x, pos_y: Current ship position
            psi: Current heading
            obstacles_list: List of (ox, oy, ov, o_psi) tuples
        """
        self.virtual_waypoint = self.compute_V1_dynamic(pos_x, pos_y, psi, obstacles_list)
        if self.virtual_waypoint:
            self.virtual_waypoint_history.append(self.virtual_waypoint)

    def compute_dynamics(self, t: float, x: float, y: float, psi: float) -> np.ndarray:
        """
        Compute state derivatives for collision avoidance mode.

        Returns:
            np.array([dx/dt, dy/dt, dpsi/dt])
        """
        if self.virtual_waypoint is None:
            raise ValueError("Virtual waypoint not set")

        vx, vy = self.virtual_waypoint

        psi_dg = np.arctan2(vy - y, vx - x)

        dx = vx - x
        dy = vy - y
        d_squared = dx**2 + dy**2

        if d_squared < 1e-6:
            psi_dg_dot = 0.0
        else:
            psi_dg_dot = (-self.v * dx * np.sin(psi) + self.v * dy * np.cos(psi)) / d_squared

        e = self.normalize_angle(psi - psi_dg)

        if t < self.tp:
            time_varying_term = self.eta * e / (self.a * (self.tp - t + 1e-6))
            u = (1/self.a) * psi_dg_dot + psi - time_varying_term
        else:
            u = (1/self.a) * psi_dg_dot + psi

        self.last_control = u

        dx_dt = self.v * np.cos(psi)
        dy_dt = self.v * np.sin(psi)
        dpsi_dt = -self.a * psi + self.a * u

        return np.array([dx_dt, dy_dt, dpsi_dt])

    def compute_constant_dynamics(self, x: float, y: float, psi: float) -> np.ndarray:
        """
        Compute state derivatives for constant control mode.
        Maintains current heading for straight-line motion.

        Returns:
            np.array([dx/dt, dy/dt, dpsi/dt])
        """
        dx_dt = self.v * np.cos(psi)
        dy_dt = self.v * np.sin(psi)
        dpsi_dt = 0.0  # Constant heading for straight motion

        return np.array([dx_dt, dy_dt, dpsi_dt])

    def reset(self):
        """Reset for new avoidance maneuver."""
        self.virtual_waypoint = None
        self.last_control = 0.0
