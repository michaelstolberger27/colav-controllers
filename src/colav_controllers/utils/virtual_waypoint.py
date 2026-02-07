"""
Virtual waypoint computation and buffering for collision avoidance.

Handles V1 selection from unsafe set vertices and optional buffering.
"""

from typing import List, Tuple, Optional, Callable
import numpy as np
from colav_controllers.utils.helpers import normalize_angle
from colav_controllers.utils.geometry import (
    point_in_polygon,
    polygon_centroid,
    offset_point_from_centroid,
    distance_to_point,
)


def default_vertex_provider(
    pos_x: float,
    pos_y: float,
    obstacles_list: List[Tuple[float, float, float, float]],
    Cs: float,
    psi: float = 0.0
) -> Optional[List[Tuple[float, float]]]:
    """
    Default vertex provider using circular obstacle approximation.

    Creates 8 vertices around each obstacle at distance Cs, representing
    the unsafe set boundary.

    Args:
        pos_x, pos_y: Ship position (currently unused in circular approximation)
        obstacles_list: List of (ox, oy, ov, o_psi) tuples
        Cs: Safety radius around obstacles
        psi: Ship heading (currently unused in circular approximation)

    Returns:
        List of (vx, vy) vertices or None if no obstacles
    """
    if not obstacles_list:
        return None

    vertices = []
    for ox, oy, _, _ in obstacles_list:
        for i in range(8):
            angle = i * np.pi / 4
            vertices.append((ox + Cs * np.cos(angle), oy + Cs * np.sin(angle)))

    return vertices if vertices else None


def compute_v1(
    pos_x: float,
    pos_y: float,
    psi: float,
    obstacles_list: List[Tuple[float, float, float, float]],
    Cs: float,
    vertex_provider: Callable[
        [float, float, List[Tuple[float, float, float, float]], float, float],
        Optional[List[Tuple[float, float]]]
    ],
    buffer_distance: float = 0.0
) -> Optional[Tuple[float, float]]:
    """
    Compute virtual waypoint V1 (starboard-most vertex ahead of ship).

    Selects the unsafe set vertex that is:
    1. Within ±90° of ship heading ("ahead")
    2. Starboard-most (most negative relative angle among ahead vertices)
    3. Optionally applies outward buffer for extra safety margin

    Args:
        pos_x, pos_y: Current ship position
        psi: Current heading (radians)
        obstacles_list: List of (ox, oy, ov, o_psi) tuples
        Cs: Safety distance from obstacles
        vertex_provider: Function to generate unsafe set vertices
        buffer_distance: Optional buffer distance to apply (default 0.0)

    Returns:
        Tuple (v1_x, v1_y) or None if no valid vertex ahead
    """
    if not obstacles_list:
        return None

    vertices = vertex_provider(pos_x, pos_y, obstacles_list, Cs, psi)
    if not vertices:
        return None

    best_vertex = None
    best_angle = np.inf

    for vx, vy in vertices:
        # Angle from ship to vertex
        angle_to_vertex = np.arctan2(vy - pos_y, vx - pos_x)
        # Angle relative to ship heading
        relative_angle = normalize_angle(angle_to_vertex - psi)

        # Filter: only vertices ahead (within ±π/2 of heading)
        if -np.pi/2 < relative_angle < np.pi/2:
            # Among ahead vertices, prefer starboard (most negative angle)
            if relative_angle < best_angle:
                best_angle = relative_angle
                best_vertex = (vx, vy)

    if best_vertex is None:
        return None

    # Apply buffering if configured
    if buffer_distance > 0:
        best_vertex = apply_v1_buffer(
            best_vertex[0], best_vertex[1], vertices, obstacles_list, 
            buffer_distance, Cs
        )

    return best_vertex


def apply_v1_buffer(
    v1_x: float,
    v1_y: float,
    vertices: List[Tuple[float, float]],
    obstacles_list: List[Tuple[float, float, float, float]],
    buffer_distance: float,
    Cs: float
) -> Tuple[float, float]:
    """
    Apply buffer to V1 by moving it outward from polygon centroid.

    The buffer is rejected (original V1 returned) if:
    - Buffered V1 would be inside the unsafe polygon
    - Buffered V1 would be closer to any obstacle than original V1

    Args:
        v1_x, v1_y: Original virtual waypoint position
        vertices: Unsafe set polygon vertices
        obstacles_list: List of obstacles
        buffer_distance: Distance to offset V1 outward
        Cs: Safety distance (used for rejection checks)

    Returns:
        Tuple (final_v1_x, final_v1_y)
    """
    if buffer_distance <= 0 or not vertices or len(vertices) < 3:
        return (v1_x, v1_y)

    centroid_x, centroid_y = polygon_centroid(vertices)
    buffered_x, buffered_y = offset_point_from_centroid(
        v1_x, v1_y, centroid_x, centroid_y, buffer_distance
    )

    # Reject if buffered point is same as original (offset failed)
    if (buffered_x, buffered_y) == (v1_x, v1_y):
        return (v1_x, v1_y)

    # Reject if buffered V1 would be inside polygon
    if point_in_polygon(buffered_x, buffered_y, vertices):
        return (v1_x, v1_y)

    # Reject if buffered V1 is closer to any obstacle
    for ox, oy, _, _ in obstacles_list:
        orig_dist = distance_to_point(v1_x, v1_y, ox, oy)
        buffered_dist = distance_to_point(buffered_x, buffered_y, ox, oy)
        if buffered_dist < orig_dist - 0.1:
            return (v1_x, v1_y)

    return (buffered_x, buffered_y)
