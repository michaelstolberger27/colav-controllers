"""
Geometric utilities for collision avoidance computations.

Handles polygon operations, point-in-polygon tests, and geometric calculations.
"""

from typing import List, Tuple
import numpy as np


def point_in_polygon(px: float, py: float, vertices: List[Tuple[float, float]]) -> bool:
    """
    Check if point (px, py) is inside a polygon using ray casting algorithm.

    Args:
        px, py: Point coordinates to check
        vertices: List of (x, y) polygon vertices in order

    Returns:
        True if point is inside polygon, False otherwise
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


def polygon_centroid(vertices: List[Tuple[float, float]]) -> Tuple[float, float]:
    """
    Compute centroid (geometric center) of a polygon.

    Args:
        vertices: List of (x, y) polygon vertices

    Returns:
        Tuple (centroid_x, centroid_y)
    """
    if not vertices:
        return (0.0, 0.0)
    
    centroid_x = sum(v[0] for v in vertices) / len(vertices)
    centroid_y = sum(v[1] for v in vertices) / len(vertices)
    return (centroid_x, centroid_y)


def offset_point_from_centroid(
    point_x: float,
    point_y: float,
    centroid_x: float,
    centroid_y: float,
    offset_distance: float
) -> Tuple[float, float]:
    """
    Move a point outward from a centroid by a specified distance.

    Args:
        point_x, point_y: Original point coordinates
        centroid_x, centroid_y: Centroid coordinates
        offset_distance: Distance to move outward (positive values move away from centroid)

    Returns:
        Tuple (new_x, new_y) or original point if offset fails
    """
    outward = np.array([point_x - centroid_x, point_y - centroid_y])
    outward_len = np.linalg.norm(outward)
    
    if outward_len < 1e-6:
        return (point_x, point_y)
    
    outward_normalized = outward / outward_len
    new_x = point_x + offset_distance * outward_normalized[0]
    new_y = point_y + offset_distance * outward_normalized[1]
    
    return (new_x, new_y)


def distance_to_point(
    p1_x: float,
    p1_y: float,
    p2_x: float,
    p2_y: float
) -> float:
    """
    Compute Euclidean distance between two points.

    Args:
        p1_x, p1_y: First point
        p2_x, p2_y: Second point

    Returns:
        Euclidean distance
    """
    return np.sqrt((p1_x - p2_x)**2 + (p1_y - p2_y)**2)
