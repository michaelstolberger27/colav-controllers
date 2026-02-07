"""
Ship navigation controllers for prescribed-time heading control and
COLREGs-compliant collision avoidance.
"""

from colav_controllers.__about__ import __version__
from colav_controllers.utils import (
    normalize_angle,
    compute_straight_line_dynamics,
    point_in_polygon,
    polygon_centroid,
    offset_point_from_centroid,
    distance_to_point,
    default_vertex_provider,
    compute_v1,
    apply_v1_buffer,
)
from colav_controllers.prescribed_time import PrescribedTimeController
from colav_controllers.collision_avoidance import CollisionAvoidanceController

__all__ = [
    "__version__",
    "normalize_angle",
    "compute_straight_line_dynamics",
    "PrescribedTimeController",
    "CollisionAvoidanceController",
    "point_in_polygon",
    "polygon_centroid",
    "offset_point_from_centroid",
    "distance_to_point",
    "default_vertex_provider",
    "compute_v1",
    "apply_v1_buffer",
]