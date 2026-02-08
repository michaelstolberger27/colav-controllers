"""
Ship navigation controllers for prescribed-time heading control and
COLREGs-compliant collision avoidance.
"""

from colav_controllers.__about__ import __version__
from colav_controllers.prescribed_time import PrescribedTimeController
from colav_controllers.virtual_waypoint import compute_v1, default_vertex_provider
from colav_controllers.unsafe_sets import (
    get_unsafe_set_vertices,
    create_los_cone,
    compute_unified_unsafe_region,
    check_collision_threat,
)

__all__ = [
    "__version__",
    "PrescribedTimeController",
    "compute_v1",
    "default_vertex_provider",
    "get_unsafe_set_vertices",
    "create_los_cone",
    "compute_unified_unsafe_region",
    "check_collision_threat",
]
