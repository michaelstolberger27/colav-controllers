"""
Ship navigation controllers for prescribed-time heading control and
COLREGs-compliant collision avoidance.
"""

from colav_controllers.__about__ import __version__
from colav_controllers.prescribed_time import PrescribedTimeController
from colav_controllers.collision_avoidance import CollisionAvoidanceController

__all__ = [
    "__version__",
    "PrescribedTimeController",
    "CollisionAvoidanceController",
]
