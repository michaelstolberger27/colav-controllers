# colav-controllers

Ship navigation controllers for prescribed-time heading control and COLREGs-compliant collision avoidance.

## Installation

```bash
pip install colav-controllers
```

## Controllers

### PrescribedTimeController

Prescribed-time heading controller that guarantees heading convergence to line-of-sight (LOS) in prescribed time tp.

```python
from colav_controllers import PrescribedTimeController

controller = PrescribedTimeController(a=1.0, v=12.0, eta=2.0, tp=15.0)
dynamics = controller.compute_dynamics(t, x, y, psi, waypoint_x, waypoint_y)
```

### CollisionAvoidanceController

COLREGs-compliant collision avoidance controller that:
- Creates unsafe sets from obstacle positions
- Computes virtual waypoint V1 (starboard vertex of unsafe set)
- Uses prescribed-time control to navigate around obstacles

```python
from colav_controllers import CollisionAvoidanceController

controller = CollisionAvoidanceController(
    a=1.0,
    v=12.0,
    eta=2.0,
    tp=15.0,
    Cs=50.0,  # Safety radius
    vertex_provider=my_vertex_function  # Optional custom vertex provider
)

# Set virtual waypoint from obstacles
controller.set_virtual_waypoint_dynamic(pos_x, pos_y, psi, obstacles_list)

# Compute dynamics
dynamics = controller.compute_dynamics(t, x, y, psi)
```

## Custom Vertex Provider

The `CollisionAvoidanceController` accepts an optional `vertex_provider` callable:

```python
def my_vertex_provider(pos_x, pos_y, obstacles_list, Cs):
    """
    Args:
        pos_x, pos_y: Ship position
        obstacles_list: List of (ox, oy, ov, o_psi) tuples
        Cs: Safety radius

    Returns:
        List of (vx, vy) vertex tuples or None
    """
    # Your implementation here
    return vertices
```
