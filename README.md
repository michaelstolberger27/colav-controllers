# colav-controllers

Ship navigation controllers for prescribed-time heading control and COLREGs-compliant collision avoidance.

## Features

- **Prescribed-Time Controller**: Guaranteed heading convergence to line-of-sight (LOS) in prescribed time
- **Collision Avoidance Controller**: COLREGs-compliant obstacle avoidance with virtual waypoint generation
- **Configurable Safety Margins**: Adjustable safety distance (`Cs`) and optional V1 buffer for extra clearance
- **Custom Unsafe Sets**: Support for custom vertex providers to define obstacle unsafe sets
- **Multi-Mode Operation**: Avoidance mode and constant heading mode

## Installation

```bash
pip install colav-controllers
```

For development:
```bash
git clone https://github.com/michaelstolberger27/colav-controllers.git
cd colav-controllers
pip install -e .
```

## Quick Start

### PrescribedTimeController

Guarantees heading convergence to waypoint line-of-sight within prescribed time `tp`.

```python
import numpy as np
from colav_controllers import PrescribedTimeController

# Initialize controller
controller = PrescribedTimeController(
    a=1.0,      # Heading dynamics coefficient
    v=2.0,      # Ship velocity (m/s)
    eta=2.5,    # Controller gain (must be > 1)
    tp=5.0      # Prescribed time (seconds)
)

# Compute dynamics at each time step
t = 0.0
x, y, psi = 0.0, 0.0, np.radians(45)  # Ship state
waypoint_x, waypoint_y = 50.0, 50.0   # Target waypoint

# Get state derivatives [dx/dt, dy/dt, dpsi/dt]
dynamics = controller.compute_dynamics(t, x, y, psi, waypoint_x, waypoint_y)

# Or get control input directly
u = controller.compute_control(t, x, y, psi, waypoint_x, waypoint_y)
```

### CollisionAvoidanceController

COLREGs-compliant collision avoidance using virtual waypoint generation.

```python
from colav_controllers import CollisionAvoidanceController

# Initialize controller
controller = CollisionAvoidanceController(
    a=1.0,          # Heading dynamics coefficient
    v=2.0,          # Ship velocity (m/s)
    eta=2.0,        # Controller gain (must be > 1)
    tp=3.0,         # Prescribed time (seconds)
    Cs=15.0,        # Safety distance from obstacles (m)
    v1_buffer=0.0   # Optional: extra buffer distance for V1 (m)
)

# Single obstacle avoidance
ship_x, ship_y, ship_psi = 0.0, 0.0, np.radians(30)
obstacle_x, obstacle_y = 40.0, 25.0

# Compute and set virtual waypoint V1
controller.set_virtual_waypoint(ship_x, ship_y, ship_psi, obstacle_x, obstacle_y)

# Compute dynamics
dynamics = controller.compute_dynamics(t, ship_x, ship_y, ship_psi)
```

### Multiple Obstacles

```python
# Multiple obstacles: (x, y, velocity, heading)
obstacles = [
    (30.0, 15.0, 0.0, 0.0),
    (50.0, 35.0, 0.0, 0.0),
]

# Compute virtual waypoint from multiple obstacles
controller.set_virtual_waypoint_dynamic(ship_x, ship_y, ship_psi, obstacles)

# Compute dynamics
dynamics = controller.compute_dynamics(t, ship_x, ship_y, ship_psi)
```

### Constant Heading Mode

After obstacle avoidance, maintain constant heading:

```python
# Switch to constant heading mode (no controller action)
dynamics = controller.compute_constant_dynamics(ship_x, ship_y, ship_psi)
```

## Advanced Usage

### V1 Buffer for Extra Clearance

The `v1_buffer` parameter offsets the virtual waypoint V1 outward from the unsafe set polygon for additional safety margin:

```python
controller = CollisionAvoidanceController(
    a=1.0, v=2.0, eta=2.0, tp=3.0,
    Cs=15.0,        # Base safety distance
    v1_buffer=5.0   # Additional 5m buffer
)
```

### Custom Vertex Provider

Define custom unsafe set shapes by providing a vertex provider function:

```python
def custom_vertex_provider(pos_x, pos_y, obstacles_list, Cs, psi):
    """
    Custom vertex provider for obstacle unsafe sets.

    Args:
        pos_x, pos_y: Ship position (m)
        obstacles_list: List of (ox, oy, ov, o_psi) tuples
        Cs: Safety distance (m)
        psi: Ship heading (radians)

    Returns:
        List of (vx, vy) vertex tuples or None
    """
    vertices = []
    for ox, oy, _, _ in obstacles_list:
        # Example: Square unsafe set
        vertices.extend([
            (ox + Cs, oy + Cs),
            (ox - Cs, oy + Cs),
            (ox - Cs, oy - Cs),
            (ox + Cs, oy - Cs),
        ])
    return vertices if vertices else None

# Use custom vertex provider
controller = CollisionAvoidanceController(
    a=1.0, v=2.0, eta=2.0, tp=3.0, Cs=15.0,
    vertex_provider=custom_vertex_provider
)
```

## Examples

Interactive Jupyter notebooks demonstrating the controllers are available in the `notebooks/` directory:

- **[prescribed_time_demo.ipynb](notebooks/prescribed_time_demo.ipynb)** - Waypoint navigation with heading convergence guarantees
- **[collision_avoidance_demo.ipynb](notebooks/collision_avoidance_demo.ipynb)** - Obstacle avoidance scenarios including:
  - Single and multiple obstacle avoidance
  - V1 buffer demonstration
  - Custom vertex providers
  - Two-phase navigation (avoidance + constant heading)

To run the notebooks:

```bash
pip install jupyter matplotlib scipy
jupyter notebook notebooks/
```

## Parameters

### Common Parameters

| Parameter | Description | Units | Constraints |
|-----------|-------------|-------|-------------|
| `a` | Heading dynamics coefficient | 1/s | > 0 |
| `v` | Ship velocity | m/s | > 0 |
| `eta` | Controller gain | - | > 1 (for guaranteed convergence) |
| `tp` | Prescribed time | s | > 0 |

### CollisionAvoidanceController Parameters

| Parameter | Description | Units | Default |
|-----------|-------------|-------|---------|
| `Cs` | Safety distance from obstacles | m | Required |
| `v1_buffer` | Extra buffer for V1 waypoint | m | 0.0 |
| `vertex_provider` | Custom unsafe set vertex function | callable | `None` (uses default circular) |

## Algorithm Details

### Prescribed-Time Control

The controller implements a time-varying control law that guarantees heading error convergence by time `tp`:

```
u = (1/a)·ψ̇_dg + ψ - η·e/(a·(tp - t))    for t < tp
u = (1/a)·ψ̇_dg + ψ                        for t ≥ tp
```

where:
- `e = ψ - ψ_dg` is the heading error
- `ψ_dg = atan2(yw - y, xw - x)` is the desired heading (LOS to waypoint)

### Collision Avoidance

1. **Unsafe Set Generation**: Create vertices around obstacles at safety distance `Cs`
2. **V1 Selection**: Choose starboard-most vertex ahead of ship (COLREGs-compliant)
3. **V1 Buffering** (optional): Offset V1 outward from polygon centroid
4. **Prescribed-Time Navigation**: Use prescribed-time control to navigate to V1

## Dependencies

- `numpy>=1.20`

Development/examples require:
- `scipy` - For ODE integration in examples
- `matplotlib` - For visualization in notebooks

