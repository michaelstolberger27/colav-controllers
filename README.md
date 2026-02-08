# colav-controllers

Ship navigation controllers for prescribed-time heading control and COLREGs-compliant collision avoidance.

**Note**: This package provides the control layer. For a complete hybrid automaton-based collision avoidance system, see [usv-navigation](https://github.com/michaelstolberger27/usv-navigation).

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

### Computing Virtual Waypoint V1

V1 is the **starboard-most unsafe set vertex ahead of the ship**, used as an intermediate waypoint during collision avoidance.

```python
from colav_controllers import compute_v1, default_vertex_provider

ship_x, ship_y, ship_psi = 0.0, 0.0, np.radians(30)
obstacles = [(40.0, 25.0, 0.0, 0.0)]  # (x, y, velocity, heading)

v1 = compute_v1(
    pos_x=ship_x, pos_y=ship_y, psi=ship_psi,
    obstacles_list=obstacles, Cs=15.0,
    vertex_provider=default_vertex_provider,
    buffer_distance=5.0  # Optional: apply 5m outward buffer
)

if v1:
    v1_x, v1_y = v1
    print(f"Virtual waypoint V1 at ({v1_x}, {v1_y})")
```

### Unsafe Set Computation

Generate dynamic unsafe regions around obstacles using DCPA/TCPA metrics:

```python
from colav_controllers import get_unsafe_set_vertices, create_los_cone, check_collision_threat

obstacles = [
    (30.0, 15.0, 2.0, np.radians(180)),  # Moving obstacle
    (50.0, 35.0, 0.0, 0.0),               # Stationary obstacle
]

# Get convex hull vertices of dynamic unsafe set
vertices = get_unsafe_set_vertices(
    ship_x=0.0, ship_y=0.0,
    obstacles_list=obstacles, Cs=15.0,
    ship_psi=0.0, ship_v=12.0,
    use_swept_region=True  # Predict future obstacle positions
)

# Check if any obstacle poses a collision threat
threat = check_collision_threat(
    pos_x=0.0, pos_y=0.0, psi=0.0,
    obstacles_list=obstacles,
    ship_v=12.0, Cs=15.0, dsafe=30.0
)
```

## Advanced Usage

### Understanding Virtual Waypoint Selection

The V1 selection algorithm:
- Vertices are selected only if they are within +/-90 deg of the ship's current heading ("ahead")
- Among ahead vertices, the starboard-most (most negative relative angle) is selected
- This ensures COLREGs-compliant starboard avoidance
- If no vertices are ahead, no virtual waypoint is computed

### V1 Buffer for Extra Clearance

The `buffer_distance` parameter offsets V1 outward from the unsafe set polygon centroid. The buffer is rejected if it would place V1 inside the polygon or closer to any obstacle.

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
        for i in range(8):
            angle = i * np.pi / 4
            vertices.append((ox + Cs * np.cos(angle), oy + Cs * np.sin(angle)))
    return vertices if vertices else None

v1 = compute_v1(
    pos_x=0.0, pos_y=0.0, psi=0.0,
    obstacles_list=obstacles, Cs=15.0,
    vertex_provider=custom_vertex_provider
)
```

### Swept Regions for Moving Obstacles

The `get_unsafe_set_vertices` function can predict future obstacle positions to create a "swept" region covering the obstacle's trajectory. This is controlled by the `use_swept_region` parameter:
- `True`: Predicts positions at 50% and 100% of estimated maneuver time (use for V1 computation)
- `False`: Uses current positions only (use for guard condition checks)

## Parameters

### PrescribedTimeController

| Parameter | Description | Units | Constraints |
|-----------|-------------|-------|-------------|
| `a` | Heading dynamics coefficient | 1/s | > 0 |
| `v` | Ship velocity | m/s | > 0 |
| `eta` | Controller gain | - | > 1 (for guaranteed convergence) |
| `tp` | Prescribed time | s | > 0 |

## Algorithm Details

### Prescribed-Time Control

The controller implements a time-varying control law:

```
u = (1/a)*psi_dg_dot + psi - eta*e/(a*(tp - t))    for t < tp
u = (1/a)*psi_dg_dot + psi                          for t >= tp
```

where:
- `e = atan2(sin(psi - psi_dg), cos(psi - psi_dg))` is the heading error (normalized to [-pi, pi])
- `psi_dg = atan2(yw - y, xw - x)` is the desired heading (line-of-sight to waypoint)
- `psi_dg_dot` is the time derivative of desired heading

As t approaches tp, the time-varying term drives rapid convergence.

### Collision Avoidance

1. **Unsafe Set Generation**: Create vertices around obstacles using `vertex_provider`
   - Default: 8 vertices in circular pattern at distance `Cs` from each obstacle
   - Dynamic: `get_unsafe_set_vertices` uses `colav_unsafe_set` for DCPA/TCPA-based regions

2. **V1 Selection**: Choose starboard-most vertex ahead of ship
   - Filter vertices to those within +/-pi/2 of ship heading
   - Among ahead vertices, select the one with most negative (starboard) relative angle

3. **V1 Buffering** (optional): Offset V1 outward from polygon centroid
   - Rejected if buffer would place V1 inside polygon or closer to obstacles

4. **Prescribed-Time Navigation**: Use prescribed-time control to navigate to V1

## API Reference

### PrescribedTimeController

- `compute_control(t, x, y, psi, xw, yw)` -> float: Returns control input u
- `compute_dynamics(t, x, y, psi, xw, yw)` -> ndarray: Returns [dx/dt, dy/dt, dpsi/dt]
- `reset()`: No-op (stateless controller)

### Virtual Waypoint (virtual_waypoint)

- `compute_v1(pos_x, pos_y, psi, obstacles_list, Cs, vertex_provider, buffer_distance=0)` -> (float, float) or None: Select starboard-most vertex ahead with optional buffering
- `default_vertex_provider(pos_x, pos_y, obstacles_list, Cs, psi)` -> [(float, float)] or None: Generate 8-vertex circular unsafe sets

### Unsafe Sets (unsafe_sets)

- `get_unsafe_set_vertices(ship_x, ship_y, obstacles_list, Cs, dsf=None, ship_psi=0, ship_v=12, use_swept_region=True)` -> List[List[float]] or None: Generate dynamic unsafe set vertices via `colav_unsafe_set`
- `create_los_cone(pos_x, pos_y, xw, yw, v, tp)` -> Polygon: Create LOS cone F(p(t)) = conv(B_2(p(t), v*tp), pw)
- `compute_unified_unsafe_region(pos_x, pos_y, obstacles_list, Cs, ship_psi=0, ship_v=12)` -> Polygon or None: Unified unsafe region from all obstacles
- `check_collision_threat(pos_x, pos_y, psi, obstacles_list, ship_v, Cs, dsafe)` -> bool: Check if any obstacle threatens collision using DCPA/TCPA

## Architecture

```
colav_controllers/
├── __init__.py             Public API (8 exports)
├── __about__.py            Package version
├── prescribed_time.py      Prescribed-time heading control law
├── virtual_waypoint.py     V1 selection algorithm and buffering
└── unsafe_sets.py          Dynamic unsafe sets, LOS cones, collision checks
```

## Dependencies

- `numpy>=1.20`
- `shapely` - Polygon intersection for LOS cone checks
- `colav_unsafe_set` - Dynamic unsafe set computation (DCPA/TCPA)

Development/examples require:
- `scipy` - For ODE integration in examples
- `matplotlib` - For visualization
