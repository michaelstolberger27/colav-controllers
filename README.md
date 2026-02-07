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

### Straight Line Dynamics

Simple helper for straight-line motion without control:

```python
import numpy as np
from colav_controllers import compute_straight_line_dynamics

# Get state derivatives [dx/dt, dy/dt, dpsi/dt] for straight-line motion
v = 2.0  # Velocity (m/s)
psi = np.radians(45)  # Heading (radians)

dynamics = compute_straight_line_dynamics(v, psi)
# Returns: [v*cos(psi), v*sin(psi), 0]
```

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

# Obstacles as list of (x, y, velocity, heading) tuples
# Note: velocity and heading currently not used in default implementation
ship_x, ship_y, ship_psi = 0.0, 0.0, np.radians(30)
obstacles = [(40.0, 25.0, 0.0, 0.0)]

# Compute and set virtual waypoint V1
controller.set_virtual_waypoint(ship_x, ship_y, ship_psi, obstacles)

# Compute dynamics
dynamics = controller.compute_dynamics(t, ship_x, ship_y, ship_psi)
```

### Multiple Obstacles

```python
obstacles = [
    (30.0, 15.0, 0.0, 0.0),
    (50.0, 35.0, 0.0, 0.0),
]

# Same API for single or multiple obstacles
controller.set_virtual_waypoint(ship_x, ship_y, ship_psi, obstacles)
dynamics = controller.compute_dynamics(t, ship_x, ship_y, ship_psi)
```

### Computing Virtual Waypoint V1 Directly

To compute V1 without storing it in the controller:

```python
from colav_controllers import compute_v1

# Compute V1 with optional buffering
v1 = compute_v1(
    pos_x=ship_x, pos_y=ship_y, psi=ship_psi, 
    obstacles_list=obstacles, Cs=15.0,
    vertex_provider=default_vertex_provider,
    buffer_distance=5.0  # Optional: apply 5m buffer
)

if v1:
    v1_x, v1_y = v1
    print(f"Virtual waypoint V1 at ({v1_x}, {v1_y})")
```

## Advanced Usage

### Understanding Virtual Waypoint Selection

The collision avoidance controller selects V1 (the virtual waypoint) as the **starboard-most vertex ahead of the ship**:

- Vertices are selected only if they are within ±90° of the ship's current heading ("ahead")
- Among ahead vertices, the starboard-most (rightmost, negative relative angle) is selected
- This ensures COLREGs-compliant starboard avoidance
- If no vertices are ahead, no virtual waypoint is computed

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

### PrescribedTimeController Parameters

| Parameter | Description | Units | Constraints |
|-----------|-------------|-------|-------------|
| `a` | Heading dynamics coefficient | 1/s | > 0 |
| `v` | Ship velocity | m/s | > 0 |
| `eta` | Controller gain | - | > 1 (for guaranteed convergence) |
| `tp` | Prescribed time | s | > 0 |

### CollisionAvoidanceController Additional Parameters

| Parameter | Description | Units | Default |
|-----------|-------------|-------|---------|
| `Cs` | Safety distance from obstacles | m | Required |
| `v1_buffer` | Extra buffer for V1 waypoint | m | 0.0 |
| `vertex_provider` | Custom unsafe set vertex function | callable | `None` (uses default 8-vertex circular) |

Inherits `a`, `v`, `eta`, `tp` from PrescribedTimeController.

## Algorithm Details

### Prescribed-Time Control

The controller implements a time-varying control law:

```
u = (1/a)·ψ̇_dg + ψ - η·e/(a·(tp - t))    for t < tp
u = (1/a)·ψ̇_dg + ψ                        for t ≥ tp
```

where:
- `e = normalize_angle(ψ - ψ_dg)` is the heading error (normalized to [-π, π])
- `ψ_dg = atan2(yw - y, xw - x)` is the desired heading (line-of-sight to waypoint)
- `ψ_dg_dot` is the time derivative of desired heading

As t approaches tp, the time-varying term η·e/(a·(tp - t)) increases, driving rapid convergence.

### Collision Avoidance

1. **Unsafe Set Generation**: Create vertices around obstacles using `vertex_provider`
   - Default: 8 vertices in circular pattern at distance `Cs` from each obstacle
   - Custom: User can provide custom vertex provider function

2. **V1 Selection**: Choose starboard-most vertex ahead of ship
   - Filter vertices to those within ±π/2 of ship heading
   - Among ahead vertices, select the one with most negative (starboard) relative angle
   - Returns None if no vertices are ahead

3. **V1 Buffering** (optional): Offset V1 outward from polygon centroid
   - Buffer distance controlled by `v1_buffer` parameter
   - Rejected if buffer would place V1 inside polygon or closer to obstacles

4. **Prescribed-Time Navigation**: Use prescribed-time control to navigate to V1

## API Reference

### PrescribedTimeController

**Methods:**
- `compute_control(t, x, y, psi, xw, yw)` → float: Returns control input u
- `compute_dynamics(t, x, y, psi, xw, yw)` → ndarray: Returns [dx/dt, dy/dt, dpsi/dt]
- `reset()`: No-op (stateless controller)

### CollisionAvoidanceController

**Methods:**
- `set_virtual_waypoint(pos_x, pos_y, psi, obstacles_list)`: Compute and store V1
- `compute_dynamics(t, x, y, psi)` → ndarray: Returns [dx/dt, dy/dt, dpsi/dt] (requires V1 set)

### Geometry Module (utils.geometry)

Low-level geometric utilities for polygon operations:

- `point_in_polygon(px, py, vertices)` → bool: Ray-casting point-in-polygon test
- `polygon_centroid(vertices)` → (float, float): Compute polygon center
- `offset_point_from_centroid(px, py, cx, cy, distance)` → (float, float): Move point outward from centroid
- `distance_to_point(p1_x, p1_y, p2_x, p2_y)` → float: Euclidean distance

### Virtual Waypoint Module (utils.virtual_waypoint)

Virtual waypoint computation and buffering:

- `compute_v1(pos_x, pos_y, psi, obstacles, Cs, vertex_provider, buffer_distance=0)` → (float, float) or None: Select starboard-most vertex ahead with optional buffering
- `apply_v1_buffer(v1_x, v1_y, vertices, obstacles, buffer_distance, Cs)` → (float, float): Apply safety buffer with rejection checks
- `default_vertex_provider(pos_x, pos_y, obstacles, Cs, psi)` → [(float, float)] or None: Generate 8-vertex circular unsafe sets

### Utility Functions (utils.helpers)

- `compute_straight_line_dynamics(v, psi)` → ndarray: Returns [v*cos(psi), v*sin(psi), 0]
- `normalize_angle(angle)` → float: Normalizes angle to [-π, π]

## Architecture

### Module Organization

```
colav_controllers/
├── collision_avoidance.py     Main controller orchestrating avoidance logic
├── prescribed_time.py         Prescribed-time heading control law
└── utils/
    ├── geometry.py            Geometric utilities (polygon operations, point tests)
    ├── helpers.py             Shared utilities (angle normalization, dynamics)
    └── virtual_waypoint.py    V1 selection algorithm and buffering
```

## Dependencies

- `numpy>=1.20`

Development/examples require:
- `scipy` - For ODE integration in examples
- `matplotlib` - For visualization in notebooks

