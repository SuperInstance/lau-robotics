# lau-robotics

Robotics fundamentals — kinematics, dynamics, and path planning for agents with spatial awareness and movement.

## Features

- **Rigid Body Transformations**: Rotation matrices (SO(3)), quaternions, homogeneous transforms (SE(3)), DH parameters
- **Forward Kinematics**: DH parameter serial chains
- **Inverse Kinematics**: Numerical Jacobian-based (damped least squares)
- **Velocity Kinematics**: Geometric Jacobian computation, manipulability analysis
- **Path Planning**: RRT, A* on grids, artificial potential fields
- **PID Motion Control**: With anti-windup and output clamping
- **Collision Detection**: AABB, bounding spheres, sphere-AABB
- **Sensor Models**: Range sensor with noise, odometry with drift
- **Agent Physical Modeling**: SpatialAgent with path following and collision awareness

## Usage

```rust
use lau_robotics::transforms::{RotationMatrix, Quaternion, HomogeneousTransform};
use lau_robotics::kinematics::{DHLink, SerialChain, forward_kinematics};
use lau_robotics::path_planning::astar::{astar_grid, GridCell};
use lau_robotics::control::PIDController;
use lau_robotics::agent::SpatialAgent;
```

## License

MIT
