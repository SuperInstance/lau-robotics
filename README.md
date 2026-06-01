# lau-robotics

**Robotics fundamentals in Rust** — spatial transforms, serial-link kinematics, PID control, path planning, collision detection, sensor simulation, and multi-agent coordination.

## What This Does

This crate provides the building blocks for 2D/3D robotics:

- **Spatial transforms** — rotation matrices (SO(3)), unit quaternions, homogeneous transforms (SE(3)), Euler angles, SLERP interpolation, axis-angle conversions
- **Denavit-Hartenberg kinematics** — define serial robot arms via DH parameters, compute forward kinematics, and get all intermediate joint transforms
- **Inverse kinematics** — damped least-squares IK solver (Levenberg-Marquardt style) for position targets
- **Jacobian analysis** — geometric 6×n Jacobian for serial chains, Yoshikawa manipulability index, condition number, singular values
- **Path planning** — A* on occupancy grids, RRT in continuous 2D space, artificial potential fields
- **PID control** — proportional-integral-derivative controller with output clamping and anti-windup
- **Collision detection** — AABB-AABB, sphere-sphere, sphere-AABB intersection tests
- **Sensor simulation** — odometry model with configurable noise (differential drive), range sensor with beam cone and Gaussian noise
- **Spatial agents** — autonomous agents with bounding volumes, path following, heading/speed PID, odometry, and collision checking

## Key Idea

Robotics software is built on a stack: transforms → kinematics → control → planning → sensing. This library implements each layer and wires them together through the `SpatialAgent` type, which combines a collision volume, PID controllers, odometry, and waypoint following into a single usable unit.

All math uses `nalgebra` for matrix operations and everything is `Serialize`/`Deserialize` for logging and state persistence.

## Install

```toml
[dependencies]
lau-robotics = "0.1.0"
```

Requires Rust 2021 edition. Depends on `nalgebra` (linear algebra), `serde`/`serde_json` (serialization), and `rand` (RRT sampling, sensor noise).

## Quick Start

### Forward Kinematics for a 2-Link Planar Arm

```rust
use lau_robotics::kinematics::{DHLink, SerialChain};
use lau_robotics::transforms::HomogeneousTransform;

let links = vec![
    DHLink::revolute(0.0, 0.0, 1.0, 0.0),  // link length = 1.0
    DHLink::revolute(0.0, 0.0, 1.0, 0.0),  // link length = 1.0
];
let chain = SerialChain::new(links);
let ee = chain.fk(&[std::f64::consts::FRAC_PI_4, std::f64::consts::FRAC_PI_4]);
println!("End-effector position: {:?}", ee.translation());
```

### Inverse Kinematics

```rust
use lau_robotics::kinematics::inverse::inverse_kinematics;

let target = HomogeneousTransform::from_translation(&nalgebra::Vector3::new(1.5, 0.5, 0.0));
let solution = inverse_kinematics(&chain, &target, &[0.0, 0.0], 100, 1e-4, 0.1);
if let Some(joints) = solution {
    println!("IK solution: {:?}", joints);
}
```

### A* Path Planning

```rust
use lau_robotics::path_planning::astar::{astar_grid, GridCell, euclidean_distance};

let obstacles = vec![(5, 3), (5, 4), (5, 5)];
let path = astar_grid(
    GridCell::new(0, 0),
    GridCell::new(10, 10),
    |cell| obstacles.iter().any(|&(x, y)| cell.x == x && cell.y == y),
    euclidean_distance,
    true,  // 8-connected
);
```

### RRT Planning

```rust
use lau_robotics::path_planning::rrt::RRT;
use nalgebra::Vector2;

let mut rrt = RRT::new(
    Vector2::new(0.0, 0.0),
    0.5,   // step size
    5000,  // max iterations
    (Vector2::new(-10.0, -10.0), Vector2::new(10.0, 10.0)),
);
let path = rrt.plan(&Vector2::new(5.0, 5.0), 0.3, |p| {
    // Collision check: obstacle at (2, 2) with radius 0.5
    (p - Vector2::new(2.0, 2.0)).norm() < 0.5
});
```

### PID Control

```rust
use lau_robotics::control::PIDController;

let mut pid = PIDController::new(2.0, 0.5, 0.1)
    .with_limits(-1.0, 1.0)
    .with_integral_limit(5.0);
pid.set_setpoint(10.0);

let dt = 0.01;
let mut measurement = 0.0;
for _ in 0..500 {
    let output = pid.update(measurement, dt);
    measurement += output * dt; // simplified plant
}
```

### Spatial Agent with Path Following

```rust
use lau_robotics::agent::SpatialAgent;
use nalgebra::Vector3;

let mut agent = SpatialAgent::new("robot-1", Vector3::new(0.0, 0.0, 0.0), 0.0, 0.3);
agent.set_path(vec![
    nalgebra::Vector2::new(2.0, 0.0),
    nalgebra::Vector2::new(2.0, 3.0),
    nalgebra::Vector2::new(5.0, 3.0),
]);

let dt = 0.05;
while agent.step(dt) {
    println!("Pose: {:?}", agent.pose_2d());
}
```

## API Reference

### `transforms` — SO(3) and SE(3)

| Type / Fn | Description |
|-----------|-------------|
| `RotationMatrix` | 3×3 rotation matrix in SO(3) |
| `.from_axis_x/y/z(angle)` | Rotation about a principal axis |
| `.from_axis_angle(axis, angle)` | Rodrigues' rotation formula |
| `.from_euler_zyx(roll, pitch, yaw)` | Build from Euler angles |
| `.to_euler_zyx()` | Extract (roll, pitch, yaw) |
| `.is_valid(tol)` | Verify RᵀR = I and det = 1 |
| `Quaternion` | Unit quaternion (w, x, y, z) |
| `.from_axis_angle(axis, angle)` | Axis-angle → quaternion |
| `.from_rotation_matrix(R)` | Matrix → quaternion (Shepperd's method) |
| `.to_rotation_matrix()` | Quaternion → matrix |
| `.slerp(other, t)` | Spherical linear interpolation |
| `.rotate_vector(v)` | Rotate a 3D vector by the quaternion |
| `HomogeneousTransform` | 4×4 rigid-body transform in SE(3) |
| `.from_rotation_translation(R, t)` | Build from rotation + translation |
| `.from_dh(θ, d, a, α)` | Build from DH parameters |
| `.inverse()` | Efficient SE(3) inverse (Rᵀ, −Rᵀt) |
| `.transform_point(p)` | Apply transform to a 3D point |

### `kinematics` — Serial-Chain Robot Arms

| Type / Fn | Description |
|-----------|-------------|
| `DHLink` | Single DH parameter set (θ, d, a, α) |
| `DHLink::revolute/prismatic(...)` | Construct a joint |
| `.with_joint_value(q)` | Set the joint variable |
| `SerialChain` | A chain of DH links |
| `.fk(joint_values)` | Forward kinematics → end-effector transform |
| `.fk_all(joint_values)` | All intermediate transforms |
| `forward_kinematics(links, q)` | Convenience function |
| `inverse::inverse_kinematics(...)` | Damped least-squares IK |
| `jacobian::compute_jacobian(chain, q)` | 6×n geometric Jacobian |
| `jacobian::Manipulability::compute(J)` | Yoshikawa index, condition number, singular values |

### `path_planning` — A*, RRT, Potential Fields

| Type / Fn | Description |
|-----------|-------------|
| `astar::GridCell` | 2D grid coordinate |
| `astar::astar_grid(start, goal, occupied, heuristic, 8-connected)` | A* on a grid |
| `astar::manhattan_distance / euclidean_distance` | Built-in heuristics |
| `rrt::RRT` | Rapidly-exploring Random Tree planner |
| `.plan(goal, tolerance, collision_fn)` | Plan a path (10% goal bias) |
| `potential_field::PotentialField` | Artificial potential field planner |
| `.plan(start, goal, obstacles)` | Gradient descent toward goal |

### `control` — PID

| Type / Fn | Description |
|-----------|-------------|
| `PIDController` | PID with anti-windup and output clamping |
| `.new(kp, ki, kd)` | Create a controller |
| `.with_limits(min, max)` | Clamp output |
| `.with_integral_limit(limit)` | Anti-windup |
| `.update(measurement, dt)` | One PID step |
| `.reset()` | Clear internal state |

### `collision` — Bounding Volumes

| Type / Fn | Description |
|-----------|-------------|
| `AABB` | Axis-aligned bounding box |
| `.intersects(other)` / `.contains_point(p)` / `.merge(other)` | Standard operations |
| `SphereBound` | Bounding sphere |
| `CollisionDetector::sphere_sphere / aabb_aabb / sphere_aabb` | Pairwise tests |

### `sensor` — Odometry and Range

| Type / Fn | Description |
|-----------|-------------|
| `OdometryModel` | 2D odometry with configurable noise |
| `.update(distance, dheading)` | Update from arc model |
| `.update_from_wheels(left, right, wheel_base)` | Update from differential drive |
| `RangeSensor` | Simulated range finder with beam cone |
| `.measure(pos, heading, obstacles)` | Get noisy range reading |

### `agent` — SpatialAgent

| Type / Fn | Description |
|-----------|-------------|
| `SpatialAgent` | An agent with position, heading, collision sphere, PID, and odometry |
| `.new(id, position, heading, radius)` | Create an agent |
| `.set_path(waypoints)` | Assign a path |
| `.step(dt)` | Advance one timestep (returns false when done) |
| `.collides_with_aabb / .collides_with_agent` | Collision checks |
| `.transform()` | Get SE(3) world-to-body transform |

## How It Works

1. **Transforms are 4×4 matrices.** Rotation matrices use `nalgebra::Matrix3`, quaternions use the (w,x,y,z) convention with automatic normalization, and homogeneous transforms stack rotation and translation into `Matrix4`. SE(3) inverse uses the efficient formula (Rᵀ, −Rᵀt) rather than a full 4×4 inverse.

2. **Kinematics chain DH transforms.** Forward kinematics multiplies successive DH transforms left-to-right: T₀₁ · T₁₂ · ⋯ · Tₙ₋₁ₙ. The Jacobian is computed geometrically: for revolute joint i, the angular velocity column is zᵢ and the linear velocity column is zᵢ × (pₑₑ − pᵢ).

3. **IK uses damped least squares.** At each iteration, compute the position error, build a finite-difference Jacobian, and solve Δq = Jᵀ(JJᵀ + λ²I)⁻¹e. Step size is clamped to 0.5 rad for stability. This is equivalent to Levenberg-Marquardt on the position residual.

4. **A* uses a binary heap** with 4- or 8-connectivity. RRT samples uniformly in bounds with 10% goal bias, steers toward samples by a fixed step size, and terminates when a node falls within tolerance of the goal. Potential fields use linear attractive force near the goal, constant-magnitude far away, and inverse-square repulsive force within an influence radius.

5. **PID includes anti-windup** by clamping the integral term, and derivative-on-error (not derivative-on-measurement). Output is clamped to configurable limits.

## The Math

**Rodrigues' Rotation Formula:** For rotation by angle θ about unit axis û, R = I cos θ + (1 − cos θ)ûûᵀ + sin θ [û]×, where [û]× is the skew-symmetric cross-product matrix.

**DH Transform:** The standard Denavit-Hartenberg convention gives Tᵢ = Rot_z(θ) · Trans_z(d) · Trans_x(a) · Rot_x(α), producing the 4×4 matrix shown in the code.

**Geometric Jacobian:** For revolute joint i with cumulative transform T₀ᵢ, the i-th column of J is [zᵢ; zᵢ × (pₑₑ − pᵢ)], where zᵢ is the third column of R₀ᵢ.

**Damped Least Squares (DLS):** Δq = Jᵀ(JJᵀ + λ²I)⁻¹e. The damping term λ prevents singularity blowup. As λ → 0, this reduces to the pseudoinverse.

**Yoshikawa Manipulability:** w = √det(JJᵀ). Higher w means the robot can move in more directions — it measures the volume of the velocity ellipsoid.

**Quaternion SLERP:** q(t) = q₁ sin((1−t)θ)/sin θ + q₂ sin(tθ)/sin θ, where θ = arccos(q₁ · q₂). Falls back to normalized linear interpolation when θ < 0.001 rad.

**Potential Fields:** F_att = k_att · (goal − pos) near goal, k_att · (goal − pos)/‖goal − pos‖ far away. F_rep = k_rep · (1/d − 1/d₀) · 1/d² · (pos − obs)/‖pos − obs‖ for d < d₀.

## Test Suite

73 integration tests covering:
- Rotation matrix construction and inverse (principal axes, arbitrary axis, Euler angles)
- Quaternion operations (conjugate, Hamilton product, vector rotation, matrix conversion round-trip, SLERP)
- Homogeneous transforms (composition, inverse, DH parameter round-trip)
- Forward kinematics (2-link arm, identity config, known positions)
- Inverse kinematics convergence
- Jacobian computation and manipulability
- A* grid planning (with and without obstacles, 4- and 8-connected)
- RRT planning
- Potential field planning
- PID controller step response and steady-state error
- Collision detection (AABB, sphere, mixed)
- Odometry and range sensor simulation
- SpatialAgent path following and collision

## License

MIT
