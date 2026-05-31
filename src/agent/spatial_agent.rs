use nalgebra::{Vector2, Vector3};
use serde::{Deserialize, Serialize};

use crate::transforms::HomogeneousTransform;
use crate::control::PIDController;
use crate::collision::{AABB, SphereBound, CollisionDetector};
use crate::sensor::OdometryModel;

/// An agent with spatial awareness and movement capabilities.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SpatialAgent {
    /// Unique identifier.
    pub id: String,
    /// 3D position in world frame.
    pub position: Vector3<f64>,
    /// Heading angle (radians, about Z axis).
    pub heading: f64,
    /// Maximum linear speed.
    pub max_speed: f64,
    /// Maximum angular speed.
    pub max_angular_speed: f64,
    /// Collision bounding volume (sphere).
    pub bounding_sphere: SphereBound,
    /// Odometry model.
    pub odometry: OdometryModel,
    /// Linear velocity PID.
    pub velocity_pid: PIDController,
    /// Heading PID.
    pub heading_pid: PIDController,
    /// Path to follow.
    pub path: Option<Vec<Vector2<f64>>>,
    /// Current waypoint index.
    pub current_waypoint: usize,
    /// Waypoint tolerance.
    pub waypoint_tolerance: f64,
}

impl SpatialAgent {
    pub fn new(id: impl Into<String>, position: Vector3<f64>, heading: f64, radius: f64) -> Self {
        let bounding = SphereBound::new(position, radius);
        Self {
            id: id.into(),
            position,
            heading,
            max_speed: 1.0,
            max_angular_speed: std::f64::consts::PI,
            bounding_sphere: bounding,
            odometry: OdometryModel::with_position(
                Vector2::new(position.x, position.y),
                heading,
            ),
            velocity_pid: PIDController::new(1.0, 0.1, 0.05),
            heading_pid: PIDController::new(2.0, 0.0, 0.3),
            path: None,
            current_waypoint: 0,
            waypoint_tolerance: 0.2,
        }
    }

    /// Set a path for the agent to follow.
    pub fn set_path(&mut self, path: Vec<Vector2<f64>>) {
        self.path = Some(path);
        self.current_waypoint = 0;
    }

    /// Step the agent forward by dt seconds.
    /// Returns true if following path and not yet at goal.
    pub fn step(&mut self, dt: f64) -> bool {
        let waypoints = match &self.path {
            Some(p) => p,
            None => return false,
        };

        if self.current_waypoint >= waypoints.len() {
            return false;
        }

        let target = waypoints[self.current_waypoint];
        let dx = target.x - self.position.x;
        let dy = target.y - self.position.y;
        let distance = (dx * dx + dy * dy).sqrt();

        if distance < self.waypoint_tolerance {
            self.current_waypoint += 1;
            return self.current_waypoint < waypoints.len();
        }

        let desired_heading = dy.atan2(dx);
        let heading_error = desired_heading - self.heading;
        // Normalize to [-PI, PI]
        let heading_error = ((heading_error + std::f64::consts::PI) % (2.0 * std::f64::consts::PI))
            - std::f64::consts::PI;
        let heading_error = if heading_error < -std::f64::consts::PI {
            heading_error + 2.0 * std::f64::consts::PI
        } else {
            heading_error
        };

        // Heading control
        self.heading_pid.set_setpoint(desired_heading);
        let angular_vel = self.heading_pid.update(self.heading, dt)
            .clamp(-self.max_angular_speed, self.max_angular_speed);

        // Speed control - slow down when heading error is large
        let speed_factor = (heading_error.abs() / std::f64::consts::PI).cos().max(0.0);
        let linear_speed = (self.max_speed * speed_factor)
            .clamp(0.0, self.max_speed);

        // Update heading
        self.heading += angular_vel * dt;

        // Update position
        self.position.x += linear_speed * self.heading.cos() * dt;
        self.position.y += linear_speed * self.heading.sin() * dt;

        // Update bounding sphere
        self.bounding_sphere.center = self.position;

        // Update odometry
        self.odometry.update(linear_speed * dt, angular_vel * dt);

        true
    }

    /// Check collision with an AABB.
    pub fn collides_with_aabb(&self, aabb: &AABB) -> bool {
        CollisionDetector::sphere_aabb(&self.bounding_sphere, aabb)
    }

    /// Check collision with another agent.
    pub fn collides_with_agent(&self, other: &SpatialAgent) -> bool {
        CollisionDetector::sphere_sphere(&self.bounding_sphere, &other.bounding_sphere)
    }

    /// Get the 2D pose as (x, y, theta).
    pub fn pose_2d(&self) -> (f64, f64, f64) {
        (self.position.x, self.position.y, self.heading)
    }

    /// Get transform from world to agent frame.
    pub fn transform(&self) -> HomogeneousTransform {
        use crate::transforms::RotationMatrix;
        let rot = RotationMatrix::from_axis_z(self.heading);
        let trans = Vector3::new(self.position.x, self.position.y, self.position.z);
        HomogeneousTransform::from_rotation_translation(&rot, &trans)
    }
}
