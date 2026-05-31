use nalgebra::Vector2;
use serde::{Deserialize, Serialize};

/// Simple odometry model with noise.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OdometryModel {
    /// Current estimated position.
    pub position: Vector2<f64>,
    /// Current estimated heading (radians).
    pub heading: f64,
    /// Distance noise factor (fraction of distance traveled).
    pub distance_noise: f64,
    /// Heading noise factor (radians per radian turned).
    pub heading_noise: f64,
}

impl OdometryModel {
    pub fn new() -> Self {
        Self {
            position: Vector2::zeros(),
            heading: 0.0,
            distance_noise: 0.01,
            heading_noise: 0.01,
        }
    }

    pub fn with_position(position: Vector2<f64>, heading: f64) -> Self {
        Self {
            position,
            heading,
            distance_noise: 0.01,
            heading_noise: 0.01,
        }
    }

    /// Update odometry from a motion command.
    /// `distance`: distance traveled.
    /// `dheading`: change in heading.
    pub fn update(&mut self, distance: f64, dheading: f64) {
        // Add noise
        let noisy_distance = distance * (1.0 + rand::Rng::gen_range(&mut rand::thread_rng(), -1.0..1.0) * self.distance_noise);
        let noisy_dheading = dheading * (1.0 + rand::Rng::gen_range(&mut rand::thread_rng(), -1.0..1.0) * self.heading_noise);

        self.heading += noisy_dheading;
        let dx = noisy_distance * self.heading.cos();
        let dy = noisy_distance * self.heading.sin();
        self.position.x += dx;
        self.position.y += dy;
    }

    /// Update from wheel encoders (differential drive).
    /// `left_dist`: left wheel distance.
    /// `right_dist`: right wheel distance.
    /// `wheel_base`: distance between wheels.
    pub fn update_from_wheels(&mut self, left_dist: f64, right_dist: f64, wheel_base: f64) {
        let distance = (left_dist + right_dist) / 2.0;
        let dheading = (right_dist - left_dist) / wheel_base;
        self.update(distance, dheading);
    }
}
