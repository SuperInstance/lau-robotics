use nalgebra::Vector2;
use serde::{Deserialize, Serialize};

/// Simple range sensor model with Gaussian noise.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RangeSensor {
    /// Maximum range (meters).
    pub max_range: f64,
    /// Minimum range (meters).
    pub min_range: f64,
    /// Noise standard deviation.
    pub noise_stddev: f64,
    /// Beam angle (radians).
    pub beam_angle: f64,
}

impl RangeSensor {
    pub fn new(max_range: f64, noise_stddev: f64) -> Self {
        Self {
            max_range,
            min_range: 0.1,
            noise_stddev,
            beam_angle: std::f64::consts::FRAC_PI_8,
        }
    }

    /// Simulate a range measurement.
    /// `sensor_pos`: sensor position in world frame.
    /// `sensor_heading`: sensor heading in radians.
    /// `obstacles`: positions of point obstacles.
    /// Returns the measured range (with noise).
    pub fn measure(
        &self,
        sensor_pos: &Vector2<f64>,
        sensor_heading: f64,
        obstacles: &[Vector2<f64>],
    ) -> f64 {
        let mut closest = self.max_range;

        let _dir = Vector2::new(sensor_heading.cos(), sensor_heading.sin());

        for obs in obstacles {
            let diff = obs - sensor_pos;
            let dist = diff.norm();
            if dist < self.min_range || dist > self.max_range {
                continue;
            }

            // Check if obstacle is within beam cone
            let angle_to_obs = diff.y.atan2(diff.x);
            let angle_diff = (angle_to_obs - sensor_heading).abs();
            let angle_diff = if angle_diff > std::f64::consts::PI {
                2.0 * std::f64::consts::PI - angle_diff
            } else {
                angle_diff
            };

            if angle_diff < self.beam_angle / 2.0 && dist < closest {
                closest = dist;
            }
        }

        // Add noise
        let noise = rand::Rng::gen_range(&mut rand::thread_rng(), -3.0..3.0) * self.noise_stddev;
        (closest + noise).clamp(self.min_range, self.max_range)
    }
}
