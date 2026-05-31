use nalgebra::{Matrix3, Vector3};
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

/// 3x3 rotation matrix representing SO(3).
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct RotationMatrix {
    pub matrix: Matrix3<f64>,
}

impl RotationMatrix {
    pub fn identity() -> Self {
        Self {
            matrix: Matrix3::identity(),
        }
    }

    pub fn from_matrix(m: Matrix3<f64>) -> Self {
        Self { matrix: m }
    }

    /// Rotation about X axis by angle (radians).
    pub fn from_axis_x(angle: f64) -> Self {
        let c = angle.cos();
        let s = angle.sin();
        Self {
            matrix: Matrix3::new(1.0, 0.0, 0.0, 0.0, c, -s, 0.0, s, c),
        }
    }

    /// Rotation about Y axis by angle (radians).
    pub fn from_axis_y(angle: f64) -> Self {
        let c = angle.cos();
        let s = angle.sin();
        Self {
            matrix: Matrix3::new(c, 0.0, s, 0.0, 1.0, 0.0, -s, 0.0, c),
        }
    }

    /// Rotation about Z axis by angle (radians).
    pub fn from_axis_z(angle: f64) -> Self {
        let c = angle.cos();
        let s = angle.sin();
        Self {
            matrix: Matrix3::new(c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0),
        }
    }

    /// Rotation about an arbitrary unit axis.
    pub fn from_axis_angle(axis: &Vector3<f64>, angle: f64) -> Self {
        let ux = axis.x;
        let uy = axis.y;
        let uz = axis.z;
        let c = angle.cos();
        let s = angle.sin();
        let t = 1.0 - c;
        let m = Matrix3::new(
            t * ux * ux + c,
            t * ux * uy - s * uz,
            t * ux * uz + s * uy,
            t * ux * uy + s * uz,
            t * uy * uy + c,
            t * uy * uz - s * ux,
            t * ux * uz - s * uy,
            t * uy * uz + s * ux,
            t * uz * uz + c,
        );
        Self { matrix: m }
    }

    /// Transpose (equals inverse for rotation matrices).
    pub fn transpose(&self) -> Self {
        Self {
            matrix: self.matrix.transpose(),
        }
    }

    pub fn inverse(&self) -> Self {
        self.transpose()
    }

    /// Compose two rotations: self followed by other.
    pub fn compose(&self, other: &RotationMatrix) -> Self {
        Self {
            matrix: self.matrix * other.matrix,
        }
    }

    /// Check if this is a valid rotation matrix (det ≈ 1, R^T * R ≈ I).
    pub fn is_valid(&self, tol: f64) -> bool {
        let should_be_identity = self.matrix.transpose() * self.matrix;
        let det = self.matrix.determinant();
        (det - 1.0).abs() < tol && (should_be_identity - Matrix3::identity()).norm() < tol
    }

    /// Extract Euler angles (roll, pitch, yaw) ZYX convention.
    pub fn to_euler_zyx(&self) -> (f64, f64, f64) {
        let pitch = (-self.matrix[(2, 0)]).asin();
        let (roll, yaw) = if pitch.cos().abs() > 1e-6 {
            let roll = self.matrix[(2, 1)].atan2(self.matrix[(2, 2)]);
            let yaw = self.matrix[(1, 0)].atan2(self.matrix[(0, 0)]);
            (roll, yaw)
        } else {
            (0.0, (-self.matrix[(0, 1)]).atan2(self.matrix[(1, 1)]))
        };
        (roll, pitch, yaw)
    }

    /// Create from Euler angles (roll, pitch, yaw) ZYX convention.
    pub fn from_euler_zyx(roll: f64, pitch: f64, yaw: f64) -> Self {
        let rx = Self::from_axis_x(roll);
        let ry = Self::from_axis_y(pitch);
        let rz = Self::from_axis_z(yaw);
        rz.compose(&ry).compose(&rx)
    }
}
