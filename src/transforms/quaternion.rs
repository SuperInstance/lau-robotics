use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

/// Unit quaternion for 3D rotation.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Quaternion {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Quaternion {
    pub fn identity() -> Self {
        Self { w: 1.0, x: 0.0, y: 0.0, z: 0.0 }
    }

    pub fn new(w: f64, x: f64, y: f64, z: f64) -> Self {
        let norm = (w * w + x * x + y * y + z * z).sqrt();
        Self { w: w / norm, x: x / norm, y: y / norm, z: z / norm }
    }

    /// From axis-angle representation.
    pub fn from_axis_angle(axis: &Vector3<f64>, angle: f64) -> Self {
        let half = angle / 2.0;
        let s = half.sin();
        let n = axis.norm();
        if n < 1e-10 {
            return Self::identity();
        }
        let a = axis / n;
        Self {
            w: half.cos(),
            x: a.x * s,
            y: a.y * s,
            z: a.z * s,
        }
    }

    pub fn norm(&self) -> f64 {
        (self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    pub fn conjugate(&self) -> Self {
        Self { w: self.w, x: -self.x, y: -self.y, z: -self.z }
    }

    /// Hamilton product.
    pub fn multiply(&self, other: &Quaternion) -> Self {
        Self {
            w: self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
            x: self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
            y: self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
            z: self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w,
        }
    }

    /// Rotate a 3D vector.
    pub fn rotate_vector(&self, v: &Vector3<f64>) -> Vector3<f64> {
        let qv = Quaternion { w: 0.0, x: v.x, y: v.y, z: v.z };
        let result = self.multiply(&qv).multiply(&self.conjugate());
        Vector3::new(result.x, result.y, result.z)
    }

    /// Convert to rotation matrix.
    pub fn to_rotation_matrix(&self) -> crate::transforms::RotationMatrix {
        use crate::transforms::RotationMatrix;
        use nalgebra::Matrix3;
        let (w, x, y, z) = (self.w, self.x, self.y, self.z);
        let m = Matrix3::new(
            1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y),
            2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x),
            2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y),
        );
        RotationMatrix::from_matrix(m)
    }

    /// From rotation matrix (Shepperd's method).
    pub fn from_rotation_matrix(r: &crate::transforms::RotationMatrix) -> Self {
        let m = r.matrix;
        let trace = m[(0, 0)] + m[(1, 1)] + m[(2, 2)];
        if trace > 0.0 {
            let s = (trace + 1.0).sqrt() * 2.0; // s = 4*w
            let w = 0.25 * s;
            let x = (m[(2, 1)] - m[(1, 2)]) / s;
            let y = (m[(0, 2)] - m[(2, 0)]) / s;
            let z = (m[(1, 0)] - m[(0, 1)]) / s;
            Self::new(w, x, y, z)
        } else if m[(0, 0)] > m[(1, 1)] && m[(0, 0)] > m[(2, 2)] {
            let s = (1.0 + m[(0, 0)] - m[(1, 1)] - m[(2, 2)]).sqrt() * 2.0; // s = 4*x
            let w = (m[(2, 1)] - m[(1, 2)]) / s;
            let x = 0.25 * s;
            let y = (m[(0, 1)] + m[(1, 0)]) / s;
            let z = (m[(0, 2)] + m[(2, 0)]) / s;
            Self::new(w, x, y, z)
        } else if m[(1, 1)] > m[(2, 2)] {
            let s = (1.0 + m[(1, 1)] - m[(0, 0)] - m[(2, 2)]).sqrt() * 2.0; // s = 4*y
            let w = (m[(0, 2)] - m[(2, 0)]) / s;
            let x = (m[(0, 1)] + m[(1, 0)]) / s;
            let y = 0.25 * s;
            let z = (m[(1, 2)] + m[(2, 1)]) / s;
            Self::new(w, x, y, z)
        } else {
            let s = (1.0 + m[(2, 2)] - m[(0, 0)] - m[(1, 1)]).sqrt() * 2.0; // s = 4*z
            let w = (m[(1, 0)] - m[(0, 1)]) / s;
            let x = (m[(0, 2)] + m[(2, 0)]) / s;
            let y = (m[(1, 2)] + m[(2, 1)]) / s;
            let z = 0.25 * s;
            Self::new(w, x, y, z)
        }
    }

    /// Spherical linear interpolation.
    pub fn slerp(&self, other: &Quaternion, t: f64) -> Self {
        let dot = self.w * other.w + self.x * other.x + self.y * other.y + self.z * other.z;
        let other = if dot < 0.0 {
            Quaternion { w: -other.w, x: -other.x, y: -other.y, z: -other.z }
        } else {
            *other
        };
        let dot = dot.abs();

        if dot > 0.9995 {
            // Linear interpolation for very close quaternions
            let result = Quaternion {
                w: self.w + t * (other.w - self.w),
                x: self.x + t * (other.x - self.x),
                y: self.y + t * (other.y - self.y),
                z: self.z + t * (other.z - self.z),
            };
            let n = result.norm();
            return Quaternion { w: result.w / n, x: result.x / n, y: result.y / n, z: result.z / n };
        }

        let theta_0 = dot.acos();
        let theta = theta_0 * t;
        let sin_theta = theta.sin();
        let sin_theta_0 = theta_0.sin();

        if sin_theta_0.abs() < 1e-10 {
            // Nearly identical quaternions
            return *self;
        }

        let s0 = ((1.0 - t) * theta_0).sin() / sin_theta_0;
        let s1 = sin_theta / sin_theta_0;

        let raw = Quaternion {
            w: s0 * self.w + s1 * other.w,
            x: s0 * self.x + s1 * other.x,
            y: s0 * self.y + s1 * other.y,
            z: s0 * self.z + s1 * other.z,
        };
        // Normalize
        let n = raw.norm();
        Quaternion { w: raw.w / n, x: raw.x / n, y: raw.y / n, z: raw.z / n }
    }
}
