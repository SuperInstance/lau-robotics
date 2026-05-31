use nalgebra::{Matrix4, Vector3};
use serde::{Deserialize, Serialize};

use super::{Quaternion, RotationMatrix};

/// 4x4 homogeneous transformation matrix representing SE(3).
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct HomogeneousTransform {
    pub matrix: Matrix4<f64>,
}

impl HomogeneousTransform {
    pub fn identity() -> Self {
        Self {
            matrix: Matrix4::identity(),
        }
    }

    pub fn from_rotation_translation(rotation: &RotationMatrix, translation: &Vector3<f64>) -> Self {
        let mut m = Matrix4::identity();
        m.fixed_view_mut::<3, 3>(0, 0).copy_from(&rotation.matrix);
        m[(0, 3)] = translation.x;
        m[(1, 3)] = translation.y;
        m[(2, 3)] = translation.z;
        Self { matrix: m }
    }

    pub fn from_translation(translation: &Vector3<f64>) -> Self {
        let mut m = Matrix4::identity();
        m[(0, 3)] = translation.x;
        m[(1, 3)] = translation.y;
        m[(2, 3)] = translation.z;
        Self { matrix: m }
    }

    pub fn from_rotation(rotation: &RotationMatrix) -> Self {
        Self::from_rotation_translation(rotation, &Vector3::zeros())
    }

    /// DH transform from DH parameters (standard convention).
    pub fn from_dh(theta: f64, d: f64, a: f64, alpha: f64) -> Self {
        let ct = theta.cos();
        let st = theta.sin();
        let ca = alpha.cos();
        let sa = alpha.sin();
        let m = Matrix4::new(
            ct, -st * ca, st * sa, a * ct,
            st, ct * ca, -ct * sa, a * st,
            0.0, sa, ca, d,
            0.0, 0.0, 0.0, 1.0,
        );
        Self { matrix: m }
    }

    pub fn rotation(&self) -> RotationMatrix {
        let r = self.matrix.fixed_view::<3, 3>(0, 0).into_owned();
        RotationMatrix::from_matrix(r)
    }

    pub fn translation(&self) -> Vector3<f64> {
        Vector3::new(self.matrix[(0, 3)], self.matrix[(1, 3)], self.matrix[(2, 3)])
    }

    /// Compose: self then other.
    pub fn compose(&self, other: &HomogeneousTransform) -> Self {
        Self {
            matrix: self.matrix * other.matrix,
        }
    }

    pub fn inverse(&self) -> Self {
        let r = self.rotation();
        let t = self.translation();
        let rt = r.transpose();
        let inv_t = rt.matrix * -t;
        Self::from_rotation_translation(&rt, &inv_t)
    }

    /// Transform a 3D point.
    pub fn transform_point(&self, p: &Vector3<f64>) -> Vector3<f64> {
        let r = self.rotation();
        r.matrix * p + self.translation()
    }
}

/// Convenience alias.
pub type Transform3 = HomogeneousTransform;
