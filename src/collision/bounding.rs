use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

/// Axis-Aligned Bounding Box.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct AABB {
    pub min: Vector3<f64>,
    pub max: Vector3<f64>,
}

impl AABB {
    pub fn new(min: Vector3<f64>, max: Vector3<f64>) -> Self {
        Self { min, max }
    }

    pub fn from_center_half_extents(center: &Vector3<f64>, half: &Vector3<f64>) -> Self {
        Self {
            min: center - half,
            max: center + half,
        }
    }

    pub fn center(&self) -> Vector3<f64> {
        (self.min + self.max) * 0.5
    }

    pub fn half_extents(&self) -> Vector3<f64> {
        (self.max - self.min) * 0.5
    }

    pub fn intersects(&self, other: &AABB) -> bool {
        self.min.x <= other.max.x && self.max.x >= other.min.x &&
        self.min.y <= other.max.y && self.max.y >= other.min.y &&
        self.min.z <= other.max.z && self.max.z >= other.min.z
    }

    /// Test if a point is inside this AABB.
    pub fn contains_point(&self, p: &Vector3<f64>) -> bool {
        p.x >= self.min.x && p.x <= self.max.x &&
        p.y >= self.min.y && p.y <= self.max.y &&
        p.z >= self.min.z && p.z <= self.max.z
    }

    /// Compute the enclosing AABB of two AABBs.
    pub fn merge(&self, other: &AABB) -> AABB {
        AABB {
            min: self.min.inf(&other.min),
            max: self.max.sup(&other.max),
        }
    }

    pub fn volume(&self) -> f64 {
        let d = self.max - self.min;
        d.x * d.y * d.z
    }
}

/// Bounding sphere.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct SphereBound {
    pub center: Vector3<f64>,
    pub radius: f64,
}

impl SphereBound {
    pub fn new(center: Vector3<f64>, radius: f64) -> Self {
        Self { center, radius }
    }

    pub fn intersects(&self, other: &SphereBound) -> bool {
        let dist = (self.center - other.center).norm();
        dist < self.radius + other.radius
    }

    pub fn contains_point(&self, p: &Vector3<f64>) -> bool {
        (p - self.center).norm() <= self.radius
    }

    pub fn intersects_aabb(&self, aabb: &AABB) -> bool {
        CollisionDetector::sphere_aabb(self, aabb)
    }
}

/// General collision detector.
pub struct CollisionDetector;

impl CollisionDetector {
    pub fn sphere_sphere(a: &SphereBound, b: &SphereBound) -> bool {
        a.intersects(b)
    }

    pub fn aabb_aabb(a: &AABB, b: &AABB) -> bool {
        a.intersects(b)
    }

    pub fn sphere_aabb(sphere: &SphereBound, aabb: &AABB) -> bool {
        let closest = Vector3::new(
            sphere.center.x.clamp(aabb.min.x, aabb.max.x),
            sphere.center.y.clamp(aabb.min.y, aabb.max.y),
            sphere.center.z.clamp(aabb.min.z, aabb.max.z),
        );
        (sphere.center - closest).norm() <= sphere.radius
    }
}
