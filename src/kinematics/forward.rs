use nalgebra::Vector3;

use crate::transforms::HomogeneousTransform;

use super::DHLink;

/// A serial kinematic chain of DH links.
#[derive(Debug, Clone)]
pub struct SerialChain {
    pub links: Vec<DHLink>,
}

impl SerialChain {
    pub fn new(links: Vec<DHLink>) -> Self {
        Self { links }
    }

    pub fn num_joints(&self) -> usize {
        self.links.len()
    }

    /// Compute forward kinematics for given joint values.
    /// Returns the end-effector transform.
    pub fn fk(&self, joint_values: &[f64]) -> HomogeneousTransform {
        assert_eq!(joint_values.len(), self.links.len(), "Joint values must match number of links");
        let mut transform = HomogeneousTransform::identity();
        for (link, &q) in self.links.iter().zip(joint_values.iter()) {
            let link_with_q = link.with_joint_value(q);
            transform = transform.compose(&link_with_q.to_transform());
        }
        transform
    }

    /// Get all intermediate transforms (one per joint).
    pub fn fk_all(&self, joint_values: &[f64]) -> Vec<HomogeneousTransform> {
        assert_eq!(joint_values.len(), self.links.len());
        let mut transforms = Vec::with_capacity(self.links.len());
        let mut transform = HomogeneousTransform::identity();
        for (link, &q) in self.links.iter().zip(joint_values.iter()) {
            let link_with_q = link.with_joint_value(q);
            transform = transform.compose(&link_with_q.to_transform());
            transforms.push(transform);
        }
        transforms
    }
}

/// Convenience function: compute FK for a chain of DH links.
pub fn forward_kinematics(links: &[DHLink], joint_values: &[f64]) -> HomogeneousTransform {
    let chain = SerialChain::new(links.to_vec());
    chain.fk(joint_values)
}
