use serde::{Deserialize, Serialize};

/// DH parameters for a single link (standard convention).
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct DHLink {
    /// Joint angle / variable (theta for revolute).
    pub theta: f64,
    /// Link offset along z.
    pub d: f64,
    /// Link length along x.
    pub a: f64,
    /// Link twist about x.
    pub alpha: f64,
    /// Is this a revolute joint?
    pub revolute: bool,
}

impl DHLink {
    pub fn revolute(theta: f64, d: f64, a: f64, alpha: f64) -> Self {
        Self { theta, d, a, alpha, revolute: true }
    }

    pub fn prismatic(theta: f64, d: f64, a: f64, alpha: f64) -> Self {
        Self { theta, d, a, alpha, revolute: false }
    }

    /// Set the joint variable and return a new link.
    pub fn with_joint_value(&self, q: f64) -> Self {
        let mut link = *self;
        if self.revolute {
            link.theta = q;
        } else {
            link.d = q;
        }
        link
    }

    pub fn to_transform(&self) -> crate::transforms::HomogeneousTransform {
        crate::transforms::HomogeneousTransform::from_dh(self.theta, self.d, self.a, self.alpha)
    }
}
