use nalgebra::Vector2;

/// Artificial potential field path planner.
pub struct PotentialField {
    /// Attractive gain.
    pub k_att: f64,
    /// Repulsive gain.
    pub k_rep: f64,
    /// Influence distance of obstacles.
    pub influence_distance: f64,
    /// Step size for gradient descent.
    pub step_size: f64,
    /// Maximum iterations.
    pub max_iterations: usize,
    /// Goal tolerance.
    pub tolerance: f64,
}

impl PotentialField {
    pub fn new(k_att: f64, k_rep: f64, influence_distance: f64) -> Self {
        Self {
            k_att,
            k_rep,
            influence_distance,
            step_size: 0.05,
            max_iterations: 5000,
            tolerance: 0.1,
        }
    }

    /// Attractive force toward goal.
    fn attractive_force(&self, pos: &Vector2<f64>, goal: &Vector2<f64>) -> Vector2<f64> {
        let diff = goal - pos;
        let dist = diff.norm();
        if dist < 1.0 {
            // Linear (conic) near goal
            self.k_att * diff
        } else {
            // Constant magnitude far from goal
            self.k_att * diff / dist
        }
    }

    /// Repulsive force from a single obstacle.
    fn repulsive_force(&self, pos: &Vector2<f64>, obstacle: &Vector2<f64>) -> Vector2<f64> {
        let diff = pos - obstacle;
        let dist = diff.norm();
        if dist < 1e-6 {
            // Push in random direction to escape
            return Vector2::new(self.k_rep, 0.0);
        }
        if dist < self.influence_distance {
            let magnitude = self.k_rep * (1.0 / dist - 1.0 / self.influence_distance) / (dist * dist);
            magnitude * diff / dist
        } else {
            Vector2::zeros()
        }
    }

    /// Plan a path from start to goal avoiding obstacles.
    /// Returns the path or None if it doesn't converge.
    pub fn plan(
        &self,
        start: &Vector2<f64>,
        goal: &Vector2<f64>,
        obstacles: &[Vector2<f64>],
    ) -> Option<Vec<Vector2<f64>>> {
        let mut path = vec![*start];
        let mut pos = *start;

        for _ in 0..self.max_iterations {
            let f_att = self.attractive_force(&pos, goal);
            let f_rep: Vector2<f64> = obstacles.iter()
                .map(|obs| self.repulsive_force(&pos, obs))
                .fold(Vector2::zeros(), |acc, f| acc + f);

            let total_force = f_att + f_rep;
            if total_force.norm() < 1e-8 {
                // Stuck in local minimum
                return None;
            }

            pos = pos + total_force.normalize() * self.step_size;
            path.push(pos);

            if (pos - goal).norm() < self.tolerance {
                return Some(path);
            }
        }

        None
    }
}
