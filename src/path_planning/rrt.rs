use nalgebra::Vector2;
use rand::Rng;
use serde::{Deserialize, Serialize};

/// A node in the RRT tree.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RRTNode {
    pub position: Vector2<f64>,
    pub parent: Option<usize>,
}

/// Rapidly-exploring Random Tree planner.
pub struct RRT {
    pub nodes: Vec<RRTNode>,
    pub step_size: f64,
    pub max_iterations: usize,
    pub bounds: (Vector2<f64>, Vector2<f64>), // min, max
}

impl RRT {
    pub fn new(start: Vector2<f64>, step_size: f64, max_iterations: usize, bounds: (Vector2<f64>, Vector2<f64>)) -> Self {
        Self {
            nodes: vec![RRTNode { position: start, parent: None }],
            step_size,
            max_iterations,
            bounds,
        }
    }

    /// Find the nearest node to a given point.
    pub fn nearest(&self, point: &Vector2<f64>) -> usize {
        let mut best_idx = 0;
        let mut best_dist = f64::MAX;
        for (i, node) in self.nodes.iter().enumerate() {
            let dist = (node.position - point).norm();
            if dist < best_dist {
                best_dist = dist;
                best_idx = i;
            }
        }
        best_idx
    }

    /// Steer from `from` towards `to` by at most `step_size`.
    pub fn steer(&self, from: &Vector2<f64>, to: &Vector2<f64>) -> Vector2<f64> {
        let diff = to - from;
        let dist = diff.norm();
        if dist <= self.step_size {
            *to
        } else {
            from + diff / dist * self.step_size
        }
    }

    /// Check if a point is within bounds.
    pub fn in_bounds(&self, p: &Vector2<f64>) -> bool {
        p.x >= self.bounds.0.x && p.x <= self.bounds.1.x &&
        p.y >= self.bounds.0.y && p.y <= self.bounds.1.y
    }

    /// Plan a path to the goal. Returns the path (start → goal) or None.
    /// `collision_fn` returns true if a configuration is in collision.
    pub fn plan<F: Fn(&Vector2<f64>) -> bool>(
        &mut self,
        goal: &Vector2<f64>,
        goal_tolerance: f64,
        collision_fn: F,
    ) -> Option<Vec<Vector2<f64>>> {
        let mut rng = rand::thread_rng();

        for _ in 0..self.max_iterations {
            // Random sample (with 10% goal bias)
            let sample = if rng.gen::<f64>() < 0.1 {
                *goal
            } else {
                Vector2::new(
                    rng.gen_range(self.bounds.0.x..=self.bounds.1.x),
                    rng.gen_range(self.bounds.0.y..=self.bounds.1.y),
                )
            };

            let nearest_idx = self.nearest(&sample);
            let nearest_pos = self.nodes[nearest_idx].position;
            let new_pos = self.steer(&nearest_pos, &sample);

            if !self.in_bounds(&new_pos) || collision_fn(&new_pos) {
                continue;
            }

            let new_idx = self.nodes.len();
            self.nodes.push(RRTNode {
                position: new_pos,
                parent: Some(nearest_idx),
            });

            if (new_pos - goal).norm() < goal_tolerance {
                // Extract path
                return Some(self.extract_path(new_idx));
            }
        }

        None
    }

    fn extract_path(&self, goal_idx: usize) -> Vec<Vector2<f64>> {
        let mut path = Vec::new();
        let mut current = Some(goal_idx);
        while let Some(idx) = current {
            path.push(self.nodes[idx].position);
            current = self.nodes[idx].parent;
        }
        path.reverse();
        path
    }
}
