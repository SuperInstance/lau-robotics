use nalgebra::{DVector, DMatrix};
use crate::transforms::HomogeneousTransform;
use super::forward::SerialChain;

/// Numerical inverse kinematics using damped least squares.
/// Position-only IK using the linear velocity Jacobian.
pub fn inverse_kinematics(
    chain: &SerialChain,
    target: &HomogeneousTransform,
    initial_guess: &[f64],
    max_iterations: usize,
    tolerance: f64,
    damping: f64,
) -> Option<Vec<f64>> {
    let n = chain.num_joints();
    assert_eq!(initial_guess.len(), n);

    let mut q = initial_guess.to_vec();
    let target_pos = target.translation();

    for _ in 0..max_iterations {
        let current = chain.fk(&q);
        let pos_error = target_pos - current.translation();
        let error = DVector::from_vec(pos_error.as_slice().to_vec());

        if error.norm() < tolerance {
            return Some(q);
        }

        // Use finite-difference Jacobian for robustness
        let eps = 1e-6;
        let mut jacobian = DMatrix::zeros(3, n);
        for j in 0..n {
            let mut q_plus = q.clone();
            let mut q_minus = q.clone();
            q_plus[j] += eps;
            q_minus[j] -= eps;
            let p_plus = chain.fk(&q_plus).translation();
            let p_minus = chain.fk(&q_minus).translation();
            let col = (p_plus - p_minus) / (2.0 * eps);
            for i in 0..3 {
                jacobian[(i, j)] = col[i];
            }
        }

        // Damped least squares: delta_q = J^T (J J^T + λ²I)^{-1} error
        let jjt = &jacobian * &jacobian.transpose();
        let damping_matrix = DMatrix::identity(3, 3) * (damping * damping);
        let to_invert = jjt + damping_matrix;
        let inverted = match to_invert.try_inverse() {
            Some(inv) => inv,
            None => return None,
        };
        let delta_q = jacobian.transpose() * inverted * error;

        // Limit step size
        let step_limit = 0.5;
        for i in 0..n {
            let step = if delta_q[i].abs() > step_limit {
                step_limit * delta_q[i].signum()
            } else {
                delta_q[i]
            };
            q[i] += step;
        }
    }

    // Check if we're close enough
    let current = chain.fk(&q);
    let pos_err = (target_pos - current.translation()).norm();
    if pos_err < tolerance * 10.0 {
        Some(q)
    } else {
        None
    }
}
