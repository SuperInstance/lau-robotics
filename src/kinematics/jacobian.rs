use nalgebra::{DMatrix, Vector3};
use super::forward::SerialChain;
use crate::transforms::HomogeneousTransform;

/// Compute the geometric Jacobian (6×n) for a serial chain.
/// Top 3 rows: angular velocity; bottom 3 rows: linear velocity.
pub fn compute_jacobian(chain: &SerialChain, joint_values: &[f64]) -> DMatrix<f64> {
    let n = chain.num_joints();
    assert_eq!(joint_values.len(), n);

    let transforms = chain.fk_all(joint_values);
    let ee_pos = transforms.last().unwrap().translation();

    let mut jacobian = DMatrix::zeros(6, n);

    let mut t_cumulative = HomogeneousTransform::identity();

    for (i, link) in chain.links.iter().enumerate() {
        let link_with_q = link.with_joint_value(joint_values[i]);
        t_cumulative = t_cumulative.compose(&link_with_q.to_transform());

        let z_i = Vector3::new(
            t_cumulative.matrix[(0, 2)],
            t_cumulative.matrix[(1, 2)],
            t_cumulative.matrix[(2, 2)],
        );
        let p_i = t_cumulative.translation();

        if link.revolute {
            // Angular: z_i; Linear: z_i × (p_ee - p_i)
            let cross = z_i.cross(&(ee_pos - p_i));
            jacobian[(0, i)] = z_i.x;
            jacobian[(1, i)] = z_i.y;
            jacobian[(2, i)] = z_i.z;
            jacobian[(3, i)] = cross.x;
            jacobian[(4, i)] = cross.y;
            jacobian[(5, i)] = cross.z;
        } else {
            // Prismatic: angular = 0; linear = z_i
            jacobian[(0, i)] = 0.0;
            jacobian[(1, i)] = 0.0;
            jacobian[(2, i)] = 0.0;
            jacobian[(3, i)] = z_i.x;
            jacobian[(4, i)] = z_i.y;
            jacobian[(5, i)] = z_i.z;
        }
    }

    jacobian
}

/// Manipulability analysis for a robot.
pub struct Manipulability {
    pub manipulability_index: f64,
    pub condition_number: f64,
    pub singular_values: Vec<f64>,
}

impl Manipulability {
    pub fn compute(jacobian: &DMatrix<f64>) -> Self {
        let jjt = jacobian * jacobian.transpose();
        let svd = jjt.clone().svd(false, false);
        let singular_values: Vec<f64> = svd.singular_values.iter().map(|s| s.abs().sqrt()).collect();

        // Yoshikawa manipulability = sqrt(det(J J^T))
        let manipulability = if jjt.nrows() == jjt.ncols() {
            jjt.determinant().abs().sqrt()
        } else {
            0.0
        };

        let max_sv = singular_values.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let min_sv = singular_values.iter().cloned().fold(f64::INFINITY, f64::min);
        let condition_number = if min_sv > 1e-10 { max_sv / min_sv } else { f64::INFINITY };

        Self {
            manipulability_index: manipulability,
            condition_number,
            singular_values,
        }
    }
}
