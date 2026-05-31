#[cfg(test)]
mod tests {
    use nalgebra::{Vector2, Vector3, Matrix3};
    use std::f64::consts::PI;

    // ==================== TRANSFORMS ====================

    mod transform_tests {
        use super::*;

        #[test]
        fn rotation_identity_is_valid() {
            use lau_robotics::transforms::RotationMatrix;
            let r = RotationMatrix::identity();
            assert!(r.is_valid(1e-10));
        }

        #[test]
        fn rotation_x_90deg() {
            use lau_robotics::transforms::RotationMatrix;
            let r = RotationMatrix::from_axis_x(PI / 2.0);
            assert!(r.is_valid(1e-10));
            let v = Vector3::new(0.0, 1.0, 0.0);
            let rv = r.matrix * v;
            assert!((rv - Vector3::new(0.0, 0.0, 1.0)).norm() < 1e-10);
        }

        #[test]
        fn rotation_y_90deg() {
            use lau_robotics::transforms::RotationMatrix;
            let r = RotationMatrix::from_axis_y(PI / 2.0);
            assert!(r.is_valid(1e-10));
            let v = Vector3::new(1.0, 0.0, 0.0);
            let rv = r.matrix * v;
            assert!((rv - Vector3::new(0.0, 0.0, -1.0)).norm() < 1e-10);
        }

        #[test]
        fn rotation_z_90deg() {
            use lau_robotics::transforms::RotationMatrix;
            let r = RotationMatrix::from_axis_z(PI / 2.0);
            assert!(r.is_valid(1e-10));
            let v = Vector3::new(1.0, 0.0, 0.0);
            let rv = r.matrix * v;
            assert!((rv - Vector3::new(0.0, 1.0, 0.0)).norm() < 1e-10);
        }

        #[test]
        fn rotation_compose_is_valid() {
            use lau_robotics::transforms::RotationMatrix;
            let r1 = RotationMatrix::from_axis_x(0.5);
            let r2 = RotationMatrix::from_axis_y(0.7);
            let r3 = RotationMatrix::from_axis_z(1.2);
            let composed = r1.compose(&r2).compose(&r3);
            assert!(composed.is_valid(1e-10));
        }

        #[test]
        fn rotation_inverse_equals_transpose() {
            use lau_robotics::transforms::RotationMatrix;
            let r = RotationMatrix::from_axis_x(1.0).compose(&RotationMatrix::from_axis_y(0.5));
            let inv = r.inverse();
            let product = r.compose(&inv);
            assert!(product.is_valid(1e-10));
            // Should be identity
            assert!((product.matrix - Matrix3::identity()).norm() < 1e-10);
        }

        #[test]
        fn euler_zyx_roundtrip() {
            use lau_robotics::transforms::RotationMatrix;
            // Test several non-degenerate angle combinations
            for &(roll, pitch, yaw) in &[
                (0.3, 0.3, 0.7),
                (0.1, -0.2, 0.5),
                (-0.5, 0.1, -0.3),
            ] {
                let r = RotationMatrix::from_euler_zyx(roll, pitch, yaw);
                let (r2, p2, y2) = r.to_euler_zyx();
                // Verify roundtrip by comparing rotation matrices
                let r_rt = RotationMatrix::from_euler_zyx(r2, p2, y2);
                assert!((r.matrix - r_rt.matrix).norm() < 1e-6,
                    "Euler roundtrip failed: ({}, {}, {}) -> ({}, {}, {})",
                    roll, pitch, yaw, r2, p2, y2);
            }
        }

        #[test]
        fn axis_angle_rotation() {
            use lau_robotics::transforms::RotationMatrix;
            let axis = Vector3::new(0.0, 0.0, 1.0);
            let r = RotationMatrix::from_axis_angle(&axis, PI / 2.0);
            assert!(r.is_valid(1e-10));
        }

        #[test]
        fn homogeneous_identity() {
            use lau_robotics::transforms::HomogeneousTransform;
            let t = HomogeneousTransform::identity();
            let p = Vector3::new(1.0, 2.0, 3.0);
            let tp = t.transform_point(&p);
            assert!((tp - p).norm() < 1e-10);
        }

        #[test]
        fn homogeneous_translation() {
            use lau_robotics::transforms::HomogeneousTransform;
            let t = HomogeneousTransform::from_translation(&Vector3::new(1.0, 2.0, 3.0));
            let p = Vector3::new(0.0, 0.0, 0.0);
            let tp = t.transform_point(&p);
            assert!((tp - Vector3::new(1.0, 2.0, 3.0)).norm() < 1e-10);
        }

        #[test]
        fn homogeneous_compose() {
            use lau_robotics::transforms::{HomogeneousTransform, RotationMatrix};
            let r = RotationMatrix::from_axis_z(PI / 2.0);
            let t1 = HomogeneousTransform::from_rotation_translation(&r, &Vector3::new(1.0, 0.0, 0.0));
            let t2 = HomogeneousTransform::from_translation(&Vector3::new(0.0, 1.0, 0.0));
            let composed = t1.compose(&t2);
            let p = Vector3::new(0.0, 0.0, 0.0);
            let tp = composed.transform_point(&p);
            // t1: R_z(90) + [1,0,0]; t2: translate [0,1,0]
            // composed(t2 then t1): R_z(90)*[0,1,0] + [1,0,0] = [-1,0,0] + [1,0,0] = [0,0,0]
            assert!((tp - Vector3::new(0.0, 0.0, 0.0)).norm() < 1e-10);
        }

        #[test]
        fn homogeneous_inverse() {
            use lau_robotics::transforms::{HomogeneousTransform, RotationMatrix};
            let r = RotationMatrix::from_axis_z(0.7);
            let t = HomogeneousTransform::from_rotation_translation(&r, &Vector3::new(1.0, 2.0, 3.0));
            let inv = t.inverse();
            let product = t.compose(&inv);
            let ident = HomogeneousTransform::identity();
            assert!((product.matrix - ident.matrix).norm() < 1e-10);
        }

        #[test]
        fn dh_transform_zero() {
            use lau_robotics::transforms::HomogeneousTransform;
            let t = HomogeneousTransform::from_dh(0.0, 0.0, 0.0, 0.0);
            let ident = HomogeneousTransform::identity();
            assert!((t.matrix - ident.matrix).norm() < 1e-10);
        }
    }

    // ==================== QUATERNION ====================

    mod quaternion_tests {
        use super::*;
        use lau_robotics::transforms::Quaternion;

        #[test]
        fn quaternion_identity() {
            let q = Quaternion::identity();
            assert!((q.norm() - 1.0).abs() < 1e-10);
            let v = Vector3::new(1.0, 0.0, 0.0);
            let rv = q.rotate_vector(&v);
            assert!((rv - v).norm() < 1e-10);
        }

        #[test]
        fn quaternion_rotation_x() {
            let q = Quaternion::from_axis_angle(&Vector3::new(1.0, 0.0, 0.0), PI / 2.0);
            let v = Vector3::new(0.0, 1.0, 0.0);
            let rv = q.rotate_vector(&v);
            assert!((rv - Vector3::new(0.0, 0.0, 1.0)).norm() < 1e-10);
        }

        #[test]
        fn quaternion_rotation_y() {
            let q = Quaternion::from_axis_angle(&Vector3::new(0.0, 1.0, 0.0), PI / 2.0);
            let v = Vector3::new(1.0, 0.0, 0.0);
            let rv = q.rotate_vector(&v);
            assert!((rv - Vector3::new(0.0, 0.0, -1.0)).norm() < 1e-10);
        }

        #[test]
        fn quaternion_multiply_identity() {
            let q = Quaternion::from_axis_angle(&Vector3::new(1.0, 1.0, 0.0).normalize(), 1.5);
            let qi = Quaternion::identity();
            let result = q.multiply(&qi);
            assert!((result.w - q.w).abs() < 1e-10);
            assert!((result.x - q.x).abs() < 1e-10);
        }

        #[test]
        fn quaternion_to_rotation_matrix_roundtrip() {
            use lau_robotics::transforms::RotationMatrix;
            let q = Quaternion::from_axis_angle(&Vector3::new(1.0, 2.0, 3.0).normalize(), 2.0);
            let r = q.to_rotation_matrix();
            assert!(r.is_valid(1e-6));
            let q2 = Quaternion::from_rotation_matrix(&r);
            // Verify they produce the same rotation
            let v = Vector3::new(1.0, 1.0, 1.0);
            let v1 = q.rotate_vector(&v);
            let v2 = q2.rotate_vector(&v);
            assert!((v1 - v2).norm() < 1e-6);
        }

        #[test]
        fn quaternion_slerp_endpoints() {
            let q1 = Quaternion::from_axis_angle(&Vector3::new(0.0, 0.0, 1.0), 0.0);
            let q2 = Quaternion::from_axis_angle(&Vector3::new(0.0, 0.0, 1.0), PI / 4.0);
            let s0 = q1.slerp(&q2, 0.0);
            let s1 = q1.slerp(&q2, 1.0);
            // Check rotations match
            let v = Vector3::new(1.0, 0.0, 0.0);
            assert!((s0.rotate_vector(&v) - q1.rotate_vector(&v)).norm() < 1e-4);
            assert!((s1.rotate_vector(&v) - q2.rotate_vector(&v)).norm() < 1e-4);
        }

        #[test]
        fn quaternion_slerp_midpoint() {
            let q1 = Quaternion::from_axis_angle(&Vector3::new(0.0, 0.0, 1.0), 0.0);
            let q2 = Quaternion::from_axis_angle(&Vector3::new(0.0, 0.0, 1.0), PI);
            let mid = q1.slerp(&q2, 0.5);
            // Midpoint should be ~90 degrees
            let angle = 2.0 * mid.w.acos();
            assert!((angle - PI / 2.0).abs() < 1e-4);
        }
    }

    // ==================== KINEMATICS ====================

    mod kinematics_tests {
        use super::*;
        use lau_robotics::kinematics::{DHLink, SerialChain, forward_kinematics, inverse_kinematics, compute_jacobian, Manipulability};

        fn make_2dof_planar() -> Vec<DHLink> {
            vec![
                DHLink::revolute(0.0, 0.0, 1.0, 0.0),
                DHLink::revolute(0.0, 0.0, 1.0, 0.0),
            ]
        }

        fn make_3dof_planar() -> Vec<DHLink> {
            vec![
                DHLink::revolute(0.0, 0.0, 1.0, 0.0),
                DHLink::revolute(0.0, 0.0, 1.0, 0.0),
                DHLink::revolute(0.0, 0.0, 0.5, 0.0),
            ]
        }

        #[test]
        fn fk_zero_config_2dof() {
            let links = make_2dof_planar();
            let chain = SerialChain::new(links);
            let ee = chain.fk(&[0.0, 0.0]);
            let pos = ee.translation();
            assert!((pos.x - 2.0).abs() < 1e-10);
            assert!(pos.y.abs() < 1e-10);
            assert!(pos.z.abs() < 1e-10);
        }

        #[test]
        fn fk_90deg_first_joint() {
            let links = make_2dof_planar();
            let chain = SerialChain::new(links);
            let ee = chain.fk(&[PI / 2.0, 0.0]);
            let pos = ee.translation();
            assert!(pos.x.abs() < 1e-10);
            assert!((pos.y - 2.0).abs() < 1e-10);
        }

        #[test]
        fn fk_180deg_first_joint() {
            let links = make_2dof_planar();
            let chain = SerialChain::new(links);
            let ee = chain.fk(&[PI, 0.0]);
            let pos = ee.translation();
            assert!((pos.x - (-2.0)).abs() < 1e-10);
            assert!(pos.y.abs() < 1e-10);
        }

        #[test]
        fn fk_folded() {
            let links = make_2dof_planar();
            let chain = SerialChain::new(links);
            let ee = chain.fk(&[0.0, PI]);
            let pos = ee.translation();
            assert!(pos.x.abs() < 1e-10);
            assert!(pos.y.abs() < 1e-10);
        }

        #[test]
        fn fk_convenience_function() {
            let links = make_2dof_planar();
            let ee = forward_kinematics(&links, &[0.0, 0.0]);
            let pos = ee.translation();
            assert!((pos.x - 2.0).abs() < 1e-10);
        }

        #[test]
        fn fk_3dof_zero() {
            let links = make_3dof_planar();
            let chain = SerialChain::new(links);
            let ee = chain.fk(&[0.0, 0.0, 0.0]);
            let pos = ee.translation();
            assert!((pos.x - 2.5).abs() < 1e-10);
        }

        #[test]
        fn ik_2dof_reach() {
            let links = make_2dof_planar();
            let chain = SerialChain::new(links);
            use lau_robotics::transforms::{HomogeneousTransform, RotationMatrix};
            let target = HomogeneousTransform::from_rotation_translation(
                &RotationMatrix::identity(),
                &Vector3::new(1.5, 0.5, 0.0),
            );
            let result = inverse_kinematics(&chain, &target, &[0.3, 0.3], 500, 1e-3, 0.5);
            assert!(result.is_some());
            let q = result.unwrap();
            let ee = chain.fk(&q);
            let pos_err = (ee.translation() - target.translation()).norm();
            assert!(pos_err < 0.2, "Position error too large: {}", pos_err);
        }

        #[test]
        fn ik_2dof_fails_out_of_reach() {
            let links = make_2dof_planar();
            let chain = SerialChain::new(links);
            use lau_robotics::transforms::{HomogeneousTransform, RotationMatrix};
            let target = HomogeneousTransform::from_rotation_translation(
                &RotationMatrix::identity(),
                &Vector3::new(10.0, 10.0, 0.0),
            );
            let result = inverse_kinematics(&chain, &target, &[0.0, 0.0], 100, 1e-3, 0.01);
            assert!(result.is_none() || {
                if let Some(ref q) = result {
                    let ee = chain.fk(q);
                    (ee.translation() - target.translation()).norm() > 0.5
                } else { true }
            });
        }

        #[test]
        fn jacobian_dimensions() {
            let links = make_3dof_planar();
            let chain = SerialChain::new(links);
            let j = compute_jacobian(&chain, &[0.0, 0.0, 0.0]);
            assert_eq!(j.nrows(), 6);
            assert_eq!(j.ncols(), 3);
        }

        #[test]
        fn jacobian_zero_config_2dof() {
            let links = make_2dof_planar();
            let chain = SerialChain::new(links);
            let j = compute_jacobian(&chain, &[0.0, 0.0]);
            // At zero config (both links along x), linear velocity jacobian should be:
            // Joint 0: z × (p_ee - p_0) = [0,0,1] × [1,0,0] = [0,1,0]
            // Joint 1: z × (p_ee - p_1) = [0,0,1] × [0,0,0] = [0,0,0]
            assert!((j[(3, 0)] - 0.0).abs() < 1e-10);
            assert!((j[(4, 0)] - 1.0).abs() < 1e-10);
            assert!((j[(3, 1)] - 0.0).abs() < 1e-10);
            assert!((j[(4, 1)] - 0.0).abs() < 1e-10);
        }

        #[test]
        fn manipulability_positive() {
            let links = make_2dof_planar();
            let chain = SerialChain::new(links);
            let j = compute_jacobian(&chain, &[0.5, 0.5]);
            let m = Manipulability::compute(&j);
            // For a 2-DOF planar arm, JJT is 6×6 but rank 2 so det is 0
            // Check that singular values exist and the max is positive
            assert!(m.singular_values.iter().cloned().fold(0.0f64, f64::max) > 0.0);
        }

        #[test]
        fn fk_all_intermediate() {
            let links = make_2dof_planar();
            let chain = SerialChain::new(links);
            let transforms = chain.fk_all(&[0.0, 0.0]);
            assert_eq!(transforms.len(), 2);
            // First joint at x=1
            assert!((transforms[0].translation().x - 1.0).abs() < 1e-10);
            // End effector at x=2
            assert!((transforms[1].translation().x - 2.0).abs() < 1e-10);
        }

        #[test]
        fn dh_link_with_joint_value() {
            let link = DHLink::revolute(0.0, 0.5, 1.0, 0.0);
            let modified = link.with_joint_value(1.5);
            assert!((modified.theta - 1.5).abs() < 1e-10);
            assert!((modified.d - 0.5).abs() < 1e-10); // d unchanged for revolute
        }
    }

    // ==================== PATH PLANNING ====================

    mod path_planning_tests {
        use super::*;
        use lau_robotics::path_planning::astar::{astar_grid, GridCell, manhattan_distance, euclidean_distance};
        use lau_robotics::path_planning::rrt::RRT;

        #[test]
        fn astar_simple_path() {
            let start = GridCell::new(0, 0);
            let goal = GridCell::new(4, 4);
            let path = astar_grid(start, goal, |_| false, manhattan_distance, false);
            assert!(path.is_some());
            let p = path.unwrap();
            assert_eq!(*p.first().unwrap(), start);
            assert_eq!(*p.last().unwrap(), goal);
        }

        #[test]
        fn astar_with_obstacle() {
            let start = GridCell::new(0, 0);
            let goal = GridCell::new(4, 0);
            let occupied = |c: GridCell| c.y == 0 && c.x > 0 && c.x < 4;
            let path = astar_grid(start, goal, occupied, manhattan_distance, true);
            assert!(path.is_some());
            let p = path.unwrap();
            assert_eq!(*p.first().unwrap(), start);
            assert_eq!(*p.last().unwrap(), goal);
            // Should not go through occupied cells
            for c in &p {
                if c.y == 0 && c.x > 0 && c.x < 4 {
                    panic!("Path goes through obstacle at ({}, {})", c.x, c.y);
                }
            }
        }

        #[test]
        fn astar_no_path() {
            // Create a wall that completely separates start and goal
            let start = GridCell::new(0, 0);
            let goal = GridCell::new(4, 0);
            // Block the entire column x=2 and the goal
            let occupied = |c: GridCell| c.x == 2;
            let path = astar_grid(start, goal, occupied, manhattan_distance, false);
            // With only 4-connected movement and a full wall, no path exists
            // Actually with an infinite grid, A* could go around. Let's block the goal instead.
            let start = GridCell::new(0, 0);
            let goal = GridCell::new(1, 0);
            let occupied = |c: GridCell| c.x >= 1 && c.x <= 2 && c.y == 0;
            let path = astar_grid(start, goal, occupied, manhattan_distance, false);
            // Goal is blocked, so no path
            assert!(path.is_none());
        }

        #[test]
        fn astar_adjacent_start_goal() {
            let start = GridCell::new(0, 0);
            let goal = GridCell::new(1, 0);
            let path = astar_grid(start, goal, |_| false, manhattan_distance, false);
            assert!(path.is_some());
            let p = path.unwrap();
            assert_eq!(p.len(), 2);
        }

        #[test]
        fn astar_8_connected_shorter() {
            let start = GridCell::new(0, 0);
            let goal = GridCell::new(3, 3);
            let path4 = astar_grid(start, goal, |_| false, euclidean_distance, false).unwrap();
            let path8 = astar_grid(start, goal, |_| false, euclidean_distance, true).unwrap();
            // 8-connected should find a shorter (fewer nodes) path
            assert!(path8.len() <= path4.len());
        }

        #[test]
        fn rrt_basic() {
            let start = Vector2::new(0.0, 0.0);
            let goal = Vector2::new(5.0, 5.0);
            let mut rrt = RRT::new(
                start,
                0.5,
                5000,
                (Vector2::new(-1.0, -1.0), Vector2::new(6.0, 6.0)),
            );
            let path = rrt.plan(&goal, 0.5, |_| false);
            assert!(path.is_some());
            let p = path.unwrap();
            assert!((p.first().unwrap() - &start).norm() < 1e-10);
            assert!((p.last().unwrap() - &goal).norm() < 0.5);
        }

        #[test]
        fn rrt_with_obstacle() {
            let start = Vector2::new(0.0, 0.0);
            let goal = Vector2::new(5.0, 5.0);
            let mut rrt = RRT::new(
                start,
                0.3,
                10000,
                (Vector2::new(-1.0, -1.0), Vector2::new(6.0, 6.0)),
            );
            // Obstacle in the middle
            let collision = |p: &Vector2<f64>| {
                (p.x - 2.5).abs() < 0.5 && (p.y - 2.5).abs() < 0.5
            };
            let path = rrt.plan(&goal, 0.5, collision);
            assert!(path.is_some());
            let p = path.unwrap();
            // Verify no point is in the obstacle
            for pt in &p {
                assert!(!((pt.x - 2.5).abs() < 0.5 && (pt.y - 2.5).abs() < 0.5));
            }
        }

        #[test]
        fn potential_field_basic() {
            use lau_robotics::path_planning::potential_field::PotentialField;
            let pf = PotentialField::new(1.0, 1.0, 1.0);
            let start = Vector2::new(0.0, 0.0);
            let goal = Vector2::new(5.0, 0.0);
            let path = pf.plan(&start, &goal, &[]);
            assert!(path.is_some());
        }

        #[test]
        fn potential_field_with_obstacle() {
            use lau_robotics::path_planning::potential_field::PotentialField;
            let pf = PotentialField::new(1.5, 5.0, 1.5);
            let start = Vector2::new(0.0, 0.0);
            let goal = Vector2::new(5.0, 0.0);
            let obstacles = vec![Vector2::new(2.5, 0.3)];
            let path = pf.plan(&start, &goal, &obstacles);
            // May or may not find path due to local minima — just verify it runs
            // If it does find one, the path should avoid the obstacle
            if let Some(p) = &path {
                for pt in p.iter().take(p.len().saturating_sub(1)) {
                    let dist = (pt - obstacles[0]).norm();
                    assert!(dist > 0.1, "Path goes through obstacle: dist={}", dist);
                }
            }
        }
    }

    // ==================== CONTROL ====================

    mod control_tests {
        use super::*;
        use lau_robotics::control::PIDController;

        #[test]
        fn pid_convergence() {
            let mut pid = PIDController::new(1.0, 0.5, 0.1);
            pid.set_setpoint(10.0);
            let mut measurement = 0.0;
            let dt = 0.01;
            for _ in 0..5000 {
                let output = pid.update(measurement, dt);
                measurement += output * dt;
            }
            assert!((measurement - 10.0).abs() < 0.1, "PID didn't converge: {}", measurement);
        }

        #[test]
        fn pid_zero_error_zero_output() {
            let mut pid = PIDController::new(1.0, 0.0, 0.0);
            pid.set_setpoint(5.0);
            // At setpoint initially: no integral, no derivative
            let output = pid.update(5.0, 0.01);
            assert!(output.abs() < 1e-10);
        }

        #[test]
        fn pid_proportional_response() {
            let mut pid = PIDController::new(2.0, 0.0, 0.0);
            pid.set_setpoint(10.0);
            let output = pid.update(0.0, 0.01);
            assert!((output - 20.0).abs() < 1e-10); // kp * error = 2 * 10
        }

        #[test]
        fn pid_output_clamping() {
            let mut pid = PIDController::new(100.0, 0.0, 0.0).with_limits(-5.0, 5.0);
            pid.set_setpoint(100.0);
            let output = pid.update(0.0, 0.01);
            assert!((output - 5.0).abs() < 1e-10);
        }

        #[test]
        fn pid_integral_accumulation() {
            let mut pid = PIDController::new(0.0, 1.0, 0.0);
            pid.set_setpoint(1.0);
            let o1 = pid.update(0.0, 1.0); // integral = 1.0
            let o2 = pid.update(0.0, 1.0); // integral = 2.0
            assert!((o1 - 1.0).abs() < 1e-10);
            assert!((o2 - 2.0).abs() < 1e-10);
        }

        #[test]
        fn pid_reset() {
            let mut pid = PIDController::new(1.0, 1.0, 1.0);
            pid.set_setpoint(10.0);
            pid.update(0.0, 0.01);
            pid.reset();
            assert!(!pid.initialized);
            assert!(pid.integral.abs() < 1e-10);
        }

        #[test]
        fn pid_anti_windup() {
            let mut pid = PIDController::new(0.0, 1.0, 0.0).with_integral_limit(2.0);
            pid.set_setpoint(10.0);
            for _ in 0..100 {
                pid.update(0.0, 1.0);
            }
            assert!(pid.integral.abs() <= 2.1); // Should be clamped
        }
    }

    // ==================== COLLISION ====================

    mod collision_tests {
        use super::*;
        use lau_robotics::collision::{AABB, SphereBound, CollisionDetector};

        #[test]
        fn aabb_overlap() {
            let a = AABB::new(Vector3::new(0.0, 0.0, 0.0), Vector3::new(2.0, 2.0, 2.0));
            let b = AABB::new(Vector3::new(1.0, 1.0, 1.0), Vector3::new(3.0, 3.0, 3.0));
            assert!(a.intersects(&b));
        }

        #[test]
        fn aabb_no_overlap() {
            let a = AABB::new(Vector3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0));
            let b = AABB::new(Vector3::new(2.0, 2.0, 2.0), Vector3::new(3.0, 3.0, 3.0));
            assert!(!a.intersects(&b));
        }

        #[test]
        fn aabb_contains_point() {
            let aabb = AABB::new(Vector3::new(-1.0, -1.0, -1.0), Vector3::new(1.0, 1.0, 1.0));
            assert!(aabb.contains_point(&Vector3::new(0.0, 0.0, 0.0)));
            assert!(!aabb.contains_point(&Vector3::new(2.0, 0.0, 0.0)));
        }

        #[test]
        fn aabb_merge() {
            let a = AABB::new(Vector3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0));
            let b = AABB::new(Vector3::new(-1.0, -1.0, -1.0), Vector3::new(0.5, 0.5, 0.5));
            let merged = a.merge(&b);
            assert!((merged.min.x - (-1.0)).abs() < 1e-10);
            assert!((merged.max.x - 1.0).abs() < 1e-10);
        }

        #[test]
        fn aabb_volume() {
            let aabb = AABB::new(Vector3::new(0.0, 0.0, 0.0), Vector3::new(2.0, 3.0, 4.0));
            assert!((aabb.volume() - 24.0).abs() < 1e-10);
        }

        #[test]
        fn sphere_overlap() {
            let a = SphereBound::new(Vector3::new(0.0, 0.0, 0.0), 1.0);
            let b = SphereBound::new(Vector3::new(1.5, 0.0, 0.0), 1.0);
            assert!(a.intersects(&b));
        }

        #[test]
        fn sphere_no_overlap() {
            let a = SphereBound::new(Vector3::new(0.0, 0.0, 0.0), 1.0);
            let b = SphereBound::new(Vector3::new(3.0, 0.0, 0.0), 1.0);
            assert!(!a.intersects(&b));
        }

        #[test]
        fn sphere_contains_point() {
            let s = SphereBound::new(Vector3::new(0.0, 0.0, 0.0), 2.0);
            assert!(s.contains_point(&Vector3::new(1.0, 1.0, 0.0)));
            assert!(!s.contains_point(&Vector3::new(3.0, 0.0, 0.0)));
        }

        #[test]
        fn sphere_aabb_collision() {
            let sphere = SphereBound::new(Vector3::new(0.0, 0.0, 0.0), 1.0);
            let aabb = AABB::new(Vector3::new(0.5, 0.5, 0.5), Vector3::new(2.0, 2.0, 2.0));
            assert!(CollisionDetector::sphere_aabb(&sphere, &aabb));
        }

        #[test]
        fn sphere_aabb_no_collision() {
            let sphere = SphereBound::new(Vector3::new(0.0, 0.0, 0.0), 0.5);
            let aabb = AABB::new(Vector3::new(2.0, 2.0, 2.0), Vector3::new(3.0, 3.0, 3.0));
            assert!(!CollisionDetector::sphere_aabb(&sphere, &aabb));
        }

        #[test]
        fn aabb_center_and_half_extents() {
            let aabb = AABB::from_center_half_extents(
                &Vector3::new(1.0, 2.0, 3.0),
                &Vector3::new(0.5, 1.0, 1.5),
            );
            assert!((aabb.center() - Vector3::new(1.0, 2.0, 3.0)).norm() < 1e-10);
            assert!((aabb.half_extents() - Vector3::new(0.5, 1.0, 1.5)).norm() < 1e-10);
        }
    }

    // ==================== SENSOR ====================

    mod sensor_tests {
        use super::*;
        use lau_robotics::sensor::range::RangeSensor;
        use lau_robotics::sensor::odometry::OdometryModel;

        #[test]
        fn range_sensor_no_obstacle() {
            let sensor = RangeSensor::new(10.0, 0.0);
            let pos = Vector2::new(0.0, 0.0);
            let obstacles = vec![Vector2::new(20.0, 0.0)]; // Beyond max range
            let range = sensor.measure(&pos, 0.0, &obstacles);
            assert!((range - 10.0).abs() < 0.01); // Should return max range
        }

        #[test]
        fn range_sensor_detects_obstacle() {
            let sensor = RangeSensor::new(10.0, 0.0);
            let pos = Vector2::new(0.0, 0.0);
            let obstacles = vec![Vector2::new(5.0, 0.0)];
            let range = sensor.measure(&pos, 0.0, &obstacles);
            assert!((range - 5.0).abs() < 0.01);
        }

        #[test]
        fn range_sensor_behind_no_detect() {
            let sensor = RangeSensor::new(10.0, 0.0);
            let pos = Vector2::new(0.0, 0.0);
            let obstacles = vec![Vector2::new(-5.0, 0.0)]; // Behind sensor
            let range = sensor.measure(&pos, 0.0, &obstacles);
            assert!((range - 10.0).abs() < 0.01); // Should return max range
        }

        #[test]
        fn odometry_basic() {
            let mut odom = OdometryModel::new();
            odom.update(1.0, 0.0); // Move 1m forward
            assert!(odom.position.x > 0.9);
            assert!(odom.position.x < 1.1);
        }

        #[test]
        fn odometry_turn() {
            let mut odom = OdometryModel::new();
            odom.update(0.0, PI / 2.0); // Turn 90 degrees
            assert!((odom.heading - PI / 2.0).abs() < 0.1);
        }

        #[test]
        fn odometry_from_wheels() {
            let mut odom = OdometryModel::new();
            // Move straight with both wheels same distance
            odom.update_from_wheels(1.0, 1.0, 0.5);
            assert!(odom.position.x > 0.9);
            assert!(odom.heading.abs() < 0.1);
        }

        #[test]
        fn odometry_wheel_turn() {
            let mut odom = OdometryModel::new();
            // Turn: left wheel 0, right wheel = PI * wheel_base
            odom.update_from_wheels(0.0, PI * 0.5, 0.5);
            assert!((odom.heading - PI).abs() < 0.2);
        }
    }

    // ==================== AGENT ====================

    mod agent_tests {
        use super::*;
        use lau_robotics::agent::SpatialAgent;

        #[test]
        fn agent_creation() {
            let agent = SpatialAgent::new("test", Vector3::new(0.0, 0.0, 0.0), 0.0, 0.5);
            assert_eq!(agent.id, "test");
            assert_eq!(agent.pose_2d(), (0.0, 0.0, 0.0));
        }

        #[test]
        fn agent_follows_path() {
            let mut agent = SpatialAgent::new("test", Vector3::new(0.0, 0.0, 0.0), 0.0, 0.3);
            agent.max_speed = 2.0;
            agent.set_path(vec![
                Vector2::new(1.0, 0.0),
                Vector2::new(2.0, 0.0),
                Vector2::new(3.0, 0.0),
            ]);

            for _ in 0..200 {
                agent.step(0.05);
            }

            // Should have moved toward the goal
            assert!(agent.position.x > 2.0, "Agent didn't move enough: x={}", agent.position.x);
        }

        #[test]
        fn agent_collision_with_aabb() {
            let agent = SpatialAgent::new("test", Vector3::new(0.0, 0.0, 0.0), 0.0, 1.0);
            use lau_robotics::collision::AABB;
            let aabb = AABB::new(Vector3::new(0.5, 0.5, 0.0), Vector3::new(2.0, 2.0, 2.0));
            assert!(agent.collides_with_aabb(&aabb));
        }

        #[test]
        fn agent_no_collision_distant_aabb() {
            let agent = SpatialAgent::new("test", Vector3::new(0.0, 0.0, 0.0), 0.0, 0.1);
            use lau_robotics::collision::AABB;
            let aabb = AABB::new(Vector3::new(10.0, 10.0, 10.0), Vector3::new(11.0, 11.0, 11.0));
            assert!(!agent.collides_with_aabb(&aabb));
        }

        #[test]
        fn agent_collision_with_agent() {
            let a1 = SpatialAgent::new("a1", Vector3::new(0.0, 0.0, 0.0), 0.0, 1.0);
            let a2 = SpatialAgent::new("a2", Vector3::new(1.0, 0.0, 0.0), 0.0, 1.0);
            assert!(a1.collides_with_agent(&a2));
        }

        #[test]
        fn agent_transform() {
            let agent = SpatialAgent::new("test", Vector3::new(1.0, 2.0, 3.0), 0.0, 0.5);
            let t = agent.transform();
            assert!((t.translation() - Vector3::new(1.0, 2.0, 3.0)).norm() < 1e-10);
        }
    }
}
