use dh_arm_model::dh_arm_model::DHArmModel;
use dh_arm_model::inverse_kinematics_solvers::{UrtIkSolver};
use dh_arm_model::setups::{setup_default_urt, urt_arm, NUM_JOINTS, NUM_FRAMES, NUM_LINKS};
use nalgebra::{SMatrix, SVector, Vector3};


fn numerical_jacobian(
    arm: &mut DHArmModel<NUM_FRAMES, NUM_JOINTS, NUM_LINKS, UrtIkSolver>,
    joint_positions_degrees: [f64; NUM_JOINTS],
) -> SMatrix<f64, 6, NUM_JOINTS> {
    // set_joint_positions accepts degrees, while a Jacobian maps radians to task-space motion.
    let step_radians: f64 = 1.0e-6;
    let step_degrees = step_radians.to_degrees();
    let mut result = SMatrix::<f64, 6, NUM_JOINTS>::zeros();

    for joint_index in 0..NUM_JOINTS {
        let mut plus = joint_positions_degrees;
        let mut minus = joint_positions_degrees;
        plus[joint_index] += step_degrees;
        minus[joint_index] -= step_degrees;

        arm.set_joint_positions(&plus);
        let plus_pose = arm.frame_pose(NUM_FRAMES);
        arm.set_joint_positions(&minus);
        let minus_pose = arm.frame_pose(NUM_FRAMES);

        let linear = (plus_pose.position - minus_pose.position) / (2.0 * step_radians);

        // R(q + h) R(q - h)^T = I + 2h [omega]x + O(h^2).
        // Extracting the skew-symmetric part gives angular velocity in the world frame,
        // which is the convention used by compute_jacobian.
        let rotation_delta = plus_pose.rotation * minus_pose.rotation.transpose();
        let angular = Vector3::new(
            rotation_delta[(2, 1)] - rotation_delta[(1, 2)],
            rotation_delta[(0, 2)] - rotation_delta[(2, 0)],
            rotation_delta[(1, 0)] - rotation_delta[(0, 1)],
        ) / (4.0 * step_radians);

        result
            .column_mut(joint_index)
            .copy_from(&SVector::<f64, 6>::new(
                linear.x, linear.y, linear.z, angular.x, angular.y, angular.z,
            ));
    }

    arm.set_joint_positions(&joint_positions_degrees);
    result
}

#[test]
fn geometric_jacobian_matches_forward_kinematics_finite_difference() {
    let configurations = [
        [0.0; NUM_JOINTS],
        [15.0, -25.0, 45.0, 30.0, 35.0, -20.0],
        [-40.0, 10.0, 70.0, -35.0, 25.0, 55.0],
    ];
    let mut arm = setup_default_urt();

    for joint_positions in configurations {
        arm.set_joint_positions(&joint_positions);
        let analytic = *arm.jacobian();
        let numerical = numerical_jacobian(&mut arm, joint_positions);
        let error = (analytic - numerical).norm();

        assert!(
            error <= 1.0e-6,
            "Jacobian finite-difference error {error} exceeded tolerance at {joint_positions:?}"
        );
    }
}

#[test]
fn damped_pseudo_inverse_satisfies_regularized_right_inverse_equation() {
    let damping: f64 = 1.0e-3;
    let mut arm = urt_arm(Some(damping), UrtIkSolver);
    let configurations = [
        [0.0; NUM_JOINTS],
        [15.0, -25.0, 45.0, 30.0, 35.0, -20.0],
        [-40.0, 10.0, 70.0, -35.0, 25.0, 55.0],
    ];

    for joint_positions in configurations {
        arm.set_joint_positions(&joint_positions);
        let jacobian = *arm.jacobian();
        let pseudo_inverse = *arm.inv_jacobian();
        let regularized =
            jacobian * jacobian.transpose() + SMatrix::<f64, 6, 6>::identity() * damping.powi(2);

        // For J^+_lambda = J^T (J J^T + lambda^2 I)^-1, multiplying through
        // by the regularized factor gives this identity. A damped pseudo-inverse
        // is not expected to make J J^+ exactly equal to the identity matrix.
        let residual = pseudo_inverse * regularized - jacobian.transpose();

        assert!(
            residual.norm() <= 1.0e-8,
            "regularized pseudo-inverse residual {} exceeded tolerance at {:?}",
            residual.norm(),
            joint_positions
        );
        assert!(
            pseudo_inverse.iter().all(|value| value.is_finite()),
            "pseudo-inverse contained a non-finite value at {joint_positions:?}"
        );
    }
}
