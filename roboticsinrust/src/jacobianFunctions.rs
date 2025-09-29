use nalgebra::{DMatrix, Matrix4, Matrix3, Vector3};

use crate::kinematicsFunctions::trans_max;

/// Computes the Jacobian matrix assuming revolute joints.
/// 
/// # Arguments
/// * `dh_table` - Denavit-Hartenberg table as a matrix (rows = joints+1, cols = 4)
///                Format per row: [a, alpha, d, theta]
/// * `trans_max` - Function to compute the cumulative transformation matrix
///                 from frame 0 up to frame `i` given the DH table.
/// 
/// # Returns
/// * `DMatrix<f64>` - 6 x num_joints Jacobian
pub fn compute_jacobian(
    dh_table: &DMatrix<f64>,
    trans_max: &dyn Fn(usize, usize, &DMatrix<f64>) -> Matrix4<f64>,
) -> DMatrix<f64> {
    // Number of joints (exclude last row = end effector)
    let num_joints = dh_table.nrows() - 1;
    let end_effector_dh_row = num_joints;

    // Initialize Jacobian (6 x num_joints)
    let mut j = DMatrix::<f64>::zeros(6, num_joints);

    // Compute end-effector transformation and position
    let t_end = trans_max(0, end_effector_dh_row, dh_table);
    let p_end = t_end.fixed_slice::<3, 1>(0, 3).into_owned();

    // Loop over joints
    for i in 0..num_joints {
        // Compute transformation up to joint i
        let t_i = trans_max(0, i, dh_table);

        // Extract joint position
        let p_i = t_i.fixed_slice::<3, 1>(0, 3).into_owned();

        // Extract z-axis of the i-th frame
        let z_i = t_i.fixed_slice::<3, 1>(0, 2).into_owned();

        // Linear velocity part: cross(Z_i, (P_end - P_i))
        let linear = z_i.cross(&(p_end.clone() - p_i));

        // Angular velocity part: Z_i
        let angular = z_i;

        // Fill Jacobian column
        for row in 0..3 {
            j[(row, i)] = linear[row];
            j[(row + 3, i)] = angular[row];
        }
    }

    j
}
