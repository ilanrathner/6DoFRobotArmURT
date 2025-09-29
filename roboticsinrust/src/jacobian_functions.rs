use nalgebra::{DMatrix, Matrix4, Matrix3, DVector, Vector3};

use crate::kinematics_functions::{create_dh_table_6DOF, trans_max};

/// Computes the Jacobian matrix assuming revolute joints.
/// 
/// # Arguments
/// * `dh_table` - Denavit-Hartenberg table as a matrix (rows = joints+1, cols = 4)
///                Format per row: [a, alpha, d, theta] 
/// # Returns
/// * `DMatrix<f64>` - 6 x num_joints Jacobian
pub fn compute_jacobian(dh_table: &DMatrix<f64>) -> DMatrix<f64> {
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

//Computes the damped moore penrose pseudo inverse. Use a small labda value 
fn damped_moore_penrose_pseudo_inverse(j: &DMatrix<f64>, lambda: f64) -> DMatrix<f64> {
    let jt = j.transpose();
    let jtj = &jt * j;
    let n = jtj.nrows();

    // Add damping term: (JᵀJ + λ²I)
    let damped = jtj + DMatrix::identity(n, n) * lambda.powi(2);

    // Invert safely
    let inv = damped.try_inverse().expect("Matrix not invertible even with damping");

    inv * jt
}


/// Task-space velocity control for 6-DOF robot
///
/// # Arguments
/// * `v_ref` - 3x1 linear velocity reference
/// * `pos_ref` - 3x1 position reference
/// * `w_ref` - 3x1 angular velocity reference
/// * `rot_ref` - 3x3 desired rotation matrix
/// * `cur_angles` - 6x1 current joint angles
/// * `k_p` - proportional gain (scalar or 6x6 diagonal)
/// * `k_i` - integral gain (scalar or 6x6 diagonal)
/// * `error_integral` - 6x1 previous integral error
/// * `timestep` - time step for integration
/// * `link_lengths` - 5-element array of link lengths
///
/// # Returns
/// * `(joint_velocities, error_integral, current_position)`
///   - `joint_velocities` is 6x1 DVector
///   - `error_integral` is updated 6x1 DVector
///   - `current_position` is 3x1 Vector3<f64>
pub fn task_space_velocity_control(
    v_ref: &Vector3<f64>,
    pos_ref: &Vector3<f64>,
    w_ref: &Vector3<f64>,
    rot_ref: &Matrix3<f64>,
    cur_angles: &[f64; 6],
    k_p: f64,
    k_i: f64,
    k_d: f64,
    mut error_integral: DVector<f64>,
    prev_error: &DVector<f64>,  // for derivative term
    timestep: f64,
    link_lengths: &[f64; 5],
) -> (DVector<f64>, DVector<f64>, Vector3<f64>) {

    // Combine linear and angular velocity reference
    let mut v_w_ref = DVector::from_vec(vec![]);
    v_w_ref.extend(v_ref.iter());
    v_w_ref.extend(w_ref.iter());

    // Create DH table
    let dh_table = create_dh_table(link_lengths, cur_angles);

    // Compute current forward transform
    let forward_transform = trans_max(0, 6, &dh_table);

    // Extract current position
    let current_position = Vector3::from_column_slice(
        forward_transform.fixed_slice::<3, 1>(0, 3).as_slice()
    );

    // Extract current rotation axes (columns of rotation)
    let cur_rot_x = Vector3::from_column_slice(forward_transform.fixed_slice::<3, 1>(0, 0).as_slice());
    let cur_rot_y = Vector3::from_column_slice(forward_transform.fixed_slice::<3, 1>(0, 1).as_slice());
    let cur_rot_z = Vector3::from_column_slice(forward_transform.fixed_slice::<3, 1>(0, 2).as_slice());

    // Compute rotation error
    let x_rot_error = cur_rot_x.cross(&rot_ref.column(0));
    let y_rot_error = cur_rot_y.cross(&rot_ref.column(1));
    let z_rot_error = cur_rot_z.cross(&rot_ref.column(2));
    let rot_error = (x_rot_error + y_rot_error + z_rot_error) * 0.5;

    // Compute position error
    let pos_error = pos_ref - current_position;

    // Combine errors (6x1 vector)
    let mut error = DVector::from_vec(vec![]);
    error.extend(pos_error.iter());
    error.extend(rot_error.iter());

    // Update integral error
    error_integral += &error * timestep;

    // Compute derivative error
    let error_derivative = (&error - prev_error) / timestep;

    // Compute commanded task-space velocity (PID)
    let v_w_command = &v_w_ref
        + error.scale(k_p)
        + error_integral.scale(k_i)
        + error_derivative.scale(k_d);

    // Compute Jacobian (6x6)
    let j = compute_jacobian(&dh_table);

    // Compute pseudoinverse of Jacobian
    let j_inv = pseudo_inverse(&j, 1e-3); // small damping factor

    // Compute joint velocities
    let joint_velocities = j_inv * v_w_command;

    (joint_velocities, error_integral, current_position)
}