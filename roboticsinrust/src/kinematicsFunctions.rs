// kinematics.rs (or shared module)
// Uses nalgebra for matrices/vectors
use nalgebra::{Matrix3, Matrix4, Vector3};
use std::f64::consts::PI;

/// Translation matrix given x, y, z
fn translate(x: f64, y: f64, z: f64) -> Matrix4<f64> {
    // Matrix4::new takes entries in row-major order: m11, m12, m13, m14, m21, ...
    Matrix4::new(
        1.0, 0.0, 0.0, x,
        0.0, 1.0, 0.0, y,
        0.0, 0.0, 1.0, z,
        0.0, 0.0, 0.0, 1.0,
    )
}

/// Rotation matrix about principal axis (x, y, or z)
fn rot_max(axis: char, angle: f64) -> Matrix4<f64> {
    match axis {
        'x' => Matrix4::new(
            1.0, 0.0,          0.0,         0.0,
            0.0, angle.cos(), -angle.sin(), 0.0,
            0.0, angle.sin(),  angle.cos(), 0.0,
            0.0, 0.0,          0.0,         1.0,
        ),
        'y' => Matrix4::new(
             angle.cos(), 0.0, angle.sin(), 0.0,
             0.0,         1.0, 0.0,         0.0,
            -angle.sin(), 0.0, angle.cos(), 0.0,
             0.0,         0.0, 0.0,         1.0,
        ),
        'z' => Matrix4::new(
            angle.cos(), -angle.sin(), 0.0, 0.0,
            angle.sin(),  angle.cos(), 0.0, 0.0,
            0.0,          0.0,         1.0, 0.0,
            0.0,          0.0,         0.0, 1.0,
        ),
        _ => panic!("Invalid axis, must be 'x', 'y', or 'z'"),
    }
}

/// Transformation for one DH row: [a, alpha, d, theta]
fn transform(dh_row: [f64; 4]) -> Matrix4<f64> {
    let a = dh_row[0];
    let alpha = dh_row[1];
    let d = dh_row[2];
    let theta = dh_row[3];

    // DH convention: Tx(a) * Rx(alpha) * Tz(d) * Rz(theta)
    translate(a, 0.0, 0.0) *
        rot_max('x', alpha) *
        translate(0.0, 0.0, d) *
        rot_max('z', theta)
}

/// Final transformation matrix from frame j to i (j <= i).
/// dh is a slice of DH rows (each [a, alpha, d, theta]) where the last row
/// is typically the end-effector row. This multiplies transforms from j..i-1
/// in order: T = T_j * T_{j+1} * ... * T_{i-1}
pub fn trans_max(j: usize, i: usize, dh: &[[f64; 4]]) -> Matrix4<f64> {
    let r = dh.len();
    if dh.is_empty() {
        panic!("DH table is empty");
    }
    // allowed i == r (meaning multiply up to last row)
    if i > r || j > r {
        panic!("Index out of bounds for DH table: j={}, i={}, rows={}", j, i, r);
    }
    if j > i {
        panic!("Require j <= i, got j={} > i={}", j, i);
    }

    let mut transformation_matrix = Matrix4::<f64>::identity();

    // multiplier order: multiply transform(dh[f]) for f in j..i (i excluded)
    for f in j..i {
        transformation_matrix = transformation_matrix * transform(dh[f]);
    }

    transformation_matrix
}

/// Create the DH table from link lengths and joint angles.
/// Note: link_lengths length = 5, thetas length = 6 (example taken from user's original code)
fn create_dh_table_6DOF(link_lengths: &[f64; 5], thetas: &[f64; 6]) -> Vec<[f64; 4]> {
    let theta1 = thetas[0];
    let theta2 = thetas[1];
    let theta3 = thetas[2];
    let theta4 = thetas[3];
    let theta5 = thetas[4];
    let theta6 = thetas[5];

    let l1 = link_lengths[0];
    let l2 = link_lengths[1];
    let l3 = link_lengths[2];
    let l4 = link_lengths[3];
    let l5 = link_lengths[4];

    let pi2 = PI / 2.0;

    vec![
        [0.0,   0.0,   l1,        theta1],
        [0.0,  -pi2,   0.0,       theta2 - pi2],
        [l2,    0.0,   0.0,       theta3 + pi2],
        [0.0,   pi2,   l3,        theta4],
        [0.0,  -pi2,   0.0,       theta5],
        [0.0,   pi2,   l4,        theta6],
        [0.0,   0.0,   l5,        0.0],
    ]
}

/// Compute orientation matrix from yaw (Z), pitch (Y), roll (X)
fn orientation_mat(yaw: f64, pitch: f64, roll: f64) -> Matrix3<f64> {
    // rotation order Z * Y * X (yaw, pitch, roll)
    let x_rot = Matrix3::new(
        1.0, 0.0, 0.0,
        0.0, roll.cos(), -roll.sin(),
        0.0, roll.sin(),  roll.cos(),
    );

    let y_rot = Matrix3::new(
        pitch.cos(), 0.0, pitch.sin(),
        0.0,         1.0, 0.0,
       -pitch.sin(), 0.0, pitch.cos(),
    );

    let z_rot = Matrix3::new(
        yaw.cos(), -yaw.sin(), 0.0,
        yaw.sin(),  yaw.cos(), 0.0,
        0.0,        0.0,       1.0,
    );

    z_rot * y_rot * x_rot
}

/// Solve inverse kinematics for a 6-DOF arm. Returns joint angles [theta1..theta6].
/// NOTE: this uses the standard planar 2R solution for theta2/theta3 (see comments).
fn inverse_kinematics(
    x: f64, y: f64, z: f64,
    yaw: f64, pitch: f64, roll: f64,
    link_lengths: &[f64; 5],
) -> Result<[f64; 6], String> {
    let l1 = link_lengths[0];
    let l2 = link_lengths[1];
    let l3 = link_lengths[2];
    let l4 = link_lengths[3];
    let l5 = link_lengths[4];

    // Step 1: orientation matrix for end-effector (rotation from base)
    let r = orientation_mat(yaw, pitch, roll);

    // Step 2: wrist center (subtract distance along effector Z)
    let d = l4 + l5;
    let wx = x - d * r[(0, 2)];
    let wy = y - d * r[(1, 2)];
    let wz = z - d * r[(2, 2)];

    // Step 3: theta1
    let theta1 = wy.atan2(wx); // atan2(y, x)

    // Step 4: planar distances for first 3 joints
    let r_val = (wx.powi(2) + wy.powi(2)).sqrt();
    let s = wz - l1;

    // Step 5: theta3 (using law of cosines)
    let numerator = r_val.powi(2) + s.powi(2) - l2.powi(2) - l3.powi(2);
    let denom = 2.0 * l2 * l3;
    let cos_theta3 = numerator / denom;
    if cos_theta3.abs() > 1.0 {
        return Err("Target out of workspace: theta3 complex".into());
    }
    // two possible solutions for theta3: + and - (elbow up / elbow down)
    let sin_theta3 = (1.0 - cos_theta3 * cos_theta3).sqrt(); // positive branch; you can choose sign
    let theta3 = sin_theta3.atan2(cos_theta3); // atan2(sin, cos) gives correct quadrant

    // Step 6: theta2 (standard 2R geometry)
    // theta2 = atan2(s, r_val) - atan2(l3*sin(theta3), l2 + l3*cos(theta3))
    let theta2 = (s).atan2(r_val) - (l3 * sin_theta3).atan2(l2 + l3 * cos_theta3);

    // Validate first three joints are finite
    if !theta1.is_finite() || !theta2.is_finite() || !theta3.is_finite() {
        return Err("Target out of workspace: base joints complex".into());
    }

    // Precompute sines/cosines used for wrist orientation
    let c1 = theta1.cos();
    let s1 = theta1.sin();
    let c23 = (theta2 + theta3).cos();
    let s23 = (theta2 + theta3).sin();

    // Step 7..9: wrist Euler angles (θ4..θ6)
    // These were kept similar to your original structure but MUST be tested and
    // verified for your chosen Euler/wrist convention. Watch signs & atan2 argument order.
    let theta4 = ( r[(1, 2)] * c1 - r[(0, 2)] * s1 )
        .atan2( r[(0, 2)] * c23 * c1 - r[(2, 2)] * s23 + r[(1, 2)] * c23 * s1 );

    let expr = -r[(2, 2)] * c23 - r[(0, 2)] * s23 * c1 - r[(1, 2)] * s23 * s1;
    let theta5 = ( (1.0 - expr.powi(2)).sqrt() ).atan2(-expr);

    let theta6 = ( -r[(2, 1)] * c23 - r[(0, 1)] * s23 * c1 - r[(1, 1)] * s23 * s1 )
        .atan2( -r[(2, 0)] * c23 - r[(0, 0)] * s23 * c1 - r[(1, 0)] * s23 * s1 );

    // Final check
    let thetas = [theta1, theta2, theta3, theta4, theta5, theta6];
    if thetas.iter().any(|t| !t.is_finite()) {
        return Err("One or more joint angles are invalid".into());
    }

    Ok(thetas)
}
