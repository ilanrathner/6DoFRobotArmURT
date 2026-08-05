use nalgebra::Matrix3;

// ----------------------------------------------------------------------
// 1. GENERIC TRAIT DEFINITION
// ----------------------------------------------------------------------

/// Defines the interface that all Inverse Kinematics solvers must implement.
pub trait IkSolver<const J: usize> {
    /// Solves the inverse kinematics problem for a given target pose components and link lengths.
    /// 
    /// Returns: Some([theta1..thetaJ]) if a valid workspace solution exists, or None.
    fn solve_ik(
        &self,
        x: f64, 
        y: f64, 
        z: f64, 
        r: &Matrix3<f64>,
        link_lengths: &[f64], 
    ) -> Option<[f64; J]>;
}

// ----------------------------------------------------------------------
// 2. URT ROBOT SPECIFIC IMPLEMENTATION
// ----------------------------------------------------------------------

/// Concrete struct for the URT arm's closed-form IK solver.
pub struct UrtIkSolver;

impl IkSolver<6> for UrtIkSolver {
    fn solve_ik(
        &self,
        x: f64, y: f64, z: f64,
        r: &Matrix3<f64>,
        link_lengths: &[f64],
    ) -> Option<[f64; 6]> {
        
        // --- CHECK: Ensure the correct number of link lengths were provided ---
        if link_lengths.len() != 5 {
            println!("URT IK Solver requires 5 link parameters, but {} were provided.", link_lengths.len());
            return None;
        }

        // --- Print input position (x, y, z) and rotation matrix (r) ---
        println!("--- IK Solver Input ---");
        println!("Target Position (x, y, z): ({:.4}, {:.4}, {:.4})", x, y, z);
        println!("Target Rotation Matrix (R):");
        
        // Print the 3x3 matrix row-by-row for readability
        for i in 0..3 {
            println!("\t| {:.4}  {:.4}  {:.4} |", 
                r[(i, 0)], r[(i, 1)], r[(i, 2)]);
        }
        println!("-----------------------");

        let l1 = link_lengths[0];
        let l2 = link_lengths[1];
        let l3 = link_lengths[2];
        let l4 = link_lengths[3];
        let l5 = link_lengths[4];
        
        // Step 2: wrist center (subtract distance along effector Z)
        let d = l4 + l5;
        let wx = x - d * r[(0, 2)];
        let wy = y - d * r[(1, 2)];
        let wz = z - d * r[(2, 2)];

        // Step 3: theta1
        let theta1 = wy.atan2(wx);

        // Step 4: planar distances for first 3 joints
        let r_val = (wx.powi(2) + wy.powi(2)).sqrt();
        let s = wz - l1;

        // Step 5: theta3 (Law of Cosines)
        let numerator = r_val.powi(2) + s.powi(2) - l2.powi(2) - l3.powi(2);
        let denom = 2.0 * l2 * l3;
        let cos_theta3 = numerator / denom;

        // Workspace reach check
        if cos_theta3.abs() > 1.0 {
            println!("Target out of workspace: theta3 complex (cos_theta3 = {:.4})", cos_theta3);
            return None;
        }

        let sin_theta3 = (1.0 - cos_theta3 * cos_theta3).sqrt();
        let theta3 = sin_theta3.atan2(cos_theta3);

        // Step 6: theta2 (standard 2R geometry)
        let theta2 = r_val.atan2(s) - (l3 * sin_theta3).atan2(l2 + l3 * cos_theta3);

        // Precompute sines/cosines used for wrist orientation
        let c1 = theta1.cos();
        let s1 = theta1.sin();
        let c23 = (theta2 + theta3).cos();
        let s23 = (theta2 + theta3).sin();

        // Step 7..9: wrist Euler angles (θ4..θ6)
        let theta4 = (r[(1, 2)] * c1 - r[(0, 2)] * s1)
            .atan2(r[(0, 2)] * c23 * c1 - r[(2, 2)] * s23 + r[(1, 2)] * c23 * s1);

        let expr = -r[(2, 2)] * c23 - r[(0, 2)] * s23 * c1 - r[(1, 2)] * s23 * s1;
        
        // Clamp to [-1.0, 1.0] to prevent floating-point precision NaN in sqrt
        let expr_clamped = expr.clamp(-1.0, 1.0);
        let theta5 = ((1.0 - expr_clamped.powi(2)).sqrt()).atan2(-expr_clamped);

        let theta6 = (-(-r[(2, 1)] * c23 - r[(0, 1)] * s23 * c1 - r[(1, 1)] * s23 * s1))
            .atan2(-r[(2, 0)] * c23 - r[(0, 0)] * s23 * c1 - r[(1, 0)] * s23 * s1);

        let thetas = [theta1, theta2, theta3, theta4, theta5, theta6];

        if thetas.iter().any(|t| !t.is_finite()) {
            let thetas_str: Vec<String> = thetas.iter().map(|t| format!("{:.4}", t)).collect();
            println!(
                "One or more joint angles are invalid (NaN or Inf). Calculated: [{}]",
                thetas_str.join(", ")
            );
            return None;
        }

        println!("IK solution theta degrees: [{:.4}, {:.4}, {:.4}, {:.4}, {:.4}, {:.4}]",
            theta1.to_degrees(), theta2.to_degrees(), theta3.to_degrees(),
            theta4.to_degrees(), theta5.to_degrees(), theta6.to_degrees());
        
        Some(thetas)
    }
}