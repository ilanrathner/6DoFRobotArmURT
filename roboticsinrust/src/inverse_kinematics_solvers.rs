use nalgebra::Matrix3;

// ----------------------------------------------------------------------
// 1. GENERIC TRAIT DEFINITION
// ----------------------------------------------------------------------

/// Defines the interface that all Inverse Kinematics solvers must implement.
pub trait IkSolver: Send + Sync {
    /// Solves the inverse kinematics problem for a given target pose components and link lengths.
    /// The number of required link lengths is specific to the solver implementation.
    /// 
    /// Returns: Result containing the joint angles [theta1..theta6] or an error string.
    fn solve_ik(
        &self,
        x: f64, 
        y: f64, 
        z: f64, 
        r: &Matrix3<f64>,
        link_lengths: &[f64], // <--- CHANGE: Now a dynamically sized slice
    ) -> Result<[f64; 6], String>;
}

// ----------------------------------------------------------------------
// 2. URT ROBOT SPECIFIC IMPLEMENTATION
// ----------------------------------------------------------------------

/// Concrete struct for the URT arm's closed-form IK solver.
pub struct UrtIkSolver;

impl IkSolver for UrtIkSolver {
    /// Solves IK for the URT arm, which requires exactly 5 link lengths.
    fn solve_ik(
        &self,
        x: f64, y: f64, z: f64,
        r: &Matrix3<f64>,
        link_lengths: &[f64], // <--- Slice input
    ) -> Result<[f64; 6], String> {
        
        // --- CHECK: Ensure the correct number of link lengths were provided ---
        if link_lengths.len() != 5 {
            return Err(format!(
                "URT IK Solver requires 5 link parameters, but {} were provided.", 
                link_lengths.len()
            ));
        }

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

        // Step 5: theta3 (using law of cosines)
        let numerator = r_val.powi(2) + s.powi(2) - l2.powi(2) - l3.powi(2);
        let denom = 2.0 * l2 * l3;
        let cos_theta3 = numerator / denom;
        if cos_theta3.abs() > 1.0 {
            return Err("Target out of workspace: theta3 complex".into());
        }
        let sin_theta3 = (1.0 - cos_theta3 * cos_theta3).sqrt();
        let theta3 = sin_theta3.atan2(cos_theta3);

        // Step 6: theta2 (standard 2R geometry)
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
}