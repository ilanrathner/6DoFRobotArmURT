use crate::dh::{DHTable, Pose};
use crate::inverse_kinematics_solvers::IkSolver; // <-- IMPORT TRAIT FROM CONSOLIDATED FILE

use nalgebra::{DMatrix};


pub struct Arm {
    dh_table: DHTable,           // The robot's DH table
    jacobian: Option<DMatrix<f64>>,  // Cached Jacobian
    inv_jacobian: Option<DMatrix<f64>>, // Cached damped pseudo-inverse
    dirty: bool,                 // True if DH table changed since last FK / Jacobian
    damping: f64,                // Default damping for pseudo-inverse
    ik_solver: Box<dyn IkSolver>, // Inverse Kinematics solver
    /// Generic list of link parameters needed by the specific IkSolver.
    ik_link_parameters: Vec<f64>,
}

impl Arm {
    /// Initialize arm with a DH table and optional damping
    pub fn new(dh_table: DHTable, damping: Option<f64>, ik_solver: Box<dyn IkSolver>, ik_link_parameters: Vec<f64>) -> Self {
        Self {
            dh_table,
            jacobian: None,
            inv_jacobian: None,
            dirty: true,
            damping: damping.unwrap_or(1e-4),
            ik_solver,
            ik_link_parameters, // Store the received parameters
        }
    }

    pub fn dh_table(&self) -> &DHTable {
        &self.dh_table
    }

    /// Update a single joint variable by joint number
    pub fn set_joint_variable(&mut self, joint_num: usize, new_var: f64) {
        self.dh_table.set_joint_variable(joint_num, new_var);
        self.dirty = true;
    }

    /// Update multiple joint variables
    pub fn set_joint_variables(&mut self, vars: &[f64]) {
        self.dh_table.set_joint_variables(vars);
        self.dirty = true;
    }


    /// Compute / update cached FK, Jacobian, and inverse if dirty
    pub fn update(&mut self) {
        if self.dirty {
            // Update Jacobian
            let j = self.dh_table.compute_jacobian();
            let inv_j = self.dh_table.damped_moore_penrose_pseudo_inverse(Some(&j), Some(self.damping));

            self.jacobian = Some(j);
            self.inv_jacobian = Some(inv_j);

            self.dirty = false;
        }
    }

    /// Get the current end-effector pose (computes if dirty)
    pub fn ee_pose(&mut self) -> Pose {
        self.dh_table.get_frame_pose(self.dh_table.num_frames() - 1)
    }

    pub fn frame_poses(&self) -> Vec<Pose> {
        self.dh_table.all_poses()
    }


    /// Get the current Jacobian (computes if dirty)
    pub fn jacobian(&mut self) -> &DMatrix<f64> {
        self.update();
        self.jacobian.as_ref().unwrap()
    }

    /// Get the current inverse Jacobian (computes if dirty)
    pub fn inv_jacobian(&mut self) -> &DMatrix<f64> {
        self.update();
        self.inv_jacobian.as_ref().unwrap()
    }

    
    /// Solves IK using the End-Effector target pose (position + rotation matrix).
    pub fn solve_ik_from_pose(&self, target_pose: &Pose) -> Result<Vec<f64>, String> {
        let x = target_pose.position.x;
        let y = target_pose.position.y;
        let z = target_pose.position.z;
        let r = &target_pose.rotation;
        
        let link_lengths = &self.ik_link_parameters; // Get the stored link parameters
        
        // Calls the method on the stored trait object, passing a slice reference
        self.ik_solver.solve_ik(x, y, z, r, &link_lengths) // <--- PASSES &[f64]
    }

    /// Solves IK using the End-Effector target position (x,y,z) and Euler angles (yaw, pitch, roll).
    pub fn solve_ik_from_components(
        &self, 
        x: f64, y: f64, z: f64, 
        yaw: f64, pitch: f64, roll: f64
    ) -> Result<Vec<f64>, String> {
        
        let r = Pose::orientation_mat(yaw, pitch, roll); 
        let link_lengths = &self.ik_link_parameters; // Get the stored link parameters
        
        // Calls the method on the stored trait object, passing a slice reference
        self.ik_solver.solve_ik(x, y, z, &r, &link_lengths) // <--- PASSES &[f64]
    }

}