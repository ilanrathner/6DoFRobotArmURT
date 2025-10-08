use crate::dh::{DHRow, DHTable, FrameType, Pose};

use nalgebra::{Matrix3, DMatrix, Matrix4, Vector3};


pub struct Arm {
    dh_table: DHTable,           // The robot's DH table
    joint_variables: Vec<f64>,   // Current joint values (optional duplicate of DHRow joint_variable)
    ee_pose: Option<Pose>,       // Cached end-effector pose
    jacobian: Option<DMatrix<f64>>,  // Cached Jacobian
    inv_jacobian: Option<DMatrix<f64>>, // Cached damped pseudo-inverse
    dirty: bool,                 // True if DH table changed since last FK / Jacobian
    damping: f64,                // Default damping for pseudo-inverse
}

impl Arm {
    /// Initialize arm with a DH table and optional damping
    pub fn new(dh_table: DHTable, damping: Option<f64>) -> Self {
        Self {
            dh_table,
            joint_variables: Vec::new(),
            ee_pose: None,
            jacobian: None,
            inv_jacobian: None,
            dirty: true,
            damping: damping.unwrap_or(1e-4),
        }
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
            // Update end-effector pose
            self.ee_pose = Some(Pose::from_homogeneous(&self.dh_table.forward_kinematics()));

            // Update Jacobian
            let j = self.dh_table.compute_jacobian();
            self.jacobian = Some(j.clone());

            // Update damped pseudo-inverse
            let inv_j = self.dh_table.damped_moore_penrose_pseudo_inverse(Some(&j), Some(self.damping));
            self.inv_jacobian = Some(inv_j);

            self.dirty = false;
        }
    }

    /// Get the current end-effector pose (computes if dirty)
    pub fn ee_pose(&mut self) -> &Pose {
        self.update();
        self.ee_pose.as_ref().unwrap()
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
}