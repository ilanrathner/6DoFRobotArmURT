use crate::dh::{DHTable, Pose};
use crate::joint::{Joint};

use crate::inverse_kinematics_solvers::IkSolver; // <-- IMPORT TRAIT 

use nalgebra::{DMatrix};


pub struct Arm {
    dh_table: DHTable,           // The robot's DH table
    joints: Vec<Joint>,        // The robot's joints
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
    pub fn new(
        dh_table: DHTable,
        joints: Vec<Joint>,
        damping: Option<f64>,
        ik_solver: Box<dyn IkSolver>,
        ik_link_parameters: Vec<f64>
    ) -> Self {
        Self {
            dh_table,
            joints,
            jacobian: None,
            inv_jacobian: None,
            dirty: true,
            damping: damping.unwrap_or(1e-4),
            ik_solver,
            ik_link_parameters,
        }
    }

    pub fn dh_table(&self) -> &DHTable {
        &self.dh_table
    }

    pub fn joints(&self) -> &Vec<Joint> {
        &self.joints
    }

    pub fn update_joint(&mut self, index: usize, joint: Joint) {
        self.joints[index] = joint;
        self.dirty = true;
    }

    pub fn update_joints(&mut self, joints: Vec<Joint>) {
        self.joints = joints;
        self.dirty = true;
    }

    /// Compute / update cached FK, Jacobian, and inverse if dirty
    pub fn update(&mut self) {
        if self.dirty {
            let j = self.dh_table.compute_jacobian(&self.joints);
            let inv_j = self.dh_table.damped_moore_penrose_pseudo_inverse(
                &self.joints,
                Some(&j),
                Some(self.damping),
            );

            self.jacobian = Some(j);
            self.inv_jacobian = Some(inv_j);
            self.dirty = false;
        }
    }

    /// Get the current end-effector pose (computes if dirty)
    pub fn ee_pose(&self) -> Pose {
        // Pass self.joints to DHTable
        self.dh_table.get_frame_pose(self.dh_table.num_frames() - 1, &self.joints)
    }

    pub fn frame_poses(&self) -> Vec<Pose> {
        self.dh_table.all_poses(&self.joints)
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

    /// Solves IK using the End-Effector target pose (position + rotation matrix)
    pub fn solve_ik_from_pose(&self, target_pose: &Pose) -> Result<Vec<f64>, String> {
        let x = target_pose.position.x;
        let y = target_pose.position.y;
        let z = target_pose.position.z;
        let r = &target_pose.rotation;
        let link_lengths = &self.ik_link_parameters;

        self.ik_solver.solve_ik(x, y, z, r, link_lengths)
    }

    /// Solves IK using the End-Effector target position (x,y,z) and Euler angles (yaw, pitch, roll)
    pub fn solve_ik_from_components(
        &self, 
        x: f64, y: f64, z: f64, 
        yaw: f64, pitch: f64, roll: f64
    ) -> Result<Vec<f64>, String> {
        let r = Pose::orientation_mat(yaw, pitch, roll); 
        let link_lengths = &self.ik_link_parameters;

        self.ik_solver.solve_ik(x, y, z, &r, link_lengths)
    }
}