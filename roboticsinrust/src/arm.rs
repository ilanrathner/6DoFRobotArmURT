use std::usize;

use crate::dh::{DHTable, Pose};
use crate::joint::{Joint, JointType};

use crate::inverse_kinematics_solvers::IkSolver; // <-- IMPORT TRAIT 

use nalgebra::{SMatrix, SVector};


pub struct Arm<const F: usize, const J: usize> {
    dh_table: DHTable<F, J>,           // The robot's DH table
    joints: [Joint ; J],        // The robot's joints

    jacobian: Option<SMatrix<f64, 6, J>>,  // Cached Jacobian
    inv_jacobian: Option<SMatrix<f64, J, 6>>, // Cached damped pseudo-inverse

    dirty: bool,                 // True if DH table changed since last FK / Jacobian
    damping: f64,                // Default damping for pseudo-inverse

    ik_solver: Box<dyn IkSolver>, // Inverse Kinematics solver
    /// Generic list of link parameters needed by the specific IkSolver.
    ik_link_parameters: Vec<f64>,
}

impl<const F: usize, const J: usize> Arm<F, J> {
    /// Initialize arm with a DH table and optional damping
    pub fn new(
        dh_table: DHTable<F, J>,
        joints: [Joint; J],
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

    pub fn dh_table(&self) -> &DHTable<F, J> {
        &self.dh_table
    }

    /// Update joint positions from a slice of f32
    pub fn set_joint_positions(&mut self, positions: &[f64; J]) {
        assert_eq!(positions.len(), self.joints.len(), "Position vector length mismatch");
        for (joint, &pos) in self.joints.iter_mut().zip(positions.iter()) {
            match joint.joint_type {
                JointType::Revolute => joint.set_position_deg(pos as f64),
                JointType::Prismatic => joint.set_position(pos as f64),
            }
        }
        self.dirty = true;
    }

    /// Update joint velocities from a slice of f32
    pub fn set_joint_velocities(&mut self, velocities: &[f64; J]) {
        assert_eq!(velocities.len(), self.joints.len(), "Velocity vector length mismatch");
        for (joint, &vel) in self.joints.iter_mut().zip(velocities.iter()) {
            joint.set_velocity(vel as f64);
        }
        self.dirty = true;
    }

    pub fn joints(&self) -> &[Joint; J] {
        &self.joints
    }

    pub fn joint_positions(&self) -> SVector<f64, J> {
        SVector::from_iterator(self.joints.iter().map(|j| j.position as f64))
    }

    pub fn joint_velocities(&self) -> SVector<f64, J> {
        SVector::from_iterator(self.joints.iter().map(|j| j.velocity as f64))
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
        self.dh_table.get_frame_pose(F - 1, &self.joints)
    }

    pub fn frame_poses(&self) -> [Pose; F] {
        self.dh_table.all_poses(&self.joints)
    }

    /// Get the current Jacobian (computes if dirty)
    pub fn jacobian(&mut self) -> &SMatrix<f64, 6, J> {
        self.update();
        self.jacobian.as_ref().unwrap()
    }

    /// Get the current inverse Jacobian (computes if dirty)
    pub fn inv_jacobian(&mut self) -> &SMatrix<f64, J, 6> {
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