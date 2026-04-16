use std::usize;

use crate::dh::{DHTable, Pose};
use crate::joint::{Joint};

use crate::inverse_kinematics_solvers::IkSolver; // <-- IMPORT TRAIT 

use nalgebra::{SMatrix, SVector};

/// High-level controller for a robotic arm defined by Denavit-Hartenberg parameters.
/// 
/// This struct acts as the central "brain," coordinating the kinematic table, 
/// joint states, and the IK solver. It uses a lazy-update pattern to cache 
/// expensive computations like the Jacobian and its pseudo-inverse.
///
/// # Type Parameters
/// * `F`: Number of coordinate frames in the kinematic chain.
/// * `J`: Number of movable Joints.
/// * `S`: The Inverse Kinematics solver implementation.
pub struct DHArmModel<const F: usize, const J: usize, S: IkSolver<J>> {
    /// Internal DH representation for Forward Kinematics and Jacobian math.
    dh_table: DHTable<F, J>,          
    /// State of each physical joint (position, velocity, limits).
    joints: [Joint ; J],        
    /// Cached geometric Jacobian
    jacobian: Option<SMatrix<f64, 6, J>>,  
    /// Cached damped Moore-Penrose pseudo-inverse of the Jacobian
    inv_jacobian: Option<SMatrix<f64, J, 6>>, 

    /// State flag indicating if joint positions have changed since the last update.
    /// When true, kinematics must be recomputed.
    dirty: bool,                 
    /// Damping factor ($\lambda$) used in pseudo-inverse to handle singularities.
    damping: f64,                

    ik_solver: S, // Inverse Kinematics solver
    /// Generic list of link parameters needed by the specific IkSolver.
    ik_link_parameters: Vec<f64>,
}

impl<const F: usize, const J: usize, S: IkSolver<J>> DHArmModel<F, J, S> {
    /// Creates a new arm model instance.
    /// 
    /// If `damping` is not provided, it defaults to a stable value of $1e-4$.
    pub fn new(
        dh_table: DHTable<F, J>,
        joints: [Joint; J],
        damping: Option<f64>,
        ik_solver: S,
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

    /// Updates the position of all joints and marks the kinematics as "dirty."
    /// 
    /// # Panics
    /// Panics if the input slice length does not match the joint count `J`.
    pub fn set_joint_positions(&mut self, positions: &[f64; J]) {
        assert_eq!(positions.len(), self.joints.len(), "Position vector length mismatch");
        for (joint, &pos) in self.joints.iter_mut().zip(positions.iter()) {
            joint.set_position(pos);
        }
        self.dirty = true;
    }

    /// Update joint velocities
    pub fn set_joint_velocities(&mut self, velocities: &[f64; J]) {
        assert_eq!(velocities.len(), self.joints.len(), "Velocity vector length mismatch");
        for (joint, &vel) in self.joints.iter_mut().zip(velocities.iter()) {
            joint.set_velocity(vel);
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
    pub fn frame_pose(&self, frame_index: usize) -> Pose {
        // Pass self.joints to DHTable
        self.dh_table.get_frame_pose(frame_index, &self.joints)
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
    pub fn solve_ik_from_pose(&self, target_pose: &Pose) -> Result<[f64; J], String> {
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
    ) -> Result<[f64; J], String> {
        let r = Pose::orientation_mat(yaw, pitch, roll); 
        let link_lengths = &self.ik_link_parameters;

        self.ik_solver.solve_ik(x, y, z, &r, link_lengths)
    }
}