use crate::joint::{Joint, JointType};
use nalgebra::{Matrix4, Matrix3,  Vector3, SMatrix};


pub enum FrameType {
    Fixed,
    Joint,
}

impl FrameType {
    pub fn is_joint(&self) -> bool {
        matches!(self, FrameType::Joint)
    }
    pub fn is_fixed(&self) -> bool {
        matches!(self, FrameType::Fixed)
    }
}

// -----------------------------------------------------------------------------
// DHRow: manages all functions and data for a single frame
// -----------------------------------------------------------------------------
pub struct DHRow {
    a: f64,      
    alpha: f64,  
    d: f64,       
    theta: f64,  
    frame_type: FrameType,
    joint_index: Option<usize>, // Index of the joint if this frame is a joint
}

impl DHRow {
    pub fn new(a: f64, alpha: f64, d: f64, theta: f64, frame_type:FrameType, joint_index: Option<usize>) -> Self {
        Self  {
            a,
            alpha: alpha.to_radians(),
            d,
            theta: theta.to_radians(),
            frame_type,
            joint_index,
        }
    }


    // Setters for DH parameters if initially fixed but need to be changed later
    pub fn set_new_a(&mut self, new_a: f64) { self.a = new_a; }
    pub fn set_new_alpha(&mut self, new_alpha: f64) { self.alpha = new_alpha.to_radians(); }
    pub fn set_new_theta(&mut self, new_theta: f64) { self.theta = new_theta.to_radians(); }
    pub fn set_new_d(&mut self, new_d: f64) { self.d = new_d; }

    pub fn theta_deg(&self) -> f64 { self.theta.to_degrees() }
    pub fn alpha_deg(&self) -> f64 { self.alpha.to_degrees() }


    fn dh_row_matrix(a: f64, alpha: f64, d: f64, theta: f64) -> Matrix4<f64> {
        let (st, ct) = theta.sin_cos();
        let (sa, ca) = alpha.sin_cos();

        Matrix4::new(
            ct, -st,  0.0, a,
            ca*st,  ca * ct, -sa, -d*sa,
            sa*st,    sa*ct,      ca,       d *ca,
            0.0,    0.0,     0.0,     1.0,
        )
    }

    pub fn get_row_trans_mat(&self, joints: &[Joint]) -> Matrix4<f64> {
        let theta_total = match self.frame_type {
            FrameType::Fixed => self.theta,
            FrameType::Joint => {
                let idx = self.joint_index.expect("Joint row missing joint_index");
                match joints[idx].joint_type {
                    JointType::Revolute => self.theta + joints[idx].position,
                    JointType::Prismatic => self.theta,
                }
            }
        };

        let d_total = match self.frame_type {
            FrameType::Fixed => self.d,
            FrameType::Joint => {
                let idx = self.joint_index.expect("Joint row missing joint_index");
                match joints[self.joint_index.unwrap()].joint_type {
                    JointType::Revolute => self.d,
                    JointType::Prismatic => self.d + joints[idx].position,
                }
            }
        };

        Self::dh_row_matrix(self.a, self.alpha, d_total, theta_total)
    }

    /// Print DH row info, showing joint type and current joint value if applicable
    pub fn print_row(&self, row_index: usize,  joints: &[Joint]) {
        let (ftype_str, joint_val) = match self.frame_type {
            FrameType::Fixed => ("Fixed".to_string(), None),
            FrameType::Joint => {
                let idx = self.joint_index.expect("Joint row missing joint_index");
                let joint = &joints[idx];
                let val = joint.position;
                (format!("{:?}", joint.joint_type), Some(val))
            }
        };

        match joint_val {
            Some(val) => println!(
                "Row {:2} | {:9} | a={:.4}, alpha={:.4}, d={:.4}, theta={:.4}, joint_value={:.4}",
                row_index, ftype_str, self.a, self.alpha, self.d, self.theta, val
            ),
            None => println!(
                "Row {:2} | {:9} | a={:.4}, alpha={:.4}, d={:.4}, theta={:.4}",
                row_index, ftype_str, self.a, self.alpha, self.d, self.theta
            ),
        }
    }
}

// -----------------------------------------------------------------------------
// DHTable: manages all frames and joints
// -----------------------------------------------------------------------------
pub struct DHTable<const F: usize, const J: usize> {
    rows: [DHRow; F],
}

impl<const F: usize, const J: usize> DHTable<F, J> {
    pub fn new(rows: [DHRow; F]) -> Self {
        Self { rows }
    }

    pub fn num_frames(&self) -> usize { F }

    pub fn transformation_matrix_j_i(&self, initial_row_index: usize, final_row_index:usize, joints: &[Joint; J]) -> Matrix4<f64> {

        let r = F;

        let j = initial_row_index;
        let i = final_row_index;

        assert!(
            j < i && i <= r,
            "Invalid frame range: require 0 <= j < i <= {}, got j={}, i={}",
            r, j, i
        );

        let mut transformation_matrix = Matrix4::<f64>::identity();

        //multiply transformation matrices from j to i-1
        for f in j..i {
            transformation_matrix *=  self.rows[f].get_row_trans_mat(joints);
        }

        transformation_matrix
    }

    pub fn forward_kinematics(&self, joints: &[Joint; J]) -> Matrix4<f64> {
        self.transformation_matrix_j_i(0, F, joints)
    }

     /// Compute poses for each frame relative to base frame (0).
    pub fn all_poses(&self, joints: &[Joint; J]) -> [Pose; F] {
        let mut poses: [Pose; F] = std::array::from_fn(|_|  Pose::identity());
        let mut transform = Matrix4::<f64>::identity();

        for i in 0..F {
            transform *= self.rows[i].get_row_trans_mat(joints);
            poses[i] = Pose::from_homogeneous(&transform);
        }

        poses
    }

    pub fn get_frame_pose(&self, frame_index: usize, joints: &[Joint; J]) -> Pose {
        assert!(frame_index < F);
        let mut transform = Matrix4::<f64>::identity();
        for k in 0..frame_index {
            transform *= self.rows[k].get_row_trans_mat(joints);
        }
        Pose::from_homogeneous(&transform)
    }
    /// Get pose between frame j and frame i (exclusive i index convention)
    pub fn pose_between_j_i(&self, j: usize, i: usize, joints: &[Joint; J]) -> Pose {
        assert!(j < i && i <= F);
        let mut transform = Matrix4::<f64>::identity();
        for k in j..i {
            transform *= self.rows[k].get_row_trans_mat(joints);
        }
        Pose::from_homogeneous(&transform)
    }

    //Jacobian matrix. Always has 6 rows for linear and angular portion
    pub fn compute_jacobian(&self, joints: &[Joint; J]) -> SMatrix<f64, 6, J> {
        let poses = self.all_poses(joints);
        let p_end = poses[F - 1].position;

        let mut j = SMatrix::<f64,6, J>::zeros(); 

        for (i, row) in self.rows.iter().enumerate() {
            if row.frame_type.is_fixed() { continue; }
            let joint_index = row.joint_index.expect("Joint row missing joint_index");

            let pose_i = &poses[i];
            let z_i = pose_i.z_axis();
            let p_i = pose_i.position;
            let p_diff = p_end - p_i;


            let (linear, angular) = match joints[joint_index].joint_type {
                JointType::Revolute => (z_i.cross(&p_diff), z_i),
                JointType::Prismatic => (z_i, Vector3::zeros()),
            };

            for k in 0..3 {
                j[(k, joint_index)] = linear[k];
                j[(k + 3, joint_index)] = angular[k];
            }
        }       
        j
    }

    //Computes the damped moore penrose pseudo inverse. Use a small labda value
    pub fn damped_moore_penrose_pseudo_inverse(
        &self,
        joints: &[Joint; J],
        maybe_j: Option<&SMatrix<f64, 6, J>>,
        lambda: Option<f64>,
    ) -> SMatrix<f64, J, 6> {
        // Get or compute the Jacobian (stack-allocated)
        let j_storage;
        let j = match maybe_j {
            Some(j_ref) => j_ref,
            None => {
                j_storage = self.compute_jacobian(joints); // returns SMatrix<6,N>
                &j_storage
            }
        };

        // Compute Jᵀ once (J x 6)
        let jt: SMatrix<f64, J, 6> = j.transpose();

        // Compute Jᵀ * J (J x J)
        let mut damped: SMatrix<f64, J, J> = &jt * j;

        // Add damping lambda^2 to diagonal
        let lambda_val = lambda.unwrap_or(1e-4);
        for i in 0..J {
            damped[(i, i)] += lambda_val.powi(2);
        }

        // Invert damped matrix (J x J)
        let inv = damped.try_inverse().expect("Matrix not invertible even with damping");

        // Multiply by Jᵀ (J x 6) to get pseudo-inverse
        inv * jt // SMatrix<J, 6>
    }


    pub fn print_table(&self, joints: &[Joint; J]) {
        println!("================ DH TABLE ================");
        for (i, row) in self.rows.iter().enumerate() {
            row.print_row(i, joints);
        }
        println!("==========================================");
    }

}


// -----------------------------------------------------------------------------
// Pose struct turns a homogeneous matrix into position + rotation and functions for reverse as well
// -----------------------------------------------------------------------------
#[derive(Debug)]
pub struct Pose {
    pub position: Vector3<f64>,
    pub rotation: Matrix3<f64>,
}

impl Pose {
    pub fn new(position: Vector3<f64>, rotation: Matrix3<f64>) -> Self {
        Self { position, rotation }
    }

    pub fn to_homogeneous(&self) -> Matrix4<f64> {
        let mut m = Matrix4::identity();
        m.fixed_slice_mut::<3, 3>(0, 0).copy_from(&self.rotation);
        m.fixed_slice_mut::<3, 1>(0, 3).copy_from(&self.position);
        m
    }

    pub fn from_homogeneous(m: &Matrix4<f64>) -> Self {
        let rotation = m.fixed_slice::<3, 3>(0, 0).into();
        let position = Vector3::new(m[(0, 3)], m[(1, 3)], m[(2, 3)]);
        Self { position, rotation }
    }

    pub fn identity() -> Self {
        Self {
            position: Vector3::zeros(),
            rotation: Matrix3::identity(),
        }
    }

    /// Returns the x-axis of this frame.
    pub fn x_axis(&self) -> Vector3<f64> { self.rotation.column(0).into() }

    /// Returns the y-axis of this frame.
    pub fn y_axis(&self) -> Vector3<f64> { self.rotation.column(1).into() }

    /// Returns the z-axis of this frame (the joint axis direction).
    pub fn z_axis(&self) -> Vector3<f64> { self.rotation.column(2).into() }

    /// Compute orientation matrix from yaw (Z), pitch (Y), roll (X).
    /// Rotation order: Z * Y * X (yaw, pitch, roll).
    pub fn orientation_mat(yaw: f64, pitch: f64, roll: f64) -> Matrix3<f64> {
        // Rotation about X (Roll)
        let x_rot = Matrix3::new(
            1.0, 0.0, 0.0,
            0.0, roll.cos(), -roll.sin(),
            0.0, roll.sin(),  roll.cos(),
        );

        // Rotation about Y (Pitch)
        let y_rot = Matrix3::new(
            pitch.cos(), 0.0, pitch.sin(),
            0.0, 1.0, 0.0,
           -pitch.sin(), 0.0, pitch.cos(),
        );

        // Rotation about Z (Yaw)
        let z_rot = Matrix3::new(
            yaw.cos(), -yaw.sin(), 0.0,
            yaw.sin(),  yaw.cos(), 0.0,
            0.0, 0.0, 1.0,
        );

        // Combined Rotation: Z * Y * X
        z_rot * y_rot * x_rot
    }

    /// Constructor helper to create a Pose directly from components.
    pub fn from_components(x: f64, y: f64, z: f64, yaw: f64, pitch: f64, roll: f64) -> Self {
        let position = Vector3::new(x, y, z);
        let rotation = Self::orientation_mat(yaw, pitch, roll);
        Self { position, rotation }
    }
}