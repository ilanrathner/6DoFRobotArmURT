use crate::joint::{Joint, JointType};
use nalgebra::{Matrix4, Matrix3,  Vector3, SMatrix};


// -----------------------------------------------------------------------------
// DHRow: manages all functions and data for a single frame
// -----------------------------------------------------------------------------
pub struct DHRow {
    a: f64,      
    alpha: f64,  
    d: f64,       
    theta: f64,  
    fixed_frame: bool, // True if this row is a fixed frame (no joint)
    joint_index: Option<usize>, // Index of the joint if this frame is a joint
}

impl DHRow {
    pub fn new(a: f64, alpha: f64, d: f64, theta: f64, fixed_frame: bool, joint_index: Option<usize>) -> Self {
        Self  {
            a,
            alpha: alpha.to_radians(),
            d,
            theta: theta.to_radians(),
            fixed_frame,
            joint_index,
        }
    }

    fn dh_row_matrix(a: f64, alpha: f64, d: f64, theta: f64) -> Matrix4<f64> {
        let (st, ct) = theta.sin_cos();
        let (sa, ca) = alpha.sin_cos();

        // DH Transformation Matrix T(x)*R(alpha)*T(z)*R(theta)
        Matrix4::new(
            ct, -st,  0.0, a,
            ca*st,  ca * ct, -sa, -d*sa,
            sa*st,    sa*ct,      ca,       d *ca,
            0.0,    0.0,     0.0,     1.0,
        )
    }

    pub fn get_row_trans_mat(&self, joints: &[Joint]) -> Matrix4<f64> {
        
        let theta_total = if self.fixed_frame {
            self.theta
        } else {
            let idx = self.joint_index.expect("Joint row missing joint_index");
            match joints[idx].joint_type {
                JointType::Revolute => self.theta + joints[idx].position,
                JointType::Prismatic => self.theta,
            }
        };
        

        let d_total = if self.fixed_frame {
            self.d
        } else {
            let idx = self.joint_index.expect("Joint row missing joint_index");
            match joints[idx].joint_type {
                JointType::Revolute => self.d,
                JointType::Prismatic => self.d + joints[idx].position,
            }
        };

        Self::dh_row_matrix(self.a, self.alpha, d_total, theta_total)
    }

    /// Print DH row info, showing joint type and current joint value if applicable
    pub fn print_row(&self, row_index: usize, joints: &[Joint]) {
        if self.fixed_frame {
            println!("Frame {}: Fixed Frame | a={:.2}, alpha={:.2}, d={:.2}, theta={:.2}",
                row_index, self.a, self.alpha.to_degrees(), self.d, self.theta.to_degrees());
        } else {
            let idx = self.joint_index.expect("Joint row missing joint_index");
            let joint = &joints[idx];
            let joint_info = match joint.joint_type {
                JointType::Revolute => format!("Revolute Joint {} | angle={:.2} deg", idx + 1, joint.position.to_degrees()),
                JointType::Prismatic => format!("Prismatic Joint {} | extension={:.2} units", idx + 1, joint.position),
            };
            println!("Frame {}: {} | a={:.2}, alpha={:.2}, d={:.2}, theta={:.2}",
                row_index, joint_info, self.a, self.alpha.to_degrees(), self.d, self.theta.to_degrees());
        };
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

        /// Get pose between frame j and frame i (exclusive i index convention)
    pub fn pose_between_j_i(&self, j: usize, i: usize, joints: &[Joint; J]) -> Pose {
        assert!(j < i && i <= F);
        let mut transform = Matrix4::<f64>::identity();
        for k in j..i {
            transform *= self.rows[k].get_row_trans_mat(joints);
        }
        Pose::from_homogeneous(&transform)
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

    //Jacobian matrix. Always has 6 rows for linear and angular portion
    pub fn compute_jacobian(&self, joints: &[Joint; J]) -> SMatrix<f64, 6, J> {
        let poses = self.all_poses(joints);
        let p_end = poses[F - 1].position;

        let mut j = SMatrix::<f64,6, J>::zeros(); 

        for (i, row) in self.rows.iter().enumerate() {
            if row.fixed_frame { continue; }
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

    /// Computes the damped Moore-Penrose pseudo-inverse.
    /// Works for any number of joints J by automatically switching between
    /// the Right and Left pseudo-inverse based on the system's DOF.
    pub fn damped_moore_penrose_pseudo_inverse(
        &self,
        joints: &[Joint; J],
        maybe_j: Option<&SMatrix<f64, 6, J>>,
        lambda: Option<f64>,
    ) -> SMatrix<f64, J, 6> {
        // 1. Get or compute the Jacobian (6 x J)
        let j_storage;
        let j = match maybe_j {
            Some(j_ref) => j_ref,
            None => {
                j_storage = self.compute_jacobian(joints);
                &j_storage
            }
        };

        // 2. Pre-compute Transpose and Damping value
        let jt = j.transpose(); // (J x 6)
        let lambda_val = lambda.unwrap_or(1e-4);
        let l2 = lambda_val.powi(2);

        // 3. Conditional: Choose method based on Joint count J
        // If J >= 6, we use the Right Inverse (minimizes joint velocities).
        // If J < 6, we use the Left Inverse (minimizes task error).
        if J >= 6 {
            // --- RIGHT PSEUDO-INVERSE (Redundant/Full-DOF) ---
            // Formula: Jᵀ * (J * Jᵀ + λ²I)⁻¹
            
            let mut damped_inner: SMatrix<f64, 6, 6> = j * jt;
            
            // Add damping to the 6x6 diagonal
            for i in 0..6 {
                damped_inner[(i, i)] += l2;
            }

            // Invert 6x6 and multiply by Jᵀ
            match damped_inner.try_inverse() {
                Some(inv) => jt * inv,
                None => {
                    // Fallback if matrix is still singular (e.g. NaNs in Jacobian)
                    eprintln!("Warning: Right inverse failed, returning zeros");
                    SMatrix::<f64, J, 6>::zeros()
                }
            }
        } else {
            // --- LEFT PSEUDO-INVERSE (Under-actuated) ---
            // Formula: (Jᵀ * J + λ²I)⁻¹ * Jᵀ
            
            let mut damped_inner: SMatrix<f64, J, J> = jt * j;
            
            // Add damping to the J x J diagonal
            for i in 0..J {
                damped_inner[(i, i)] += l2;
            }

            // Invert JxJ and multiply Jᵀ
            match damped_inner.try_inverse() {
                Some(inv) => inv * jt,
                None => {
                    eprintln!("Warning: Left inverse failed, returning zeros");
                    SMatrix::<f64, J, 6>::zeros()
                }
            }
        }
    }

    pub fn print_table(&self, joints: &[Joint; J]) {
        println!("================ DH TABLE ================");
        for (i, row) in self.rows.iter().enumerate() {
            row.print_row(i,joints);
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