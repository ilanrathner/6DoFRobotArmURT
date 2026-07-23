use crate::joint::{Joint, JointType};
use nalgebra::{Matrix4, Matrix3,  Vector3, SMatrix};


pub enum DHParam {
    /// Fixed physical value (twist angle, fixed offset).
    Constant(f64),
    
    /// Fixed physical link length used by IK solvers.
    ConstantLink(f64),
    
    /// Driven joint variable (e.g., revolute theta). Base offset is the axis offset in the dh table
    Variable {
        base_offset: f64,
        joint_index: usize,
    },
    
    /// Driven joint variable that ALSO acts as a link length (e.g., prismatic d).
    VariableLink {
        base_offset: f64,
        joint_index: usize,
    },
}

impl DHParam {
    /// Resolves the current numeric value based on joint positions.
    pub fn get_val(&self, joints: &[Joint]) -> f64 {
        match *self {
            DHParam::Constant(val) | DHParam::ConstantLink(val) => val,
            DHParam::Variable { base_offset, joint_index } 
            | DHParam::VariableLink { base_offset, joint_index } => {
                base_offset + joints[joint_index].position
            }
        }
    }

    /// Fixed structural angle in degrees (e.g., fixed twist alpha or fixed offset theta).
    pub fn constant_angle(deg: f64) -> Self {
        Self::Constant(deg.to_radians())
    }

    /// Driven joint variable for angles in degrees (e.g., revolute theta).
    pub fn variable_angle(base_deg: f64, joint_index: usize) -> Self {
        Self::Variable {
            base_offset: base_deg.to_radians(),
            joint_index,
        }
    }

    /// Fixed physical length or linear offset (e.g., fixed d offset or fixed a offset).
    pub fn constant(val: f64) -> Self {
        Self::Constant(val)
    }

    /// Fixed physical link length explicitly tracked for IK solvers (e.g., link a or link d).
    pub fn constant_link(val: f64) -> Self {
        Self::ConstantLink(val)
    }

    /// Driven joint variable for linear displacements (e.g., prismatic d or prismatic a).
    pub fn variable_link(base_offset: f64, joint_index: usize) -> Self {
        Self::VariableLink {
            base_offset,
            joint_index,
        }
    }

}


/// Represents a single row in a Denavit-Hartenberg (DH) parameter table.
/// 
/// This struct manages the transformation data for a single frame, which can
/// either be a physical joint or a fixed frame offset.
pub struct DHRow {
    a: DHParam,      
    alpha: DHParam,  
    d: DHParam,       
    theta: DHParam,  
}

impl DHRow {
    /// Creates a new DH row. 
    /// 
    /// Note: `alpha` and `theta` should be provided in **degrees**; 
    /// they are converted to radians internally.
    pub fn new(a: DHParam, alpha: DHParam, d: DHParam, theta: DHParam) -> Self {
        Self  { a, alpha, d, theta }
    }

    /// Internal helper to generate a standard DH transformation matrix.
    /// 
    /// Uses the convention: T = T(x)*R(alpha)*T(z)*R(theta).
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

    /// Computes the 4x4 transformation matrix for this row given the current joint states.
    pub fn get_row_trans_mat(&self, joints: &[Joint]) -> Matrix4<f64> {
        
        // Everything resolves itself based on whether it's constant or variable
        let a = self.a.get_val(joints);
        let alpha = self.alpha.get_val(joints); // Remember to handle radians/degrees conversion
        let d = self.d.get_val(joints);
        let theta = self.theta.get_val(joints);

        Self::dh_row_matrix(a, alpha, d, theta)
    }

    /// Returns the joint index and whether it is a prismatic or revolute joint
    pub fn get_joint_info(&self) -> Option<(usize, JointType)> {
        match self.theta {
            DHParam::Variable { joint_index, .. } 
            | DHParam::VariableLink { joint_index, .. } => return Some((joint_index, JointType::Revolute)),
            _ => {}
        }

        match self.d {
            DHParam::Variable { joint_index, .. } 
            | DHParam::VariableLink { joint_index, .. } => return Some((joint_index, JointType::Prismatic)),
            _ => {}
        }

        match self.alpha {
            DHParam::Variable { joint_index, .. } 
            | DHParam::VariableLink { joint_index, .. } => return Some((joint_index, JointType::Revolute)),
            _ => {}
        }

        match self.a {
            DHParam::Variable { joint_index, .. } 
            | DHParam::VariableLink { joint_index, .. } => return Some((joint_index, JointType::Prismatic)),
            _ => {}
        }

        None // Fixed frame offset
    }

    /// Print DH row info, showing joint type and current joint value if applicable
    pub fn print_row(&self, row_index: usize, joints: &[Joint]) {
        let a = self.a.get_val(joints);
        let alpha = self.alpha.get_val(joints); // Remember to handle radians/degrees conversion
        let d = self.d.get_val(joints);
        let theta = self.theta.get_val(joints);

        println!("Frame {}: | a={:.2}, alpha={:.2}, d={:.2}, theta={:.2}",
            row_index, a, alpha.to_degrees(), d, theta.to_degrees());
        
    }
}

/// A Denavit-Hartenberg Table representing a full robotic kinematic chain.
/// 
/// # Type Parameters
/// * 'F': The number of Frames in the table.
/// * 'J': The number of movable Joints.
/// * 'L': The number of links
pub struct DHTable<const F: usize, const J: usize, const L:usize> {
    rows: [DHRow; F],

    //cached map of each row to its respective joint index and type if it has one
    row_map: [Option<(usize, JointType)>; F],
}

impl<const F: usize, const J: usize, const L:usize > DHTable<F, J, L> {
    pub fn new(rows: [DHRow; F]) -> Self {
        let mut row_map = [None; F];
        for (i, row) in rows.iter().enumerate() {
            // Do the "expensive" 4 if-lets here, only once!
            row_map[i] = row.get_joint_info(); 
        }
        Self { rows, row_map }
    }

    /// Returns all parameters marked as 'is_ik_link' from the table.
    /// This resolves both Constant links and Variable (Prismatic) links.
    pub fn get_current_link_lengths(&self, joints: &[Joint; J]) -> [f64; L] {
        let mut links = [0.0; L];
        let mut cursor = 0;

        for row in &self.rows {
            for param in [&row.a, &row.d] {
                match param {
                    DHParam::ConstantLink(val) => {
                        links[cursor] = *val;
                        cursor += 1;
                    }
                    DHParam::VariableLink { base_offset, joint_index } => {
                        links[cursor] = *base_offset + joints[*joint_index].position;
                        cursor += 1;
                    }
                    _ => {} // Constant and Variable are ignored
                }
            }
        }

        debug_assert_eq!(cursor, L, "IK Link count mismatch! Found {}, expected L={}", cursor, L);
        links
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

    pub fn get_frame_pose(&self, frame: usize, joints: &[Joint; J]) -> Pose {
        assert!(frame> 0 && frame <= F);
        let mut transform = Matrix4::<f64>::identity();
        for k in 0..frame {
            transform *= self.rows[k].get_row_trans_mat(joints);
        }
        Pose::from_homogeneous(&transform)
    }

    /// Computes the geometric Jacobian matrix ($6 \times J$) for the current configuration.
    /// 
    /// The top 3 rows represent linear velocity mapping; the bottom 3 represent angular.
    pub fn compute_jacobian(&self, joints: &[Joint; J]) -> SMatrix<f64, 6, J> {
        let poses = self.all_poses(joints);
        let p_end = poses[F - 1].position;

        let mut j = SMatrix::<f64,6, J>::zeros(); 

        for (i, row_info) in self.row_map.iter().enumerate() {
            let (joint_index, joint_type) = match *row_info {
                Some(info) => info,
                None => continue, // Skip fixed frames
            };

            let pose_i = &poses[i];
            let z_i = pose_i.z_axis();
            let p_i = pose_i.position;
            let p_diff = p_end - p_i;


            let (linear, angular) = match joint_type {
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

    /// Computes the damped Moore-Penrose pseudo-inverse of the Jacobian.
    /// 
    /// This is used to map task-space velocities back to joint velocities.
    /// It handles singularity avoidance via the `lambda` damping parameter.
    ///
    /// # Logic
    /// * If **J >= 6** (Redundant): Uses Right Pseudo-Inverse to minimize joint velocities.
    /// * If **J < 6** (Under-actuated): Uses Left Pseudo-Inverse to minimize task error.
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
            row.print_row(i, joints);
        }
        println!("==========================================");
    }

}


/// Represents the pose of a frame using a vector for position and a rotation matrix for orientation.
/// Converts between homogeneous transformation matrices and this structured format for easier manipulation in task-space control.
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