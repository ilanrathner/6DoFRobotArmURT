use nalgebra::{Matrix4, Matrix3,  Vector3, DMatrix};

pub enum FrameType {
    Revolute,
    Prismatic,
    Fixed,
}

impl FrameType {
    pub fn is_joint(&self) -> bool {
        matches!(self, FrameType::Revolute | FrameType::Prismatic)
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
    joint_variable: f64, // This is the variable part of the joint which could be added to theta or d if its revolute or prismatic respectively
}

impl DHRow {
    pub fn new(a: f64, alpha: f64, d: f64, theta: f64, frame_type:FrameType) -> Self {
        Self  {
            a,
            alpha: alpha.to_radians(),
            d,
            theta: theta.to_radians(),
            frame_type,
            joint_variable: 0.0,
        }
    }

    pub fn set_joint_variable(&mut self, new_var: f64) {
        match self.frame_type {
            FrameType::Revolute => self.joint_variable = new_var.to_radians(), //new angle in radians
            FrameType::Prismatic => self.joint_variable = new_var, //new distance in same units as d
            FrameType::Fixed => self.joint_variable = 0.0, // no variable for fixed joints
        }
    }

    // Setters for DH parameters if initially fixed but need to be changed later
    pub fn set_new_a(&mut self, new_a: f64) { self.a = new_a; }
    pub fn set_new_alpha(&mut self, new_alpha: f64) { self.alpha = new_alpha.to_radians(); }
    pub fn set_new_theta(&mut self, new_theta: f64) { self.theta = new_theta.to_radians(); }
    pub fn set_new_d(&mut self, new_d: f64) { self.d = new_d; }

    pub fn theta_deg(&self) -> f64 { self.theta.to_degrees() }
    pub fn alpha_deg(&self) -> f64 { self.alpha.to_degrees() }


    fn tx(&self) -> Matrix4<f64> {
        Matrix4::new(
            1.0, 0.0, 0.0, self.a,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0,
        )
    }

    fn rx(&self) -> Matrix4<f64> {
        Matrix4::new(
            1.0, 0.0,               0.0,              0.0,
            0.0, self.alpha.cos(), -self.alpha.sin(), 0.0,
            0.0, self.alpha.sin(),  self.alpha.cos(), 0.0,
            0.0, 0.0,               0.0,              1.0,
        )
    }

    fn tz(&self) -> Matrix4<f64> {
        let d_total: f64 = match self.frame_type {
            FrameType::Revolute => self.d, // d is constant for revolute joints
            FrameType::Prismatic => self.d + self.joint_variable, // d changes with prismatic joints
            FrameType::Fixed => self.d, // d is constant for fixed joints
        };
        Matrix4::new(
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, d_total,
            0.0, 0.0, 0.0, 1.0,
        )
    }

    fn rz(&self) -> Matrix4<f64> {
        let theta_total: f64 = match self.frame_type {
            FrameType::Revolute => self.theta + self.joint_variable, // theta changes with revolute joints
            FrameType::Prismatic => self.theta, // theta is constant for prismatic joints
            FrameType::Fixed => self.theta, // theta is constant for fixed joints
        };
        Matrix4::new(
            theta_total.cos(), -theta_total.sin(), 0.0, 0.0,
            theta_total.sin(),  theta_total.cos(), 0.0, 0.0,
            0.0,               0.0,               1.0, 0.0,
            0.0,               0.0,               0.0, 1.0,
        )
    }

    pub fn get_row_trans_mat(&self) -> Matrix4<f64> {
        self.tx() * self.rx() * self.tz() * self.rz()
    }

}

// -----------------------------------------------------------------------------
// DHTable: manages all frames and joints
// -----------------------------------------------------------------------------
pub struct DHTable {
    rows: Vec<DHRow>,
    num_joints: usize, // number of joints (this is how many prismatic and revolute frames there are)
}

impl DHTable {
    pub fn new_empty() -> Self {
        Self { rows: Vec::new(), num_joints: 0 }
    }

    pub fn insert_row(&mut self, row: DHRow) {
        if !matches!(row.frame_type, FrameType::Fixed) {
            self.num_joints += 1;
        }
        self.rows.push(row);
    }

    pub fn num_joints(&self) -> usize { self.num_joints}

    pub fn num_frames(&self) -> usize { self.rows.len() }

    pub fn set_joint_variable(&mut self, joint_index: usize, new_var: f64) {
        if joint_index < self.num_frames() {
            self.rows[joint_index].set_joint_variable(new_var);
        } else {
            panic!("Joint index out of bounds");
        }
    }

    pub fn set_joint_variables(&mut self, vars: &[f64]) {
        for (row, &val) in self.rows.iter_mut().zip(vars.iter()) {
            row.set_joint_variable(val);
        }
    }

    pub fn transformation_matrix_j_i(&self, initial_row_index: usize, final_row_index:usize) -> Matrix4<f64> {
  
        if self.rows.is_empty() {
            panic!("DH table is empty");
        }
        let r = self.num_frames();

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
            transformation_matrix = transformation_matrix * self.rows[f].get_row_trans_mat();
        }

        transformation_matrix
    }

    pub fn forward_kinematics(&self) -> Matrix4<f64> {
        self.transformation_matrix_j_i(0, self.rows.len())
    }

     /// Compute poses for each frame relative to base frame (0).
    pub fn all_poses(&self) -> Vec<Pose> {
        let mut poses = Vec::new();
        let mut transform = Matrix4::<f64>::identity();

        for row in &self.rows {
            transform *= row.get_row_trans_mat();
            poses.push(Pose::from_homogeneous(&transform));
        }

        poses
    }

    /// Get pose between frame j and frame i (exclusive i index convention)
    pub fn pose_between_j_i(&self, j: usize, i: usize) -> Pose {
        assert!(j < i && i <= self.rows.len());
        let mut transform = Matrix4::<f64>::identity();
        for k in j..i {
            transform *= self.rows[k].get_row_trans_mat();
        }
        Pose::from_homogeneous(&transform)
    }

    pub fn compute_jacobian(&self) -> DMatrix<f64> {
        // Number of frames defined in dh table. End effector is typically last frame in last row
        let num_joints = self.num_joints;
        assert!(num_joints > 0, "No joints defined in DH table");

        let poses = self.all_poses();
        let p_end = &poses.last().unwrap().position;

        let mut j = DMatrix::<f64>::zeros(6, num_joints); //Jacobian matrix. Always has 6 rows for linear and angular portion
        let mut joint_col = 0;

        for (i, row) in self.rows.iter().enumerate() {
            if row.frame_type.is_fixed() { continue; }

            let pose_i = &poses[i];
            let z_i = pose_i.z_axis();
            let p_i = pose_i.position;
            let p_diff = p_end - p_i;

            let (linear, angular) = match row.frame_type {
                FrameType::Revolute => (z_i.cross(&p_diff), z_i),
                FrameType::Prismatic => (z_i, Vector3::zeros()),
                FrameType::Fixed => continue,
            };

            for k in 0..3 {
                j[(k, joint_col)] = linear[k];
                j[(k + 3, joint_col)] = angular[k];
            }

            joint_col += 1;
        }

        j
    }

    //Computes the damped moore penrose pseudo inverse. Use a small labda value 
    pub fn damped_moore_penrose_pseudo_inverse(&self, maybe_j: Option<&DMatrix<f64>>, lambda: Option<f64>) -> DMatrix<f64> {
        // We may or may not need to store the computed Jacobian
        let j_storage; 
        // Get Jacobian either from argument or compute it
        let j = match maybe_j {
            Some(j_ref) => j_ref,  // borrow if provided
            None => {
                j_storage = self.compute_jacobian(); // store it in a variable
                &j_storage  // then take a reference to it
            }
        };

        let jt = j.transpose();
        let jtj = &jt * j;
        let n = jtj.nrows();

        let lambda_val = lambda.unwrap_or(1e-4); // default if not provided

        let damped = jtj + DMatrix::identity(n, n) * lambda_val.powi(2);

        let inv = damped
            .try_inverse()
            .expect("Matrix not invertible even with damping");

        inv * jt
    }

    pub fn jacobian_and_inverse(&self, lambda: Option<f64>) -> (DMatrix<f64>, DMatrix<f64>) {
        let j = self.compute_jacobian();
        let inv_j = self.damped_moore_penrose_pseudo_inverse(Some(&j), lambda);
        (j, inv_j)
    }

    /// Returns the indices of the rows that correspond to joints (revolute or prismatic).
    pub fn joint_indices(&self) -> Vec<usize> {
        self.rows.iter().enumerate()
            .filter_map(|(i, row)| if row.frame_type.is_joint() { Some(i) } else { None })
            .collect()
    }
}


// -----------------------------------------------------------------------------
// Pose struct turns a homogeneous matrix into position + rotation and functions for reverse as well
// -----------------------------------------------------------------------------

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
        m.fixed_view_mut::<3, 3>(0, 0).copy_from(&self.rotation);
        m.fixed_view_mut::<3, 1>(0, 3).copy_from(&self.position);
        m
    }

    pub fn from_homogeneous(m: &Matrix4<f64>) -> Self {
        let rotation = m.fixed_view::<3, 3>(0, 0).into();
        let position = Vector3::new(m[(0, 3)], m[(1, 3)], m[(2, 3)]);
        Self { position, rotation }
    }

    /// Returns the x-axis of this frame.
    pub fn x_axis(&self) -> Vector3<f64> { self.rotation.column(0).into() }

    /// Returns the y-axis of this frame.
    pub fn y_axis(&self) -> Vector3<f64> { self.rotation.column(1).into() }

    /// Returns the z-axis of this frame (the joint axis direction).
    pub fn z_axis(&self) -> Vector3<f64> { self.rotation.column(2).into() }
}