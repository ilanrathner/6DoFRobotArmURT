use nalgebra::{Matrix4};

pub enum JointType {
    Revolute,
    Prismatic,
    Fixed,
}
pub struct DHRow {
    a: f64,      
    alpha: f64,  
    d: f64,       
    theta: f64,  
    joint_type: JointType,
    joint_variable: f64, // This is the variable part of the joint which could be added to theta or d if its revolute or prismatic respectively
}

impl DHRow {
    pub fn new(a: f64, alpha: f64, d: f64, theta: f64, joint_type:JointType) -> Self {
        Self { a, alpha: alpha.to_radians(), d, theta: theta.to_radians(), joint_type, joint_variable: 0.0 }
    }

    pub fn set_joint_variable(&mut self, new_var: f64) {
        match self.joint_type {
            JointType::Revolute => self.joint_variable = new_var.to_radians(), //new angle in radians
            JointType::Prismatic => self.joint_variable = new_var, //new distance in same units as d
            JointType::Fixed => self.joint_variable = 0.0, // no variable for fixed joints
        }
    }

    // Setters for DH parameters if initially fixed but need to be changed later
    pub fn set_new_a(&mut self, new_a: f64) {
        self.a = new_a;
    }
    pub fn set_new_alpha(&mut self, new_alpha: f64) {
        self.alpha = new_alpha.to_radians();
    }
    pub fn set_new_theta(&mut self, new_theta: f64) {
        self.theta = new_theta.to_radians();
    }
    pub fn set_new_d(&mut self, new_d: f64) {
        self.d = new_d;
    }


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
        let d_total: f64 = match self.joint_type {
            JointType::Revolute => self.d, // d is constant for revolute joints
            JointType::Prismatic => self.d + self.joint_variable, // d changes with prismatic joints
            JointType::Fixed => self.d, // d is constant for fixed joints
        };
        Matrix4::new(
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, d_total,
            0.0, 0.0, 0.0, 1.0,
        )
    }

    fn rz(&self) -> Matrix4<f64> {
        let theta_total: f64 = match self.joint_type {
            JointType::Revolute => self.theta + self.joint_variable, // theta changes with revolute joints
            JointType::Prismatic => self.theta, // theta is constant for prismatic joints
            JointType::Fixed => self.theta, // theta is constant for fixed joints
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

pub struct DHTable {
    rows: Vec<DHRow>,
}

impl DHTable {
    pub fn new(rows: Vec<DHRow>) -> Self {
        Self { rows }
    }

    pub fn set_joint_variable(&mut self, joint_index: usize, new_var: f64) {
        if joint_index < self.rows.len() {
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
        let r = self.rows.len();

        let j = initial_row_index;
        let i = final_row_index;

        assert!(
            j < i && i <= r,
            "Invalid frame range: require 0 <= j < i <= {}, got j={}, i={}",
            j, i, r
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
}


