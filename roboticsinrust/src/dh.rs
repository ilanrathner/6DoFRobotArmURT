use nalgebra::{Matrix4};

pub enum LinkVariable{
    A, // corresponds to 'a' in DH table
    D, // corresponds to 'd' in DH table
}

pub struct DHRow {
    a: f64,      
    alpha: f64,  
    d: f64,       
    theta: f64,  
    joint_angle: f64, // This is the variable part of theta and is the change in angle of the revolurte joint
    link_var: LinkVariable,
}

impl DHRow {
    pub fn new(a: f64, alpha: f64, d: f64, theta: f64, link_var: LinkVariable) -> Self {
        Self { a, alpha, d, theta, joint_angle: 0.0, link_var }
    }

    pub fn set_joint_angle(&mut self, angle: f64) {
        self.joint_angle = angle;
    }

    pub fn set_new_link_length(&mut self, new_length: f64) {
        match self.link_var {
            LinkVariable::A => self.a = new_length,
            LinkVariable::D => self.d = new_length,
        }
    }

    fn tx(& mut self) -> Matrix4<f64> {
        Matrix4::new(
            1.0, 0.0, 0.0, self.a,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0,
        )
    }

    fn rx(& mut self) -> Matrix4<f64> {
        Matrix4::new(
            1.0, 0.0,               0.0,              0.0,
            0.0, self.alpha.cos(), -self.alpha.sin(), 0.0,
            0.0, self.alpha.sin(),  self.alpha.cos(), 0.0,
            0.0, 0.0,               0.0,              1.0,
        )
    }

    fn tz(& mut self) -> Matrix4<f64> {
        Matrix4::new(
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, self.d,
            0.0, 0.0, 0.0, 1.0,
        )
    }

    fn rz(& mut self) -> Matrix4<f64> {
        let theta_total = self.theta + self.joint_angle;
        Matrix4::new(
            theta_total.cos(), -theta_total.sin(), 0.0, 0.0,
            theta_total.sin(),  theta_total.cos(), 0.0, 0.0,
            0.0,               0.0,               1.0, 0.0,
            0.0,               0.0,               0.0, 1.0,
        )
    }

    pub fn get_row_trans_mat(& mut self) -> Matrix4<f64> {
        self.tx() * self.rx() * self.tz() * self.rz()
    }

}