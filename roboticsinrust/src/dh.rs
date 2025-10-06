use nalgebra::{Matrix4};

pub enum JointType {
    Revolute,
    Prismatic,
}
pub enum LinkVariable{
    A, // corresponds to 'a' in DH table
    D, // corresponds to 'd' in DH table
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
        Self { a, alpha: alpha.to_radians(), d, theta: theta.to_radians(), joint_type, joint_variable: 0.0, }
    }

    pub fn set_joint_variable(&mut self, new_var: f64) {
        match self.joint_type {
            JointType::Revolute => self.joint_variable = new_var.to_radians(), //new angle in radians
            JointType::Prismatic => self.joint_variable = new_var, //new distance in same units as d
        }
    }

    pub fn set_new_link_length(&mut self, new_length: f64, link_var: &LinkVariable) {
        match link_var {
            LinkVariable::A => self.a = new_length,
            LinkVariable::D => self.d = new_length,
        }
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


pub struct Link{
    length: f64,
    dh_row_index: usize,
    link_var: LinkVariable,
    // Other Link-specific data if necessary (e.g., mass, inertia)
}
impl Link{
    pub fn new(length: f64, dh_row_index: usize, link_var: LinkVariable) -> Self {
        Self { length, dh_row_index, link_var }
    }

    pub fn set_length(&mut self, new_length: f64) {
        self.length = new_length;
    }

    pub fn get_length(&self) -> f64 {
        self.length
    }

    pub fn get_dh_row_index(&self) -> usize {
        self.dh_row_index
    }

    pub fn get_link_var(&self) -> &LinkVariable {
        &self.link_var
    }

}

pub struct RevoluteJoint {
    angle: f64,
    velocity: f64,
    dh_row_index: usize,
}

impl RevoluteJoint {
    pub fn new(angle: f64, velocity: f64, dh_row_index: usize) -> Self {
        Self { angle, velocity, dh_row_index }
    }

    pub fn set_angle(&mut self, new_angle: f64) {
        self.angle = new_angle;
    }

    pub fn get_angle(&self) -> f64 {
        self.angle
    }

    pub fn set_velocity(&mut self, new_velocity: f64) {
        self.velocity = new_velocity;
    }

    pub fn get_velocity(&self) -> f64 {
        self.velocity
    }

    pub fn get_dh_row_index(&self) -> usize {
        self.dh_row_index
    }
}

pub struct DHTable {
    links: Vec<Link>,
    joints: Vec<Joint>,
    rows: Vec<DHRow>,
}

impl DHTable {
    pub fn new(links: Vec<Link>, joints: Vec<Joint>, rows: Vec<DHRow>) -> Self {
        Self { links, joints, rows }
    }

    pub fn update_joint_angle(&mut self, joint_index: usize, new_angle: f64) {
        if let Some(joint) = self.joints.get_mut(joint_index) {
            joint.set_angle(new_angle);
            let dh_index = joint.get_dh_row_index();
            if let Some(dh_row) = self.rows.get_mut(dh_index) {
                dh_row.set_joint_angle(new_angle);
            }
        }
    }

    pub fn update_link_length(&mut self, link_index: usize, new_length: f64) {
        if let Some(link) = self.links.get_mut(link_index) {
            link.set_length(new_length);
            let dh_index = link.get_dh_row_index();
            if let Some(dh_row) = self.rows.get_mut(dh_index) {
                dh_row.set_new_link_length(new_length);
            }
        }
    }


    pub fn forward_kinematics(&self) -> Matrix4<f64> {
        // start with identity
        let mut accumulation = Matrix4::identity();

        // iterate by reference so we don't move rows out of self
        for row in self.rows.iter() {
            // compute the transform for this DH row (borrow row, produce a Matrix4)
            let t = row.get_row_trans_mat();

            // multiply accumulative transform by the new one
            accumulation = t * accumulation * t;
        }

        accumulation
    }
}


