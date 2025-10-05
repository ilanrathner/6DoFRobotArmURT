use crate::dh::{DHRow, LinkVariable};

use nalgebra::{Matrix3, Matrix4, Vector3};
use std::f64::consts::PI;

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


pub struct Arm{
    links: Vec<Link>,
    joint_angles: Vec<f64>,
    joint_velocities: Vec<f64>,
    dh_table: Vec<DHRow>,
}