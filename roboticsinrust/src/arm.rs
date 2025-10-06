use crate::dh::{DHRow, LinkVariable};

use nalgebra::{Matrix3, Matrix4, Vector3};
use std::f64::consts::PI;



pub struct Arm{
    joint_angles: Vec<f64>,
    joint_velocities: Vec<f64>,
    dh_table: Vec<DHRow>,
    links: Vec<Link>,
}