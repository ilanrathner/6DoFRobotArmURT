mod dh;
mod arm;
mod inverse_kinematics_solvers;
mod arm_sim;

use dh::{DHTable, DHRow, FrameType};
use arm::Arm;
use arm_sim::ArmSim;

use std::f64::consts::PI;

fn main() {
    // URT robot 6 dof arm
    // Replace these rows with your real robot DH parameters.
    let mut table = DHTable::new_empty();

    // DHRow::new(a, alpha_deg, d, theta_deg, FrameType)
    table.insert_row(DHRow::new(0.0, 0.0, 9.0, 0.0, FrameType::Revolute)); // joint 1
    table.insert_row(DHRow::new(0.0, -90.0, 0.0, -90.0, FrameType::Revolute)); // joint 2
    table.insert_row(DHRow::new(34.0, 0.0, 0.0, 90.0, FrameType::Revolute)); // joint 3
    table.insert_row(DHRow::new(0.0, 90.0, 32.0, 0.0, FrameType::Revolute)); // joint 4
    table.insert_row(DHRow::new(0.0, -90.0, 0.0, 0.0, FrameType::Revolute)); // joint 5
    table.insert_row(DHRow::new(0.0, 90.0, 15.0, 0.0, FrameType::Revolute)); // joint 6
    // Add an end effector fixed frame for visualization (optional)
    table.insert_row(DHRow::new(0.0, 0.0, 15.0, 0.0, FrameType::Fixed));

    let urt_ik_link_parameters = vec![
        9.0,  // l1
        34.0, // l2
        0.0,  // l3
        32.0, // l4
        15.0, // l5
    ];

    // Create Arm with default damping
    let arm = Arm::new(table, 
        None, 
        Box::new(inverse_kinematics_solvers::UrtIkSolver), 
        urt_ik_link_parameters);

    // choose dt for integration (seconds)
    let dt = 0.05; // 50 ms per keypress step

    let mut sim = ArmSim::new(arm, dt);
    sim.run();
}
