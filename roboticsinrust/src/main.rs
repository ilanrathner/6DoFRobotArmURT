mod dh;
mod arm;
mod inverse_kinematics_solvers;
mod arm_sim;
mod joint;

use joint::{Joint, JointType};
use dh::{DHTable, DHRow, FrameType};
use arm::Arm;
use arm_sim::ArmSim;

fn main() {
    // URT robot 6 DOF arm
    let mut table = DHTable::new_empty();

    // Create joints
    let joints = vec![
        Joint::new(JointType::Revolute), // joint 1
        Joint::new(JointType::Revolute), // joint 2
        Joint::new(JointType::Revolute), // joint 3
        Joint::new(JointType::Revolute), // joint 4
        Joint::new(JointType::Revolute), // joint 5
        Joint::new(JointType::Revolute), // joint 6
    ];

    // Insert DH rows with joint_index
    table.insert_row(DHRow::new(0.0, 0.0, 9.0, 0.0, FrameType::Joint, Some(0)));   // joint 1
    table.insert_row(DHRow::new(0.0, -90.0, 0.0, -90.0, FrameType::Joint, Some(1))); // joint 2
    table.insert_row(DHRow::new(34.0, 0.0, 0.0, 90.0, FrameType::Joint, Some(2)));  // joint 3
    table.insert_row(DHRow::new(0.0, 90.0, 32.0, 0.0, FrameType::Joint, Some(3)));  // joint 4
    table.insert_row(DHRow::new(0.0, -90.0, 0.0, 0.0, FrameType::Joint, Some(4)));  // joint 5
    table.insert_row(DHRow::new(0.0, 90.0, 15.0, 0.0, FrameType::Joint, Some(5)));  // joint 6

    // Add end-effector fixed frame (no joint)
    table.insert_row(DHRow::new(0.0, 0.0, 15.0, 0.0, FrameType::Fixed, None));

    let urt_ik_link_parameters = vec![
        9.0,  // l1
        34.0, // l2
        0.0,  // l3
        32.0, // l4
        15.0, // l5
    ];

    // Create Arm with default damping
    let arm = Arm::new(
        table,
        joints,
        None, // Use default damping
        Box::new(inverse_kinematics_solvers::UrtIkSolver),
        urt_ik_link_parameters,
    );

    // Choose dt for simulation (seconds)
    let dt = 0.05; // 50 ms per step

    let mut sim = ArmSim::new(arm, dt);
    sim.run();
}
