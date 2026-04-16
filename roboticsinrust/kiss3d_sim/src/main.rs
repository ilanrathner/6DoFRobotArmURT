mod arm_sim;

use dh_arm_model::task_space_pid_controller::TaskSpacePidController;
use dh_arm_model::joint::{Joint, JointType};
use dh_arm_model::dh::{DHTable, DHRow};
use dh_arm_model::dh_arm_model::DHArmModel;
use arm_sim::ArmSim;
use nalgebra::SVector;
use dh_arm_model::inverse_kinematics_solvers::UrtIkSolver;

const NUM_FRAMES: usize = 7;
const NUM_JOINTS: usize = 6;

fn main() {
    // URT robot 6 DOF arm
    let table = DHTable::<NUM_FRAMES, NUM_JOINTS>::new([
        // Insert DH rows with joint_index
        DHRow::new(0.0, 0.0, 9.0, 0.0, false, Some(0)),   // joint 1
        DHRow::new(0.0, -90.0, 0.0, -90.0, false, Some(1)), // joint 2
        DHRow::new(24.0, 0.0, 0.0, 90.0, false, Some(2)),  // joint 3
        DHRow::new(0.0, 90.0, 22.0, 0.0, false, Some(3)),  // joint 4
        DHRow::new(0.0, -90.0, 0.0, 0.0, false, Some(4)),  // joint 5
        DHRow::new(0.0, 90.0, 15.0, 0.0, false, Some(5)),  // joint 6
        // Add end-effector fixed frame (no joint)
        DHRow::new(0.0, 0.0, 15.0, 0.0, true, None)
    ]);

    // Create joints
    let joints = [
        Joint::new(JointType::Revolute, None, None), // joint 1
        Joint::new(JointType::Revolute, None, None), // joint 2
        Joint::new(JointType::Revolute, None, None), // joint 3
        Joint::new(JointType::Revolute, None, None), // joint 4
        Joint::new(JointType::Revolute, None, None), // joint 5
        Joint::new(JointType::Revolute, None, None), // joint 6
    ];

    let urt_ik_link_parameters = vec![
        9.0,  // l1
        34.0, // l2
        0.0,  // l3
        32.0, // l4
        15.0, // l5
    ];

    // Create Arm with default damping
    let arm = DHArmModel::<NUM_FRAMES, NUM_JOINTS, UrtIkSolver>::new(
        table,
        joints,
        None, // Use default damping
        UrtIkSolver,
        urt_ik_link_parameters,
    );

    // Choose dt for simulation (seconds)
    let dt = 0.05; // 50 ms per step

    let controller = TaskSpacePidController::new(
        // Proportional Gains (Kp) - [x, y, z, roll, pitch, yaw]
        SVector::<f64, 6>::from([1.0, 1.0, 1.0, 0.0, 0.0, 0.0]), 
        
        // Integral Gains (Ki)
        SVector::<f64, 6>::from([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), 
        
        // Derivative Gains (Kd)
        SVector::<f64, 6>::from([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), 
    );

    let mut sim = ArmSim::new(arm, controller,  dt);
    sim.run();
}
