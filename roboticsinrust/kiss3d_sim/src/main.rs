mod arm_sim;

use dh_arm_model::joint::{Joint, JointType};
use dh_arm_model::dh::{DHTable, DHRow, DHParam};
use dh_arm_model::dh_arm_model::DHArmModel;
use arm_sim::ArmSim;
use nalgebra::SVector;
use dh_arm_model::inverse_kinematics_solvers::UrtIkSolver;

use control::ts_vel_pid_controller::TSVelPIDController;

const NUM_FRAMES: usize = 7;
const NUM_JOINTS: usize = 6;
const NUM_LINKS: usize = 5;

fn main() {
    // URT robot 6 DOF arm
    let table = DHTable::<NUM_FRAMES, NUM_JOINTS, NUM_LINKS>::new([
        // Insert DH rows with joint_index
        DHRow::new(DHParam::constant(0.0), DHParam::constant_angle(0.0), DHParam::constant_link(9.0), DHParam::variable_angle(0.0, 0)),   // joint 1
        DHRow::new(DHParam::constant(0.0), DHParam::constant_angle(-90.0), DHParam::constant(0.0), DHParam::variable_angle(-90.0, 1)), // joint 2
        DHRow::new(DHParam::constant_link(24.0), DHParam::constant_angle(0.0), DHParam::constant(0.0), DHParam::variable_angle(90.0, 2)),  // joint 3
        DHRow::new(DHParam::constant(0.0), DHParam::constant_angle(90.0), DHParam::constant_link(22.0), DHParam::variable_angle(0.0, 3)),  // joint 4
        DHRow::new(DHParam::constant(0.0), DHParam::constant_angle(-90.0), DHParam::constant(0.0), DHParam::variable_angle(0.0, 4)),  // joint 5
        DHRow::new(DHParam::constant(0.0), DHParam::constant_angle(90.0), DHParam::constant_link(15.0), DHParam::variable_angle(0.0, 5)),  // joint 6
        // Add end-effector fixed frame (no joint)
        DHRow::new(DHParam::constant(0.0), DHParam::constant_angle(0.0), DHParam::constant_link(15.0), DHParam::constant_angle(0.0))
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

    // Create Arm with default damping
    let arm = DHArmModel::<NUM_FRAMES, NUM_JOINTS, NUM_LINKS, UrtIkSolver>::new(
        table,
        joints,
        None, // Use default damping
        UrtIkSolver,
    );

    // Choose dt for simulation (seconds)
    let dt = 0.05; // 50 ms per step

    // Instantiate the task-space velocity PID controller.
    let ts_vel_controller = TSVelPIDController::new(
        SVector::<f64, 6>::from([1.0, 1.0, 1.0, 0.0, 0.0, 0.0]),
        SVector::<f64, 6>::from([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        SVector::<f64, 6>::from([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    );

    let mut sim = ArmSim::new(arm, ts_vel_controller, dt);
    sim.run();
}
