mod arm_sim;

use arm_sim::ArmSim;
use nalgebra::SVector;
use dh_arm_model::setups::{setup_default_urt};

use control::ts_vel_pid_controller::TSVelPIDController;


fn main() {

    // Create Arm with default damping
    let arm = setup_default_urt();

    // Choose dt for simulation (seconds)
    let dt = 0.05; // 50 ms per step

    // Instantiate the task-space velocity PID controller.
    let ts_vel_controller = TSVelPIDController::new(
        SVector::<f64, 6>::from([10.0, 10.0, 10.0, 10.0, 10.0, 10.0]),
        SVector::<f64, 6>::from([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        SVector::<f64, 6>::from([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    );

    let mut sim = ArmSim::new(arm, ts_vel_controller, dt);
    sim.run();
}
