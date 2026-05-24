use dh_arm_model::dh_arm_model::DHArmModel;
use dh_arm_model::inverse_kinematics_solvers::IkSolver;
use nalgebra::{SVector, Vector3};
use crate::ts_pose_ref_from_vel::PoseRef;

pub struct TSVelPIDController {
    kp: SVector<f64, 6>,
    ki: SVector<f64, 6>,
    kd: SVector<f64, 6>,

    // PID state
    integral_error: SVector<f64, 6>,
    prev_error: SVector<f64, 6>,
}


impl TSVelPIDController {
    pub fn new(
        kp: SVector<f64, 6>,
        ki: SVector<f64, 6>,
        kd: SVector<f64, 6>,
    ) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral_error: SVector::zeros(),
            prev_error: SVector::zeros(),
        }
    }

    /// Main compute function
    /// Inputs:
    /// - ts_vel_input: Desired task-space velocity in cm/s (or m/s whichever is used for dh table. don't need to convert here) 
    /// [vx, vy, vz] in World frame, 
    /// [wx, wy, wz] in End-Effector frame (angular velocity in degrees/s, will be converted to rad/s)
    /// - motor_pos: Current joint positions from encoders  
    /// - dt: Time step for integration
    /// Output:
    /// - Joint velocity commands to send to motors in degrees/s
    pub fn compute<const F: usize, const J: usize, const L: usize,  S: IkSolver<J>>(
        &mut self,
        arm: &mut DHArmModel<F, J, L, S>,
        pose_reference: &PoseRef,
        ts_vel_input: &[f64; 6],       // Input: [vx, vy, vz] in World, [wx, wy, wz] in End-Effector
        dt: f64,
    ) -> [f64; J] {
        //  Current end-effector pose
        let wrist_pose = arm.frame_pose(F - 1); // Pose { position, rotation }

        // Parse desired task-space velocity directly from array
        // Linear (World)
        let v_des_world = Vector3::new(ts_vel_input[0], ts_vel_input[1], ts_vel_input[2]);
        // Angular (End-Effector) in rad/s, will transform to World next
        let w_des_ee = Vector3::new(ts_vel_input[3].to_radians(),
                                                                         ts_vel_input[4].to_radians(),
                                                                         ts_vel_input[5].to_radians());

        // TRANSFORM: Map EE rotation to World Frame
        let w_des_world = wrist_pose.rotation * w_des_ee;

        // Construct the unified world-frame desired velocity for Feedforward
        let mut vel_des_world = SVector::<f64, 6>::zeros();
        vel_des_world.fixed_rows_mut::<3>(0).copy_from(&v_des_world);
        vel_des_world.fixed_rows_mut::<3>(3).copy_from(&w_des_world);

        // Compute position error
        let e_pos = pose_reference.get_x_ref() - wrist_pose.position;

        // Compute orientation error using cross-product method
        let x_e = wrist_pose.x_axis();
        let y_e = wrist_pose.y_axis();
        let z_e = wrist_pose.z_axis();

        let r_ref = pose_reference.get_r_ref();
        let x_r: Vector3<f64> = r_ref.column(0).into();
        let y_r: Vector3<f64> = r_ref.column(1).into();
        let z_r: Vector3<f64> = r_ref.column(2).into();

        let e_ori = 0.5 * (x_e.cross(&x_r) + y_e.cross(&y_r) + z_e.cross(&z_r));

        // Assemble full 6D task-space error
        let mut error = SVector::<f64, 6>::zeros();
        error.fixed_rows_mut::<3>(0).copy_from(&e_pos);
        error.fixed_rows_mut::<3>(3).copy_from(&e_ori);

        // PID computation
        self.integral_error += error * dt;
        let d_error = (error - self.prev_error) / dt;

        // Feedforward (vel_des_world) + PID correction
        let u_task =
            vel_des_world
            + self.kp.component_mul(&error)
            + self.ki.component_mul(&self.integral_error)
            + self.kd.component_mul(&d_error);

        self.prev_error = error;

        // Map to joint velocities
        let qd_task = arm.inv_jacobian() * u_task;

        // Convert to array for motor output (with Rad to Deg conversion)
        let mut qd_array = [0.0f64; J];

        for (i, &rad_val) in qd_task.as_slice().iter().enumerate() {
            qd_array[i] = rad_val.to_degrees();
        }

        qd_array
    }

    pub fn print_errors(&self) {
        println!("prev_error: [{:.6}, {:.6}, {:.6}, {:.6}, {:.6}, {:.6}]",
            self.prev_error[0], self.prev_error[1], self.prev_error[2],
            self.prev_error[3], self.prev_error[4], self.prev_error[5],
        );
        println!("integral_error: [{:.6}, {:.6}, {:.6}, {:.6}, {:.6}, {:.6}]",
            self.integral_error[0], self.integral_error[1], self.integral_error[2],
            self.integral_error[3], self.integral_error[4], self.integral_error[5],
        );
    }
}



