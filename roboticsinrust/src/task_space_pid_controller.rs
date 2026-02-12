use crate::dh_arm_model::DHArmModel;

use nalgebra::{SVector, Vector3, Matrix3};
use crate::inverse_kinematics_solvers::IkSolver;

pub struct TaskSpacePidController {
    pub kp: SVector<f64, 6>,
    pub ki: SVector<f64, 6>,
    pub kd: SVector<f64, 6>,

    // PID state
    integral_error: SVector<f64, 6>,
    prev_error: SVector<f64, 6>,

    // Pose reference for position + orientation
    x_ref: Vector3<f64>,
    r_ref: Matrix3<f64>,

    // Holding logic
    holding: bool,

    // Orthonormalization scheduling
    cycle_count: usize,
    orthonorm_interval: usize, // e.g., 50 cycles
}

impl TaskSpacePidController {
    /// Constructor
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
            x_ref: Vector3::zeros(),
            r_ref: Matrix3::identity(),
            holding: false,
            cycle_count: 0,
            orthonorm_interval: 50, // adjust as needed
        }
    }

    /// Helper: small-angle orientation integration directly (world-frame)
    fn integrate_orientation(&self, r: &Matrix3<f64>, w: &Vector3<f64>, dt: f64) -> Matrix3<f64> {
        let w_x = w[0];
        let w_y = w[1];
        let w_z = w[2];

        // Small-angle update: R_new = (I + [w]_x * dt) * R
        let d_r = Matrix3::new(
            0.0, -w_z*dt,  w_y*dt,
            w_z*dt,  0.0, -w_x*dt,
           -w_y*dt, w_x*dt,  0.0
        ) + Matrix3::identity();

        d_r * r
    }

    /// Helper: SVD-based orthonormalization
    fn svd_orthonormalize(&self, r: &Matrix3<f64>) -> Matrix3<f64> {
        let svd = r.svd(true, true);
        let u = svd.u.unwrap();
        let vt = svd.v_t.unwrap();
        u * vt
    }

    /// Main compute function
    /// Inputs:
    /// - xd_des_arr: Desired task-space velocity in cm/s (or m/s whichever is used for dh table. don't need to convert here) 
    /// [vx, vy, vz] in World frame, 
    /// [wx, wy, wz] in End-Effector frame (angular velocity in degrees/s, will be converted to rad/s)
    /// - motor_pos: Current joint positions from encoders  
    /// - motor_vels: Current joint velocities from encoders
    /// - dt: Time step for integration
    /// Output:
    /// - Joint velocity commands to send to motors in degrees/s
    pub fn compute<const F: usize, const J: usize, S: IkSolver<J>>(
        &mut self,
        arm: &mut DHArmModel<F, J, S>,
        xd_des_arr: &[f64; 6],       // Input: [vx, vy, vz] in World, [wx, wy, wz] in End-Effector
        motor_pos: &[f64; J],
        motor_vels: &[f64; J],
        dt: f64,
    ) -> [f64; J] {
        // --- 1️ Update arm state from motor readings
        arm.set_joint_positions(motor_pos);
        arm.set_joint_velocities(motor_vels);

        // --- 2️ Current end-effector pose
        let wrist_pose = arm.frame_pose(F - 1); // Pose { position, rotation }
        let r_curr = wrist_pose.rotation; // Current 3x3 Rotation Matrix (R_world_ee)

        // --- 3️ Parse desired task-space velocity directly from array
        // Linear (World)
        let v_des_world = Vector3::new(xd_des_arr[0], xd_des_arr[1], xd_des_arr[2]);
        // Angular (End-Effector) in rad/s, will transform to World next
        let w_des_ee = Vector3::new(xd_des_arr[3].to_radians(),
                                                                         xd_des_arr[4].to_radians(),
                                                                         xd_des_arr[5].to_radians());

        // --- 4️ TRANSFORM: Map EE rotation to World Frame
        let w_des_world = r_curr * w_des_ee;

        // Construct the unified world-frame desired velocity for Feedforward
        let mut xd_des_world = SVector::<f64, 6>::zeros();
        xd_des_world.fixed_rows_mut::<3>(0).copy_from(&v_des_world);
        xd_des_world.fixed_rows_mut::<3>(3).copy_from(&w_des_world);

        // --- 5️ Determine if joystick is active
        let vel_eps = 1e-4;
        let joystick_active = v_des_world.norm() > vel_eps || w_des_ee.norm() > vel_eps;

        // --- 5️ Update reference pose
        if joystick_active {
            // TRACKING MODE: integrate reference
            self.holding = false;

            // Position integration (World Frame)
            self.x_ref += v_des_world * dt;

            // Orientation integration (Now using the World-transformed w_des)
            self.r_ref = self.integrate_orientation(&self.r_ref, &w_des_world, dt);

            // Cycle-based orthonormalization
            self.cycle_count += 1;
            if self.cycle_count % self.orthonorm_interval == 0 {
                self.r_ref = self.svd_orthonormalize(&self.r_ref);
            }
            //println!(">>> JOYSTICK ACTIVE | v_world: {:.3}, w_ee: {:.3}", v_des_world.norm(), w_des_ee.norm());

        } else {
            // HOLD MODE: freeze reference
            if !self.holding {
                // Capture reference ONCE at release
                self.x_ref = wrist_pose.position;
                self.r_ref = wrist_pose.rotation;
                self.holding = true;
                //println!(">>> JOYSTICK RELEASED | HOLDING POSITION");
            }
        }

        // --- 6️ Compute position error
        let e_pos = self.x_ref - wrist_pose.position;

        // --- 7️ Compute orientation error using cross-product method
        let x_e = wrist_pose.x_axis();
        let y_e = wrist_pose.y_axis();
        let z_e = wrist_pose.z_axis();

        let x_r: Vector3<f64> = self.r_ref.column(0).into();
        let y_r: Vector3<f64> = self.r_ref.column(1).into();
        let z_r: Vector3<f64> = self.r_ref.column(2).into();

        let e_ori = 0.5 * (x_e.cross(&x_r) + y_e.cross(&y_r) + z_e.cross(&z_r));

        // --- 8️ Assemble full 6D task-space error
        let mut error = SVector::<f64, 6>::zeros();
        error.fixed_rows_mut::<3>(0).copy_from(&e_pos);
        error.fixed_rows_mut::<3>(3).copy_from(&e_ori);

        // --- 9️ PID computation
        self.integral_error += error * dt;
        let d_error = (error - self.prev_error) / dt;

        // Feedforward (xd_des_world) + PID correction
        let u_task =
            xd_des_world
            + self.kp.component_mul(&error)
            + self.ki.component_mul(&self.integral_error)
            + self.kd.component_mul(&d_error);

        self.prev_error = error;

        // --- 10 Map to joint velocities
        let qd_task = arm.inv_jacobian() * u_task;

        // --- 11 Convert to array for motor output (with Rad to Deg conversion)
        let mut qd_array = [0.0f64; J];

        for (i, &rad_val) in qd_task.as_slice().iter().enumerate() {
            qd_array[i] = rad_val.to_degrees();
        }

        qd_array
    }
}
