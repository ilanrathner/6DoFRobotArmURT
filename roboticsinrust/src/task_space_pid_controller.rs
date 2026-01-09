use crate::Arm;

use nalgebra::{SVector, Vector3, Matrix3};

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
    pub fn compute<const F: usize, const J: usize>(
        &mut self,
        arm: &mut Arm<F, J>,
        xd_des_arr: &[f64; 6],       // desired task-space velocity (world-frame)
        motor_pos: &[f64; J],
        motor_vels: &[f64; J],
        dt: f64,
    ) -> [f64; J] {
        // --- 1️⃣ Update arm state from motor readings
        arm.set_joint_positions(motor_pos);
        arm.set_joint_velocities(motor_vels);

        // --- 2️⃣ Current end-effector pose
        let ee_pose = arm.ee_pose(); // Pose { position, rotation }

        // --- 3️⃣ Desired task-space velocity
        let xd_des = SVector::<f64, 6>::from_row_slice(xd_des_arr);
        let v_des = Vector3::new(xd_des[0], xd_des[1], xd_des[2]);
        let w_des = Vector3::new(xd_des[3], xd_des[4], xd_des[5]);

        // --- 4️⃣ Determine if joystick is active
        let vel_eps = 1e-4;
        let joystick_active = v_des.norm() > vel_eps || w_des.norm() > vel_eps;

        // --- 5️⃣ Update reference pose
        if joystick_active {
            // TRACKING MODE: integrate reference
            self.holding = false;

            // Position integration
            self.x_ref += v_des * dt;

            // Orientation integration (world-frame small-angle)
            self.r_ref = self.integrate_orientation(&self.r_ref, &w_des, dt);

            // Cycle-based orthonormalization
            self.cycle_count += 1;
            if self.cycle_count % self.orthonorm_interval == 0 {
                self.r_ref = self.svd_orthonormalize(&self.r_ref);
            }
            println!(">>> JOYSTICK ACTIVE | v_des: {:.3}, w_des: {:.3}", v_des.norm(), w_des.norm());

        } else {
            // HOLD MODE: freeze reference
            if !self.holding {
                // Capture reference ONCE at release
                self.x_ref = ee_pose.position;
                self.r_ref = ee_pose.rotation;
                self.holding = true;
                println!(">>> JOYSTICK RELEASED | HOLDING POSITION");
            }
            // Reference frozen, PID will still correct gravity sag
        }

        println!("--- CONTROLLER STATE ---");
        println!("Mode: {}", if self.holding { "HOLDING" } else { "TRACKING" });
        println!("EE Pos:  {:.4?}", ee_pose.position.as_slice());
        println!("Ref Pos: {:.4?}", self.x_ref.as_slice());

        // --- 6️⃣ Compute position error
        let e_pos = self.x_ref - ee_pose.position;

        // --- 7️⃣ Compute orientation error using cross-product method
        let x_e = ee_pose.x_axis();
        let y_e = ee_pose.y_axis();
        let z_e = ee_pose.z_axis();

        let x_r: Vector3<f64> = self.r_ref.column(0).into();
        let y_r: Vector3<f64> = self.r_ref.column(1).into();
        let z_r: Vector3<f64> = self.r_ref.column(2).into();

        let e_ori = 0.5 * (x_e.cross(&x_r) + y_e.cross(&y_r) + z_e.cross(&z_r));

        // --- 8️⃣ Assemble full 6D task-space error
        let mut error = SVector::<f64, 6>::zeros();
        println!("Pos Error: {:.4?} | Ori Error: {:.4?}", e_pos.as_slice(), e_ori.as_slice());
        error.fixed_rows_mut::<3>(0).copy_from(&e_pos);
        error.fixed_rows_mut::<3>(3).copy_from(&e_ori);

        // --- 9️⃣ PID computation
        self.integral_error += error * dt;
        let d_error = (error - self.prev_error) / dt;

        let u_task =
            xd_des
            + self.kp.component_mul(&error)
            + self.ki.component_mul(&self.integral_error)
            + self.kd.component_mul(&d_error);

        self.prev_error = error;

        // --- 🔟 Map to joint velocities
        let qd_task = arm.inv_jacobian() * u_task;

        // --- 11️⃣ Convert to array for motor output
        let mut qd_array = [0.0f64; J];
        qd_array.copy_from_slice(qd_task.as_slice());
        qd_array
    }
}
