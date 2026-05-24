use nalgebra::{Matrix3, Rotation3, UnitQuaternion, Vector3};

pub struct PoseRef {
    // Pose reference for position + orientation
    // Position reference in the global frame
    x_ref: Vector3<f64>,
    // Orientation reference of the end effector in the world frame
    r_ref: Matrix3<f64>,
    // Orientation reference stored as a quaternion for fast reuse
    quat_ref: UnitQuaternion<f64>,
}

impl PoseRef {
    pub fn new_zero() -> Self {
        let identity_quat = UnitQuaternion::identity();
        Self {
            x_ref: Vector3::zeros(),
            r_ref: Matrix3::identity(),
            quat_ref: identity_quat,
        }
    }

    pub fn new_from_pose(x_ref: Vector3<f64>, r_ref: Matrix3<f64>) -> Self {
        let rotation = Rotation3::from_matrix_unchecked(r_ref);
        let quat_ref = UnitQuaternion::from_rotation_matrix(&rotation);
        Self {
            x_ref,
            r_ref,
            quat_ref,
        }
    }

    pub fn to_pose(&mut self, x_ref: Vector3<f64>, r_ref: Matrix3<f64>) {
        self.x_ref = x_ref;
        self.r_ref = r_ref;
        let rotation = Rotation3::from_matrix_unchecked(self.r_ref);
        self.quat_ref = UnitQuaternion::from_rotation_matrix(&rotation);
    }

    pub fn get_x_ref(&self) -> &Vector3<f64> {
        &self.x_ref
    }

    pub fn get_r_ref(&self) -> &Matrix3<f64> {
        &self.r_ref
    }

    pub fn get_quat_ref(&self) -> &UnitQuaternion<f64> {
        &self.quat_ref
    }

    pub fn print_refs(&self) {
        println!("x_ref: [{:.6}, {:.6}, {:.6}]", self.x_ref.x, self.x_ref.y, self.x_ref.z);
        println!(
            "r_ref:\n  [{:.6}, {:.6}, {:.6}]\n  [{:.6}, {:.6}, {:.6}]\n  [{:.6}, {:.6}, {:.6}]",
            self.r_ref[(0, 0)], self.r_ref[(0, 1)], self.r_ref[(0, 2)],
            self.r_ref[(1, 0)], self.r_ref[(1, 1)], self.r_ref[(1, 2)],
            self.r_ref[(2, 0)], self.r_ref[(2, 1)], self.r_ref[(2, 2)],
        );
    }

    /// Update the Pose from task space velocity inputs, integrating orientation via the rotation matrix.
    /// ts_vel_input is made up of [vx, vy, vz] in World frame and
    /// [wx, wy, wz] in End-Effector frame (angular velocity in degrees/s, converted to rad/s).
    /// - dt: Time step for integration
    pub fn euler_step_mat(&mut self, ts_vel_input: &[f64; 6], dt: f64) {
        let v = Vector3::new(ts_vel_input[0], ts_vel_input[1], ts_vel_input[2]);
        let w_deg = Vector3::new(ts_vel_input[3], ts_vel_input[4], ts_vel_input[5]);
        let w = w_deg * std::f64::consts::PI / 180.0;

        self.x_ref += v * dt;

        let w_skew = Matrix3::new(
            0.0, -w.z, w.y,
            w.z, 0.0, -w.x,
            -w.y, w.x, 0.0,
        );

        //skew is on the right because we are converting from end effector frame orientation velocity to world frame
        self.r_ref += dt * self.r_ref * w_skew;
        // FIX 2: Re-orthogonalize the rotation matrix.
        // Direct addition introduces numerical scaling and shearing drift over time.
        if let Some(orthog_r) = self.r_ref.try_normalize(1e-6) {
            self.r_ref = orthog_r;
        } else {
            // Fallback recovery if matrix somehow degrades past numerical recovery thresholds
            let svd = self.r_ref.svd(true, true);
            if let (Some(u), Some(v_t)) = (svd.u, svd.v_t) {
                self.r_ref = u * v_t;
            }
        }

        let rotation = Rotation3::from_matrix_unchecked(self.r_ref);
        self.quat_ref = UnitQuaternion::from_rotation_matrix(&rotation);
    }

    /// Update the Pose from task space velocity inputs, integrating orientation via the quaternion.
    /// ts_vel_input is made up of [vx, vy, vz] in World frame and
    /// [wx, wy, wz] in End-Effector frame (angular velocity in degrees/s, converted to rad/s).
    /// - dt: Time step for integration
    pub fn euler_step_quat(&mut self, ts_vel_input: &[f64; 6], dt: f64) {
        let v = Vector3::new(ts_vel_input[0], ts_vel_input[1], ts_vel_input[2]);
        let w_deg = Vector3::new(ts_vel_input[3], ts_vel_input[4], ts_vel_input[5]);
        let w = w_deg * std::f64::consts::PI / 180.0;

        self.x_ref += v * dt;

        // Body-fixed (end-effector frame) angular velocity integration.
        let delta_quat = UnitQuaternion::from_scaled_axis(w * dt);
        self.quat_ref = UnitQuaternion::new_normalize(*(self.quat_ref * delta_quat));
        self.r_ref = self.quat_ref.to_rotation_matrix().into_inner();
    }


}
