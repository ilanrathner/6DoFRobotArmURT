use nalgebra::SVector;
use crate::Arm;

pub struct TaskSpacePidController {
    pub kp: SVector<f64, 6>,
    pub ki: SVector<f64, 6>,
    pub kd: SVector<f64, 6>,

    integral_error: SVector<f64, 6>,
    prev_error: SVector<f64, 6>,
}

impl TaskSpacePidController {
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

    pub fn compute<const N: usize>(
        &mut self,
        arm: &mut Arm<N>,
        xd_des_arr: [f64; 6],
        dt: f64,
    ) -> SVector<f64, N> {

        let xd_des = SVector::<f64, 6>::from_row_slice(&xd_des_arr);

        let qd = arm.joint_velocities();
        let j = arm.jacobian();
        let xd = j * qd;

        let error = xd_des - xd;

        self.integral_error += error * dt;
        let d_error = (error - self.prev_error) / dt;

        let u_task =
            xd_des
            + self.kp.component_mul(&error)
            + self.ki.component_mul(&self.integral_error)
            + self.kd.component_mul(&d_error);

        self.prev_error = error;

        let j_pinv = arm.inv_jacobian();
        j_pinv * u_task
    }
}