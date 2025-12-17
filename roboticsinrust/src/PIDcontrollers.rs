use nalgebra::{Matrix6, Vector6};

/// ---------------------------------------------------------------------------
/// Type aliases
/// ---------------------------------------------------------------------------
pub type TaskVel  = Vector6<f64>;
pub type JointVel = Vector6<f64>;
pub type GainMat  = Matrix6<f64>;

/// ---------------------------------------------------------------------------
/// Conversion helpers (boundary between Vec and fixed-size math)
/// ---------------------------------------------------------------------------

fn vec_to_vector6(v: &Vec<f64>) -> Result<Vector6<f64>, String> {
    if v.len() != 6 {
        return Err(format!("Expected Vec<f64> of length 6, got {}", v.len()));
    }
    Ok(Vector6::from_row_slice(v))
}

fn vector6_to_vec_f32(v: &Vector6<f64>, dt: f64) -> Vec<f32> {
    v.iter()
        .map(|x| (x * dt) as f32)
        .collect()
}

/// ---------------------------------------------------------------------------
/// PID trait (behavior only)
/// ---------------------------------------------------------------------------
pub trait PidController {
    type Error;
    type Output;

    fn update(&mut self, error: Self::Error, dt: f64) -> Self::Output;
}

/// ---------------------------------------------------------------------------
/// 6D matrix PID (full 6x6 gains)
/// ---------------------------------------------------------------------------
pub struct MatrixPid6 {
    pub kp: GainMat,
    pub ki: GainMat,
    pub kd: GainMat,

    integral: Vector6<f64>,
    prev_error: Vector6<f64>,
}

impl MatrixPid6 {
    pub fn new(kp: GainMat, ki: GainMat, kd: GainMat) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral: Vector6::zeros(),
            prev_error: Vector6::zeros(),
        }
    }

    pub fn reset(&mut self) {
        self.integral.fill(0.0);
        self.prev_error.fill(0.0);
    }
}

impl PidController for MatrixPid6 {
    type Error = Vector6<f64>;
    type Output = Vector6<f64>;

    fn update(&mut self, error: Vector6<f64>, dt: f64) -> Vector6<f64> {
        self.integral += error * dt;
        let derivative = (error - self.prev_error) / dt;
        self.prev_error = error;

        self.kp * error + self.ki * self.integral + self.kd * derivative
    }
}

/// ---------------------------------------------------------------------------
/// High-level controller trait
/// ---------------------------------------------------------------------------
pub trait Controller {
    type Input;
    type Output;

    fn step(
        &mut self,
        arm: &Arm,
        reference: Self::Input,
        dt: f64,
    ) -> Result<Self::Output, String>;
}

/// ---------------------------------------------------------------------------
/// Task-space velocity PID controller
/// ---------------------------------------------------------------------------
pub struct TaskSpaceVelocityController {
    pub pid: MatrixPid6,
}

impl Controller for TaskSpaceVelocityController {
    type Input  = TaskVel;   // ẋ_ref
    type Output = Vec<f32>;  // joint deltas for simulation

    fn step(
        &mut self,
        arm: &Arm,
        xdot_ref: TaskVel,
        dt: f64,
    ) -> Result<Vec<f32>, String> {

        // --- Joint velocities (Vec<f64> → Vector6) ---
        let qdot_vec = arm.joint_velocities();
        let qdot = vec_to_vector6(&qdot_vec)?;

        // --- Jacobian ---
        let j = arm.jacobian();
        if j.nrows() != 6 || j.ncols() != 6 {
            return Err("Jacobian shape mismatch".into());
        }

        // --- Task-space velocity ---
        let xdot = j * qdot;

        // --- Task-space PID ---
        let error = xdot_ref - xdot;
        let ux = self.pid.update(error, dt);

        // --- Inverse Jacobian ---
        let inv_j = arm.inv_jacobian();
        if inv_j.nrows() != 6 || inv_j.ncols() != 6 {
            return Err("Inverse Jacobian shape mismatch".into());
        }

        let qdot_cmd = inv_j * ux;

        // --- Convert back to Vec<f32> for simulation integration ---
        Ok(vector6_to_vec_f32(&qdot_cmd, dt))
    }
}

/// ---------------------------------------------------------------------------
/// Joint-space velocity PID controller
/// ---------------------------------------------------------------------------
pub struct JointSpaceVelocityController {
    pub pid: MatrixPid6,
}

impl Controller for JointSpaceVelocityController {
    type Input  = JointVel;
    type Output = Vec<f32>;

    fn step(
        &mut self,
        arm: &Arm,
        qdot_ref: JointVel,
        dt: f64,
    ) -> Result<Vec<f32>, String> {

        let qdot_vec = arm.joint_velocities();
        let qdot = vec_to_vector6(&qdot_vec)?;

        let error = qdot_ref - qdot;
        let qdot_cmd = self.pid.update(error, dt);

        Ok(vector6_to_vec_f32(&qdot_cmd, dt))
    }
}
