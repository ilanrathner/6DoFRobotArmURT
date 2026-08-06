use crate::dh::{DHParam, DHRow, DHTable};
use crate::dh_arm_model::DHArmModel;
use crate::inverse_kinematics_solvers::{IkSolver, UrtIkSolver};
use crate::joint::{Joint, JointType};

pub const NUM_FRAMES: usize = 7;
pub const NUM_JOINTS: usize = 6;
pub const NUM_LINKS: usize = 5;

pub fn urt_arm<S: IkSolver<NUM_JOINTS>>(
    damping: Option<f64>,
    solver: S,
) -> DHArmModel<NUM_FRAMES, NUM_JOINTS, NUM_LINKS, S> {
    let table = DHTable::new([
        DHRow::new(
            DHParam::constant(0.0),
            DHParam::constant_angle(0.0),
            DHParam::constant_link(9.0),
            DHParam::variable_angle(0.0, 0),
        ),
        DHRow::new(
            DHParam::constant(0.0),
            DHParam::constant_angle(-90.0),
            DHParam::constant(0.0),
            DHParam::variable_angle(-90.0, 1),
        ),
        DHRow::new(
            DHParam::constant_link(24.0),
            DHParam::constant_angle(0.0),
            DHParam::constant(0.0),
            DHParam::variable_angle(90.0, 2),
        ),
        DHRow::new(
            DHParam::constant(0.0),
            DHParam::constant_angle(90.0),
            DHParam::constant_link(22.0),
            DHParam::variable_angle(0.0, 3),
        ),
        DHRow::new(
            DHParam::constant(0.0),
            DHParam::constant_angle(-90.0),
            DHParam::constant(0.0),
            DHParam::variable_angle(0.0, 4),
        ),
        DHRow::new(
            DHParam::constant(0.0),
            DHParam::constant_angle(90.0),
            DHParam::constant_link(15.0),
            DHParam::variable_angle(0.0, 5),
        ),
        DHRow::new(
            DHParam::constant(0.0),
            DHParam::constant_angle(0.0),
            DHParam::constant_link(15.0),
            DHParam::constant_angle(0.0),
        ),
    ]);
    let joints = std::array::from_fn(|_| Joint::new(JointType::Revolute, None, None));

    DHArmModel::new(table, joints, damping, solver)
}

pub fn setup_default_urt() -> DHArmModel<NUM_FRAMES, NUM_JOINTS, NUM_LINKS, UrtIkSolver> {
    urt_arm(None, UrtIkSolver)
}