use dh_arm_model::dh::{DHParam, DHRow, DHTable};
use dh_arm_model::joint::{Joint, JointType};

fn approx_eq(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-9
}

fn dh_fixed_link_row(length: f64) -> DHRow {
    DHRow::new(
        DHParam::constant(length),
        DHParam::constant(0.0),
        DHParam::constant(0.0),
        DHParam::constant(0.0),
    )
}

fn dh_revolute_joint_row(joint_index: usize) -> DHRow {
    DHRow::new(
        DHParam::constant(0.0),
        DHParam::constant(0.0),
        DHParam::constant(0.0),
        DHParam::variable(0.0, joint_index),
    )
}

fn revolute_joint(position_degrees: f64) -> Joint {
    let mut joint = Joint::new(JointType::Revolute, None, None);
    joint.set_position(position_degrees);
    joint
}

//_________________________
// Build arm with one joint

fn build_arm_one_joint() -> DHTable<2, 1, 0> {
    // Row 0: the joint itself -> pure roation, no fixed offset
    let row0 = DHRow::new(
        DHParam::constant(0.0),
        DHParam::constant(0.0),
        DHParam::constant(0.0),
        DHParam::variable(0.0, 0),
    );

    // Row 1: the rigid link -> 1 unit long, no rotation of its own
    let row1 = DHRow::new(
        DHParam::constant(1.0),
        DHParam::constant(0.0),
        DHParam::constant(0.0),
        DHParam::constant(0.0),
    );

    DHTable::new([row0, row1])
}

#[test]
fn fk_one_joint_test1() {
    let table = build_arm_one_joint();
    let mut joint = Joint::new(JointType::Revolute, None, None);
    joint.set_position(0.0);
    let joints = [joint];

    let pose = table.get_frame_pose(2, &joints);

    assert!(approx_eq(pose.position.x, 1.0));
    assert!(approx_eq(pose.position.y, 0.0));
    assert!(approx_eq(pose.position.z, 0.0));
}

#[test]
fn fk_one_joint_test2() {
    let table = build_arm_one_joint();
    let mut joint = Joint::new(JointType::Revolute, None, None);
    joint.set_position(90.0);
    let joints = [joint];

    let pose = table.get_frame_pose(2, &joints);

    assert!(approx_eq(pose.position.x, 0.0));
    assert!(approx_eq(pose.position.y, 1.0));
    assert!(approx_eq(pose.position.z, 0.0));
}

//__________________________
// Build arm with two joints

fn build_arm_two_joints(link0: f64, link1: f64) -> DHTable<4, 2, 0> {
    DHTable::new([
        dh_revolute_joint_row(0),
        dh_fixed_link_row(link0),
        dh_revolute_joint_row(1),
        dh_fixed_link_row(link1),
    ])
}

#[test]
fn fk_two_joints_test1() {
    let table = build_arm_two_joints(1.0, 2.0);
    let joints = [revolute_joint(0.0), revolute_joint(0.0)];

    let pose = table.get_frame_pose(4, &joints);

    assert!(approx_eq(pose.position.x, 3.0));
    assert!(approx_eq(pose.position.y, 0.0));
    assert!(approx_eq(pose.position.z, 0.0));
}

#[test]
fn fk_two_joints_test2() {
    let table = build_arm_two_joints(1.0, 2.0);
    let joints = [revolute_joint(0.0), revolute_joint(90.0)];

    let pose = table.get_frame_pose(4, &joints);

    assert!(approx_eq(pose.position.x, 1.0));
    assert!(approx_eq(pose.position.y, 2.0));
    assert!(approx_eq(pose.position.z, 0.0));
}

//__________________________
// Build arm with six joints

fn build_arm_six_joints(
    link0: f64,
    link1: f64,
    link2: f64,
    link3: f64,
    link4: f64,
    link5: f64,
) -> DHTable<12, 6, 0> {
    DHTable::new([
        dh_revolute_joint_row(0),
        dh_fixed_link_row(link0),
        dh_revolute_joint_row(1),
        dh_fixed_link_row(link1),
        dh_revolute_joint_row(2),
        dh_fixed_link_row(link2),
        dh_revolute_joint_row(3),
        dh_fixed_link_row(link3),
        dh_revolute_joint_row(4),
        dh_fixed_link_row(link4),
        dh_revolute_joint_row(5),
        dh_fixed_link_row(link5),
    ])
}

#[test]
fn fk_six_joints_test1() {
    let table = build_arm_six_joints(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
    let joints = [
        revolute_joint(0.0),
        revolute_joint(0.0),
        revolute_joint(0.0),
        revolute_joint(0.0),
        revolute_joint(0.0),
        revolute_joint(0.0),
    ];

    let pose = table.get_frame_pose(12, &joints);

    assert!(approx_eq(pose.position.x, 21.0));
    assert!(approx_eq(pose.position.y, 0.0));
    assert!(approx_eq(pose.position.z, 0.0));
}

#[test]
fn fk_six_joints_test2() {
    let table = build_arm_six_joints(1.0, 1.0, 1.0, 1.0, 1.0, 1.0);
    let joints = [
        revolute_joint(0.0),
        revolute_joint(90.0),
        revolute_joint(-90.0),
        revolute_joint(-90.0),
        revolute_joint(90.0),
        revolute_joint(0.0),
    ];

    let pose = table.get_frame_pose(12, &joints);

    assert!(approx_eq(pose.position.x, 4.0));
    assert!(approx_eq(pose.position.y, 0.0));
    assert!(approx_eq(pose.position.z, 0.0));
}
