use dh_arm_model::dh::{DHParam, DHRow, DHTable};
use dh_arm_model::joint::{Joint, JointType};

fn approx_eq(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-9
}

fn dh_joint_row(a: f64, alpha: f64, d: f64, base_offset: f64, joint_index: usize) -> DHRow {
    DHRow::new(
        DHParam::constant(a),
        DHParam::constant_angle(alpha),
        DHParam::constant(d),
        DHParam::variable_angle(base_offset, joint_index),
    )
}

fn dh_fixed_row(a: f64, alpha: f64, d: f64) -> DHRow {
    DHRow::new(
        DHParam::constant(a),
        DHParam::constant_angle(alpha),
        DHParam::constant(d),
        DHParam::constant_angle(0.0),
    )
}

fn revolute_joint(position_degrees: f64) -> Joint {
    let mut joint = Joint::new(JointType::Revolute, None, None);
    joint.set_position(position_degrees);
    joint
}

#[test]
fn fk_one_joint_test1() {
    let table = DHTable::<2, 1, 0>::new([
        dh_joint_row(0.0, 0.0, 0.0, 0.0, 0),
        dh_fixed_row(1.0, 0.0, 0.0),
    ]);
    let joints = [revolute_joint(0.0)];

    let pose = table.get_frame_pose(2, &joints);

    println!(
        "x = {}, y = {}, z = {}",
        pose.position.x, pose.position.y, pose.position.z
    );

    assert!(approx_eq(pose.position.x, 1.0));
    assert!(approx_eq(pose.position.y, 0.0));
    assert!(approx_eq(pose.position.z, 0.0));
}

#[test]
fn fk_one_joint_test2() {
    let table = DHTable::<2, 1, 0>::new([
        dh_joint_row(0.0, 0.0, 0.0, 0.0, 0),
        dh_fixed_row(1.0, 0.0, 0.0),
    ]);
    let joints = [revolute_joint(90.0)];

    let pose = table.get_frame_pose(2, &joints);

    println!(
        "x = {}, y = {}, z = {}",
        pose.position.x, pose.position.y, pose.position.z
    );

    assert!(approx_eq(pose.position.x, 0.0));
    assert!(approx_eq(pose.position.y, 1.0));
    assert!(approx_eq(pose.position.z, 0.0));
}

#[test]
fn fk_one_joint_test3() {
    let table = DHTable::<2, 1, 0>::new([
        dh_joint_row(0.0, 0.0, 0.0, 0.0, 0),
        dh_fixed_row(0.0, 0.0, 1.0),
    ]);
    let joints = [revolute_joint(0.0)];

    let pose = table.get_frame_pose(2, &joints);

    println!(
        "x = {}, y = {}, z = {}",
        pose.position.x, pose.position.y, pose.position.z
    );

    assert!(approx_eq(pose.position.x, 0.0));
    assert!(approx_eq(pose.position.y, 0.0));
    assert!(approx_eq(pose.position.z, 1.0));
}

#[test]
fn fk_six_joints_test1() {
    let table = DHTable::<7, 6, 5>::new([
        dh_joint_row(0.0, 0.0, 0.0, 0.0, 0),
        dh_joint_row(1.0, 0.0, 0.0, 0.0, 1),
        dh_joint_row(1.0, 0.0, 0.0, 0.0, 2),
        dh_joint_row(1.0, 0.0, 0.0, 0.0, 3),
        dh_joint_row(1.0, 0.0, 0.0, 0.0, 4),
        dh_joint_row(1.0, 0.0, 0.0, 0.0, 5),
        dh_fixed_row(1.0, 0.0, 0.0),
    ]);
    let joints = [
        revolute_joint(0.0),
        revolute_joint(0.0),
        revolute_joint(0.0),
        revolute_joint(0.0),
        revolute_joint(0.0),
        revolute_joint(0.0),
    ];

    let pose = table.get_frame_pose(7, &joints);

    println!(
        "x = {}, y = {}, z = {}",
        pose.position.x, pose.position.y, pose.position.z
    );

    assert!(approx_eq(pose.position.x, 6.0));
    assert!(approx_eq(pose.position.y, 0.0));
    assert!(approx_eq(pose.position.z, 0.0));
}

#[test]
fn fk_six_joints_test2() {
    let table = DHTable::<7, 6, 5>::new([
        dh_joint_row(0.0, 0.0, 0.0, 0.0, 0),
        dh_joint_row(1.0, 0.0, 0.0, 0.0, 1),
        dh_joint_row(1.0, 0.0, 0.0, 0.0, 2),
        dh_joint_row(1.0, 0.0, 0.0, 0.0, 3),
        dh_joint_row(1.0, 0.0, 0.0, 0.0, 4),
        dh_joint_row(1.0, 0.0, 0.0, 0.0, 5),
        dh_fixed_row(1.0, 0.0, 0.0),
    ]);
    let joints = [
        revolute_joint(0.0),
        revolute_joint(0.0),
        revolute_joint(0.0),
        revolute_joint(0.0),
        revolute_joint(0.0),
        revolute_joint(90.0),
    ];

    let pose = table.get_frame_pose(7, &joints);

    println!(
        "x = {}, y = {}, z = {}",
        pose.position.x, pose.position.y, pose.position.z
    );

    assert!(approx_eq(pose.position.x, 5.0));
    assert!(approx_eq(pose.position.y, 1.0));
    assert!(approx_eq(pose.position.z, 0.0));
}

#[test]
fn fk_six_joints_test3() {
    let table = DHTable::<7, 6, 5>::new([
        dh_joint_row(0.0, 0.0, 0.0, 0.0, 0),
        dh_joint_row(1.0, 0.0, 0.0, 0.0, 1),
        dh_joint_row(1.0, 0.0, 0.0, 0.0, 2),
        dh_joint_row(1.0, 0.0, 0.0, 0.0, 3),
        dh_joint_row(1.0, 0.0, 0.0, 0.0, 4),
        dh_joint_row(1.0, 0.0, 0.0, 0.0, 5),
        dh_fixed_row(0.0, 0.0, 1.0),
    ]);
    let joints = [
        revolute_joint(0.0),
        revolute_joint(0.0),
        revolute_joint(0.0),
        revolute_joint(0.0),
        revolute_joint(0.0),
        revolute_joint(0.0),
    ];

    let pose = table.get_frame_pose(7, &joints);

    println!(
        "x = {}, y = {}, z = {}",
        pose.position.x, pose.position.y, pose.position.z
    );

    assert!(approx_eq(pose.position.x, 5.0));
    assert!(approx_eq(pose.position.y, 0.0));
    assert!(approx_eq(pose.position.z, 1.0));
}

#[test]
fn fk_six_joints_test4() {
    let table = DHTable::<7, 6, 5>::new([
        dh_joint_row(0.0, 0.0, 0.0, 0.0, 0),
        dh_joint_row(1.0, 0.0, 0.0, 0.0, 1),
        dh_joint_row(1.0, 0.0, 0.0, 0.0, 2),
        dh_joint_row(1.0, 0.0, 0.0, 0.0, 3),
        dh_joint_row(1.0, 0.0, 0.0, 0.0, 4),
        dh_joint_row(1.0, 0.0, 0.0, 0.0, 5),
        dh_fixed_row(1.0, 0.0, 0.0),
    ]);
    let joints = [
        revolute_joint(0.0),
        revolute_joint(0.0),
        revolute_joint(0.0),
        revolute_joint(0.0),
        revolute_joint(90.0),
        revolute_joint(0.0),
    ];

    let pose = table.get_frame_pose(7, &joints);

    println!(
        "x = {}, y = {}, z = {}",
        pose.position.x, pose.position.y, pose.position.z
    );

    assert!(approx_eq(pose.position.x, 4.0));
    assert!(approx_eq(pose.position.y, 2.0));
    assert!(approx_eq(pose.position.z, 0.0));
}

#[test]
fn fk_six_joints_test5() {
    let table = DHTable::<7, 6, 5>::new([
        dh_joint_row(0.0, 0.0, 0.0, 0.0, 0),
        dh_joint_row(1.0, 0.0, 0.0, 0.0, 1),
        dh_joint_row(1.0, 0.0, 0.0, 0.0, 2),
        dh_joint_row(1.0, 0.0, 0.0, 0.0, 3),
        dh_joint_row(1.0, 0.0, 0.0, 0.0, 4),
        dh_joint_row(1.0, 0.0, 0.0, 0.0, 5),
        dh_fixed_row(0.0, 0.0, 1.0),
    ]);
    let joints = [
        revolute_joint(0.0),
        revolute_joint(0.0),
        revolute_joint(0.0),
        revolute_joint(0.0),
        revolute_joint(90.0),
        revolute_joint(0.0),
    ];

    let pose = table.get_frame_pose(7, &joints);

    println!(
        "x = {}, y = {}, z = {}",
        pose.position.x, pose.position.y, pose.position.z
    );

    assert!(approx_eq(pose.position.x, 4.0));
    assert!(approx_eq(pose.position.y, 1.0));
    assert!(approx_eq(pose.position.z, 1.0));
}

#[test]
fn fk_six_joints_test6() {
    let table = DHTable::<7, 6, 5>::new([
        dh_joint_row(0.0, 0.0, 0.0, 0.0, 0),
        dh_joint_row(1.0, 0.0, 0.0, 0.0, 1),
        dh_joint_row(1.0, 0.0, 0.0, 0.0, 2),
        dh_joint_row(1.0, 0.0, 0.0, 0.0, 3),
        dh_joint_row(1.0, 0.0, 0.0, 0.0, 4),
        dh_joint_row(1.0, 90.0, 0.0, 0.0, 5),
        dh_fixed_row(0.0, 0.0, 1.0),
    ]);
    let joints = [
        revolute_joint(0.0),
        revolute_joint(0.0),
        revolute_joint(0.0),
        revolute_joint(0.0),
        revolute_joint(0.0),
        revolute_joint(0.0),
    ];

    let pose = table.get_frame_pose(7, &joints);

    println!(
        "x = {}, y = {}, z = {}",
        pose.position.x, pose.position.y, pose.position.z
    );

    assert!(approx_eq(pose.position.x, 5.0));
    assert!(approx_eq(pose.position.y, -1.0));
    assert!(approx_eq(pose.position.z, 0.0));
}
