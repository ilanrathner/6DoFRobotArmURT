use dh_arm_model::dh::{DHParam, DHRow, DHTable};
use dh_arm_model::inverse_kinematics_solvers::{IkSolver, UrtIkSolver};
use dh_arm_model::joint::{Joint, JointType};

fn approx_eq(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-9
}

fn revolute_joint(position_degrees: f64) -> Joint {
    let mut joint = Joint::new(JointType::Revolute, None, None);
    joint.set_position(position_degrees);
    joint
}

// #[test]
fn build_six_joints(theta0: f64, theta1: f64, theta2: f64, theta3: f64, theta4: f64, theta5: f64) {
    let table = DHTable::<7, 6, 5>::new([
        // Insert DH rows with joint_index
        DHRow::new(
            DHParam::constant(0.0),
            DHParam::constant_angle(0.0),
            DHParam::constant_link(9.0),
            DHParam::variable_angle(0.0, 0),
        ), // joint 1
        DHRow::new(
            DHParam::constant(0.0),
            DHParam::constant_angle(-90.0),
            DHParam::constant(0.0),
            DHParam::variable_angle(-90.0, 1),
        ), // joint 2
        DHRow::new(
            DHParam::constant_link(24.0),
            DHParam::constant_angle(0.0),
            DHParam::constant(0.0),
            DHParam::variable_angle(90.0, 2),
        ), // joint 3
        DHRow::new(
            DHParam::constant(0.0),
            DHParam::constant_angle(90.0),
            DHParam::constant_link(22.0),
            DHParam::variable_angle(0.0, 3),
        ), // joint 4
        DHRow::new(
            DHParam::constant(0.0),
            DHParam::constant_angle(-90.0),
            DHParam::constant(0.0),
            DHParam::variable_angle(0.0, 4),
        ), // joint 5
        DHRow::new(
            DHParam::constant(0.0),
            DHParam::constant_angle(90.0),
            DHParam::constant_link(15.0),
            DHParam::variable_angle(0.0, 5),
        ), // joint 6
        // Add end-effector fixed frame (no joint)
        DHRow::new(
            DHParam::constant(0.0),
            DHParam::constant_angle(0.0),
            DHParam::constant_link(15.0),
            DHParam::constant_angle(0.0),
        ),
    ]);

    let joints = [
        revolute_joint(theta0),
        revolute_joint(theta1),
        revolute_joint(theta2),
        revolute_joint(theta3),
        revolute_joint(theta4),
        revolute_joint(theta5),
    ];

    let pose = table.get_frame_pose(7, &joints);

    println!(
        "Original Position (x, y, z): ({:.4}, {:.4}, {:.4})",
        pose.position.x, pose.position.y, pose.position.z
    );

    let link_lengths = table.get_current_link_lengths(&joints);

    let solver = UrtIkSolver;

    let result = solver.solve_ik(
        pose.position.x,
        pose.position.y,
        pose.position.z,
        &pose.rotation,
        &link_lengths,
    );

    let angles = result.unwrap();

    let mut check_joints = joints;
    for i in 0..6 {
        check_joints[i].set_position(angles[i].to_degrees());
    }

    let check_pose = table.get_frame_pose(7, &check_joints);
    println!(
        "Roundtrip Position (x, y, z): ({:.4}, {:.4}, {:.4})",
        check_pose.position.x, check_pose.position.y, check_pose.position.z
    );
    assert!(
        approx_eq(check_pose.position.x, pose.position.x),
        "position mismatch at x: origin={:.4}, roundtrip={:.4}",
        pose.position.x,
        check_pose.position.x
    );
    assert!(
        approx_eq(check_pose.position.y, pose.position.y),
        "position mismatch at y: origin={:.4}, roundtrip={:.4}",
        pose.position.y,
        check_pose.position.y
    );
    assert!(
        approx_eq(check_pose.position.z, pose.position.z),
        "position mismatch at z: origin={:.4}, roundtrip={:.4}",
        pose.position.z,
        check_pose.position.z
    );

    println!("Roundtrip Rotation Matrix (R):");
    println!(
        "        | {:.4} {:.4} {:.4} |",
        check_pose.rotation[(0, 0)],
        check_pose.rotation[(0, 1)],
        check_pose.rotation[(0, 2)]
    );
    println!(
        "        | {:.4} {:.4} {:.4} |",
        check_pose.rotation[(1, 0)],
        check_pose.rotation[(1, 1)],
        check_pose.rotation[(1, 2)]
    );
    println!(
        "        | {:.4} {:.4} {:.4} |",
        check_pose.rotation[(2, 0)],
        check_pose.rotation[(2, 1)],
        check_pose.rotation[(2, 2)]
    );
    for row in 0..3 {
        for col in 0..3 {
            assert!(
                approx_eq(check_pose.rotation[(row, col)], pose.rotation[(row, col)]),
                "rotation mismatch at ({}, {}): original={}, roundtrip={}",
                row,
                col,
                pose.rotation[(row, col)],
                check_pose.rotation[(row, col)]
            );
        }
    }
}

#[test]
fn ik_test1() {
    build_six_joints(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

#[test]
fn ik_test2() {
    build_six_joints(90.0, 90.0, 90.0, 90.0, 90.0, 90.0);
}

#[test]
fn ik_test3() {
    build_six_joints(90.0, 90.0, 0.0, 0.0, 0.0, 0.0);
}

#[test]
fn ik_test4() {
    build_six_joints(90.0, 90.0, 90.0, 0.0, 0.0, 0.0);
}
