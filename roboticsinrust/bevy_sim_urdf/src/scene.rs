use bevy::input::mouse::{MouseMotion, MouseWheel};
use bevy::prelude::*;
use std::collections::HashMap;

use crate::constants::{JOINT_SPEED, TARGET_MOVE_SPEED, TARGET_ROTATE_SPEED, TASK_TARGET_LINK};
use crate::kinematics::{
    KinematicsState, create_kinematics_state, link_position_from_joint_values,
    solve_task_space_ik_values,
};
use crate::mesh::load_binary_stl_mesh;
use crate::model::RobotModelResource;
use crate::settings::ViewerSettings;
use crate::urdf::resolve_mesh_path;

#[derive(Component)]
pub(crate) struct JointState {
    name: String,
    origin_xyz: Vec3,
    origin_rotation: Quat,
    axis: Vec3,
    value: f32,
    lower: f32,
    upper: f32,
    increase_key: KeyCode,
    decrease_key: KeyCode,
}

#[derive(Component)]
pub(crate) struct OrbitCamera {
    target: Vec3,
    radius: f32,
    yaw: f32,
    pitch: f32,
}

#[derive(Component)]
pub(crate) struct TargetMarker;

#[derive(Resource)]
pub(crate) struct TaskSpaceControl {
    enabled: bool,
    target: Vec3,
    target_rpy: Vec3,
    target_orientation_enabled: bool,
    target_link: String,
}

pub(crate) fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    settings: Res<ViewerSettings>,
    model: Res<RobotModelResource>,
) {
    commands.spawn((
        PointLight {
            intensity: 3500.0,
            range: 8.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(0.4, -0.8, 1.2),
    ));

    commands.spawn((
        DirectionalLight {
            illuminance: 2500.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -0.8, -0.4, -0.5)),
    ));

    spawn_ground(&mut commands, &mut meshes, &mut materials);

    let mut kinematics = create_kinematics_state(&settings.urdf_path)
        .expect("failed to initialize k kinematics state");
    let task_target = settings.target_xyz.unwrap_or_else(|| {
        link_position_from_joint_values(
            &mut kinematics,
            &settings.initial_joint_values,
            TASK_TARGET_LINK,
        )
        .unwrap_or(Vec3::new(-0.20, 0.30, 0.28))
    });
    commands.insert_resource(TaskSpaceControl {
        enabled: settings.target_xyz.is_some(),
        target: task_target,
        target_rpy: settings.target_rpy,
        target_orientation_enabled: settings.target_orientation_enabled,
        target_link: TASK_TARGET_LINK.to_string(),
    });
    commands.insert_resource(kinematics);
    spawn_target_marker(
        &mut commands,
        &mut meshes,
        &mut materials,
        task_target,
        settings.target_rpy,
    );

    let mut link_entities = HashMap::new();
    for link in &model.0.links {
        let entity = commands
            .spawn((Transform::default(), Name::new(link.name.clone())))
            .id();
        link_entities.insert(link.name.clone(), entity);
    }

    let moving_joints = model
        .0
        .joints
        .iter()
        .filter(|joint| joint.is_moving())
        .count();
    println!(
        "Loaded {} links, {} joints, {} moving joints",
        model.0.links.len(),
        model.0.joints.len(),
        moving_joints
    );

    for joint in &model.0.joints {
        let Some(parent) = link_entities.get(&joint.parent).copied() else {
            warn!(
                "joint {} references missing parent {}",
                joint.name, joint.parent
            );
            continue;
        };
        let Some(child) = link_entities.get(&joint.child).copied() else {
            warn!(
                "joint {} references missing child {}",
                joint.name, joint.child
            );
            continue;
        };

        let origin_rotation = rpy_quat(joint.origin_rpy);
        let axis = joint.axis.normalize_or_zero();
        let initial_value = settings
            .initial_joint_values
            .get(&joint.name)
            .copied()
            .unwrap_or(0.0)
            .clamp(joint.lower, joint.upper);
        let joint_rotation = if joint.is_moving() {
            Quat::from_axis_angle(axis, initial_value)
        } else {
            Quat::IDENTITY
        };
        commands.entity(child).insert(
            Transform::from_translation(joint.origin_xyz)
                .with_rotation(origin_rotation * joint_rotation),
        );

        if joint.is_moving() {
            if let (Some(increase_key), Some(decrease_key)) =
                (joint.increase_key, joint.decrease_key)
            {
                commands.entity(child).insert(JointState {
                    name: joint.name.clone(),
                    origin_xyz: joint.origin_xyz,
                    origin_rotation,
                    axis,
                    value: initial_value,
                    lower: joint.lower,
                    upper: joint.upper,
                    increase_key,
                    decrease_key,
                });
            }
        }

        commands.entity(parent).add_child(child);
    }

    let urdf_dir = settings
        .urdf_path
        .parent()
        .expect("URDF should have a parent directory");

    for link in &model.0.links {
        let Some(entity) = link_entities.get(&link.name).copied() else {
            continue;
        };

        for visual in &link.visuals {
            let mesh_path = resolve_mesh_path(
                urdf_dir,
                settings.mesh_dir_override.as_deref(),
                &visual.mesh_file,
            );
            let mesh = load_binary_stl_mesh(&mesh_path, visual.scale, settings.triangle_cap)
                .unwrap_or_else(|error| panic!("failed to load {}: {error}", mesh_path.display()));
            let material = materials.add(StandardMaterial {
                base_color: visual.color,
                perceptual_roughness: 0.65,
                metallic: 0.05,
                cull_mode: None,
                ..default()
            });

            commands.entity(entity).with_children(|parent| {
                parent.spawn((
                    Mesh3d(meshes.add(mesh)),
                    MeshMaterial3d(material),
                    Transform::from_translation(visual.xyz).with_rotation(rpy_quat(visual.rpy)),
                ));
            });
        }
    }

    let target = Vec3::new(0.03, 0.01, 0.18);
    commands.spawn((
        Camera3d::default(),
        camera_transform(-0.7, -0.55, 1.8, target),
        OrbitCamera {
            target,
            radius: 1.8,
            yaw: -0.7,
            pitch: -0.55,
        },
    ));
}

pub(crate) fn drive_joints(
    time: Res<Time>,
    keyboard: Res<ButtonInput<KeyCode>>,
    task_control: Option<Res<TaskSpaceControl>>,
    mut joints: Query<(&mut JointState, &mut Transform)>,
) {
    if let Some(task_control) = task_control {
        if task_control.enabled {
            return;
        }
    }
    for (mut joint, mut transform) in &mut joints {
        let mut delta = 0.0;
        if keyboard.pressed(joint.increase_key) {
            delta += JOINT_SPEED * time.delta_secs();
        }
        if keyboard.pressed(joint.decrease_key) {
            delta -= JOINT_SPEED * time.delta_secs();
        }
        if keyboard.just_pressed(KeyCode::Space) {
            joint.value = 0.0;
        } else if delta != 0.0 {
            joint.value = (joint.value + delta).clamp(joint.lower, joint.upper);
            println!("{}: {:.3} rad", joint.name, joint.value);
        }

        transform.translation = joint.origin_xyz;
        transform.rotation = joint.origin_rotation * Quat::from_axis_angle(joint.axis, joint.value);
    }
}

pub(crate) fn drive_task_space_target(
    time: Res<Time>,
    keyboard: Res<ButtonInput<KeyCode>>,
    mut task_control: ResMut<TaskSpaceControl>,
    mut kinematics: ResMut<KinematicsState>,
    mut marker: Query<&mut Transform, (With<TargetMarker>, Without<JointState>)>,
    mut joints: Query<(&mut JointState, &mut Transform), Without<TargetMarker>>,
) {
    if keyboard.just_pressed(KeyCode::KeyM) {
        task_control.enabled = !task_control.enabled;
        println!(
            "task-space IK: {}",
            if task_control.enabled {
                "enabled"
            } else {
                "disabled"
            }
        );
    }

    if !task_control.enabled {
        return;
    }

    let mut direction = Vec3::ZERO;
    if keyboard.pressed(KeyCode::ArrowRight) {
        direction.x += 1.0;
    }
    if keyboard.pressed(KeyCode::ArrowLeft) {
        direction.x -= 1.0;
    }
    if keyboard.pressed(KeyCode::ArrowUp) {
        direction.y += 1.0;
    }
    if keyboard.pressed(KeyCode::ArrowDown) {
        direction.y -= 1.0;
    }
    if keyboard.pressed(KeyCode::PageUp) {
        direction.z += 1.0;
    }
    if keyboard.pressed(KeyCode::PageDown) {
        direction.z -= 1.0;
    }

    let mut rotation_delta = Vec3::ZERO;
    if keyboard.pressed(KeyCode::KeyZ) {
        rotation_delta.x += 1.0;
    }
    if keyboard.pressed(KeyCode::KeyX) {
        rotation_delta.x -= 1.0;
    }
    if keyboard.pressed(KeyCode::KeyC) {
        rotation_delta.y += 1.0;
    }
    if keyboard.pressed(KeyCode::KeyV) {
        rotation_delta.y -= 1.0;
    }
    if keyboard.pressed(KeyCode::KeyB) {
        rotation_delta.z += 1.0;
    }
    if keyboard.pressed(KeyCode::KeyN) {
        rotation_delta.z -= 1.0;
    }

    let moved = direction.length_squared() > 0.0;
    let rotated = rotation_delta.length_squared() > 0.0;
    if moved {
        task_control.target += direction.normalize() * TARGET_MOVE_SPEED * time.delta_secs();
    }
    if rotated {
        task_control.target_orientation_enabled = true;
        task_control.target_rpy +=
            rotation_delta.normalize() * TARGET_ROTATE_SPEED * time.delta_secs();
    }
    for mut transform in &mut marker {
        transform.translation = task_control.target;
        transform.rotation = rpy_quat(task_control.target_rpy);
    }

    if !(moved || rotated) {
        return;
    }

    let current_values = joints
        .iter_mut()
        .map(|(joint, _)| (joint.name.clone(), joint.value))
        .collect::<HashMap<_, _>>();

    match solve_task_space_ik_values(
        &mut kinematics,
        &current_values,
        &task_control.target_link,
        task_control.target,
        task_control
            .target_orientation_enabled
            .then_some(task_control.target_rpy),
    ) {
        Ok(values) => {
            for (mut joint, mut transform) in &mut joints {
                if let Some(value) = values.get(&joint.name) {
                    joint.value = value.clamp(joint.lower, joint.upper);
                    transform.translation = joint.origin_xyz;
                    transform.rotation =
                        joint.origin_rotation * Quat::from_axis_angle(joint.axis, joint.value);
                }
            }
        }
        Err(_error) => {
            warn!(
                "task-space IK did not converge; target may be unreachable or near a singularity"
            );
        }
    }
}

pub(crate) fn orbit_camera(
    mut mouse_motion: MessageReader<MouseMotion>,
    mut mouse_wheel: MessageReader<MouseWheel>,
    mouse_buttons: Res<ButtonInput<MouseButton>>,
    mut cameras: Query<(&mut OrbitCamera, &mut Transform)>,
) {
    let mut orbit_delta = Vec2::ZERO;
    if mouse_buttons.pressed(MouseButton::Left) || mouse_buttons.pressed(MouseButton::Right) {
        for event in mouse_motion.read() {
            orbit_delta += event.delta;
        }
    }

    let mut zoom_delta = 0.0;
    for event in mouse_wheel.read() {
        zoom_delta += event.y;
    }

    for (mut camera, mut transform) in &mut cameras {
        if orbit_delta != Vec2::ZERO {
            camera.yaw -= orbit_delta.x * 0.006;
            camera.pitch = (camera.pitch - orbit_delta.y * 0.006).clamp(-1.35, 1.35);
        }
        if zoom_delta != 0.0 {
            camera.radius = (camera.radius * (1.0 - zoom_delta * 0.08)).clamp(0.2, 8.0);
        }
        *transform = camera_transform(camera.yaw, camera.pitch, camera.radius, camera.target);
    }
}

pub(crate) fn draw_joint_axes(mut gizmos: Gizmos, joints: Query<(&JointState, &GlobalTransform)>) {
    for (joint, global_transform) in &joints {
        let transform = global_transform.compute_transform();
        let origin = transform.translation;
        let axis = (transform.rotation * joint.axis).normalize_or_zero();
        let color =
            if joint.axis.x.abs() > joint.axis.y.abs() && joint.axis.x.abs() > joint.axis.z.abs() {
                Color::srgb(1.0, 0.15, 0.15)
            } else if joint.axis.y.abs() > joint.axis.z.abs() {
                Color::srgb(0.2, 1.0, 0.25)
            } else {
                Color::srgb(0.25, 0.45, 1.0)
            };

        gizmos.line(origin - axis * 0.08, origin + axis * 0.08, color);
    }
}

fn camera_transform(yaw: f32, pitch: f32, radius: f32, target: Vec3) -> Transform {
    let direction = Vec3::new(
        yaw.cos() * pitch.cos(),
        yaw.sin() * pitch.cos(),
        pitch.sin(),
    );
    Transform::from_translation(target + direction * radius).looking_at(target, Vec3::Z)
}

fn spawn_ground(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(3.0, 3.0, 0.01))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.12, 0.13, 0.13),
            perceptual_roughness: 0.9,
            ..default()
        })),
        Transform::from_xyz(0.0, 0.0, -0.008),
    ));
}

fn spawn_target_marker(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
    target: Vec3,
    target_rpy: Vec3,
) {
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(0.035, 0.035, 0.035))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(1.0, 0.2, 0.15),
            emissive: Color::srgb(0.45, 0.04, 0.02).into(),
            ..default()
        })),
        Transform::from_translation(target).with_rotation(rpy_quat(target_rpy)),
        TargetMarker,
        Name::new("task_space_target"),
    ));
}

fn rpy_quat(rpy: Vec3) -> Quat {
    Quat::from_rotation_z(rpy.z) * Quat::from_rotation_y(rpy.y) * Quat::from_rotation_x(rpy.x)
}
