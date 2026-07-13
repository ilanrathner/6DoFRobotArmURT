use bevy::input::mouse::{MouseMotion, MouseWheel};
use bevy::log::{Level, LogPlugin};
use bevy::prelude::*;
use bevy::render::mesh::PrimitiveTopology;
use bevy::render::render_asset::RenderAssetUsages;
use k::nalgebra::UnitQuaternion;
use k::InverseKinematicsSolver;
use std::collections::HashMap;
use std::env;
use std::fs;
use std::path::{Path, PathBuf};

const DEFAULT_TRIANGLE_CAP: usize = 25_000;
const JOINT_SPEED: f32 = 1.2;
const TARGET_MOVE_SPEED: f32 = 0.25;
const TARGET_ROTATE_SPEED: f32 = 1.2;
const TASK_TARGET_LINK: &str = "link6";
const DEFAULT_HOME_JOINTS: [f64; 6] = [-0.6, 2.5, 1.5, -1.2, -1.4, 2.0];
const DEFAULT_LIMIT: f32 = std::f32::consts::PI;
const KEY_PAIRS: [(KeyCode, KeyCode); 8] = [
    (KeyCode::KeyQ, KeyCode::KeyA),
    (KeyCode::KeyW, KeyCode::KeyS),
    (KeyCode::KeyE, KeyCode::KeyD),
    (KeyCode::KeyR, KeyCode::KeyF),
    (KeyCode::KeyT, KeyCode::KeyG),
    (KeyCode::KeyY, KeyCode::KeyH),
    (KeyCode::KeyU, KeyCode::KeyJ),
    (KeyCode::KeyI, KeyCode::KeyK),
];

#[derive(Clone)]
struct LinkSpec {
    name: String,
    visuals: Vec<VisualSpec>,
}

#[derive(Clone)]
struct VisualSpec {
    mesh_file: String,
    xyz: Vec3,
    rpy: Vec3,
    scale: Vec3,
    color: Color,
}

#[derive(Clone)]
struct JointSpec {
    name: String,
    joint_type: String,
    parent: String,
    child: String,
    origin_xyz: Vec3,
    origin_rpy: Vec3,
    axis: Vec3,
    lower: f32,
    upper: f32,
    increase_key: Option<KeyCode>,
    decrease_key: Option<KeyCode>,
}

struct RobotModel {
    links: Vec<LinkSpec>,
    joints: Vec<JointSpec>,
}

#[derive(Component)]
struct JointState {
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
struct OrbitCamera {
    target: Vec3,
    radius: f32,
    yaw: f32,
    pitch: f32,
}

#[derive(Component)]
struct TargetMarker;

#[derive(Resource)]
struct TaskSpaceControl {
    enabled: bool,
    target: Vec3,
    target_rpy: Vec3,
    target_orientation_enabled: bool,
    target_link: String,
}

#[derive(Resource)]
struct KinematicsState {
    chain: k::Chain<f64>,
    joint_names: Vec<String>,
}

#[derive(Resource)]
struct ViewerSettings {
    urdf_path: PathBuf,
    mesh_dir_override: Option<PathBuf>,
    triangle_cap: usize,
    initial_joint_values: HashMap<String, f32>,
    target_xyz: Option<Vec3>,
    target_rpy: Vec3,
    target_orientation_enabled: bool,
    dry_run: bool,
}

fn main() {
    let repo_root = repo_root();
    let settings = parse_viewer_settings(&repo_root);
    let model = parse_urdf(&settings.urdf_path).expect("failed to parse URDF");

    println!("Bevy URDF viewer");
    println!("URDF: {}", settings.urdf_path.display());
    if let Some(mesh_dir) = &settings.mesh_dir_override {
        println!("Mesh override dir: {}", mesh_dir.display());
    }
    if settings.triangle_cap == 0 {
        println!("Triangle cap per STL: disabled");
    } else {
        println!("Triangle cap per STL: {}", settings.triangle_cap);
    }
    if let Some(target_xyz) = settings.target_xyz {
        if settings.target_orientation_enabled {
            println!(
                "k IK target for link6: xyz=[{:.3}, {:.3}, {:.3}], rpy=[{:.3}, {:.3}, {:.3}]",
                target_xyz.x,
                target_xyz.y,
                target_xyz.z,
                settings.target_rpy.x,
                settings.target_rpy.y,
                settings.target_rpy.z
            );
        } else {
            println!(
                "k IK target for link6: xyz=[{:.3}, {:.3}, {:.3}], orientation=unconstrained",
                target_xyz.x, target_xyz.y, target_xyz.z
            );
        }
    }
    if !settings.initial_joint_values.is_empty() {
        println!("Initial joint values from k:");
        let mut values = settings.initial_joint_values.iter().collect::<Vec<_>>();
        values.sort_by(|left, right| left.0.cmp(right.0));
        for (name, value) in values {
            println!("  {name}: {value:.6} rad");
        }
    }

    println!("Moving joint controls:");
    for joint in model.joints.iter().filter(|joint| joint.is_moving()) {
        if let (Some(inc), Some(dec)) = (joint.increase_key, joint.decrease_key) {
            println!("  {:?}/{:?} {}", inc, dec, joint.name);
        }
    }
    println!("  M toggle task-space IK");
    println!("  Arrow keys move target X/Y, PageUp/PageDown move target Z");
    println!("  Z/X target roll, C/V target pitch, B/N target yaw");
    println!("  Space reset, mouse drag orbit, wheel zoom");
    if settings.dry_run {
        println!("Dry run complete; window was not opened.");
        return;
    }

    App::new()
        .insert_resource(ClearColor(Color::srgb(0.04, 0.045, 0.05)))
        .insert_resource(settings)
        .insert_resource(model_resource(model))
        .add_plugins(
            DefaultPlugins
                .set(WindowPlugin {
                    primary_window: Some(Window {
                        title: "URT Arm Bevy Viewer".to_string(),
                        resolution: (1280.0, 840.0).into(),
                        ..default()
                    }),
                    ..default()
                })
                .set(LogPlugin {
                    filter: "info,k::urdf=warn".to_string(),
                    level: Level::INFO,
                    ..default()
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (
                drive_task_space_target,
                drive_joints,
                orbit_camera,
                draw_joint_axes,
            ),
        )
        .run();
}

#[derive(Resource)]
struct RobotModelResource(RobotModel);

fn model_resource(model: RobotModel) -> RobotModelResource {
    RobotModelResource(model)
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    settings: Res<ViewerSettings>,
    model: Res<RobotModelResource>,
) {
    commands.spawn(PointLightBundle {
        point_light: PointLight {
            intensity: 3500.0,
            range: 8.0,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(0.4, -0.8, 1.2),
        ..default()
    });

    commands.spawn(DirectionalLightBundle {
        directional_light: DirectionalLight {
            illuminance: 2500.0,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -0.8, -0.4, -0.5)),
        ..default()
    });

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
            .spawn((SpatialBundle::default(), Name::new(link.name.clone())))
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
                parent.spawn(PbrBundle {
                    mesh: meshes.add(mesh),
                    material,
                    transform: Transform::from_translation(visual.xyz)
                        .with_rotation(rpy_quat(visual.rpy)),
                    ..default()
                });
            });
        }
    }

    let target = Vec3::new(0.03, 0.01, 0.18);
    commands.spawn((
        Camera3dBundle {
            transform: camera_transform(-0.7, -0.55, 1.8, target),
            ..default()
        },
        OrbitCamera {
            target,
            radius: 1.8,
            yaw: -0.7,
            pitch: -0.55,
        },
    ));
}

impl JointSpec {
    fn is_moving(&self) -> bool {
        self.joint_type == "revolute" || self.joint_type == "continuous"
    }
}

fn drive_joints(
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
            delta += JOINT_SPEED * time.delta_seconds();
        }
        if keyboard.pressed(joint.decrease_key) {
            delta -= JOINT_SPEED * time.delta_seconds();
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

fn drive_task_space_target(
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
        task_control.target += direction.normalize() * TARGET_MOVE_SPEED * time.delta_seconds();
    }
    if rotated {
        task_control.target_orientation_enabled = true;
        task_control.target_rpy +=
            rotation_delta.normalize() * TARGET_ROTATE_SPEED * time.delta_seconds();
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
fn orbit_camera(
    mut mouse_motion: EventReader<MouseMotion>,
    mut mouse_wheel: EventReader<MouseWheel>,
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

fn draw_joint_axes(mut gizmos: Gizmos, joints: Query<(&JointState, &GlobalTransform)>) {
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
    commands.spawn(PbrBundle {
        mesh: meshes.add(Cuboid::new(3.0, 3.0, 0.01)),
        material: materials.add(StandardMaterial {
            base_color: Color::srgb(0.12, 0.13, 0.13),
            perceptual_roughness: 0.9,
            ..default()
        }),
        transform: Transform::from_xyz(0.0, 0.0, -0.008),
        ..default()
    });
}

fn spawn_target_marker(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
    target: Vec3,
    target_rpy: Vec3,
) {
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Cuboid::new(0.035, 0.035, 0.035)),
            material: materials.add(StandardMaterial {
                base_color: Color::srgb(1.0, 0.2, 0.15),
                emissive: Color::srgb(0.45, 0.04, 0.02).into(),
                ..default()
            }),
            transform: Transform::from_translation(target).with_rotation(rpy_quat(target_rpy)),
            ..default()
        },
        TargetMarker,
        Name::new("task_space_target"),
    ));
}
fn load_binary_stl_mesh(path: &Path, scale: Vec3, triangle_cap: usize) -> Result<Mesh, String> {
    let bytes = fs::read(path).map_err(|error| error.to_string())?;
    if bytes.len() < 84 {
        return Err("file is too small to be a binary STL".to_string());
    }

    let triangle_count = u32::from_le_bytes(bytes[80..84].try_into().unwrap()) as usize;
    let expected_len = 84 + triangle_count * 50;
    if expected_len > bytes.len() {
        return Err(format!(
            "binary STL length mismatch: expected at least {expected_len}, got {}",
            bytes.len()
        ));
    }

    let stride = if triangle_cap == 0 {
        1
    } else {
        triangle_count.div_ceil(triangle_cap).max(1)
    };

    let sampled_count = triangle_count.div_ceil(stride);
    let mut positions = Vec::with_capacity(sampled_count * 3);
    let mut normals = Vec::with_capacity(sampled_count * 3);
    let mut uvs = Vec::with_capacity(sampled_count * 3);

    for triangle_index in (0..triangle_count).step_by(stride) {
        let offset = 84 + triangle_index * 50;
        let file_normal = read_vec3(&bytes, offset).normalize_or_zero();
        let vertices = [
            read_vec3(&bytes, offset + 12) * scale,
            read_vec3(&bytes, offset + 24) * scale,
            read_vec3(&bytes, offset + 36) * scale,
        ];
        let normal = if file_normal.length_squared() > 1.0e-10 {
            file_normal
        } else {
            (vertices[1] - vertices[0])
                .cross(vertices[2] - vertices[0])
                .normalize_or_zero()
        };

        for vertex in vertices {
            positions.push([vertex.x, vertex.y, vertex.z]);
            normals.push([normal.x, normal.y, normal.z]);
            uvs.push([0.0, 0.0]);
        }
    }

    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);
    Ok(mesh)
}

fn read_vec3(bytes: &[u8], offset: usize) -> Vec3 {
    Vec3::new(
        read_f32(bytes, offset),
        read_f32(bytes, offset + 4),
        read_f32(bytes, offset + 8),
    )
}

fn read_f32(bytes: &[u8], offset: usize) -> f32 {
    f32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap())
}

fn rpy_quat(rpy: Vec3) -> Quat {
    Quat::from_rotation_z(rpy.z) * Quat::from_rotation_y(rpy.y) * Quat::from_rotation_x(rpy.x)
}

fn rpy_quat_f64(rpy: Vec3) -> UnitQuaternion<f64> {
    UnitQuaternion::from_euler_angles(rpy.x as f64, rpy.y as f64, rpy.z as f64)
}

fn parse_viewer_settings(repo_root: &Path) -> ViewerSettings {
    let mut triangle_cap = None;
    let mut mesh_dir_override = None;
    let mut urdf_path = None;
    let mut joints_arg = None;
    let mut target_xyz_arg = None;
    let mut target_rpy_arg = None;
    let mut dry_run = false;

    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        if let Some(value) = arg.strip_prefix("--joints=") {
            joints_arg = Some(value.to_string());
            continue;
        }
        if let Some(value) = arg.strip_prefix("--target-xyz=") {
            target_xyz_arg = Some(value.to_string());
            continue;
        }
        if let Some(value) = arg.strip_prefix("--target-rpy=") {
            target_rpy_arg = Some(value.to_string());
            continue;
        }

        match arg.as_str() {
            "--dry-run" => {
                dry_run = true;
            }
            "--tri-cap" => {
                if let Some(value) = args.next() {
                    if let Ok(parsed) = value.parse() {
                        triangle_cap = Some(parsed);
                    }
                }
            }
            "--mesh-dir" => {
                if let Some(value) = args.next() {
                    mesh_dir_override = Some(PathBuf::from(value));
                }
            }
            "--urdf" => {
                if let Some(value) = args.next() {
                    urdf_path = Some(PathBuf::from(value));
                }
            }
            "--joints" => {
                if let Some(value) = args.next() {
                    joints_arg = Some(value);
                }
            }
            "--target-xyz" => {
                if let Some(value) = args.next() {
                    target_xyz_arg = Some(value);
                }
            }
            "--target-rpy" => {
                if let Some(value) = args.next() {
                    target_rpy_arg = Some(value);
                }
            }
            _ => {}
        }
    }

    let urdf_path = resolve_repo_path(
        repo_root,
        urdf_path.unwrap_or_else(|| {
            PathBuf::from("assets")
                .join("urdf")
                .join("urdf_assembly_rigid_stl_collapsed.urdf")
        }),
    );
    let mesh_dir_override = mesh_dir_override.map(|path| resolve_repo_path(repo_root, path));
    let target_xyz = target_xyz_arg
        .as_deref()
        .map(parse_csv_vec3)
        .transpose()
        .expect("--target-xyz must be three comma-separated numbers");
    let target_rpy = target_rpy_arg
        .as_deref()
        .map(parse_csv_vec3)
        .transpose()
        .expect("--target-rpy must be three comma-separated numbers");
    let target_orientation_enabled = target_rpy.is_some();
    let target_rpy = target_rpy.unwrap_or(Vec3::ZERO);
    let initial_joint_values = compute_initial_joint_values(
        &urdf_path,
        joints_arg.as_deref(),
        target_xyz,
        target_orientation_enabled.then_some(target_rpy),
    )
    .expect("failed to calculate initial joint values with k");

    ViewerSettings {
        urdf_path,
        mesh_dir_override,
        triangle_cap: triangle_cap.unwrap_or(DEFAULT_TRIANGLE_CAP),
        initial_joint_values,
        target_xyz,
        target_rpy,
        target_orientation_enabled,
        dry_run,
    }
}

fn resolve_repo_path(repo_root: &Path, path: PathBuf) -> PathBuf {
    if path.is_absolute() {
        path
    } else {
        repo_root.join(path)
    }
}

fn parse_csv_f32(value: &str) -> Result<Vec<f32>, String> {
    value
        .split(',')
        .map(|part| {
            part.trim()
                .parse::<f32>()
                .map_err(|error| format!("failed to parse '{part}' as a number: {error}"))
        })
        .collect()
}

fn parse_csv_vec3(value: &str) -> Result<Vec3, String> {
    let values = parse_csv_f32(value)?;
    if values.len() != 3 {
        return Err(format!(
            "expected three comma-separated values, got {}",
            values.len()
        ));
    }
    Ok(Vec3::new(values[0], values[1], values[2]))
}

fn compute_initial_joint_values(
    urdf_path: &Path,
    joints_arg: Option<&str>,
    target_xyz: Option<Vec3>,
    target_rpy: Option<Vec3>,
) -> Result<HashMap<String, f32>, String> {
    let chain = k::Chain::<f64>::from_urdf_file(urdf_path)
        .map_err(|error| format!("failed to load URDF with k: {error}"))?;
    let joint_names = chain
        .iter_joints()
        .map(|joint| joint.name.clone())
        .collect::<Vec<_>>();
    let initial_positions = if let Some(joints_arg) = joints_arg {
        parse_csv_f32(joints_arg)?
            .into_iter()
            .map(|value| value as f64)
            .collect::<Vec<_>>()
    } else if chain.dof() == DEFAULT_HOME_JOINTS.len() {
        DEFAULT_HOME_JOINTS.to_vec()
    } else {
        vec![0.0; chain.dof()]
    };

    if initial_positions.len() != chain.dof() {
        return Err(format!(
            "expected {} joint values, got {}",
            chain.dof(),
            initial_positions.len()
        ));
    }

    chain
        .set_joint_positions(&initial_positions)
        .map_err(|error| format!("failed to set joint positions: {error}"))?;
    chain.update_transforms();

    if let Some(target_xyz) = target_xyz {
        solve_task_space_ik(&chain, TASK_TARGET_LINK, target_xyz, target_rpy)?;
    }

    Ok(joint_names
        .into_iter()
        .zip(
            chain
                .joint_positions()
                .into_iter()
                .map(|value| value as f32),
        )
        .collect())
}

fn create_kinematics_state(urdf_path: &Path) -> Result<KinematicsState, String> {
    let chain = k::Chain::<f64>::from_urdf_file(urdf_path)
        .map_err(|error| format!("failed to load URDF with k: {error}"))?;
    let joint_names = chain
        .iter_joints()
        .map(|joint| joint.name.clone())
        .collect::<Vec<_>>();
    Ok(KinematicsState { chain, joint_names })
}

fn link_position_from_joint_values(
    kinematics: &mut KinematicsState,
    joint_values: &HashMap<String, f32>,
    target_link: &str,
) -> Result<Vec3, String> {
    let positions = kinematics
        .joint_names
        .iter()
        .map(|name| joint_values.get(name).copied().unwrap_or(0.0) as f64)
        .collect::<Vec<_>>();
    kinematics
        .chain
        .set_joint_positions(&positions)
        .map_err(|error| format!("failed to set joint positions: {error}"))?;
    kinematics.chain.update_transforms();

    let target_node = kinematics
        .chain
        .find_link(target_link)
        .ok_or_else(|| format!("target link '{target_link}' was not found in the URDF"))?;
    let target_transform = target_node
        .world_transform()
        .ok_or_else(|| format!("target link '{target_link}' has no world transform"))?;
    let translation = target_transform.translation.vector;
    Ok(Vec3::new(
        translation.x as f32,
        translation.y as f32,
        translation.z as f32,
    ))
}

fn solve_task_space_ik_values(
    kinematics: &mut KinematicsState,
    current_values: &HashMap<String, f32>,
    target_link: &str,
    target_xyz: Vec3,
    target_rpy: Option<Vec3>,
) -> Result<HashMap<String, f32>, String> {
    let positions = kinematics
        .joint_names
        .iter()
        .map(|name| current_values.get(name).copied().unwrap_or(0.0) as f64)
        .collect::<Vec<_>>();

    kinematics
        .chain
        .set_joint_positions(&positions)
        .map_err(|error| format!("failed to set joint positions: {error}"))?;
    kinematics.chain.update_transforms();
    solve_task_space_ik(&kinematics.chain, target_link, target_xyz, target_rpy)?;

    Ok(kinematics
        .joint_names
        .iter()
        .cloned()
        .zip(
            kinematics
                .chain
                .joint_positions()
                .into_iter()
                .map(|value| value as f32),
        )
        .collect())
}

fn solve_task_space_ik(
    chain: &k::Chain<f64>,
    target_link: &str,
    target_xyz: Vec3,
    target_rpy: Option<Vec3>,
) -> Result<(), String> {
    let target_node = chain
        .find_link(target_link)
        .ok_or_else(|| format!("target link '{target_link}' was not found in the URDF"))?;
    let mut target_transform = target_node
        .world_transform()
        .ok_or_else(|| format!("target link '{target_link}' has no world transform"))?;
    target_transform.translation.vector.x = target_xyz.x as f64;
    target_transform.translation.vector.y = target_xyz.y as f64;
    target_transform.translation.vector.z = target_xyz.z as f64;
    if let Some(target_rpy) = target_rpy {
        target_transform.rotation = rpy_quat_f64(target_rpy);
    }

    let arm = k::SerialChain::from_end(target_node);
    let solver = k::JacobianIkSolver::default();

    solver
        .solve(&arm, &target_transform)
        .map_err(|error| format!("full-pose IK solver failed: {error}"))?;
    chain.update_transforms();
    Ok(())
}
fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

fn resolve_mesh_path(
    urdf_dir: &Path,
    mesh_dir_override: Option<&Path>,
    mesh_file: &str,
) -> PathBuf {
    let mesh_path = Path::new(mesh_file);
    if mesh_path.is_absolute() {
        return mesh_path.to_path_buf();
    }

    if let Some(mesh_dir) = mesh_dir_override {
        return mesh_dir.join(mesh_path.file_name().unwrap_or_default());
    }

    urdf_dir.join(mesh_path)
}

fn parse_urdf(path: &Path) -> Result<RobotModel, String> {
    let xml = fs::read_to_string(path).map_err(|error| error.to_string())?;
    let mut links = Vec::new();
    let mut joints = Vec::new();

    for block in tag_blocks(&xml, "link") {
        let Some(name) = attr_value(&block.start_tag, "name") else {
            continue;
        };
        let mut visuals = Vec::new();
        for visual_block in tag_blocks(&block.body, "visual") {
            let Some(mesh_tag) = first_tag(&visual_block.body, "mesh") else {
                continue;
            };
            let Some(mesh_file) = attr_value(&mesh_tag, "filename") else {
                continue;
            };

            let origin_tag = first_tag(&visual_block.body, "origin").unwrap_or_default();
            let color_tag = first_tag(&visual_block.body, "color").unwrap_or_default();
            let scale = attr_value(&mesh_tag, "scale")
                .map(|value| parse_vec3(&value, Vec3::ONE))
                .unwrap_or(Vec3::ONE);
            let color = attr_value(&color_tag, "rgba")
                .map(|value| parse_color(&value))
                .unwrap_or(Color::srgb(0.7, 0.7, 0.7));

            visuals.push(VisualSpec {
                mesh_file,
                xyz: attr_value(&origin_tag, "xyz")
                    .map(|value| parse_vec3(&value, Vec3::ZERO))
                    .unwrap_or(Vec3::ZERO),
                rpy: attr_value(&origin_tag, "rpy")
                    .map(|value| parse_vec3(&value, Vec3::ZERO))
                    .unwrap_or(Vec3::ZERO),
                scale,
                color,
            });
        }
        links.push(LinkSpec { name, visuals });
    }

    let mut moving_index = 0usize;
    for block in tag_blocks(&xml, "joint") {
        let Some(name) = attr_value(&block.start_tag, "name") else {
            continue;
        };
        let joint_type =
            attr_value(&block.start_tag, "type").unwrap_or_else(|| "fixed".to_string());
        let parent = first_tag(&block.body, "parent")
            .and_then(|tag| attr_value(&tag, "link"))
            .unwrap_or_default();
        let child = first_tag(&block.body, "child")
            .and_then(|tag| attr_value(&tag, "link"))
            .unwrap_or_default();
        let origin_tag = first_tag(&block.body, "origin").unwrap_or_default();
        let axis_tag = first_tag(&block.body, "axis").unwrap_or_default();
        let limit_tag = first_tag(&block.body, "limit").unwrap_or_default();

        let is_moving = joint_type == "revolute" || joint_type == "continuous";
        let (increase_key, decrease_key) = if is_moving {
            let pair = KEY_PAIRS.get(moving_index).copied();
            moving_index += 1;
            pair.map(|(increase, decrease)| (Some(increase), Some(decrease)))
                .unwrap_or((None, None))
        } else {
            (None, None)
        };

        let (mut lower, mut upper) = if joint_type == "continuous" {
            (f32::NEG_INFINITY, f32::INFINITY)
        } else {
            (
                attr_value(&limit_tag, "lower")
                    .and_then(|value| value.parse::<f32>().ok())
                    .unwrap_or(-DEFAULT_LIMIT),
                attr_value(&limit_tag, "upper")
                    .and_then(|value| value.parse::<f32>().ok())
                    .unwrap_or(DEFAULT_LIMIT),
            )
        };
        if lower >= upper {
            lower = -DEFAULT_LIMIT;
            upper = DEFAULT_LIMIT;
        }
        joints.push(JointSpec {
            name,
            joint_type,
            parent,
            child,
            origin_xyz: attr_value(&origin_tag, "xyz")
                .map(|value| parse_vec3(&value, Vec3::ZERO))
                .unwrap_or(Vec3::ZERO),
            origin_rpy: attr_value(&origin_tag, "rpy")
                .map(|value| parse_vec3(&value, Vec3::ZERO))
                .unwrap_or(Vec3::ZERO),
            axis: attr_value(&axis_tag, "xyz")
                .map(|value| parse_vec3(&value, Vec3::Z))
                .unwrap_or(Vec3::Z),
            lower,
            upper,
            increase_key,
            decrease_key,
        });
    }

    Ok(RobotModel { links, joints })
}

struct TagBlock {
    start_tag: String,
    body: String,
}

fn tag_blocks(xml: &str, tag: &str) -> Vec<TagBlock> {
    let mut blocks = Vec::new();
    let open_pattern = format!("<{tag}");
    let close_pattern = format!("</{tag}>");
    let mut search_from = 0usize;

    while let Some(open_rel) = xml[search_from..].find(&open_pattern) {
        let open = search_from + open_rel;
        let Some(start_end_rel) = xml[open..].find('>') else {
            break;
        };
        let start_end = open + start_end_rel + 1;
        let start_tag = xml[open..start_end].to_string();

        if start_tag.trim_end().ends_with("/>") {
            blocks.push(TagBlock {
                start_tag,
                body: String::new(),
            });
            search_from = start_end;
            continue;
        }

        let Some(close_rel) = xml[start_end..].find(&close_pattern) else {
            break;
        };
        let close = start_end + close_rel;
        let close_end = close + close_pattern.len();
        blocks.push(TagBlock {
            start_tag,
            body: xml[start_end..close].to_string(),
        });
        search_from = close_end;
    }

    blocks
}

fn first_tag(xml: &str, tag: &str) -> Option<String> {
    let open_pattern = format!("<{tag}");
    let open = xml.find(&open_pattern)?;
    let start_end = open + xml[open..].find('>')? + 1;
    Some(xml[open..start_end].to_string())
}

fn attr_value(tag: &str, attr: &str) -> Option<String> {
    let pattern = format!("{attr}=\"");
    let value_start = tag.find(&pattern)? + pattern.len();
    let value_end = value_start + tag[value_start..].find('"')?;
    Some(tag[value_start..value_end].to_string())
}

fn parse_vec3(value: &str, fallback: Vec3) -> Vec3 {
    let mut parts = value
        .split_whitespace()
        .filter_map(|part| part.parse::<f32>().ok());
    let Some(x) = parts.next() else {
        return fallback;
    };
    let Some(y) = parts.next() else {
        return fallback;
    };
    let Some(z) = parts.next() else {
        return fallback;
    };
    Vec3::new(x, y, z)
}

fn parse_color(value: &str) -> Color {
    let mut parts = value
        .split_whitespace()
        .filter_map(|part| part.parse::<f32>().ok());
    let r = parts.next().unwrap_or(0.7);
    let g = parts.next().unwrap_or(0.7);
    let b = parts.next().unwrap_or(0.7);
    let a = parts.next().unwrap_or(1.0);
    Color::srgba(r, g, b, a)
}
