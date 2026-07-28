mod constants;
mod kinematics;
mod mesh;
mod model;
mod scene;
mod settings;
mod urdf;

use bevy::log::{Level, LogPlugin};
use bevy::prelude::*;
use std::env;
use std::path::PathBuf;

use model::model_resource;
use scene::{draw_joint_axes, drive_joints, drive_task_space_target, orbit_camera, setup};
use settings::parse_viewer_settings;
use urdf::parse_urdf;

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
                        resolution: (1280, 840).into(),
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

fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}
