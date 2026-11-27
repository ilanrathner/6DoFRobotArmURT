use std::collections::VecDeque;

use bevy::prelude::*;

use crate::scene::{JointState, TaskSpaceControl};

const MAX_LOG_LINES: usize = 10;

#[derive(Resource)]
pub(crate) struct UiFont(pub(crate) Handle<Font>);

#[derive(Resource)]
pub(crate) struct MonoFont(pub(crate) Handle<Font>);

#[derive(Resource, Default)]
pub(crate) struct LogBuffer {
    lines: VecDeque<String>,
}

#[derive(Component)]
pub(crate) struct JointAnglesText;

#[derive(Component)]
pub(crate) struct JointSliderThumb {
    joint_name: String,
}

#[derive(Component)]
pub(crate) struct LogPanelText;

pub(crate) fn load_ui_font(mut commands: Commands, asset_server: Res<AssetServer>) {
    let font = asset_server.load("fonts/Inter.ttf");
    commands.insert_resource(UiFont(font));

    let mono_font = asset_server.load("fonts/SpaceMono-Regular.ttf");
    commands.insert_resource(MonoFont(mono_font));
}

pub(crate) fn spawn_joint_angles_ui(
    commands: &mut Commands,
    mono_font: &MonoFont,
    joint_names: &[String],
) {
    commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                left: px(20),
                top: px(20),
                width: px(520),
                padding: UiRect::all(px(16)),
                flex_direction: FlexDirection::Row,
                column_gap: px(16),
                ..default()
            },
            BackgroundColor(Color::srgba(0.02, 0.025, 0.03, 0.9)),
        ))
        .with_children(|parent| {
            // parent.spawn((
            //     Text::new("JOINT POSITIONS"),
            //     TextFont {
            //         font_size: 22.0,
            //         ..default()
            //     },
            // ));
            parent.spawn((
                Node {
                    width: px(280),
                    ..default()
                },
                Text::new("J1   0.00°\nJ2   0.00°\nJ3   0.00°\nJ4   0.00°\nJ5   0.00°\nJ6   0.00°"),
                TextFont {
                    font: mono_font.0.clone(),
                    font_size: 18.0,
                    ..default()
                },
                JointAnglesText,
            ));
            parent
                .spawn(Node {
                    flex_direction: FlexDirection::Column,
                    row_gap: px(12),
                    ..default()
                })
                .with_children(|slider_col| {
                    for joint_name in joint_names {
                        slider_col
                            .spawn((
                                Node {
                                    top: px(37),
                                    width: px(220),
                                    height: px(10),
                                    position_type: PositionType::Relative,
                                    ..default()
                                },
                                BackgroundColor(Color::srgba(0.15, 0.15, 0.18, 1.0)),
                            ))
                            .with_children(|track| {
                                track.spawn((
                                    Node {
                                        position_type: PositionType::Absolute,
                                        left: Val::Percent(50.0),
                                        width: px(10),
                                        height: px(10),
                                        border_radius: BorderRadius::MAX,
                                        ..default()
                                    },
                                    BackgroundColor(Color::srgb(0.9, 0.75, 0.2)),
                                    JointSliderThumb {
                                        joint_name: joint_name.clone(),
                                    },
                                ));
                            });
                    }
                });
        });
}

pub(crate) fn update_joint_angles_ui(
    joints: Query<&JointState>,
    mut text_query: Query<&mut Text, With<JointAnglesText>>,
    task_control: Res<TaskSpaceControl>,
) {
    let Ok(mut text) = text_query.single_mut() else {
        return;
    };

    let mut output = String::from("JOINT POSITIONS\n\n");

    for joint in &joints {
        output.push_str(&format!(
            "{}   {:>7.2}°\n",
            joint.name,
            joint.value.to_degrees(),
        ));
    }

    if task_control.enabled {
        output.push_str(&format!(
            "\nTarget:\nx = {:.3}\ny = {:.3}\nz = {:.3}",
            task_control.target.x, task_control.target.y, task_control.target.z,
        ));
    }

    **text = output;
}

pub(crate) fn update_joint_slider_ui(
    joints: Query<&JointState>,
    mut thumbs: Query<(&JointSliderThumb, &mut Node)>,
) {
    for (thumb, mut node) in &mut thumbs {
        let Some(joint) = joints.iter().find(|j| j.name == thumb.joint_name) else {
            continue;
        };
        let degrees = joint.value.to_degrees().clamp(-180.0, 180.0);
        let percent = ((degrees + 180.0) / 360.0) * 100.0;
        node.left = Val::Percent(percent);
    }
}

impl LogBuffer {
    pub(crate) fn push(&mut self, message: impl Into<String>) {
        let message = message.into();
        println!("{message}");
        self.lines.push_back(message);
        if self.lines.len() > MAX_LOG_LINES {
            self.lines.pop_front();
        }
    }

    fn joined(&self) -> String {
        self.lines.iter().cloned().collect::<Vec<_>>().join("\n")
    }
}

pub(crate) fn spawn_log_panel_ui(commands: &mut Commands, mono_font: &MonoFont) {
    commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                bottom: px(20),
                left: px(20),
                width: px(420),
                height: px(200),
                padding: UiRect::all(px(12)),
                overflow: Overflow::clip(),
                ..default()
            },
            BackgroundColor(Color::srgba(0.02, 0.025, 0.03, 0.9)),
        ))
        .with_children(|parent| {
            parent.spawn((
                Text::new(""),
                TextFont {
                    font: mono_font.0.clone(),
                    font_size: 14.0,
                    ..default()
                },
                LogPanelText,
            ));
        });
}

pub(crate) fn update_log_panel_ui(
    log: Res<LogBuffer>,
    mut text_query: Query<&mut Text, With<LogPanelText>>,
) {
    let Ok(mut text) = text_query.single_mut() else {
        return;
    };
    **text = log.joined();
}
