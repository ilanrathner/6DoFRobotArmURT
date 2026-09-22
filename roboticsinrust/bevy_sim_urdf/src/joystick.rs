//! Joystick input adapter for the Bevy arm simulation.
//!
//! The simulator supports two control modes that consume the same normalized
//! [`JoystickInput`] resource:
//!
//! - **Task space:** joystick 1 moves the end-effector in X/Y, twists it around
//!   Z, and uses buttons 1/2 for vertical movement.
//! - **Joint space:** joystick 1 controls joints 1-3 and joystick 2 controls
//!   joints 4-6. Each joystick contributes X, Y, and twist axes.
//!
//! Windows uses the legacy WinMM joystick API because Bevy/gilrs exposes the
//! T.16000M X/Y axes but may omit its twist and throttle axes. Other platforms
//! retain a Bevy gamepad fallback.

use bevy::prelude::*;

/// Ignore small axis movements caused by centering noise or sensor drift.
const DEAD_ZONE: f32 = 0.08;

#[derive(Resource, Default)]
pub(crate) struct JoystickInput {
    /// True when at least one joystick is available this frame.
    pub(crate) connected: bool,
    /// Number of active devices, capped at two by the Windows adapter.
    pub(crate) connected_count: usize,
    /// Task-space linear command: X/Y from joystick 1 and Z from buttons 1/2.
    pub(crate) translation: Vec3,
    /// Task-space angular command. Currently joystick 1 twist controls yaw.
    pub(crate) rotation: Vec3,
    /// Joint-space velocities in URDF joint order:
    /// `[left X, left Y, left twist, right X, right Y, right twist]`.
    pub(crate) joint_axes: [f32; 6],
}

#[cfg(target_os = "windows")]
mod platform {
    use super::*;
    use std::mem::size_of;
    use windows_sys::Win32::Media::Multimedia::{
        JOY_RETURNBUTTONS, JOY_RETURNPOV, JOY_RETURNR, JOY_RETURNU, JOY_RETURNV, JOY_RETURNX,
        JOY_RETURNY, JOY_RETURNZ, JOYCAPSW, JOYERR_NOERROR, JOYINFOEX, joyGetDevCapsW,
        joyGetNumDevs, joyGetPosEx,
    };

    #[derive(Clone, Copy)]
    pub(crate) struct WindowsJoystick {
        /// WinMM device index used by `joyGetPosEx`.
        id: u32,
        /// Per-axis minimum/maximum values reported by the driver.
        caps: JOYCAPSW,
    }

    #[derive(Clone, Copy, Default)]
    pub(crate) struct RawState {
        /// Normalized `[X, Y, Z, R, U, V]` values in the range `[-1, 1]`.
        axes: [f32; 6],
        /// WinMM button bitmask; bit 0 is button 1, bit 1 is button 2, etc.
        buttons: u32,
        /// POV hat angle in hundredths of a degree, or the WinMM neutral value.
        pov: u32,
    }

    /// Poll up to two Windows joysticks and publish one controller snapshot.
    ///
    /// Device enumeration only occurs when the cached list is empty. If either
    /// cached device disconnects, the list is cleared and rebuilt next frame.
    pub(crate) fn read_joystick(
        mut input: ResMut<JoystickInput>,
        mut devices: Local<Vec<WindowsJoystick>>,
        mut previous: Local<[Option<RawState>; 2]>,
    ) {
        if devices.is_empty() {
            *devices = find_joysticks();
        }
        if devices.is_empty() {
            *input = JoystickInput::default();
            return;
        }

        let mut states = [RawState::default(); 2];
        let mut active = 0;
        for (slot, device) in devices.iter().take(2).enumerate() {
            let Some(raw) = read_state(*device) else {
                warn!("Windows joystick {} disconnected", device.id);
                *devices = Vec::new();
                *input = JoystickInput::default();
                return;
            };
            states[slot] = raw;
            active += 1;

            let changed = previous[slot].is_none_or(|old| {
                raw.axes
                    .iter()
                    .zip(old.axes.iter())
                    .any(|(new, old)| (new - old).abs() > 0.02)
                    || raw.buttons != old.buttons
                    || raw.pov != old.pov
            });
            if changed {
                info!(
                    "WinMM joystick {}: X={:+.3} Y={:+.3} Z={:+.3} R={:+.3} U={:+.3} V={:+.3} buttons=0x{:08X} pov={}",
                    slot + 1,
                    raw.axes[0],
                    raw.axes[1],
                    raw.axes[2],
                    raw.axes[3],
                    raw.axes[4],
                    raw.axes[5],
                    raw.buttons,
                    raw.pov
                );
                previous[slot] = Some(raw);
            }
        }

        input.connected = active > 0;
        input.connected_count = active;
        // Button bits follow WinMM numbering: button 1 is bit 0, button 2 bit 1.
        let move_up = states[0].buttons & 0x1 != 0;
        let move_down = states[0].buttons & 0x2 != 0;
        let vertical = i8::from(move_up) as f32 - i8::from(move_down) as f32;
        input.translation = Vec3::new(
            dead_zone(states[0].axes[0]),
            -dead_zone(states[0].axes[1]),
            vertical,
        );
        input.rotation = Vec3::new(0.0, 0.0, dead_zone(states[0].axes[3]));
        // The scene assigns these six entries to moving URDF joints in order.
        input.joint_axes = [
            dead_zone(states[0].axes[0]),
            -dead_zone(states[0].axes[1]),
            dead_zone(states[0].axes[3]),
            dead_zone(states[1].axes[0]),
            -dead_zone(states[1].axes[1]),
            dead_zone(states[1].axes[3]),
        ];
    }

    /// Read and normalize every axis, button, and POV value for one device.
    fn read_state(device: WindowsJoystick) -> Option<RawState> {
        let mut state = JOYINFOEX {
            dwSize: size_of::<JOYINFOEX>() as u32,
            dwFlags: (JOY_RETURNX
                | JOY_RETURNY
                | JOY_RETURNZ
                | JOY_RETURNR
                | JOY_RETURNU
                | JOY_RETURNV
                | JOY_RETURNPOV
                | JOY_RETURNBUTTONS) as u32,
            ..Default::default()
        };
        // SAFETY: state is a valid JOYINFOEX with the required size and flags.
        if unsafe { joyGetPosEx(device.id, &mut state) } != JOYERR_NOERROR {
            return None;
        }
        Some(RawState {
            axes: [
                normalize(state.dwXpos, device.caps.wXmin, device.caps.wXmax),
                normalize(state.dwYpos, device.caps.wYmin, device.caps.wYmax),
                normalize(state.dwZpos, device.caps.wZmin, device.caps.wZmax),
                normalize(state.dwRpos, device.caps.wRmin, device.caps.wRmax),
                normalize(state.dwUpos, device.caps.wUmin, device.caps.wUmax),
                normalize(state.dwVpos, device.caps.wVmin, device.caps.wVmax),
            ],
            buttons: state.dwButtons,
            pov: state.dwPOV,
        })
    }

    /// Enumerate the first two WinMM joystick devices available to Windows.
    fn find_joysticks() -> Vec<WindowsJoystick> {
        let mut devices = Vec::new();
        // SAFETY: WinMM enumeration accepts IDs in 0..joyGetNumDevs().
        let count = unsafe { joyGetNumDevs() };
        for id in 0..count {
            let mut caps = JOYCAPSW::default();
            // SAFETY: caps points to a correctly sized JOYCAPSW value.
            let result =
                unsafe { joyGetDevCapsW(id as usize, &mut caps, size_of::<JOYCAPSW>() as u32) };
            if result == JOYERR_NOERROR {
                let product_name = caps.szPname;
                let end = product_name
                    .iter()
                    .position(|character| *character == 0)
                    .unwrap_or(product_name.len());
                let name = String::from_utf16_lossy(&product_name[..end]);
                info!(
                    "Windows joystick {} connected: id={id}, name={name}",
                    devices.len() + 1
                );
                devices.push(WindowsJoystick { id, caps });
                if devices.len() == 2 {
                    break;
                }
            }
        }
        devices
    }

    /// Convert a driver-specific unsigned axis range to Bevy's `[-1, 1]` range.
    fn normalize(value: u32, minimum: u32, maximum: u32) -> f32 {
        if maximum <= minimum {
            return 0.0;
        }
        ((value.saturating_sub(minimum)) as f32 / (maximum - minimum) as f32) * 2.0 - 1.0
    }
}

#[cfg(not(target_os = "windows"))]
mod platform {
    use super::*;

    /// Bevy-native fallback for Linux and other non-Windows platforms.
    ///
    /// On Ubuntu the T.16000M is reported as X, Y, Rz, Throttle, Hat0X and
    /// Hat0Y. Bevy maps the first three motion axes to LeftStickX,
    /// LeftStickY and RightZ respectively. The first enumerated device drives
    /// joints 1-3 and the second drives joints 4-6.
    pub(crate) fn read_joystick(
        gamepads: Query<(&Name, &Gamepad)>,
        mut input: ResMut<JoystickInput>,
        mut previous_names: Local<[Option<String>; 2]>,
    ) {
        let mut devices = gamepads.iter();
        let Some((first_name, first)) = devices.next() else {
            *input = JoystickInput::default();
            *previous_names = [None, None];
            return;
        };

        if previous_names[0].as_deref() != Some(first_name.as_str()) {
            info!("joystick 1 connected: {first_name}");
            previous_names[0] = Some(first_name.to_string());
        }
        let second = devices.next();
        if let Some((second_name, _)) = second {
            if previous_names[1].as_deref() != Some(second_name.as_str()) {
                info!("joystick 2 connected: {second_name}");
                previous_names[1] = Some(second_name.to_string());
            }
        } else {
            previous_names[1] = None;
        }

        input.connected = true;
        input.connected_count = if second.is_some() { 2 } else { 1 };

        let first_x = dead_zone(first.get(GamepadAxis::LeftStickX).unwrap_or(0.0));
        let first_y = -dead_zone(first.get(GamepadAxis::LeftStickY).unwrap_or(0.0));
        let first_rz = dead_zone(first.get(GamepadAxis::RightZ).unwrap_or(0.0));
        let (second_x, second_y, second_rz) = second
            .map(|(_, gamepad)| {
                (
                    dead_zone(gamepad.get(GamepadAxis::LeftStickX).unwrap_or(0.0)),
                    -dead_zone(gamepad.get(GamepadAxis::LeftStickY).unwrap_or(0.0)),
                    dead_zone(gamepad.get(GamepadAxis::RightZ).unwrap_or(0.0)),
                )
            })
            .unwrap_or((0.0, 0.0, 0.0));

        // SDL/gilrs mappings vary for flight sticks. On the T.16000M the
        // physical Trigger and ThumbBtn may be exposed as standard gamepad
        // buttons or as raw `Other(0)` / `Other(1)` buttons.
        for button in first.get_just_pressed() {
            info!("joystick 1 button pressed: {button:?}");
        }
        if let Some((_, second_gamepad)) = second {
            for button in second_gamepad.get_just_pressed() {
                info!("joystick 2 button pressed: {button:?}");
            }
        }

        // Accept the vertical controls from either device. This is important
        // on Ubuntu because the operator may use /dev/input/js1 for task-space
        // while /dev/input/js0 remains assigned to the other hand.
        let move_up = is_move_up(first)
            || second
                .map(|(_, gamepad)| is_move_up(gamepad))
                .unwrap_or(false);
        let move_down = is_move_down(first)
            || second
                .map(|(_, gamepad)| is_move_down(gamepad))
                .unwrap_or(false);
        let vertical = i8::from(move_up) as f32 - i8::from(move_down) as f32;
        input.translation = Vec3::new(first_x, first_y, vertical);
        input.rotation = Vec3::new(0.0, 0.0, first_rz);
        input.joint_axes = [first_x, first_y, first_rz, second_x, second_y, second_rz];
    }

    /// Return true when any platform-specific alias for a control is pressed.
    fn pressed_any(gamepad: &Gamepad, buttons: &[GamepadButton]) -> bool {
        buttons.iter().any(|button| gamepad.pressed(*button))
    }

    fn is_move_up(gamepad: &Gamepad) -> bool {
        pressed_any(
            gamepad,
            &[
                GamepadButton::South,
                GamepadButton::LeftTrigger,
                GamepadButton::Other(0),
            ],
        )
    }

    fn is_move_down(gamepad: &Gamepad) -> bool {
        pressed_any(
            gamepad,
            &[
                GamepadButton::East,
                GamepadButton::LeftThumb,
                GamepadButton::Other(1),
            ],
        )
    }
}

/// Return zero near the physical centre while preserving full-scale commands.
fn dead_zone(value: f32) -> f32 {
    if value.abs() < DEAD_ZONE { 0.0 } else { value }
}

pub(crate) use platform::read_joystick;
