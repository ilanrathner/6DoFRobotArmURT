use crate::arm::Arm;
use nalgebra::{Point3};
use kiss3d::window::Window;
use kiss3d::scene::SceneNode;
use kiss3d::nalgebra as na;
use crossterm::event::{self, Event, KeyCode};
use std::{thread, time::Duration};
use std::time::Instant;

/// Simulation for task-space velocity control with continuous loop and non-blocking input.
pub struct ArmSim {
    arm: Arm,
    task_vel: nalgebra::DVector<f64>,   // [vx, vy, vz, ω_roll, ω_pitch, ω_yaw]
    joint_vars: Vec<f64>,
    dt: f64,
}

impl ArmSim {
    pub fn new(mut arm: Arm, dt: f64) -> Self {
        let n = arm.dh_table().num_joints();
        let joint_vars = vec![0.0f64; n];
        arm.set_joint_variables(&joint_vars);

        Self {
            arm,
            task_vel: nalgebra::DVector::zeros(6),
            joint_vars,
            dt,
        }
    }

    /// Integrate one step using the current velocities
    fn step(&mut self) -> Result<(), String> {
        let inv_j = self.arm.inv_jacobian();
        let n = inv_j.nrows();

        if n == 0 || inv_j.ncols() != 6 {
            return Err("Jacobian shape mismatch".into());
        }

        let theta_dot = inv_j * &self.task_vel;
        for i in 0..n {
            self.joint_vars[i] += theta_dot[i] * self.dt;
        }

        self.arm.set_joint_variables(&self.joint_vars);
        Ok(())
    }

    fn adjust_velocity(&mut self, key: KeyCode) {
        let mag = 1.0;
        match key {
            KeyCode::Char('x') => self.task_vel[0] += mag,
            KeyCode::Char('X') => self.task_vel[0] -= mag,
            KeyCode::Char('y') => self.task_vel[1] += mag,
            KeyCode::Char('Y') => self.task_vel[1] -= mag,
            KeyCode::Char('z') => self.task_vel[2] += mag,
            KeyCode::Char('Z') => self.task_vel[2] -= mag,
            KeyCode::Char('r') => self.task_vel[3] += mag,
            KeyCode::Char('R') => self.task_vel[3] -= mag,
            KeyCode::Char('p') => self.task_vel[4] += mag,
            KeyCode::Char('P') => self.task_vel[4] -= mag,
            KeyCode::Char('w') => self.task_vel[5] += mag,
            KeyCode::Char('W') => self.task_vel[5] -= mag,
            _ => {}
        }
    }

    pub fn reset(&mut self) {
        self.task_vel.fill(0.0);
        for v in &mut self.joint_vars { *v = 0.0; }
        self.arm.set_joint_variables(&self.joint_vars);
        println!("Reset velocities and joint vars to zero.");
    }

    pub fn run(&mut self) {
        println!("=== Continuous Arm Simulation (Kiss3d) ===");
        println!("Controls:");
        println!("x/X y/Y z/Z  -> linear +/-");
        println!("r/R p/P w/W  -> angular +/-");
        println!("space        -> reset");
        println!("q            -> quit\n");

        let mut window = Window::new("Robotic Arm Simulation");

        // Create a scene node for each joint
        let mut joint_nodes: Vec<SceneNode> = Vec::new();
        for _ in 0..=self.joint_vars.len() {
            let mut s = window.add_sphere(0.05);
            s.set_color(1.0, 0.0, 0.0);
            joint_nodes.push(s);
        }

        let dt_duration = Duration::from_secs_f64(self.dt);

        while window.render() {
            // Non-blocking input check
            if event::poll(Duration::from_millis(1)).unwrap() {
                if let Event::Key(ev) = event::read().unwrap() {
                    match ev.code {
                        KeyCode::Char('q') => {
                            println!("Quitting simulation.");
                            break;
                        }
                        KeyCode::Char(' ') => self.reset(),
                        key => self.adjust_velocity(key),
                    }
                }
            }

            // Physics step
            if let Err(e) = self.step() {
                println!("Step failed: {}", e);
            }

            // Update joint node positions
            let poses = self.arm.frame_poses();
            for (i, pose) in poses.iter().enumerate() {
                joint_nodes[i].set_local_translation(Point3::new(
                    pose.position.x as f32,
                    pose.position.y as f32,
                    pose.position.z as f32,
                ));
            }

            // Optionally, draw lines between joints for links
            // Could use `window.add_cube()` or `window.add_capsule()` for arm segments
        }
    }
}
