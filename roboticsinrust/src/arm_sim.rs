use kiss3d::window::Window; 
use kiss3d::camera::ArcBall;
use kiss3d::scene::SceneNode;
use kiss3d::text::Font;
use kiss3d::nalgebra::{Translation3, Point2, Point3, Vector3, Matrix3, UnitQuaternion}; 
use kiss3d::event::{Key, Action};
use std::time::Duration;
use std::fmt::Write;
use crate::dh_arm_model::DHArmModel;
use crate::dh::Pose;
use crate::task_space_pid_controller::TaskSpacePidController;
use crate::inverse_kinematics_solvers::IkSolver;


/// Simulation for task-space velocity control with continuous loop and non-blocking input.
pub struct ArmSim<const F: usize, const J: usize, S: IkSolver<J>> {
    arm: DHArmModel<F, J, S>,
    controller: TaskSpacePidController,
    task_vel: [f64; 6],   // [vx, vy, vz, ω_roll, ω_pitch, ω_yaw]
    joint_vel: [f64; J],
    joint_pos: [f64; J],
    dt: f64,
}

impl<const F: usize, const J: usize, S: IkSolver<J>> ArmSim<F, J, S> {
    pub fn new(mut arm: DHArmModel<F, J, S>, controller: TaskSpacePidController, dt: f64) -> Self {
        
        arm.set_joint_positions(&[0.0f64; J]);
        arm.set_joint_velocities(&[0.0f64; J]);

        Self {
            arm,
            controller,
            task_vel: [0.0; 6],
            joint_vel: [0.0; J],
            joint_pos: [0.0; J],
            dt,
        }
    }

    /// Step simulation using task-space velocity (Jacobian inverse)
    fn step(&mut self) -> Result<(), String> {
        let theta_dot = self.controller.compute(&mut self.arm, &self.task_vel, &self.joint_pos, &self.joint_vel, self.dt);
        //println!("{:?} -> {:?}", self.task_vel, theta_dot);
        // Update internal joint state
        for i in 0..J {
            self.joint_vel[i] = theta_dot[i];
            self.joint_pos[i] += self.joint_vel[i] * self.dt;
        }

        Ok(())
    }

    pub fn reset(&mut self) {
        self.task_vel = [0.0; 6];
        self.joint_vel = [0.0; J];
        self.joint_pos = [0.0; J];
        self.arm.set_joint_positions(&[0.0f64; J]);
        self.arm.set_joint_velocities(&[0.0f64; J]);
        println!("Reset velocities and joint positions to zero.");
    }

    // ----- Visualization Helpers -----
    fn draw_frame_axes(window: &mut Window, pose: &Pose, length: f32) {
        let pos = Point3::new(
            pose.position.x as f32,
            pose.position.y as f32,
            pose.position.z as f32,
        );
        let rot_mat = pose.rotation.cast::<f32>();
        let x_dir: Vector3<f32> = rot_mat.column(0).into_owned();
        let y_dir: Vector3<f32> = rot_mat.column(1).into_owned();
        let z_dir: Vector3<f32> = rot_mat.column(2).into_owned();

        window.draw_line(&pos, &(pos + x_dir * length), &Point3::new(1.0, 0.0, 0.0)); //red
        window.draw_line(&pos, &(pos + y_dir * length), &Point3::new(0.0, 1.0, 0.0)); //green
        window.draw_line(&pos, &(pos + z_dir * length), &Point3::new(0.0, 0.0, 1.0)); //blue
    }

    fn draw_board(window: &mut Window, height: f64, x_offset: f64, width: f64, depth: f64) {
        let center_pos = Point3::new(x_offset as f32, 0.0, (height + depth / 2.0) as f32);
        let rotation_quaternion = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), std::f32::consts::FRAC_PI_2);
        let mut target_quad = window.add_quad(depth as f32, width as f32, 1, 1);
        target_quad.set_color(1.0, 1.0, 0.0);
        target_quad.set_local_rotation(rotation_quaternion);
        target_quad.set_local_translation(Translation3::from(center_pos.coords));
    }

    fn draw_dh_arm(
        window: &mut Window,
        arm: &DHArmModel<F, J, S>,
        joint_nodes: &mut [SceneNode],
        world_pose: &Pose,
        world_axis_len: f32,
        frame_axis_len: f32,
    ) {
        let poses = arm.frame_poses();

        // Draw world frame
        Self::draw_frame_axes(window, world_pose, world_axis_len);

        let mut prev_pos = Point3::new(
            world_pose.position.x as f32,
            world_pose.position.y as f32,
            world_pose.position.z as f32,
        );

        for (i, pose) in poses.iter().enumerate() {
            let current_pos = Point3::new(
                pose.position.x as f32,
                pose.position.y as f32,
                pose.position.z as f32,
            );

            // Update joint marker
            joint_nodes[i].set_local_translation(Translation3::from(current_pos));

            // Draw link
            window.draw_line(&prev_pos, &current_pos, &Point3::new(0.0, 0.0, 1.0));

            // Draw frame axes
            Self::draw_frame_axes(window, pose, frame_axis_len);

            prev_pos = current_pos;
        }
    }

    

    fn get_keyboard_input(&mut self, window: &Window) {
        // Placeholder for future keyboard input handling if needed
        if window.get_key(Key::Space) == Action::Press { self.reset(); }

        // Linear velocities
        if window.get_key(Key::Z) == Action::Press { self.task_vel[0] += 1.0; }
        if window.get_key(Key::X) == Action::Press { self.task_vel[0] -= 1.0; }
        if window.get_key(Key::C) == Action::Press { self.task_vel[1] += 1.0; }
        if window.get_key(Key::V) == Action::Press { self.task_vel[1] -= 1.0; }
        if window.get_key(Key::B) == Action::Press { self.task_vel[2] += 1.0; }
        if window.get_key(Key::N) == Action::Press { self.task_vel[2] -= 1.0; }

        // Angular velocities
        if window.get_key(Key::A) == Action::Press { self.task_vel[3] += 3.0; }
        if window.get_key(Key::S) == Action::Press { self.task_vel[3] -= 3.0; }
        if window.get_key(Key::D) == Action::Press { self.task_vel[4] += 3.0; }
        if window.get_key(Key::F) == Action::Press { self.task_vel[4] -= 3.0; }
        if window.get_key(Key::G) == Action::Press { self.task_vel[5] += 3.0; }
        if window.get_key(Key::H) == Action::Press { self.task_vel[5] -= 3.0; }
    }



    pub fn run(&mut self) {
        println!("=== Continuous Arm Simulation (Kiss3d) ===");
        println!("Controls:");
        println!("z/x, c/v, b/n  -> linear X/Y/Z +/-");
        println!("a/s, d/f, g/h  -> angular X/Y/Z +/-");
        println!("space          -> reset");
        println!("q              -> quit\n");

        let target = Point3::new(0.0f32, 0.0f32, 30.0f32);
        let eye = Point3::new(40.0f32, -80.0f32, 50.0f32);
        let up = Vector3::new(0.0f32, 0.0f32, 1.0f32);
        let mut camera = ArcBall::new(eye, target);
        camera.set_up_axis(up);

        let mut window = Window::new("Robotic Arm Simulation");
        window.set_framerate_limit(None);
        let font = Font::default();

        let mut joint_nodes: Vec<SceneNode> = Vec::new();
        for _ in 0..F {
            let mut s = window.add_sphere(0.05);
            s.set_color(1.0, 0.0, 0.0);
            joint_nodes.push(s);
        }

        let dt_duration = Duration::from_secs_f64(self.dt);
        let world_axis_len = 1.0;
        let frame_axis_len = 0.25;
        let world_pose = Pose::new(Vector3::new(0.0, 0.0, 0.0), Matrix3::identity());

        Self::draw_board(&mut window, -5.0, 35.0, 90.0, 60.0);

        while window.render_with_camera(&mut camera) {
            if window.get_key(Key::Q) == Action::Press { break; }

            self.get_keyboard_input(&window);

            let _ = self.step();
            println!("joint_vel: {:?}, joint_pos: {:?}", &self.joint_vel, &self.joint_pos);

            Self::draw_dh_arm(
                &mut window,
                &self.arm,
                &mut joint_nodes,
                &world_pose,
                world_axis_len,
                frame_axis_len,
            );

            let mut vel_text = String::new();
            write!(&mut vel_text,
                "Vx: {:.2}, Vy: {:.2}, Vz: {:.2}\nWx: {:.2}, Wy: {:.2}, Wz: {:.2}",
                self.task_vel[0], self.task_vel[1], self.task_vel[2],
                self.task_vel[3], self.task_vel[4], self.task_vel[5]
            ).unwrap();
            window.draw_text(&vel_text, &Point2::new(10.0, 10.0), 60.0, &font, &Point3::new(1.0, 1.0, 1.0));
            

            std::thread::sleep(dt_duration);
        }
    }
}
