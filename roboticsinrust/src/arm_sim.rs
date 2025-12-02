use kiss3d::window::Window;
use kiss3d::camera::ArcBall;
use kiss3d::scene::SceneNode;
use kiss3d::text::Font;
use kiss3d::nalgebra::{Translation3, Point2, Point3, Vector3, Matrix3}; 
use kiss3d::event::{Key, Action};
use std::time::Duration;
use std::fmt::Write;
use crate::Arm;
use crate::dh::Pose; // Import your custom Pose struct


// NOTE: This struct should exist in your 'arm' module or another accessible file.
// Included here for context to show where .position and .rotation come from.
// pub struct Pose {
//     pub position: Vector3<f64>,
//     pub rotation: Matrix3<f64>,
//     // ... methods ...
// }


/// Simulation for task-space velocity control with continuous loop and non-blocking input.
pub struct ArmSim {
    arm: Arm,
    task_vel: nalgebra::DVector<f64>,   // [vx, vy, vz, ω_roll, ω_pitch, ω_yaw]
    joint_vars: Vec<f64>,
    dt: f64,
}

impl ArmSim {
    // (new, step, reset functions remain unchanged)

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
        self.arm.update();
        Ok(())
    }

    
    pub fn reset(&mut self) {
        self.task_vel.fill(0.0);
        for v in &mut self.joint_vars { *v = 0.0; }
        self.arm.set_joint_variables(&self.joint_vars);
        self.arm.update();
        println!("Reset velocities and joint vars to zero.");
    }
    
    // ------------------------------------------------------------------
    // UPDATED FRAME DRAWING HELPER FUNCTION
    // Signature changed to accept a generic Pose struct.
    // Access changed from .translation to .position.
    // ------------------------------------------------------------------
    /// Draws a coordinate frame (X=Red, Y=Green, Z=Blue) given its pose and axis length.
    fn draw_frame_axes(
        window: &mut Window, 
        pose: &Pose, // Takes your custom Pose struct
        length: f32
    ) {
        // Convert the f64 components to f32 for kiss3d drawing
        // FIX: Accesses .position instead of .translation
        let pos = Point3::new(
            pose.position.x as f32,
            pose.position.y as f32,
            pose.position.z as f32,
        );

        // Get the rotation matrix (f32) to find the direction vectors
        // FIX: Directly uses .rotation (Matrix3) and casts it.
        let rot_mat = pose.rotation.cast::<f32>();
        
        // The columns of the rotation matrix are the local X, Y, Z axis direction vectors.
        // We use .column(index) to get the direction vector.
        let x_dir: Vector3<f32> = rot_mat.column(0).into_owned(); // X-axis (column 0)
        let y_dir: Vector3<f32> = rot_mat.column(1).into_owned(); // Y-axis (column 1)
        let z_dir: Vector3<f32> = rot_mat.column(2).into_owned(); // Z-axis (column 2)

        // Calculate the end points of the axes
        let x_end = pos + x_dir * length;
        let y_end = pos + y_dir * length;
        let z_end = pos + z_dir * length;

        // Define colors
        let color_x = Point3::new(1.0, 0.0, 0.0); // Red
        let color_y = Point3::new(0.0, 1.0, 0.0); // Green
        let color_z = Point3::new(0.0, 0.0, 1.0); // Blue

        // Draw the axes
        window.draw_line(&pos, &x_end, &color_x);
        window.draw_line(&pos, &y_end, &color_y);
        window.draw_line(&pos, &z_end, &color_z);
    }
    // ------------------------------------------------------------------

    pub fn run(&mut self) {
        println!("=== Continuous Arm Simulation (Kiss3d) ===");
        println!("Controls:");
        println!("x/z, y/h, c/v  -> linear X/Y/Z +/-");
        println!("a/s, d/f, g/j  -> angular Roll/Pitch/Yaw +/-");
        println!("space          -> reset");
        println!("q              -> quit\n");

        // Look-at target (focus point)
        let target = Point3::new(0.0f32, 0.0f32, 30.0f32);

        // Eye (camera position)
        let eye = Point3::new(40.0f32, -80.0f32, 50.0f32);

        // Up vector (Z up)
        let up = Vector3::new(0.0f32, 0.0f32, 1.0f32);

        // Create ArcBall
        let mut camera = ArcBall::new(eye, target);

        // Set up-vector (preferred if you want +Z to be up)
        camera.set_up_axis(up);

        // Create a window
        let mut window = Window::new("Robotic Arm Simulation");

        // optionally unlimited FPS
        window.set_framerate_limit(None);
        let font = Font::default();

        let mut joint_nodes: Vec<SceneNode> = Vec::new();
        for _ in 0..=self.joint_vars.len() {
            let mut s = window.add_sphere(0.05);
            s.set_color(1.0, 0.0, 0.0);
            joint_nodes.push(s);
        }
        

        let dt_duration = Duration::from_secs_f64(self.dt);
        let world_axis_len = 1.0;
        let frame_axis_len = 0.25;
        
        // World frame is now created as a custom Pose
        let world_pose = Pose::new(Vector3::new(0.0, 0.0, 0.0), Matrix3::identity());

        while window.render_with_camera(&mut camera) {
            // ... (Input and Physics Step remain the same)
            // --- Input Handling ---
            if window.get_key(Key::Q) == Action::Press {
                println!("Quitting simulation.");
                break;
            }

            if window.get_key(Key::Space) == Action::Press {
                self.reset();
            }
            
            // Linear velocities
            if window.get_key(Key::X) == Action::Press { self.task_vel[0] += 1.0; }
            if window.get_key(Key::Z) == Action::Press { self.task_vel[0] += -1.0; }
            if window.get_key(Key::Y) == Action::Press { self.task_vel[1] += 1.0; }
            if window.get_key(Key::H) == Action::Press { self.task_vel[1] += -1.0; }
            if window.get_key(Key::C) == Action::Press { self.task_vel[2] += 1.0; }
            if window.get_key(Key::V) == Action::Press { self.task_vel[2] += -1.0; }

            // Angular velocities
            if window.get_key(Key::A) == Action::Press { self.task_vel[3] += 1.0; }
            if window.get_key(Key::S) == Action::Press { self.task_vel[3] += -1.0; }
            if window.get_key(Key::D) == Action::Press { self.task_vel[4] += 1.0; }
            if window.get_key(Key::F) == Action::Press { self.task_vel[4] += -1.0; }
            if window.get_key(Key::G) == Action::Press { self.task_vel[5] += 1.0; }
            if window.get_key(Key::J) == Action::Press { self.task_vel[5] += -1.0; }


            // Physics step
            if let Err(e) = self.step() {
                println!("Step failed: {}", e);
            }

            if let Err(e) = self.step() {
                println!("Step failed: {}", e);
            }

            // --- Visualization Update ---
            // --- 1. Task Space Velocity Text Overlay ---
            let mut vel_text = String::new();
            // Use write! to efficiently format the vector components
            write!(&mut vel_text, 
                "Vx: {:.2}, Vy: {:.2}, Vz: {:.2}\n", 
                self.task_vel[0], self.task_vel[1], self.task_vel[2]
            ).unwrap();
            write!(&mut vel_text, 
                "Roll: {:.2}, Pitch: {:.2}, Yaw: {:.2}", 
                self.task_vel[3], self.task_vel[4], self.task_vel[5]
            ).unwrap();

            // Draw the text in the top-left corner
            window.draw_text(
                &vel_text, 
                &Point2::new(10.0, 10.0), // Screen coordinates (x, y, z=0)
                60.0, // Text size
                &font,
                &Point3::new(1.0, 1.0, 1.0) // White color
            );
            // Draw the global (World) coordinate axes using the helper function.
            // FIX: Uses the custom 'world_pose' struct instance.
            ArmSim::draw_frame_axes(
                &mut window, 
                &world_pose, 
                world_axis_len
            );
            
            //print table
            self.arm.dh_table().print_table();

            let poses = self.arm.frame_poses();
        // --- PRINTING THE POSES ---
            // The '{:?}' debug formatter will print the entire vector and its contents.
            println!("--- Current Arm Poses ({}) ---", poses.len());
            for (i, pose) in poses.iter().enumerate() {
                // You can print each pose separately for better readability:
                println!("Frame {}: {:?}", i, pose);
            }
            println!("-----------------------------\n");
            
            // --- 2. INVERSE KINEMATICS CALCULATION AND PRINTING ---
            if let Some(ee_pose) = poses.last() {
                match self.arm.solve_ik_from_pose(ee_pose) {
                    Ok(angles) => {
                        // The IK solution is typically very close to the current joint angles,
                        // but printing it confirms the solver is running and returning valid data.
                        let angles_formatted: Vec<String> = angles.iter()
                            .map(|a| format!("{:.4}", a.to_degrees()))
                            .collect();
                        
                        println!("--- Inverse Kinematics Solution ---");
                        println!("Target Pose: P={:.3}, R=[...]", ee_pose.position);
                        println!("Joint Angles (Degrees): [{}]", angles_formatted.join(", "));
                        println!("---------------------------------\n");
                    }
                    Err(e) => {
                        // This happens if the current pose is outside the calculated workspace
                        println!("--- Inverse Kinematics FAILED ---");
                        println!("Error: {}", e);
                        println!("-------------------------------\n");
                    }
                }
            }
            
            
            // Draw links (segments) and update joint sphere positions
            let mut prev_pos = Point3::new(
                world_pose.position.x as f32,
                world_pose.position.y as f32,
                world_pose.position.z as f32,
            );
            
            for i in 0..poses.len() {
                let current_pose = &poses[i];
                
                // Convert current position (f64) to f32 Point3
                let current_pos = Point3::new(
                    current_pose.position.x as f32,
                    current_pose.position.y as f32,
                    current_pose.position.z as f32,
                );
                
                // Set the sphere position
                joint_nodes[i].set_local_translation(Translation3::from(current_pos));
                
                // 1. Draw the link from the previous joint to the current joint.
                // This handles the link from World Origin (for i=0) onwards.
                window.draw_line(
                    &prev_pos, 
                    &current_pos, 
                    &Point3::new(0.0, 0.0, 1.0), // Blue color for links
                );

                //Draw the local axis for the current joint frame (Frame 1 to EE)
                ArmSim::draw_frame_axes(
                    &mut window, 
                    current_pose, 
                    frame_axis_len
                );
                
                // 2. Save the current position for the next iteration.
                // This avoids re-calculating the previous position from `poses[i-1]`.
                prev_pos = current_pos;
            }
            

            // Optional: add a small sleep to limit loop rate
            std::thread::sleep(dt_duration);
        }
    }
}