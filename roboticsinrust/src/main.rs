mod kinematics_functions;  // includes kinematics.rs
mod jacobian_functions;    // includes jacobian.rs

use kinematics_functions::*; // optionally bring functions into scope
use jacobian_functions::*;

use kiss3d::window::Window;
use kiss3d::nalgebra::{Point3, Translation3};

fn main() {
    println!("Hello, world!");

    let mut window = Window::new("Interactive Robot Arm");
    let mut joint_angles = [0.0; 6];
    let link_lengths = [1.0,1.0,1.0,1.0,1.0];

    while window.render() {
        // Clear window automatically each render loop

        // Update joint angles based on user input
        if window.key_down(kiss3d::event::Key::Up) { joint_angles[0] += 0.01; }
        if window.key_down(kiss3d::event::Key::Down) { joint_angles[0] -= 0.01; }

        // Compute joint positions
        let positions = forward_kinematics(&joint_angles, &link_lengths);

        // Draw links
        for w in positions.windows(2) {
            let p0 = Point3::new(w[0][0], w[0][1], w[0][2]);
            let p1 = Point3::new(w[1][0], w[1][1], w[1][2]);
            window.draw_line(&p0, &p1, &kiss3d::nalgebra::geometry::Color::rgb(1.0,0.0,0.0));
        }

        // Optionally draw joints as spheres
        for pos in &positions {
            let mut sphere = window.add_sphere(0.05);
            sphere.set_local_translation(Translation3::new(pos[0], pos[1], pos[2]));
            sphere.set_color(0.0, 0.0, 1.0);
        }
    }
}
