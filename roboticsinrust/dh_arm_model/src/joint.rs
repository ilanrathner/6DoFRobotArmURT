/// The mechanical classification of a joint
pub enum JointType {
    Revolute,   // angle, radians
    Prismatic,  // position, meters (or consistent linear unit)
}

/// Represents a single joint's state, physical constraints, and unit conversions.
///
/// This struct acts as a safety wrapper, ensuring that commanded positions 
/// stay within physical hardware limits and that user-facing units (like degrees) 
/// are correctly internalized as standard SI units (radians/meters).
pub struct Joint {
    pub joint_type: JointType,

    /// Current joint position:
    /// - revolute: radians
    /// - prismatic: meters
    pub position: f64,

    /// Current joint velocity:
    /// - revolute: rad/s
    /// - prismatic: m/s
    pub velocity: f64,

    /// Lower position limit (rad or meters)
    pub limit_min: Option<f64>,

    /// Upper position limit (rad or meters)
    pub limit_max: Option<f64>,
}

impl Joint {
    /// Create a joint with optional limits.
    pub fn new(joint_type: JointType, limit_min: Option<f64>, limit_max: Option<f64>) -> Self {
        let is_revolute = matches!(joint_type, JointType::Revolute);

        Self {
            joint_type,
            position: 0.0,
            velocity: 0.0,
            limit_min: limit_min.map(|val| if is_revolute { val.to_radians() } else { val }),
            limit_max: limit_max.map(|val| if is_revolute { val.to_radians() } else { val }),
        }
    }


    /// Set joint position with limit checking. For revolute joints, assume input is in degrees for user and convert to radians.
    pub fn set_position(&mut self, pos: f64) {
        match self.joint_type {
            JointType::Revolute => {
                self.position = pos.to_radians(); // Will apply limits below
            }
            JointType::Prismatic => {
                self.position = pos; // Will apply limits below
            }
        }


        if let Some(min) = self.limit_min {
            if self.position < min {
                self.position = min;
            }
        }
        if let Some(max) = self.limit_max {
            if self.position > max {
                self.position = max;
            }
        }

    }
    /// Set joint velocity. For revolute joints, assume input is in degrees/s for user and convert to radians/s.
    pub fn set_velocity(&mut self, vel: f64) {
        match self.joint_type {
            JointType::Revolute => {
                self.velocity = vel.to_radians(); // Assume input is in rad/s for revolute joints
            }
            JointType::Prismatic => {
                self.velocity = vel; // Assume input is in m/s for prismatic joints
            }
        }
    }

    // -------------------------------
    // Pretty Printer
    // -------------------------------

    pub fn print_info(&self) {
        match self.joint_type {
            JointType::Revolute => {
                println!("Joint Type: Revolute");
                println!("  Position: {:.3} rad  ({:.2}°)", self.position, self.position.to_degrees());
                println!("  Velocity: {:.3} rad/s ({:.2}°/s)", self.velocity, self.velocity.to_degrees());

                match (self.limit_min, self.limit_max) {
                    (Some(min), Some(max)) => {
                        println!(
                            "  Limits: {:.3} rad ({:.1}°)  →  {:.3} rad ({:.1}°)",
                            min, min.to_degrees(),
                            max, max.to_degrees()
                        );
                    }
                    _ => println!("  Limits: None"),
                }
            }

            JointType::Prismatic => {
                println!("Joint Type: Prismatic");
                println!("  Position: {:.4} m", self.position);
                println!("  Velocity: {:.4} m/s", self.velocity);

                match (self.limit_min, self.limit_max) {
                    (Some(min), Some(max)) => {
                        println!("  Limits: {:.4} m  →  {:.4} m", min, max);
                    }
                    _ => println!("  Limits: None"),
                }
            }
        }
    }
}
