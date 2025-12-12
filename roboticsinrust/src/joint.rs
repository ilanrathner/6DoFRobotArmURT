/// Type of joint in a kinematic chain.
#[derive(Debug, Clone, Copy)]
pub enum JointType {
    Revolute,   // angle, radians
    Prismatic,  // position, meters (or consistent linear unit)
}

/// Represents a joint with state and optional limits.
#[derive(Debug, Clone)]
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
    /// Create a joint with no limits.
    pub fn new(joint_type: JointType) -> Self {
        Self {
            joint_type,
            position: 0.0,
            velocity: 0.0,
            limit_min: None,
            limit_max: None,
        }
    }

    /// Create a joint with limits (in *internal* units).
    pub fn new_with_limits(joint_type: JointType, min: f64, max: f64) -> Self {
        Self {
            joint_type,
            position: 0.0,
            velocity: 0.0,
            limit_min: Some(min),
            limit_max: Some(max),
        }
    }

    // -------------------------------
    // Position Setters (Safe)
    // -------------------------------

    pub fn set_position(&mut self, pos: f64) {
        let mut p = pos;

        if let Some(min) = self.limit_min {
            if p < min {
                p = min;
            }
        }
        if let Some(max) = self.limit_max {
            if p > max {
                p = max;
            }
        }

        self.position = p;
    }

    pub fn set_velocity(&mut self, vel: f64) {
        self.velocity = vel;
    }

    // -------------------------------
    // User-Friendly DEGREE API
    // -------------------------------

    pub fn set_position_deg(&mut self, deg: f64) {
        match self.joint_type {
            JointType::Revolute => {
                self.set_position(deg.to_radians());
            }
            JointType::Prismatic => {
                panic!("set_position_deg called on prismatic joint");
            }
        }
    }

    pub fn position_deg(&self) -> f64 {
        match self.joint_type {
            JointType::Revolute => self.position.to_degrees(),
            JointType::Prismatic => {
                panic!("position_deg called on prismatic joint");
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
                println!("  Position: {:.3} rad  ({:.2}°)", self.position, self.position_deg());
                println!("  Velocity: {:.3} rad/s", self.velocity);

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
