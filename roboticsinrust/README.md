# RoboticsInRust

A Rust-based robotic arm modeling and simulation framework using the Denavit-Hartenberg (DH) system.

## Overview

This project uses a DH (Denavit-Hartenberg) parameter system to model a robotic arm. The core kinematics and dynamics calculations—including forward kinematics, inverse kinematics, and inverse Jacobian computations—are implemented in the `dh_arm_model` library crate for reusability across different simulations.

## Project Structure

### `dh_arm_model`
Core library containing all arm modeling logic:
- DH parameter definitions and transformations
- Forward kinematics calculations
- Inverse kinematics solvers
- Inverse Jacobian computations
- Task-space PID controller
- Joint definitions

### `kiss3d_sim`
Initial testing simulation using Kiss3D for visualization. This was created to validate the DH model with stick figure rendering for a specific arm configuration (6-DOF URT arm).

**To run Kiss3D simulation:**
```
cargo run -p kiss3d_sim
```

### `bevy_sim`
New advanced simulation framework using the Bevy engine for more complex interactions and features.

**To run Bevy simulation:**
```
cargo run -p bevy_sim
```

## Building

```
cargo build
```

## Dependencies

- **nalgebra** — Linear algebra and matrix operations
- **kiss3d** — 3D graphics (Kiss3D simulation)
- **bevy** — Game engine framework (Bevy simulation)
