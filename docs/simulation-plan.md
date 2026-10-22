# Simulation plan

## Purpose

Learn ROS 2 while building a simulation of the existing robot, then use repeatable experiments to evaluate balancing and compare with hardware.

Use the sourced reference specifications and explicit initial assumptions in [components.md](components.md). Existing Arduino code and physical motor tests are refinements, not prerequisites for starting the simulation.

## Current learning milestone

The local Python PD framework and working reference controller are implemented; the small student controller function is intentionally left for the owner to complete. Follow [first-experiment.md](first-experiment.md). This version uses perfect tilt/rate and bounded instantaneous torque; realistic actuation and sensing remain later work.

The passive ROS 2 / Gazebo model, physics world, simulated IMU, wheel-state bridge, and unpowered launch are implemented. Follow [ros-first-model.md](ros-first-model.md). A separate [preset balance demo](ros-balancing-demo.md) adds physical wheel-joint torque, pitch PD, and position feedback with automatic startup. Its [motor/driver surrogate](motor-driver-model.md) includes PWM, speed-dependent torque, losses, source-sag sensitivity, and rating diagnostics. Both passive and controlled responses have headless checks. Hardware identification, switching-current/thermal validation, backlash, and orientation estimation are later work.

## Proposed tools

- ROS 2 Jazzy for communication, controller nodes, launch, and logging.
- Gazebo Harmonic for rigid-body physics, contact, wheel actuation, and simulated IMU data.
- Python for initial controller experiments and plotting; revisit implementation choices when timing requirements are known.

Jazzy and Harmonic are an officially supported pairing. References:

- [Installing Gazebo with ROS](https://gazebosim.org/docs/harmonic/ros_installation/)
- [ROS 2 / Gazebo integration](https://gazebosim.org/docs/harmonic/ros2_integration/)

Verified environment: ROS 2 Jazzy and Gazebo Harmonic (Sim 8.15.0) in Ubuntu 24.04 on WSL 2, with VS Code connected to `Ubuntu-24.04-D`. The owner confirmed the shapes demo opens. The passive package builds with colcon, and its ROS sensor streams and unpowered fall pass the headless check.

## Milestones

1. **Environment:** verify ROS 2 and Gazebo with a minimal example.
2. **Passive model:** create a body and two wheel links with correct dimensions, inertia, joints, collision geometry, and ground contact. Check that the robot falls when unpowered.
3. **Sensors and actuation:** expose IMU and wheel state, test motor direction, and implement a bounded motor model. Simulated wheel feedback is not evidence of physical encoders.
4. **Balance:** implement an initial feedback controller, document sign conventions and update rate, and recover from a small initial tilt. Use torque/effort actuation or a physically limited motor model; avoid an ideal wheel-velocity source that hides motor limits.
5. **Evaluate:** repeat disturbances and initial tilts; record body angle, angular velocity, wheel motion, and commands. Assess drift as well as upright stability.
6. **Hardware comparison:** inspect Arduino code and actual sensors, measure real responses, and refine the model with documented evidence.
7. **Portfolio:** add plots, a demo video, setup instructions, experiment settings, and a clear explanation of limitations.

## Suggested experiments

- Recovery from small initial tilt angles with identical starting conditions.
- Response to a reproducible force pulse with specified magnitude and duration.
- Changes in body mass or centre-of-mass height.
- Sensitivity to sensor noise, motor saturation, and voltage loss.

Record the controller gains, physics step, controller period, motor assumptions, and random seed where relevant. Compare controllers under the same conditions.

## Scope of the first version

A simple box and two cylinders are sufficient. Detailed CAD, navigation, computer vision, and autonomy can follow once the balancing model is understood and validated.
