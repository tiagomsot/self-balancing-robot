# First ROS / Gazebo observations

Date:

Environment: ROS 2 Jazzy / Gazebo Harmonic in Ubuntu 24.04 through WSL.

## Explain the model

- Where is the body centre of mass relative to the axle?
- Which geometry and mass values came from measurements or estimates?
- Which wheel and contact values are assumptions?
- What does the green marker represent?

## Observe the unpowered robot

| Initial lean | Direction of fall | Sign of angular_velocity.y | Notes |
| --- | --- | --- | --- |
| +5 degrees | | | |
| -5 degrees | | | |

## Explain the ROS data

- What does gazebo_bridge do?
- What happens to /clock and sensor messages while paused?
- Does the physical IMU directly provide the orientation quaternion shown here?
- Does simulated wheel feedback imply that the physical robot has encoders?

## Next controller step

What quantities would your PD controller read, and what would it command?
