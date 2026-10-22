# First ROS 2 / Gazebo model

## What is implemented

The `balancing_robot_sim` ROS 2 package contains a passive physical robot, a
ground plane, an IMU, and a bridge carrying simulated measurements into ROS.
The robot starts paused with a 5-degree forward lean. Press Play to watch it
fall without a controller.

A [preset balancing demo](ros-balancing-demo.md) is also available with
`bash run_gazebo.sh`. This guide retains the passive launch as the unpowered
reference.

Verified development environment: Windows with VS Code connected to the
`Ubuntu-24.04-D` WSL 2 distro, Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic
(Gazebo Sim 8.15.0), and colcon. Run the following commands in an **Ubuntu
terminal**, such as Terminal > New Terminal in the connected VS Code window.

## Build and launch

Stop the shapes demo with Ctrl+C in its terminal. From the repository root:

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --symlink-install --packages-select balancing_robot_sim
source install/setup.bash
ros2 launch balancing_robot_sim passive.launch.py
```

`colcon build` installs the package into this workspace. Sourcing
`install/setup.bash` lets ROS find that package. New terminals need both the
ROS setup and the workspace setup sourced. The default launch starts paused;
wait for `Entity creation successful` and then press Play in Gazebo.
Stop with Ctrl+C. Stop and relaunch to repeat the same initial condition.

For the current checkout, a fresh terminal can source the workspace with:

```bash
source /opt/ros/jazzy/setup.bash
source /mnt/d/Work/Portofolio/self-balancing-robot/ros2_ws/install/setup.bash
```

Optional launch arguments:

```bash
ros2 launch balancing_robot_sim passive.launch.py initial_tilt_deg:=-5
ros2 launch balancing_robot_sim passive.launch.py gui:=false paused:=false
```

## Read your first ROS topics

In a second sourced terminal, while the simulation is playing:

```bash
ros2 node list
ros2 topic list
ros2 topic echo /imu/data --once
ros2 topic echo /joint_states --once
```

A **node** is a running ROS program. A **topic** is a named stream of messages.
Here Gazebo simulates the motion; `gazebo_bridge` translates Gazebo messages
into ROS messages; `robot_state_publisher` uses the robot description and wheel
joint angles to publish relative link transforms.

| ROS topic | Meaning |
| --- | --- |
| `/clock` | Simulation time, which stops advancing while paused |
| `/imu/data` | Ideal orientation, angular velocity, and specific force |
| `/joint_states` | Simulated wheel joint positions and velocities |
| `/robot_description` | Expanded URDF robot description |
| `/tf`, `/tf_static` | Relative transforms between the robot's links |

Sensor messages wait for physics steps. An `echo` command can wait while the
world is paused. The current transforms describe the robot's internal links;
publishing its global world pose into the ROS transform tree is later work.

## Geometry and assumptions

The xacro file is the single robot description. Gazebo converts its URDF to
SDF when spawning; robot_state_publisher reads the same expanded URDF.

| Property | Setting |
| --- | --- |
| Body | 0.035 m forward/back, 0.10 m along axle, 0.15 m tall |
| Body mass and COM | 0.42 kg, uniform box, COM 0.048 m above axle |
| Body bottom | 0.027 m below axle |
| Wheels | 0.04 m radius, 0.008 m axial width, 0.02 kg each (mass assumed) |
| Wheel centre separation | 0.168 m, owner confirmed center to center |
| Wheel and ground friction | 0.8 |
| IMU offset | 0.06 m above axle |
| Physics step / configured IMU rate | 0.001 s / 200 Hz |
| Initial pose | Axle 0.041 m above ground, body leaning 5 degrees forward |

Axle placement and track use the updated owner measurements. Wheel mass,
friction and IMU direction remain assumptions. Body mass includes onboard components. Box and solid
cylinder inertia tensors are calculated from these dimensions and masses.
`base_link` is at the axle midpoint; +x points forward, +y left along the axle,
and +z upward when upright. Positive pitch means a forward lean.

The world-referenced orientation is ideal simulator information. The physical
MPU-6050 provides raw accelerometer and gyro data, so an orientation estimator
will be a separate learning step. Wheel state in simulation does not establish
that the physical robot has encoders. A separate preset launch now adds wheel
torque and a ROS balance controller. An [averaged motor/driver surrogate](motor-driver-model.md)
now includes losses and current diagnostics in the controlled launch. Gearbox
backlash and realistic sensing remain pending. The Python PD experiment is a separate baseline.

## Your first exercise

Open `ros2_ws/src/balancing_robot_sim/urdf/robot.urdf.xacro` and find the body
mass, wheel radius, and IMU height. Relate each number to the physical robot.
Then launch once at +5 degrees and once at -5 degrees. Observe how
`angular_velocity.y` changes sign as it falls. Record your observations in
`experiments/first-ros.md`.

## Repeatable passive check

The check uses the real ROS IMU, wheel joint, and clock messages and advances
a fresh paused world by two simulated seconds. Keep this test isolated from
other Gazebo sessions. In a sourced terminal, from the repository root:

```bash
export GZ_PARTITION=balance_check
export ROS_DOMAIN_ID=92
ros2 launch balancing_robot_sim passive.launch.py gui:=false
```

Wait for successful robot creation. In a second terminal, from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
export GZ_PARTITION=balance_check
export ROS_DOMAIN_ID=92
python3 analysis/check_gazebo_passive.py
```

The check requires the fresh default 5-degree launch. It verifies the initial
orientation, falling beyond 45 degrees, finite gyro and wheel data, both wheel
joint names, and advancing simulation time. CSV and JSON are saved under
`outputs/gazebo-passive-check/`. Restart the launch before repeating the check.
Stop the test launch with Ctrl+C when finished. Use a new terminal for normal
work to return to its default ROS domain and Gazebo partition.

Verified on 2026-10-21: the ROS IMU began at 5.000 degrees, reached 96.051
degrees from upright, and was at 95.768 degrees near the end of the two-second
check. Both wheel positions were finite and matched by symmetry; the IMU
header frame was `imu_link`. This verifies the passive fall and message path,
not controlled balance or agreement with hardware.

## WSL graphics messages

The shapes demo opened successfully despite Qt warning about permissions on
`/mnt/wslg/runtime-dir` and EGL reporting permission denied for `/dev/dri`
devices. Window creation is verified; GPU acceleration has not been measured.
If rendering is smooth and simulation time advances, proceed with the model.
For a graphics failure, Gazebo documents this temporary software-rendering
fallback (which can be slower):

```bash
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch balancing_robot_sim passive.launch.py
```

Gazebo may also warn that `gz_frame_id` is an SDF extension. The field sets the
IMU message frame to `imu_link`, which is verified by the passive check.

## References

- [Gazebo: IMU sensors](https://gazebosim.org/docs/harmonic/sensors/)
- [ROS / Gazebo bridge](https://gazebosim.org/docs/harmonic/ros2_integration/)
- [Gazebo IMU reference-frame handling](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/imu/Imu.cc)
- [Gazebo graphics troubleshooting](https://gazebosim.org/docs/harmonic/troubleshooting/)

Package metadata still has a placeholder maintainer email and license. Select
these before publishing the repository.
