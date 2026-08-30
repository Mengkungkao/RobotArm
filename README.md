# robot_arm — a 6-DOF arm in Gazebo and in reality

A complete ROS 2 workspace for a 6-DOF robotic arm that runs **in Gazebo and on
physical hardware with the same control application**. Switching between the
two is one launch argument:

```bash
ros2 launch robot_arm_bringup bringup.launch.py use_sim:=true     # Gazebo
ros2 launch robot_arm_bringup bringup.launch.py use_sim:=false    # real robot
```

Target distribution: **ROS 2 Humble**. See [Newer ROS 2
distributions](#newer-ros-2-distributions) for what changes on Iron/Jazzy and
newer.

---

## Contents

1. [Architecture](#architecture)
2. [Repository layout](#repository-layout)
3. [Requirements](#requirements)
4. [Installing ROS 2, Gazebo and MoveIt 2](#installing-ros-2-gazebo-and-moveit-2)
5. [Workspace setup and build](#workspace-setup-and-build)
6. [Running the simulation](#running-the-simulation)
7. [RViz](#rviz)
8. [MoveIt 2](#moveit-2)
9. [Joint control](#joint-control)
10. [Cartesian control](#cartesian-control)
11. [Python API](#python-api)
12. [C++ API](#c-api)
13. [Real hardware setup](#real-hardware-setup)
14. [Encoder calibration](#encoder-calibration)
15. [Safety and emergency stop](#safety-and-emergency-stop)
16. [Simulation vs. real robot](#simulation-vs-real-robot)
17. [Configuration](#configuration)
18. [Testing](#testing)
19. [Troubleshooting](#troubleshooting)
20. [Extending the project](#extending-the-project)
21. [Newer ROS 2 distributions](#newer-ros-2-distributions)

---

## Architecture

```
                    ┌──────────────────────────────┐
                    │   User / GUI / Python / C++  │
                    │  robot_arm_tools, your app   │
                    └───────────────┬──────────────┘
                                    │
                    ┌───────────────▼──────────────┐
                    │       Motion planning        │
                    │          MoveIt 2            │
                    └───────────────┬──────────────┘
                                    │  FollowJointTrajectory
                    ┌───────────────▼──────────────┐
                    │        ros2_control          │
                    │  arm_controller (JTC)        │
                    │  joint_state_broadcaster     │
                    └───────────────┬──────────────┘
                                    │
                ┌───────────────────┴───────────────────┐
                ▼                                       ▼
      ┌───────────────────┐                 ┌───────────────────────┐
      │ Gazebo simulation │                 │    Real hardware      │
      │ GazeboSystem      │                 │ RobotArmSystemHardware│
      └───────────────────┘                 └───────────┬───────────┘
                                                        │
                                            ┌───────────▼───────────┐
                                            │   MotorProtocol       │
                                            │  (wire format)        │
                                            └───────────┬───────────┘
                                            ┌───────────▼───────────┐
                                            │     Transport         │
                                            │ serial │ RS485 │ CAN  │
                                            │ TCP    │ loopback     │
                                            └───────────┬───────────┘
                                            ┌───────────▼───────────┐
                                            │ Motor controller      │
                                            │ drives + encoders     │
                                            └───────────────────────┘
```

The split point is **ros2_control**, and it is the only place where the two
worlds differ. Everything above it — MoveIt, the controllers, the safety
monitor, the topics, the services, the Python and C++ APIs, the CLI tools — is
loaded from the same files in both modes.

Three design rules keep it that way:

* **One robot description.** `robot_arm_description` generates the URDF for
  every backend; `hardware_type:=gazebo|mock|real` swaps only the
  `<ros2_control>` block. A test asserts that all three expose *identical*
  joint interfaces.
* **One controller configuration.** `robot_arm_control/config/controllers.yaml`
  is handed to the Gazebo plugin in simulation and to `ros2_control_node` on
  hardware.
* **One planning frame.** `base_link` is the model root in every mode. The
  simulation adds a `world` anchor to the URDF because the physics engine needs
  one; MoveIt never sees it, so pose goals mean the same thing everywhere.

---

## Repository layout

```
robot_arm_ws/
└── src/
    ├── robot_arm_description/      URDF/Xacro, meshes, RViz config
    ├── robot_arm_bringup/          launch entry points for the whole system
    ├── robot_arm_control/          controllers, safety monitor, Python + C++ APIs
    ├── robot_arm_hardware/         ros2_control driver for the real robot
    ├── robot_arm_simulation/       Gazebo worlds and simulation launch
    ├── robot_arm_moveit_config/    SRDF, kinematics, planners, MoveIt launch
    ├── robot_arm_msgs/             message definitions
    ├── robot_arm_interfaces/       service definitions
    └── robot_arm_tools/            command-line tools
```

| Package | Purpose | Build type |
| --- | --- | --- |
| **robot_arm_description** | The robot: links, joints, inertias, collision geometry, joint limits, the `<ros2_control>` block for each backend and the Gazebo tags. All mechanical values live in `config/robot.yaml`; the Xacro derives the kinematic chain from them. | `ament_cmake` |
| **robot_arm_bringup** | Launch entry points (`bringup`, `sim`, `real`, `real_robot`, `moveit`, `rviz`) and the map of where every configuration file lives. Contains no logic. | `ament_cmake` |
| **robot_arm_control** | The control layer: `controllers.yaml` and `safety.yaml`, the `safety_monitor` node, the Python API (`robot_arm_control.RobotArm`) and the C++ API (`robot_arm_control::RobotArmClient`). | `ament_cmake` + `ament_cmake_python` |
| **robot_arm_hardware** | The ros2_control `SystemInterface` for the physical arm, plus the transport abstraction (serial/RS485/CAN/TCP/loopback), the motor-controller protocol abstraction, encoder and calibration handling, the software safety layer and the driver's ROS interface. | `ament_cmake` |
| **robot_arm_simulation** | Gazebo world files and the simulation launch file. Contains no control logic — the simulator is just another ros2_control backend. | `ament_cmake` |
| **robot_arm_moveit_config** | SRDF, kinematics, joint limits, planning pipelines, the mapping from MoveIt onto `arm_controller`, and the MoveIt RViz setup. One configuration for both modes. | `ament_cmake` |
| **robot_arm_msgs** | Data types: `JointDiagnostic`, `ArmStatus`, `SafetyStatus`, `JointCalibration`. | `ament_cmake` |
| **robot_arm_interfaces** | Command contracts: `SetMotorEnable`, `SetEStop`, `CalibrateJoint`, `SaveCalibration`, `GetCalibration`. Split from the messages so a node that only listens does not depend on the command interfaces. | `ament_cmake` |
| **robot_arm_tools** | `move_joint`, `move_pose`, `fk`, `ik`, `status`, `stop`, `e_stop`, `calibrate_joints`. Thin wrappers around the Python API. | `ament_python` |

### The robot

| Joint | Function | Axis | Range | Max velocity | Max effort |
| --- | --- | --- | --- | --- | --- |
| `joint_1` | base rotation | Z | ±170° | 5.03 rad/s | 300 Nm |
| `joint_2` | shoulder | Y | -100° … +135° | 4.19 rad/s | 300 Nm |
| `joint_3` | elbow | Y | -200° … +70° | 5.18 rad/s | 150 Nm |
| `joint_4` | wrist rotation | Z | ±270° | 6.98 rad/s | 40 Nm |
| `joint_5` | wrist bend | Y | ±130° | 7.07 rad/s | 40 Nm |
| `joint_6` | tool rotation | Z | ±400° | 10.47 rad/s | 20 Nm |

Frames: `world → base_link → link_1 … link_6 → tool0 → gripper_mount_link →
tool_tip`, plus `motor_1 … motor_6` rigidly attached to the links that carry
them.

The proportions, envelope and drive train follow the classic **ABB
IRB-1200 class**: 0.9 m reach, ~5 kg payload, ~52 kg, a rotating column, a
**cranked elbow** (the forearm is offset 42 mm from the elbow axis) and a
compact three-roll wrist. Joints 4/5/6 form a spherical wrist — their axes
intersect at the `joint_5` origin — which keeps inverse kinematics well
conditioned. The whole shape is built from primitives: it carries no vendor
mesh, badge or branding, and it is not affiliated with ABB.

| | |
| --- | --- |
| Reach (axis 2 → wrist centre) | 0.899 m |
| Height of axis 2 | 0.399 m |
| Wrist centre → flange | 0.082 m |
| Total mass incl. drives | 51.9 kg |
| Drive units | 6, 11.5 kg combined |

The six servo/gearbox assemblies are real links with mass and inertia, so the
dynamics account for them instead of pretending the drives are weightless.
Their `motor_id` is the same id the driver addresses on the bus, and a test
fails the build if the two ever disagree. Render the model without RViz:

```bash
xacro urdf/robot_arm.urdf.xacro > /tmp/arm.urdf
ros2 run robot_arm_description urdf_preview.py /tmp/arm.urdf /tmp/arm.png 0,40,-60,0,45,0
```

---

## Requirements

### Software

| Component | Version |
| --- | --- |
| Ubuntu | 22.04 (for Humble) |
| ROS 2 | Humble Hawksbill |
| Gazebo | Gazebo Classic 11 (`gazebo_ros_pkgs`, `gazebo_ros2_control`) |
| MoveIt | MoveIt 2 for Humble |
| ros2_control | `ros2_control`, `ros2_controllers` |
| Python | 3.10+ |
| C++ | C++17 |

### Hardware (only for the real robot)

* A 6-axis mechanism with an encoder on every joint.
* A motor controller reachable over serial, RS485, CAN or Ethernet, able to
  accept a position (or velocity) setpoint and report encoder counts at the
  control rate (100 Hz by default).
* Optionally: motor current and temperature reporting, which the diagnostics
  will surface automatically.
* A **hardware** emergency-stop chain. The software e-stop in this project is a
  complement to it, never a replacement.

No hardware is needed to develop: `hardware_type:=mock` runs the whole stack
with no backend at all, and the default `loopback` bus runs the *real driver*
against simulated drives.

---

## Installing ROS 2, Gazebo and MoveIt 2

Add the ROS 2 apt repository with the official `ros2-apt-source` package. Do
**not** use the older `curl ros.key` + `echo ... > /etc/apt/sources.list.d/ros2.list`
recipe that still circulates: `ros2-apt-source` ships the signing key inline and
updates it when it rotates, and running both methods breaks `apt` outright —
see [Troubleshooting](#troubleshooting).

```bash
# --- ROS 2 Humble ---------------------------------------------------------
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository universe

# Official apt source (supersedes the hand-rolled key + ros2.list method)
export ROS_APT_SOURCE_VERSION=$(
    curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F '"tag_name"' | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $UBUNTU_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb && rm /tmp/ros2-apt-source.deb

sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools

# --- Gazebo Classic + ros2_control ---------------------------------------
sudo apt install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers

# --- MoveIt 2 -------------------------------------------------------------
sudo apt install -y \
    ros-humble-moveit \
    ros-humble-moveit-resources \
    ros-humble-moveit-visual-tools

# optional: the Pilz PTP/LIN/CIRC planner
sudo apt install -y ros-humble-pilz-industrial-motion-planner
```

Verify:

```bash
source /opt/ros/humble/setup.bash
ros2 --version
gazebo --version
ros2 pkg list | grep -E "moveit_ros_move_group|gazebo_ros2_control"
```

Exactly one file should describe the ROS repo. If this prints more than
`/etc/apt/sources.list.d/ros2.sources`, delete the extras — two files naming the
same repo with different `Signed-By` values make every `apt update` fail:

```bash
grep -rl "packages.ros.org" /etc/apt/sources.list /etc/apt/sources.list.d/
```

---

## Workspace setup and build

```bash
git clone <this-repository> ~/RobotArm
cd ~/RobotArm/robot_arm_ws

source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` is worth using: configuration files and Python sources are
then symlinked rather than copied, so editing `robot.yaml` or a CLI tool takes
effect without rebuilding — and `calibrate_joints --save` writes back to the
source file.

Optional plain-Python helpers used by some tooling:

```bash
python3 -m pip install -r ../requirements.txt
```

Build one package at a time while developing:

```bash
colcon build --symlink-install --packages-select robot_arm_hardware
colcon build --symlink-install --packages-up-to robot_arm_bringup
```

### Verifying the robot description (no simulator needed)

```bash
source install/setup.bash

# expand the Xacro and check the URDF structure
xacro $(ros2 pkg prefix robot_arm_description)/share/robot_arm_description/urdf/robot_arm.urdf.xacro \
    > /tmp/robot_arm.urdf
check_urdf /tmp/robot_arm.urdf
urdf_to_graphiz /tmp/robot_arm.urdf        # optional: writes robot_arm.pdf

# look at the model and drag the joints
ros2 launch robot_arm_description display.launch.py
```

`check_urdf` should report `base_link` as the root and list the chain down to
`tool0`.

---

## Running the simulation

Everything at once — Gazebo, ros2_control, MoveIt and RViz:

```bash
source /opt/ros/humble/setup.bash
# local_setup, not setup: a colcon `setup.bash` replays the underlay chain
# recorded at build time, pulling in unrelated workspaces.
source ~/RobotArm/robot_arm_ws/install/local_setup.bash
ros2 launch robot_arm_bringup sim.launch.py
```

Just the simulator and the controllers, without MoveIt:

```bash
ros2 launch robot_arm_simulation simulation.launch.py
```

Headless (CI, or a machine with no GPU):

```bash
ros2 launch robot_arm_bringup sim.launch.py gazebo_gui:=false use_rviz:=false
```

Check that it came up:

```bash
ros2 control list_controllers
# joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
# arm_controller     [joint_trajectory_controller/JointTrajectoryController] active

ros2 topic echo /joint_states --once
ros2 run robot_arm_tools status
```

---

## RViz

RViz is started by the bringup launch files. To run it on its own against an
already-running robot:

```bash
ros2 launch robot_arm_bringup rviz.launch.py                    # model + TF + joint states
ros2 launch robot_arm_bringup rviz.launch.py use_moveit:=true   # + MotionPlanning panel
```

The MoveIt configuration displays the robot model, TF, the planning scene,
collision objects, the planned trajectory and the interactive goal marker. Drag
the marker on `tool0`, then **Plan** and **Execute** in the MotionPlanning
panel.

If the robot does not appear, set **Fixed Frame** to `world` (or `base_link`)
in Global Options — that is nearly always the cause.

---

## MoveIt 2

Started automatically by `sim.launch.py` / `real.launch.py`. To attach it to a
robot that is already running:

```bash
ros2 launch robot_arm_bringup moveit.launch.py hardware_type:=gazebo use_rviz:=true
```

The configuration provides:

* planning group **`arm`**: `joint_1` … `joint_6`, from `base_link` to `tool0`
* named poses: `zero`, `home`, `ready`, `folded`
* KDL inverse kinematics (see `config/kinematics.yaml` for faster alternatives)
* OMPL planners, RRTConnect by default
* optional Pilz PTP/LIN/CIRC:

  ```bash
  ros2 launch robot_arm_bringup sim.launch.py \
      planning_pipelines:="ompl pilz_industrial_motion_planner"
  ```

* execution onto `arm_controller` through `FollowJointTrajectory`

---

## Joint control

Example target: `joint_1 = 0°, joint_2 = 30°, joint_3 = -45°, joint_4 = 0°,
joint_5 = 45°, joint_6 = 0°`.

```bash
# radians
ros2 run robot_arm_tools move_joint -- \
    --j1 0 --j2 0.5 --j3 -0.8 --j4 0 --j5 0.5 --j6 0

# degrees
ros2 run robot_arm_tools move_joint -- \
    --j1 0 --j2 30 --j3 -45 --j4 0 --j5 45 --j6 0 --degrees

# jog one joint; the others keep their current value
ros2 run robot_arm_tools move_joint -- --j3 -45 --degrees

# slower, for a first run on hardware
ros2 run robot_arm_tools move_joint -- --j2 0.5 --velocity-scaling 0.1
```

Bypass MoveIt and command the trajectory controller directly (no collision
checking):

```bash
ros2 run robot_arm_tools move_joint -- --j1 0.5 --no-moveit --duration 3
```

Or by hand, which is what the tools do underneath:

```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory \
    control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6],
    points: [{positions: [0.0, 0.5, -0.8, 0.0, 0.5, 0.0],
              time_from_start: {sec: 4}}]
  }}"
```

---

## Cartesian control

Move `tool0` to x = 0.35 m, y = 0.10 m, z = 0.40 m, pointing down:

```bash
ros2 run robot_arm_tools move_pose -- \
    --x 0.35 --y 0.10 --z 0.40 --roll 0 --pitch 1.57 --yaw 0
```

Straight-line motion instead of a free-space plan:

```bash
ros2 run robot_arm_tools move_pose -- --x 0.35 --y 0.0 --z 0.30 --linear
```

Kinematics without moving anything:

```bash
# forward: joint angles -> tool0 pose (no --jN = the current pose)
ros2 run robot_arm_tools fk
ros2 run robot_arm_tools fk -- --j1 0 --j2 0.5 --j3 -0.8 --j4 0 --j5 0.5 --j6 0

# inverse: tool0 pose -> joint angles
ros2 run robot_arm_tools ik -- --x 0.35 --y 0.10 --z 0.40 --pitch 1.57

# ... and go there
ros2 run robot_arm_tools ik -- --x 0.35 --y 0.10 --z 0.40 --pitch 1.57 --execute
```

An unreachable pose is reported as "no IK solution" with exit code 1 — a normal
answer, not an error.

---

## Python API

```python
from robot_arm_control import RobotArm

with RobotArm() as robot:
    robot.enable()

    robot.move_joints([0.0, 0.5, -0.8, 0.0, 0.5, 0.0])

    robot.move_to_pose(
        x=0.35, y=0.10, z=0.40,
        roll=0.0, pitch=1.57, yaw=0.0,
    )

    print(robot.get_joint_states().as_dict())
    print(robot.get_current_pose_rpy())

    robot.stop()
    robot.disable()
```

| Method | Meaning |
| --- | --- |
| `move_joints(positions)` | plan and execute a joint-space motion |
| `move_to_pose(x, y, z, roll, pitch, yaw)` | plan and execute a Cartesian motion |
| `move_linear(x, y, z, ...)` | straight-line motion; fails rather than moving part way |
| `get_joint_states()` / `get_joint_positions()` | latest positions, velocities, efforts |
| `get_current_pose()` / `get_current_pose_rpy()` | `tool0` pose via TF |
| `forward_kinematics(joints)` / `inverse_kinematics(x, y, z, ...)` | FK/IK through MoveIt |
| `stop()` | abort the running motion and hold the pose |
| `enable()` / `disable()` | drives on the real robot, controller in simulation |
| `set_e_stop(engage)` / `emergency_stop()` | latched emergency stop |
| `home()` | move to the calibrated home pose |
| `get_calibration()` | the calibration in force (real robot) |

The API uses plain `rclpy` and `moveit_msgs`, so no `moveit_py` build is
required. `RobotArm(use_moveit=False)` drops the MoveIt dependency entirely and
commands the trajectory controller directly — handy for tests and minimal
installations.

Run the worked example against either backend:

```bash
ros2 run robot_arm_control python_api_demo.py
```

---

## C++ API

```cpp
#include "robot_arm_control/robot_arm_client.hpp"

auto node = std::make_shared<rclcpp::Node>("my_app", options);
rclcpp::executors::SingleThreadedExecutor executor;
executor.add_node(node);
std::thread spinner([&executor]() {executor.spin();});

robot_arm_control::RobotArmClient robot(node);

robot.enable();
robot.moveJoints({0.0, 0.5, -0.8, 0.0, 0.5, 0.0});
robot.moveToPose(0.35, 0.10, 0.40, 0.0, 1.57, 0.0);

const auto states = robot.getJointStates();
const auto pose = robot.getCurrentPose();

robot.stop();
robot.disable();
```

`moveJoints`, `moveToPose`, `moveLinear`, `getJointStates`, `getCurrentPose`,
`stop`, `enable`, `disable`, `setEStop`. Link against `robot_arm_client`; the
node must be spinning and must have `robot_description` and
`robot_description_semantic` (the launch file below passes both):

```bash
ros2 launch robot_arm_control cpp_api_demo.launch.py
```

The C++ API is built only when MoveIt is installed. Everything else in
`robot_arm_control` builds without it.

---

## Real hardware setup

### 1. Describe the machine

`robot_arm_description/config/robot.yaml` — link lengths, radii, masses, joint
limits, damping and friction. The kinematic chain is derived from the link
lengths, so joint origins cannot fall out of sync with the geometry.

### 2. Describe the drives

`robot_arm_hardware/config/hardware.yaml`:

```yaml
robot_arm_hardware:
  transport:
    type: serial              # loopback | serial | rs485 | can | tcp
    serial:
      port: /dev/ttyUSB0
      baudrate: 921600
    can:
      interface: can0
      base_id: 256
    tcp:
      host: 192.168.1.50
      port: 5000
    read_timeout_ms: 8        # must stay below one control period
    write_timeout_ms: 8

  protocol:
    type: simple_ascii        # loopback | simple_ascii | your own
    checksum: true

  control:
    command_timeout: 0.25     # controller watchdog [s]
    comm_timeout: 0.20        # bus watchdog [s]
    max_consecutive_errors: 5
    enable_on_activate: true

  joints:
    joint_1:
      motor_id: 1
      encoder_resolution: 4096   # counts per MOTOR revolution
      gear_ratio: 100.0          # motor turns per JOINT turn
      encoder_direction: 1
      torque_constant: 0.085     # Nm/A
      max_current: 8.0
      max_temperature: 70.0
    # ... one block per joint; nothing is assumed to be shared
```

The defaults are `loopback`/`loopback`, so a fresh clone runs the real driver
against simulated drives and **cannot move a machine by accident**.

### 3. Bring it up

```bash
# still no hardware: the whole stack, driver included, on the loopback bus
ros2 launch robot_arm_bringup real.launch.py

# with hardware, once transport.type points at your bus
sudo usermod -aG dialout $USER        # serial: log out and back in afterwards
ros2 launch robot_arm_bringup real.launch.py
```

Expected log lines:

```
[INFO] Configured 6 joints, transport 'serial(/dev/ttyUSB0@921600)', protocol 'simple_ascii(checksum)'
[INFO] Robot connected via serial(/dev/ttyUSB0@921600)
[INFO] Encoder feedback received, joint states initialised
[INFO] Joint controllers active, hardware ready
```

For a CAN setup, bring the interface up first:

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0

# or test the whole stack on a virtual bus
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
```

### 4. Speak your controller's language

`SimpleAsciiProtocol` is a complete, documented reference implementation
(`include/robot_arm_hardware/protocol/simple_ascii_protocol.hpp`):

```
->  #C 1:P409600 2:P-102400 3:P0 4:P0 5:P51200 6:P0*63     setpoints, one frame
->  #Q*51                                                  request feedback
<-  #F 1:409580,120,850,310,0 2:...*55                     counts, counts/s, mA, 0.1°C, fault
->  #E 1*54                                                enable
<-  #A OK*65                                               acknowledge
```

If your controller speaks something else, implement `MotorProtocol` (six
methods) and register it in `create_protocol()`. If it is on a bus that is not
listed, implement `Transport` (six methods) and register it in
`create_transport()`. Neither ros2_control, nor MoveIt, nor any application
changes. See `robot_arm_ws/src/robot_arm_hardware/README.md`.

---

## Encoder calibration

Encoders report *raw* angles. Calibration records the offset that makes a raw
reading equal the true joint angle:

```
q_joint = direction * (q_raw - zero_offset)
```

The driver applies it to every reading and every command, so nothing above it
needs to know the encoders are not zeroed.

```bash
# what is in force right now
ros2 run robot_arm_tools calibrate_joints -- --show

# guided: move each joint to a known reference, type the true angle
ros2 run robot_arm_tools calibrate_joints

# one joint, no prompts: "this pose is exactly 0 rad"
ros2 run robot_arm_tools calibrate_joints -- --joint joint_3 --position 0.0

# persist to robot_arm_hardware/config/calibration.yaml
ros2 run robot_arm_tools calibrate_joints -- --save
```

The tool de-energises the drives first: a powered joint fights you, and the
reading would be meaningless.

```yaml
calibration:
  joint_1:
    zero_offset: 0.0        # [rad] raw angle at the mechanical zero
    direction: 1            # +1 / -1 relative to the URDF axis
    min_position: -3.1416   # enforced soft limits
    max_position: 3.1416
    home_position: 0.0
```

`min_position` / `max_position` are enforced by the driver, not documentation:
a command outside them is clamped before it reaches the drives. Keep this file
in version control — it describes *your* machine.

---

## Safety and emergency stop

Two layers, both configurable:

**In the driver** (`robot_arm_hardware`, inside the control loop, real robot
only):

| Condition | Reaction |
| --- | --- |
| Command outside the calibrated limits | clamped, warning logged |
| Setpoint jump faster than `max_velocity` | rate-limited |
| NaN in a command | replaced by the measured pose |
| Implausible or NaN encoder value | reading rejected; repeated failures stop the arm |
| No valid feedback within `comm_timeout` | motion blocked, controlled stop |
| No `write()` within `command_timeout` | watchdog stops the arm; on resume the commands are re-seeded from the encoders, never from the stale setpoint |
| `max_consecutive_errors` bus errors | drives disabled, hardware deactivated |

**In the monitor** (`robot_arm_control/safety_monitor`, in **both** modes):
watches `/joint_states` for position, velocity and effort violations, owns the
latched emergency stop, deactivates `arm_controller` so a running trajectory is
aborted rather than resumed, and publishes `/robot_arm/safety_status` and
`/diagnostics`.

```bash
# emergency stop
ros2 run robot_arm_tools e_stop -- --engage
ros2 run robot_arm_tools e_stop -- --release --enable

# or from anywhere
ros2 topic pub --once /e_stop std_msgs/msg/Bool "{data: true}"
ros2 service call /robot_arm/set_e_stop robot_arm_interfaces/srv/SetEStop \
    "{engage: true, reason: 'operator'}"

# just stop the motion, keep power
ros2 run robot_arm_tools stop
```

When engaged: commands are zeroed, the drives are de-energised, trajectory
execution is aborted and the state is latched. Releasing clears the software
latch only — the drives stay off until they are enabled on purpose.

Tune the limits in `robot_arm_control/config/safety.yaml`. A test fails the
build if any of them is wider than the mechanical limit in `robot.yaml`.

> The software e-stop cannot help when the software itself is what failed.
> Keep a hardware e-stop chain that cuts motor power independently.

Diagnostics:

```bash
ros2 topic echo /diagnostics
ros2 topic echo /robot_arm/safety_status
ros2 topic echo /robot_arm_hardware/status      # real robot only
ros2 run robot_arm_tools status -- --watch
```

---

## Simulation vs. real robot

```bash
ros2 launch robot_arm_bringup sim.launch.py       # MoveIt -> ros2_control -> Gazebo
ros2 launch robot_arm_bringup real.launch.py      # MoveIt -> ros2_control -> hardware
```

or explicitly:

```bash
ros2 launch robot_arm_bringup bringup.launch.py use_sim:=true  use_rviz:=true use_moveit:=true
ros2 launch robot_arm_bringup bringup.launch.py use_sim:=false use_rviz:=true use_moveit:=true

# no simulator and no hardware, for offline development
ros2 launch robot_arm_bringup bringup.launch.py hardware_interface:=mock
```

| | Simulation | Real robot |
| --- | --- | --- |
| ros2_control backend | `gazebo_ros2_control/GazeboSystem` | `robot_arm_hardware/RobotArmSystemHardware` |
| Controllers | `controllers.yaml` | the same file |
| MoveIt configuration | `robot_arm_moveit_config` | the same package |
| Planning frame / tip | `base_link` / `tool0` | the same |
| Topics, actions, services | `/joint_states`, `/arm_controller/...`, `/robot_arm/...` | the same |
| Python / C++ APIs, CLI tools | unchanged | unchanged |
| Clock | simulated | wall clock |

The recommended workflow is to develop the entire application against Gazebo,
then run it against the loopback bus (`hardware_type:=real` with the default
`transport.type: loopback`) to exercise the driver, and only then connect the
machine.

---

## Configuration

Every configuration file lives next to the package that owns it; see
[`robot_arm_ws/src/robot_arm_bringup/config/README.md`](robot_arm_ws/src/robot_arm_bringup/config/README.md)
for the full map.

| File | Package | Contents |
| --- | --- | --- |
| `robot.yaml` | `robot_arm_description` | geometry, masses, joint limits, dynamics |
| `initial_positions.yaml` | `robot_arm_description` | start-up joint values |
| `controllers.yaml` | `robot_arm_control` | ros2_control controllers |
| `safety.yaml` | `robot_arm_control` | safety-monitor limits |
| `hardware.yaml` | `robot_arm_hardware` | bus, protocol, timeouts, per-joint drive data |
| `calibration.yaml` | `robot_arm_hardware` | zero offsets, directions, soft limits |
| `kinematics.yaml` | `robot_arm_moveit_config` | IK solver |
| `joint_limits.yaml` | `robot_arm_moveit_config` | limits for time parameterisation |
| `moveit_controllers.yaml` | `robot_arm_moveit_config` | MoveIt → `arm_controller` |
| `ompl_planning.yaml` | `robot_arm_moveit_config` | planners |

Nothing hardware-specific is compiled in: motor ids, encoder resolutions, gear
ratios, joint limits, zero offsets, ports, CAN ids, baud rates and the control
frequency are all YAML. Override the paths without editing anything:

```bash
ros2 launch robot_arm_bringup real.launch.py safety_config:=/etc/robot_arm/safety.yaml

xacro robot_arm.urdf.xacro hardware_type:=real \
    robot_config_file:=/etc/robot_arm/robot.yaml \
    hardware_config_file:=/etc/robot_arm/hardware.yaml \
    calibration_config_file:=/etc/robot_arm/calibration.yaml
```

---

## Testing

```bash
cd ~/RobotArm/robot_arm_ws
colcon test
colcon test-result --verbose

# one package
colcon test --packages-select robot_arm_hardware
```

| Suite | What it covers |
| --- | --- |
| `robot_arm_description/test_urdf.py` | Xacro expands in every backend mode; single-rooted TF tree with no cycles; complete joint limits, axes and dynamics; inertias positive; collision geometry primitive; **identical joint interfaces across gazebo, mock and real** |
| `robot_arm_hardware/test_joint_config` | encoder ↔ radian round trips, gear ratios, directions, zero offsets, torque conversion, implausible-value rejection, configuration validation |
| `robot_arm_hardware/test_safety_checker` | limit clamping, rate limiting, NaN handling, command and communication timeouts, latched e-stop, feedback validation |
| `robot_arm_hardware/test_transport` | factory contract, loopback round trips, failure injection, missing serial port handled cleanly |
| `robot_arm_hardware/test_protocol` | framing, checksums, feedback parsing, missing axes reported invalid, simulated-drive behaviour |
| `robot_arm_control/test_config.py` | controllers cover every joint; safety and calibration limits never exceed the mechanical ones; bus timeouts fit inside one control period |
| `robot_arm_control/test_python_api.py` | rotation round trips, argument validation, graceful behaviour with no backend running |
| `robot_arm_moveit_config/test_moveit_config.py` | group spans `base_link`→`tool0`; named poses complete and reachable; MoveIt limits match the URDF; MoveIt executes onto the controller ros2_control really loads |
| `robot_arm_simulation/test_simulation_assets.py` | valid SDF; physics step fine enough for the control period; launch arguments intact |
| `robot_arm_tools/test_cli.py` | argument parsing, degree conversion, omitted-joint fill-in, exit codes |
| `robot_arm_bringup/test_launch_files.py` | every launch file loads; sim and real differ only in `use_sim`; no duplicate RViz or safety monitor |
| `robot_arm_bringup/test_bringup_loopback.py` | **end-to-end**: the real hardware plugin on the loopback bus — controllers active, encoder feedback flowing, a trajectory reaching the drives, calibration readable, e-stop deactivating the controller and refusing to re-enable |

The C++ unit tests and every configuration test run without ROS, without a
simulator and without hardware, which makes them usable in a plain CI container.

### Without a ROS installation

The safety-critical core of the driver builds with plain CMake, so it can be
compiled and tested anywhere - including under a sanitizer:

```bash
cd robot_arm_ws/src/robot_arm_hardware
cmake -S standalone -B build-core -DCMAKE_BUILD_TYPE=Release
cmake --build build-core -j
ctest --test-dir build-core --output-on-failure     # 4 suites, 53 cases
```

The rest of the offline checks need no build at all:

```bash
cd robot_arm_ws/src
xacro robot_arm_description/urdf/robot_arm.urdf.xacro hardware_type:=real   # expands
python3 -m pytest robot_arm_control/test robot_arm_moveit_config/test \
                 robot_arm_tools/test robot_arm_simulation/test -q
python3 -m flake8 --max-line-length=99 .
```

---

## Troubleshooting

**`apt update` fails with "Conflicting values set for option Signed-By".**
Two files describe the `packages.ros.org` repo with different keys: the official
deb822 `/etc/apt/sources.list.d/ros2.sources` (from the `ros2-apt-source`
package) and a legacy one-line `ros2.list` left by the older
`curl ros.key` install recipe. Keep the first, delete the second — nothing is
uninstalled by removing a source file, and `ros2.sources` already covers the
same repo:

```bash
sudo rm /etc/apt/sources.list.d/ros2.list /usr/share/keyrings/ros-archive-keyring.gpg
sudo apt update
```

If it comes back, something re-ran the legacy recipe. See
[Installing ROS 2, Gazebo and MoveIt 2](#installing-ros-2-gazebo-and-moveit-2).

**`ros2 pkg list` shows packages from another robot project.**
A colcon `install/setup.bash` replays the underlay chain recorded when that
workspace was built, so sourcing it can drag in unrelated workspaces. Source
`install/local_setup.bash` instead — it adds only that workspace's own packages.
This matters when several ROS projects share one machine; give each its own
`ROS_DOMAIN_ID` so their nodes cannot see each other either.

**The robot does not appear in RViz.**
Set **Fixed Frame** to `world` or `base_link`. Then check that the description
is being published: `ros2 topic echo /robot_description --once`.

**`Controller 'arm_controller' not found` / the spawner times out.**
The controller_manager is not up yet. In simulation it lives inside the Gazebo
plugin, so the model must be spawned first — check for
`gazebo_ros2_control` errors in the Gazebo output, and confirm with
`ros2 control list_hardware_components`.

**MoveIt plans but nothing moves.**
The controller name in `moveit_controllers.yaml` must match the one
ros2_control loaded. Check `ros2 control list_controllers` and confirm
`arm_controller` is `active`. A test covers this pairing, so a mismatch usually
means a controller failed to activate rather than a typo.

**`No valid encoder feedback after configure`.**
The driver could not read the drives. Check the port/interface in
`hardware.yaml`, permissions (`sudo usermod -aG dialout $USER`), the baud rate,
and whether the controller answers `#Q` at all. Try `transport.type: loopback`
to confirm the rest of the stack is fine.

**`Motor controller communication lost`.**
`comm_timeout` or `max_consecutive_errors` was exceeded. Look for cabling and
termination problems first; on RS485 check whether the transceiver needs
`rs485_rts_toggle: true`. `read_timeout_ms` must stay below one control period.

**The arm moves to the wrong place, or mirrored.**
Calibration. Check `direction` and `zero_offset` per joint
(`ros2 run robot_arm_tools calibrate_joints -- --show`); a mirrored joint is
`direction` inverted, a constant offset is `zero_offset`.

**Gazebo starts, the robot spawns, but no controller ever goes active.**
Look for `parser error` from `gazebo_ros2_control` in the Gazebo output. The
plugin re-passes the whole robot description on a command line as
`-p robot_description:=<xml>`, and rcl parses that as a YAML scalar — which a
pretty-printed URDF is not. `simulation.launch.py` therefore feeds Gazebo the
model through `compact_xacro.py`, which emits it as one comment-free line. If
you build your own description for Gazebo, do the same:

```bash
ros2 run robot_arm_description compact_xacro.py \
    $(ros2 pkg prefix --share robot_arm_description)/urdf/robot_arm.urdf.xacro \
    hardware_type:=gazebo use_world_frame:=true
```

**Motion is jerky in Gazebo.**
Lower `velocity_scaling`, raise the solver iterations in
`worlds/robot_arm.world`, and make sure the physics step is well below the
control period. A machine that cannot keep real-time factor near 1.0 will look
jerky no matter what.

**`ros2 run robot_arm_tools move_joint --j1 0` says "unrecognized arguments".**
Put `--` after the executable name so `ros2 run` stops parsing:
`ros2 run robot_arm_tools move_joint -- --j1 0`.

**The e-stop cannot be released.**
Releasing clears the software latch but deliberately leaves the drives off. Use
`ros2 run robot_arm_tools e_stop -- --release --enable`, or call
`/robot_arm/set_motor_enable` afterwards.

**A colcon build fails after changing a message.**
Rebuild the dependants: `colcon build --packages-up-to robot_arm_bringup`, and
re-source `install/setup.bash`.

---

## Extending the project

| Goal | What to change |
| --- | --- |
| Different dimensions or masses | `robot_arm_description/config/robot.yaml` only — the chain is derived from it |
| CAD meshes instead of cylinders | drop files in `robot_arm_description/meshes/`, see the README there; keep collision geometry primitive |
| A gripper | attach it to `tool0` or `gripper_mount_link` from your own Xacro, add a `gripper` group to the SRDF and a controller to `controllers.yaml` |
| A different bus | implement `Transport`, register it in `create_transport()`, set `transport.type` |
| A different motor protocol | implement `MotorProtocol`, register it in `create_protocol()`, set `protocol.type` |
| Faster IK | change `kinematics_solver` in `kinematics.yaml` (pick_ik, TRAC-IK, IKFast) |
| Velocity control / servoing | activate `velocity_controller` instead of `arm_controller`; both are already configured |
| A second arm | pass `prefix:=left_` to the description and launch files |

---

## Newer ROS 2 distributions

The project targets Humble; these are the only places that need attention when
moving forward:

* **Simulator.** Gazebo Classic is end-of-life after Humble. The description
  already emits the right plugin for `sim_engine:=ignition` and `gz`; port
  `robot_arm_simulation/launch/simulation.launch.py` to `ros_gz_sim` and swap
  `gazebo_ros2_control` for `gz_ros2_control` in `package.xml`.
* **ros2_control.** From Iron onwards the controller manager can take the robot
  description from the `/robot_description` topic rather than a parameter; the
  parameter still works.
* **Hardware interface API.** Jazzy adds `get_node()` and typed state
  interfaces to hardware components; the driver's own node in
  `robot_arm_hardware_interface.cpp` can then be simplified.
* **MoveIt.** `MoveItConfigsBuilder` is stable across distributions; planner
  plugin names and the request-adapter list in `ompl_planning.yaml` occasionally
  change.

Nothing in the control architecture itself is distribution-specific.

---

## License

MIT — see [LICENSE](LICENSE).
