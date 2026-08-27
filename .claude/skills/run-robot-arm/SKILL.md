---
name: run-robot-arm
description: Launch this 6-DOF robot arm stack and verify it came up — headless mock, Gazebo simulation, or physical hardware. Use when asked to run, start, launch, or screenshot the arm, or to confirm a change works in the running system rather than only in tests.
---

# Running robot_arm

Workspace: `robot_arm_ws/`. Target distro: ROS 2 Humble (installed at `/opt/ros/humble`).

## Environment

Every command below assumes this preamble in each shell. `robot_arm_ws` is
built and sourced directly — unlike MiniRobot, its packages are not symlinked
into `~/ros2_ws`.

`~/.bashrc` defines a `robotarm` function that does all of this. It is a
function, so it does not exist in a non-interactive shell — inline it there:

```bash
source /opt/ros/humble/setup.bash
source ~/RobotArm/robot_arm_ws/install/local_setup.bash   # after the first build
export ROS_DOMAIN_ID=32
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

`local_setup.bash`, not `setup.bash`: a colcon `setup.bash` replays the underlay
chain recorded when the workspace was built, which drags unrelated workspaces
onto `AMENT_PREFIX_PATH`. `local_setup.bash` adds only this workspace.

Domain 32 is this project's lane — see *Running alongside the other projects*
below. The repo itself pins nothing, so without these exports the arm lands on
default domain 0 where anything else on the network can see it.
`ROS_LOCALHOST_ONLY=1` is right for the arm because sim and hardware both run
on this one machine; drop it only if you deliberately split the stack.

Do **not** source `~/ros2_ws/install/setup.bash` in the same shell — that
workspace carries MiniRobot's packages and its own conventions.

## Build

`install/` is absent on a clean clone, so the first run must build. Verified:
all 9 packages build clean on this machine (~18 s incremental, ~26 s cold).



```bash
colcon build --symlink-install
source install/setup.bash
```

After editing Python or config only, `--symlink-install` means no rebuild is
needed. After editing C++ (`robot_arm_hardware`, `robot_arm_control`) or any
`.msg`/`.srv`, rebuild just what changed:

```bash
colcon build --symlink-install --packages-select robot_arm_hardware
```

Nine packages: `robot_arm_bringup`, `_control`, `_description`, `_hardware`,
`_interfaces`, `_moveit_config`, `_msgs`, `_simulation`, `_tools`.

## Pick a mode

`bringup.launch.py` is the single entry point for all three. `hardware_interface`
overrides `use_sim` when both are given.

### mock — default for agent-driven runs

No simulator, no hardware. The full stack loads: controller_manager,
`joint_state_broadcaster`, `arm_controller`, MoveIt. Starts in seconds and
needs no GPU or display, so this is the right choice for checking that a
change wires up correctly.

```bash
ros2 launch robot_arm_bringup bringup.launch.py \
    hardware_interface:=mock use_rviz:=false use_moveit:=true
```

### sim — Gazebo, when the change is visual or physical

Use when the task is about geometry, collisions, contact, or when a screenshot
is wanted. Slower to start; Gazebo Classic needs the display (`DISPLAY=:0`,
session is Wayland so Gazebo runs through XWayland).

```bash
ros2 launch robot_arm_bringup bringup.launch.py \
    use_sim:=true use_rviz:=true use_moveit:=true
```

Give Gazebo up to ~30 s on a cold start before concluding it failed.

### real — physical hardware

**Never start this on your own initiative.** It energises the servos and the
arm can move at once. Run it only when the user has asked for the real robot in
this session, and say what will move before starting.

```bash
ros2 launch robot_arm_bringup bringup.launch.py \
    use_sim:=false use_rviz:=true use_moveit:=true use_safety_monitor:=true
```

Keep `use_safety_monitor:=true` on hardware. Confirm the e-stop is in reach.

## Running alongside the other projects

This machine hosts three ROS 2 projects that must not see each other. Two
stacks that disagree about domain or RMW cannot share a shell, and two that
*agree* by accident will discover each other's nodes and interleave their
topics — which looks like your change misbehaving.

| Project | Workspace sourced | Domain | RMW |
|---|---|---|---|
| robot_arm (this one) | `~/RobotArm/robot_arm_ws/install` | **32** | fastrtps |
| MiniRobot | `~/ros2_ws/install` | 31 | fastrtps |
| mdetect | `~/mdetect_ws/install` | 30 | cyclonedds |

One project per shell. Never source two of those workspaces into one shell:
the second `install/setup.bash` wins for overlapping names and you get a stack
assembled from two projects.

With distinct domains all three can run at the same time on this machine. Check
before blaming a change:

```bash
echo "domain=$ROS_DOMAIN_ID rmw=$RMW_IMPLEMENTATION"
ros2 node list          # only robot_arm nodes should appear
```

A `mini_*` or `mdetect_*` node in that list means the domains have collided.

## Verify it actually came up

Launch printing no error is not evidence. In a second sourced shell:

```bash
ros2 control list_controllers      # arm_controller + joint_state_broadcaster: active
ros2 topic hz /joint_states        # ~100 Hz (controller update_rate)
ros2 node list
```

`arm_controller` and `joint_state_broadcaster` both `active` is the signal the
stack is up. In sim, `/clock` must also be publishing, and everything runs with
`use_sim_time:=true`.

With MoveIt up, the `/move_group` node and the
`/arm_controller/follow_joint_trajectory` action are the seam between planning
and execution — check those before blaming a planner change.

A `velocity_controller` is configured but deliberately not started: it claims
the same six joints as `arm_controller`, so only one of the two can be active.
Switch, never load both.

## Drive it

```bash
ros2 topic pub -1 /arm_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  '{joint_names: [joint_1,joint_2,joint_3,joint_4,joint_5,joint_6],
    points: [{positions: [0,0,0,0,0,0], time_from_start: {sec: 3}}]}'
```

`robot_arm_tools` holds the Python API; `robot_arm_control` has
`cpp_api_demo.launch.py`. The README documents both.

## Shut down

Ctrl-C the launch, then confirm nothing survived — a stranded
`gzserver` holds the physics port and makes the next run fail confusingly:

```bash
pgrep -af 'gzserver|gzclient|ros2_control_node|move_group' || echo clean
```

## When it fails

- **`package 'robot_arm_bringup' not found`** — `install/setup.bash` not sourced,
  or sourced before the build finished. Re-source.
- **Controllers never reach `active`** — read the `controller_manager` output.
  In sim this manager lives *inside* the `gazebo_ros2_control` plugin, so its
  errors surface in the Gazebo output, not in a separate node.
- **MoveIt plans but execution aborts** — tolerance, not planning. See
  `allowed_start_tolerance` / `execution_duration_monitoring` in
  `src/robot_arm_moveit_config/config/moveit_controllers.yaml`.
- **Pilz LIN/PTP/CIRC unavailable** — `ros-humble-pilz-industrial-motion-planner`
  is not installed. OMPL is, and is the default.
