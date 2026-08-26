# Configuration map

The project keeps every configuration file next to the package that owns it,
rather than copying values into a second place where they can silently drift.
This is where the files listed in the project layout actually live:

| File | Location | Contents |
| --- | --- | --- |
| `robot.yaml` | `robot_arm_description/config/` | geometry, masses, joint limits, dynamics, Gazebo surfaces - the single source of mechanical truth, read by the Xacro model |
| `initial_positions.yaml` | `robot_arm_description/config/` | start-up joint values for the simulator and the mock backend |
| `controllers.yaml` | `robot_arm_control/config/` | ros2_control controller manager and controllers, used unchanged by Gazebo and by the real robot |
| `safety.yaml` | `robot_arm_control/config/` | safety-monitor limits and behaviour |
| `hardware.yaml` | `robot_arm_hardware/config/` | bus type and parameters, protocol, timeouts, per-joint drive data |
| `calibration.yaml` | `robot_arm_hardware/config/` | per-joint zero offset, direction, soft limits, home position |
| `kinematics.yaml` | `robot_arm_moveit_config/config/` | IK solver for the `arm` group |
| `joint_limits.yaml` | `robot_arm_moveit_config/config/` | limits MoveIt uses for time parameterisation |
| `moveit_controllers.yaml` | `robot_arm_moveit_config/config/` | how MoveIt executes onto `arm_controller` |
| `ompl_planning.yaml` | `robot_arm_moveit_config/config/` | planners and their parameters |

Consistency between them is enforced by tests, not by discipline:

* `robot_arm_control/test/test_config.py` - safety and calibration limits never
  exceed the mechanical limits; bus timeouts fit inside one control period;
  every joint has its own drive configuration.
* `robot_arm_moveit_config/test/test_moveit_config.py` - MoveIt's velocity
  limits match the URDF, the named poses are reachable, and the controller
  MoveIt executes onto is the one ros2_control actually loads.
* `robot_arm_description/test/test_urdf.py` - the URDF matches `robot.yaml`,
  and every backend exposes identical joint interfaces.

To point the stack at a different machine, override the file paths as launch
or Xacro arguments - nothing has to be edited in place:

```bash
ros2 launch robot_arm_bringup real.launch.py \
    safety_config:=/etc/robot_arm/safety.yaml

xacro robot_arm.urdf.xacro \
    hardware_type:=real \
    robot_config_file:=/etc/robot_arm/robot.yaml \
    hardware_config_file:=/etc/robot_arm/hardware.yaml \
    calibration_config_file:=/etc/robot_arm/calibration.yaml
```
