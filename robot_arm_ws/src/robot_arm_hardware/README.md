# robot_arm_hardware

ros2_control hardware interface for the **physical** 6-DOF arm.

```
ros2_control  (arm_controller, joint_state_broadcaster)
      |
RobotArmSystemHardware      units, calibration, safety, diagnostics, e-stop
      |
MotorProtocol               wire format          (loopback | simple_ascii | yours)
      |
Transport                   bytes on the bus     (loopback | serial | rs485 | can | tcp)
      |
motor controller -> drives -> motors + encoders
```

Everything above the plugin is identical to the simulated robot: the
controllers, MoveIt, the Python/C++ APIs and the CLI tools cannot tell the two
apart.

## Safe by default

`config/hardware.yaml` ships with `transport.type: loopback` and
`protocol.type: loopback`, so a fresh clone runs the **real driver code path**
against a simulated set of drives and cannot move a machine by accident. Switch
to your bus only once the configuration matches your hardware.

## Porting to your motor controller

Two extension points, and nothing else in the workspace has to change.

**1. A different bus** - implement `Transport`
(`include/robot_arm_hardware/transport/transport.hpp`): `open`, `close`,
`is_open`, `write(Frame)`, `read(Frame)`, `flush`, `name`. Register it in
`create_transport()` in `src/transport/transport.cpp`, then select it with
`transport.type` in `config/hardware.yaml`.

**2. A different wire format** - implement `MotorProtocol`
(`include/robot_arm_hardware/protocol/motor_protocol.hpp`): `initialize`,
`enable`, `stop`, `write_commands`, `read_feedback`, `name`. Register it in
`create_protocol()`, then select it with `protocol.type`.

`SimpleAsciiProtocol` is a complete, documented reference implementation - the
fastest starting point is to copy it. Protocols work in **motor units**
(encoder counts, counts/s, amps); the conversion to joint radians is done once,
by `JointConfig`, and is shared by every protocol.

CANopen, Modbus or EtherCAT fit the same shape: a protocol on top of a
transport.

## Configuration

| File | Contents |
| --- | --- |
| `config/hardware.yaml` | bus type and parameters, protocol, timeouts, per-joint drive data (motor id, encoder resolution, gear ratio, torque constant, current and temperature limits) |
| `config/calibration.yaml` | per-joint `zero_offset`, `direction`, soft limits, home position - this file describes *your* machine, keep it in version control |

Both are injected into the URDF by
`robot_arm_description/urdf/ros2_control.xacro` and arrive here as
ros2_control `<param>` entries. Nothing is compiled in.

## Safety behaviour

| Condition | Reaction |
| --- | --- |
| Position command outside the calibrated limits | clamped, `WARN` logged, `SafetyStatus` published |
| Setpoint jump faster than `max_velocity` | rate-limited to `max_velocity * period` |
| NaN in a command | replaced by the measured pose |
| Implausible/NaN encoder value | reading rejected; repeated failures stop the arm |
| No valid feedback within `comm_timeout` | motion blocked, controlled stop |
| No `write()` within `command_timeout` | watchdog stops the arm; on resume the commands are re-seeded from the encoders, never from the stale setpoint |
| `max_consecutive_errors` bus errors | `read`/`write` return ERROR, drives are disabled, the controller manager deactivates the hardware |
| `/e_stop` or `~/set_e_stop` | commands zeroed, drives disabled, latched until explicitly released |

On a watchdog timeout the drives keep their power so the arm holds its pose
instead of collapsing under gravity; motion stays blocked until the control
loop is alive again.

## ROS interface of the driver

The plugin is loaded by the controller_manager and has no node of its own, so
it spins a small one:

| Name | Type |
| --- | --- |
| `/e_stop` (sub) | `std_msgs/Bool` |
| `~/status` | `robot_arm_msgs/ArmStatus` |
| `~/safety_status` | `robot_arm_msgs/SafetyStatus` |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` |
| `~/set_e_stop` | `robot_arm_interfaces/SetEStop` |
| `~/set_motor_enable` | `robot_arm_interfaces/SetMotorEnable` |
| `~/calibrate_joint` | `robot_arm_interfaces/CalibrateJoint` |
| `~/save_calibration` | `robot_arm_interfaces/SaveCalibration` |
| `~/get_calibration` | `robot_arm_interfaces/GetCalibration` |

## Tests

```bash
colcon test --packages-select robot_arm_hardware
```

`test_joint_config` (encoder/gear/calibration conversions),
`test_safety_checker` (limits, timeouts, e-stop, NaN handling),
`test_transport` and `test_protocol` (framing, checksums, feedback parsing,
simulated drives) run without ROS, without hardware and without a simulator.
