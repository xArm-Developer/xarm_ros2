# UF850 force control (ros2_control)

The UF850 hardware interface runs the arm in `XARM_MODE::POSE` (mode 0) and commands it
with `set_position()`. Mode 0 is required by the arm's onboard force-control loop, and it
is not compatible with a per-cycle joint stream, so `joint_trajectory_controller` cannot
be used here — Cartesian pose commanding replaces it.

## Structure

`uf_robot_hardware/UFRobotSystemHardware` exports:

| component | interfaces | units | direction |
|---|---|---|---|
| `tcp` (gpio) | `x y z roll pitch yaw` | m, rad | command + state |
| `tcp` (gpio) | `servo`, `fault` | 0/1 | state |
| `ft` (gpio) | `enable`, `coord`, `c_axis_0..5`, `f_ref_0..5` | — , N/Nm | command |
| `ft` (gpio) | `enable`, `mode`, `error`, `force_x/y/z` | — , N | state |
| `ft_sensor` (sensor) | `force.x .. torque.z` | N, Nm | state |
| joints | `position`, `velocity` | rad, rad/s | state only |

`tcp/fault` is 1.0 while the arm's controller error code is non-zero, 0.0 while it is
clear, and NaN while the driver is not connected. It is updated on every `read()`, also
while the arm is not ready, and is optional: a URDF that does not declare it works as
before. List `<prefix>tcp/fault` in the safety_controller's `trip_interfaces` to lock the
base on an arm fault.

Driven by four controllers (all spawned by the launch file):

| controller | topic | purpose |
|---|---|---|
| `tcp_pose_controller` | `~/commands`, `~/gpio_states` | Cartesian pose |
| `ft_control_controller` | `~/commands`, `~/gpio_states` | arm / disarm / retune force control |
| `uf_ft_sensor_broadcaster` | `~/wrench` | `WrenchStamped` on `link_eef` |
| `joint_state_broadcaster` | `/joint_states` | joint feedback |

Force control is its own controller rather than a second group on `tcp_pose_controller`,
so a pose command can never disturb the force loop, and so it is never deactivated when
the hardware resets — it is the only way to disarm.

`enable` is the switch (`1` → mode 2, `0` → mode 0); `mode` is the controller's
confirmation of it. If they disagree, the request was rejected — believe `mode`.

`report_type` defaults to `rich`. `normal` carries no F/T data at all.

## Launch

```bash
ros2 launch xarm_controller uf850_control_rviz_display.launch.py robot_ip:=192.168.0.230
```

Force control is **off** at launch. The sensor is still enabled and zeroed on activation,
so the wrench publishes from the start. Homing defaults to joint space
(`home_joints:='"45 0 0 0 90 0"'`, degrees); pass `home_pose:='"x y z r p y"'`
(mm then degrees) to home in Cartesian space instead.

Key launch args: `ft_sensor_mode` (0 off / 2 force, default 0), `ft_coord` (0 base,
1 tool), `ft_c_axis`, `ft_f_ref`, `ft_xe_limit`, `ft_zero_on_activate`, `home_on_activate`,
`tcp_speed`.

## Testing

Read the current pose (m, rad) and the force channel:

```bash
ros2 topic echo /tcp_pose_controller/gpio_states --once     # x y z roll pitch yaw
ros2 topic echo /ft_control_controller/gpio_states          # enable mode error fx fy fz
ros2 control list_controllers
```

Move the TCP — send all six, substituting the values read above:

```bash
ros2 topic pub --once /tcp_pose_controller/commands \
  control_msgs/msg/DynamicInterfaceGroupValues '{
  interface_groups: ["tcp"],
  interface_values: [{
    interface_names: ["x", "y", "z", "roll", "pitch", "yaw"],
    values: [0.366, 0.0168, 0.3279, 1.7552, -1.5619, 1.3900]
  }]
}'
```

Arm force control — 8 N along base X, base X and RY compliant:

```bash
ros2 topic pub --once /ft_control_controller/commands \
  control_msgs/msg/DynamicInterfaceGroupValues '{
  interface_groups: ["ft"],
  interface_values: [{
    interface_names: ["enable","coord",
                      "c_axis_0","c_axis_1","c_axis_2","c_axis_3","c_axis_4","c_axis_5",
                      "f_ref_0","f_ref_1","f_ref_2","f_ref_3","f_ref_4","f_ref_5"],
    values: [1.0, 0.0,
             1.0, 0.0, 0.0, 0.0, 1.0, 0.0,
             8.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  }]
}'
```

Only the named interfaces are written, so a partial message is enough to toggle or retune:

```bash
# disarm
ros2 topic pub --once /ft_control_controller/commands \
  control_msgs/msg/DynamicInterfaceGroupValues \
  '{interface_groups: ["ft"], interface_values: [{interface_names: ["enable"], values: [0.0]}]}'

# change target to 5 N without disarming
ros2 topic pub --once /ft_control_controller/commands \
  control_msgs/msg/DynamicInterfaceGroupValues \
  '{interface_groups: ["ft"], interface_values: [{interface_names: ["f_ref_0"], values: [5.0]}]}'
```

**With a non-zero `f_ref` and nothing to push against, the arm drives along the compliant
axis until it finds contact**, at up to `ft_xe_limit` mm/s. Keep the e-stop in reach.

## Behaviour worth knowing

- The force app latches its configuration when it starts, so changing `coord`, `c_axis` or
  `f_ref` while armed restarts the loop (mode 0 → mode 2). It releases for a few ms.
- Arming re-anchors the compliant-axis standoff to the current pose. Disarming leaves the
  arm where the loop put it — no snap back; the next TCP command moves it from there.
- The sensor zero is written only at activation, never on re-arm: re-zeroing in contact
  would fold the contact force into the calibration.
- An all-zero `c_axis`, or `f_ref` zero on every compliant axis, is refused rather than
  armed.

## Limitations

- Compliant-axis pinning in `write()` indexes `ft_c_axis` against the base frame, so it is
  correct only for `ft_coord = 0`. With `ft_coord = 1` the wrong axes are pinned.
- A non-empty `prefix` needs `sensor_name`, `frame_id`, `gpios` and the controller names
  in `uf850_controllers.yaml` adjusted by hand.
