# xarm_skills_msgs

ROS 2 **action** interfaces for high-level xArm lab-automation skills. These
are the contract between a web/REST client (via `xarm_bridge`) and the
action-server implementations that drive the arm.

## Actions

| Action | Goal (summary) | Purpose |
|---|---|---|
| `PickAndPlace` | source/destination labels, grasp width, heights | Vision-guided pick from a source and place at a destination |
| `Transfer` | source/destination labels, safe transit height | Pick a container and move it safely above the bench to a destination |
| `Dispense` | container label, volume, pump rate | Move to a container and trigger an external pump to dispense N ml |
| `Stir` | target label, rpm, duration, depth | Locate a container and stir with a gripper-held rod |
| `Glue` | waypoint path, speed, dispense flag | Follow a waypoint path with a glue tip, triggering a pump between segments |
| `WaitForUser` | message, timeout | Pause and prompt the operator; resume on confirm or timeout |

Every action follows the same shape: a `Goal` with the parameters above, a
`Result` with `bool success` + `string message` (plus action-specific
fields), and `Feedback` with a `float32 progress` and a `string status`.

See the individual files in [`action/`](action) for the exact field
definitions.

## Dependencies

- `geometry_msgs` (used by `Glue` waypoints)
- `rosidl_default_generators` / `rosidl_default_runtime`

## Build

```bash
cd <your_ros2_ws>
colcon build --packages-select xarm_skills_msgs
source install/setup.bash
```

## Adding a skill

1. Add `action/MySkill.action` (Goal `---` Result `---` Feedback).
2. List it in `CMakeLists.txt` under `rosidl_generate_interfaces`.
3. Rebuild. `xarm_bridge` auto-discovers the new action and exposes a
   `POST /skills/my_skill/run` endpoint with no bridge code changes.
