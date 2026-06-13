# xarm_bridge

A FastAPI REST bridge and `rosbridge_server` launch for web control of a
single xArm. It turns the ROS 2 action interfaces in `xarm_skills_msgs`
into a typed HTTP API with persistent job tracking, and exposes a curated,
telemetry-only WebSocket allowlist for a browser client.

## Highlights

- **Auto-discovery.** At startup the bridge scans `xarm_skills_msgs.action`
  for action types (anything with `Goal`/`Result`/`Feedback`) and generates
  a typed Pydantic request model plus a `POST /skills/{name}/run` endpoint
  for each. Adding a new `.action` file and rebuilding exposes it through
  the bridge with no code changes here.
- **Persistent jobs.** Job state (queued / running / succeeded / failed /
  canceled), feedback, and results are stored in SQLite so they survive a
  restart.
- **Telemetry fan-out.** Job feedback is also published to `/job_feedback`
  (`std_msgs/String`, JSON) for web telemetry over rosbridge.
- **Auth + CORS.** Set `XARM_BRIDGE_API_KEY` to require an `X-API-Key`
  header on all mutating endpoints; set `XARM_BRIDGE_CORS_ORIGINS`
  (comma-separated) to restrict CORS.

## Layout

```
xarm_bridge/
├── README.md                       — this file
├── LICENSE                         — MIT
├── package.xml / setup.py          — ament_python package metadata
├── xarm_bridge/
│   └── bridge_node.py              — FastAPI bridge node
├── launch/
│   └── bridge.launch.py            — bridge + rosbridge_server
└── config/
    ├── bridge_params.yaml          — node parameters
    └── rosbridge_allowlist.yaml    — telemetry-only topic allowlist
```

## Dependencies

- `rclpy`, `std_msgs`
- `xarm_skills_msgs` (the action interfaces — travels with this contribution)
- `rosbridge_server`
- Python: `fastapi`, `uvicorn`, `pydantic`

## Build

```bash
cd <your_ros2_ws>
colcon build --packages-select xarm_skills_msgs xarm_bridge
source install/setup.bash
```

## Run

```bash
ros2 launch xarm_bridge bridge.launch.py
```

This starts the FastAPI bridge on `:8000` and `rosbridge_server` on `:9090`.
Override ports with `bridge_port:=` / `rosbridge_port:=`.

## API

| Endpoint | Method | Description |
|---|---|---|
| `/skills` | GET | List discovered skills with typed goal/result schemas |
| `/skills/{name}/run` | POST | Start a skill (typed body) → `{job_id}` |
| `/jobs/{id}` | GET | Job status, feedback, result |
| `/jobs` | GET | Recent jobs |
| `/jobs/{id}/cancel` | POST | Cancel a running job |
| `/estop` | POST | Emergency stop — latches `/estop` and cancels active goals |
| `/confirm` | POST | Confirm a pending `WaitForUser` prompt |
| `/healthz` | GET | Health + discovered skills |
| `/openapi.json` | GET | Auto-generated OpenAPI spec |

## rosbridge (telemetry only)

`config/rosbridge_allowlist.yaml` exposes only a small, stable set of
telemetry topics (joint states, TF, vision, prompts, job feedback) to the
web client. Command topics are deliberately excluded — all mutations go
through the authenticated REST bridge above.
