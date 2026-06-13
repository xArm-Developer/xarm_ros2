"""
FastAPI bridge node for xArm skill RPC.

Runs a FastAPI HTTP server inside a ROS 2 node. At startup it dynamically
discovers all action types from the xarm_skills_msgs.action module by
scanning for classes with Goal/Result/Feedback inner types. For each
discovered action it auto-generates:

  - A typed Pydantic Goal model (from ROS Goal field introspection)
  - A typed Pydantic Result model
  - A REST endpoint: POST /skills/{name}/run

Adding a new .action file to xarm_skills_msgs and rebuilding will
automatically expose it through the bridge without any code changes here.

Other endpoints:
  GET  /skills              — list discovered skills with typed schemas
  GET  /jobs/{job_id}       — poll job status / feedback / result
  GET  /jobs                — list recent jobs
  POST /jobs/{job_id}/cancel — cancel a running job
  POST /estop               — emergency stop (publishes to /estop topic)
  POST /confirm             — confirm a WaitForUser prompt
  GET  /healthz             — health check
  GET  /openapi.json        — auto-generated OpenAPI spec

Job state is persisted in a SQLite database so it survives restarts.
Job feedback is also published to /job_feedback (String, JSON) for
web telemetry via rosbridge.

Authentication: If XARM_BRIDGE_API_KEY env var is set, all mutating
endpoints require X-API-Key header. Set XARM_BRIDGE_CORS_ORIGINS to
restrict CORS (comma-separated origins).
"""

import asyncio
import importlib
import inspect
import json
import os
import re
import sqlite3
import threading
import time
import uuid
from enum import Enum
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_group import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool, String

try:
    from fastapi import FastAPI, HTTPException, Depends
    from fastapi.middleware.cors import CORSMiddleware
    from fastapi.security import APIKeyHeader
    from pydantic import BaseModel, create_model
    import uvicorn
    HAS_FASTAPI = True
except ImportError:
    HAS_FASTAPI = False


def _camel_to_snake(name):
    """Convert CamelCase to snake_case."""
    s1 = re.sub(r"(.)([A-Z][a-z]+)", r"\1_\2", name)
    return re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", s1).lower()


def _is_ros_action_type(obj):
    """Check if an object is a ROS 2 action type (has Goal, Result, Feedback)."""
    return (
        inspect.isclass(obj)
        and hasattr(obj, "Goal")
        and hasattr(obj, "Result")
        and hasattr(obj, "Feedback")
        and hasattr(obj.Goal, "get_fields_and_field_types")
    )


def discover_skills(package_name="xarm_skills_msgs.action"):
    """Dynamically discover all action types from a ROS 2 msgs package.

    Scans the given package module for classes that have Goal/Result/Feedback
    inner types (i.e., ROS 2 action types). Returns a dict mapping
    snake_case skill names to their action type classes.

    Adding a new .action file to xarm_skills_msgs and rebuilding will
    cause it to appear here automatically.
    """
    try:
        module = importlib.import_module(package_name)
    except ImportError as e:
        print("WARNING: Could not import %s: %s" % (package_name, e))
        return {}

    discovered = {}
    for attr_name in dir(module):
        obj = getattr(module, attr_name)
        if _is_ros_action_type(obj):
            skill_name = _camel_to_snake(attr_name)
            discovered[skill_name] = {
                "action_type": obj,
                "action_name": skill_name,
                "class_name": attr_name,
            }

    return discovered


def _ros_field_to_pydantic(field_name, ros_type_str, goal_cls):
    """Convert a ROS 2 field type to a Pydantic field tuple (type, default)."""
    goal_instance = goal_cls()

    if ros_type_str in ("string",):
        default = getattr(goal_instance, field_name, "")
        if default == "":
            return (str, ...)
        return (str, default)

    if ros_type_str in ("float64", "float32"):
        default = getattr(goal_instance, field_name, 0.0)
        return (float, default)

    if ros_type_str in ("int32", "int64", "uint32", "uint16", "uint8"):
        default = getattr(goal_instance, field_name, 0)
        return (int, default)

    if ros_type_str in ("bool", "boolean"):
        default = getattr(goal_instance, field_name, False)
        return (bool, default)

    if "sequence" in ros_type_str:
        return (list[dict], ...)

    return (Any, None)


def _build_pydantic_models(skill_name, action_type):
    """Build typed Pydantic Goal and Result models from ROS action type."""
    goal_cls = action_type.Goal
    goal_fields = goal_cls.get_fields_and_field_types()

    pydantic_fields = {}
    for fname, ftype in goal_fields.items():
        pydantic_fields[fname] = _ros_field_to_pydantic(fname, ftype, goal_cls)

    GoalModel = create_model(
        "%sGoal" % skill_name.title().replace("_", ""),
        **pydantic_fields,
    )

    result_cls = action_type.Result
    result_fields_raw = result_cls.get_fields_and_field_types()
    result_pydantic = {}
    for fname, ftype in result_fields_raw.items():
        if ftype in ("string",):
            result_pydantic[fname] = (Optional[str], None)
        elif ftype in ("float64", "float32"):
            result_pydantic[fname] = (Optional[float], None)
        elif ftype in ("int32", "int64"):
            result_pydantic[fname] = (Optional[int], None)
        elif ftype in ("bool", "boolean"):
            result_pydantic[fname] = (Optional[bool], None)
        else:
            result_pydantic[fname] = (Any, None)

    ResultModel = create_model(
        "%sResult" % skill_name.title().replace("_", ""),
        **result_pydantic,
    )

    return GoalModel, ResultModel


def _build_goal_schema(action_type):
    """Build a JSON-serializable schema of the goal fields."""
    goal_cls = action_type.Goal
    fields = goal_cls.get_fields_and_field_types()
    goal_instance = goal_cls()
    schema = {}
    for fname, ftype in fields.items():
        entry = {"ros_type": ftype}
        default = getattr(goal_instance, fname, None)
        if ftype == "string":
            entry["type"] = "string"
            if default and default != "":
                entry["default"] = default
            else:
                entry["required"] = True
        elif ftype in ("float64", "float32"):
            entry["type"] = "number"
            entry["default"] = float(default) if default else 0.0
        elif ftype in ("int32", "int64"):
            entry["type"] = "integer"
            entry["default"] = int(default) if default else 0
        elif ftype in ("bool", "boolean"):
            entry["type"] = "boolean"
            entry["default"] = bool(default)
        elif "sequence" in ftype:
            entry["type"] = "array"
            entry["required"] = True
        else:
            entry["type"] = "any"
        schema[fname] = entry
    return schema


def _convert_sequence_field(field_type, value):
    """Convert JSON list to ROS message list for sequence fields."""
    if "geometry_msgs/Point" in field_type and isinstance(value, list):
        from geometry_msgs.msg import Point
        pts = []
        for wp in value:
            p = Point()
            p.x = float(wp.get("x", 0.0))
            p.y = float(wp.get("y", 0.0))
            p.z = float(wp.get("z", 0.0))
            pts.append(p)
        return pts
    return value


class JobStatus(str, Enum):
    QUEUED = "queued"
    RUNNING = "running"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELED = "canceled"


class JobDB:
    """SQLite-backed job persistence."""

    def __init__(self, db_path):
        self._db_path = db_path
        self._lock = threading.Lock()
        self._init_db()

    def _init_db(self):
        with self._lock:
            conn = sqlite3.connect(self._db_path)
            conn.execute("""
                CREATE TABLE IF NOT EXISTS jobs (
                    id TEXT PRIMARY KEY,
                    skill_name TEXT NOT NULL,
                    status TEXT NOT NULL DEFAULT 'queued',
                    goal_json TEXT,
                    feedback_json TEXT,
                    result_json TEXT,
                    created_at REAL,
                    updated_at REAL
                )
            """)
            conn.commit()
            conn.close()

    def create_job(self, job_id, skill_name, goal_dict):
        now = time.time()
        with self._lock:
            conn = sqlite3.connect(self._db_path)
            conn.execute(
                "INSERT INTO jobs (id, skill_name, status, goal_json, created_at, updated_at) VALUES (?, ?, ?, ?, ?, ?)",
                (job_id, skill_name, JobStatus.QUEUED, json.dumps(goal_dict), now, now),
            )
            conn.commit()
            conn.close()

    def update_status(self, job_id, status, feedback=None, result=None):
        now = time.time()
        with self._lock:
            conn = sqlite3.connect(self._db_path)
            updates = ["status = ?", "updated_at = ?"]
            params = [status, now]
            if feedback is not None:
                updates.append("feedback_json = ?")
                params.append(json.dumps(feedback))
            if result is not None:
                updates.append("result_json = ?")
                params.append(json.dumps(result))
            params.append(job_id)
            conn.execute(
                "UPDATE jobs SET %s WHERE id = ?" % ", ".join(updates),
                params,
            )
            conn.commit()
            conn.close()

    def get_job(self, job_id):
        with self._lock:
            conn = sqlite3.connect(self._db_path)
            conn.row_factory = sqlite3.Row
            row = conn.execute("SELECT * FROM jobs WHERE id = ?", (job_id,)).fetchone()
            conn.close()
            if row is None:
                return None
            return dict(row)

    def list_jobs(self, limit=50):
        with self._lock:
            conn = sqlite3.connect(self._db_path)
            conn.row_factory = sqlite3.Row
            rows = conn.execute(
                "SELECT * FROM jobs ORDER BY created_at DESC LIMIT ?", (limit,)
            ).fetchall()
            conn.close()
            return [dict(r) for r in rows]


class BridgeNode(Node):
    def __init__(self):
        super().__init__("bridge_node")

        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8000)
        self.declare_parameter("db_path", "/tmp/xarm_bridge_jobs.db")
        self.declare_parameter("estop_topic", "/estop")
        self.declare_parameter("prompt_confirm_topic", "/skill_prompts/confirm")
        self.declare_parameter("job_feedback_topic", "/job_feedback")
        self.declare_parameter("skills_package", "xarm_skills_msgs.action")

        self._cb_group = ReentrantCallbackGroup()

        estop_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.estop_pub = self.create_publisher(
            Bool,
            self.get_parameter("estop_topic").value,
            estop_qos,
        )

        self.confirm_pub = self.create_publisher(
            Bool,
            self.get_parameter("prompt_confirm_topic").value,
            10,
        )

        self.job_feedback_pub = self.create_publisher(
            String,
            self.get_parameter("job_feedback_topic").value,
            10,
        )

        self.db = JobDB(self.get_parameter("db_path").value)

        self._action_clients = {}
        self._active_goal_handles = {}
        self._goal_lock = threading.Lock()

        self._skill_registry = {}
        self._skill_models = {}

        skills_package = self.get_parameter("skills_package").value
        discovered = discover_skills(skills_package)

        for name, info in discovered.items():
            client = ActionClient(
                self, info["action_type"], info["action_name"],
                callback_group=self._cb_group,
            )
            self._action_clients[name] = client

            GoalModel, ResultModel = _build_pydantic_models(name, info["action_type"])
            self._skill_models[name] = {
                "goal_model": GoalModel,
                "result_model": ResultModel,
                "schema": _build_goal_schema(info["action_type"]),
            }

            self._skill_registry[name] = info

        self.get_logger().info(
            "Bridge node initialized — discovered %d skills from %s: %s"
            % (len(self._skill_registry), skills_package, list(self._skill_registry.keys()))
        )

    def _build_goal(self, skill_name, params):
        info = self._skill_registry[skill_name]
        action_type = info["action_type"]
        goal = action_type.Goal()

        goal_fields = action_type.Goal.get_fields_and_field_types()

        for field_name, field_type in goal_fields.items():
            if field_name in params:
                value = params[field_name]
                if "sequence" in field_type:
                    value = _convert_sequence_field(field_type, value)
                setattr(goal, field_name, value)

        return goal

    def _publish_job_feedback(self, job_id, skill_name, feedback_dict):
        msg = String()
        msg.data = json.dumps({
            "job_id": job_id,
            "skill": skill_name,
            "feedback": feedback_dict,
            "timestamp": time.time(),
        })
        self.job_feedback_pub.publish(msg)

    def _extract_feedback(self, fb):
        feedback_dict = {}
        for attr in dir(fb):
            if attr.startswith("_") or attr.startswith("SLOT") or callable(getattr(fb, attr)):
                continue
            if attr in ("get_fields_and_field_types",):
                continue
            try:
                val = getattr(fb, attr)
                if isinstance(val, (int, float, str, bool)):
                    feedback_dict[attr] = val
            except Exception:
                pass

        if hasattr(fb, "get_fields_and_field_types"):
            for fname, ftype in fb.get_fields_and_field_types().items():
                val = getattr(fb, fname, None)
                if val is not None:
                    if ftype in ("float32", "float64"):
                        feedback_dict[fname] = float(val)
                    elif ftype in ("int32", "int64"):
                        feedback_dict[fname] = int(val)
                    elif ftype == "string":
                        feedback_dict[fname] = str(val)
                    elif ftype in ("bool", "boolean"):
                        feedback_dict[fname] = bool(val)
                    else:
                        feedback_dict[fname] = val

        return feedback_dict

    def _extract_result(self, r, action_type):
        result_fields = action_type.Result.get_fields_and_field_types()
        result_dict = {}
        for fname, ftype in result_fields.items():
            val = getattr(r, fname, None)
            if val is not None:
                if ftype in ("bool", "boolean"):
                    result_dict[fname] = bool(val)
                elif ftype in ("float64", "float32"):
                    result_dict[fname] = float(val)
                elif ftype in ("int32", "int64"):
                    result_dict[fname] = int(val)
                elif ftype == "string":
                    result_dict[fname] = str(val)
                else:
                    result_dict[fname] = val
        return result_dict

    async def start_skill(self, skill_name, params):
        if skill_name not in self._skill_registry:
            raise ValueError("Unknown skill: %s" % skill_name)

        client = self._action_clients[skill_name]
        if not client.wait_for_server(timeout_sec=3.0):
            raise RuntimeError("Action server '%s' not available" % skill_name)

        goal = self._build_goal(skill_name, params)
        job_id = str(uuid.uuid4())[:8]
        action_type = self._skill_registry[skill_name]["action_type"]

        self.db.create_job(job_id, skill_name, params)

        def feedback_cb(feedback_msg):
            fb = feedback_msg.feedback
            feedback_dict = self._extract_feedback(fb)
            self.db.update_status(job_id, JobStatus.RUNNING, feedback=feedback_dict)
            self._publish_job_feedback(job_id, skill_name, feedback_dict)

        goal_handle_future = await client.send_goal_async(
            goal, feedback_callback=feedback_cb,
        )

        if not goal_handle_future.accepted:
            self.db.update_status(
                job_id, JobStatus.FAILED,
                result={"success": False, "message": "Goal rejected by action server"},
            )
            return job_id

        self.db.update_status(job_id, JobStatus.RUNNING)

        with self._goal_lock:
            self._active_goal_handles[job_id] = goal_handle_future

        async def wait_for_result():
            try:
                result_resp = await goal_handle_future.get_result_async()
                r = result_resp.result
                result_dict = self._extract_result(r, action_type)

                status = JobStatus.SUCCEEDED if getattr(r, "success", True) else JobStatus.FAILED
                self.db.update_status(job_id, status, result=result_dict)

                completion_msg = String()
                completion_msg.data = json.dumps({
                    "job_id": job_id,
                    "skill": skill_name,
                    "status": status,
                    "result": result_dict,
                    "timestamp": time.time(),
                })
                self.job_feedback_pub.publish(completion_msg)
            except Exception as e:
                self.db.update_status(
                    job_id, JobStatus.FAILED,
                    result={"success": False, "message": str(e)},
                )
            finally:
                with self._goal_lock:
                    self._active_goal_handles.pop(job_id, None)

        asyncio.ensure_future(wait_for_result())
        return job_id

    async def cancel_job(self, job_id):
        with self._goal_lock:
            goal_handle = self._active_goal_handles.get(job_id)
        if goal_handle is None:
            return False
        await goal_handle.cancel_goal_async()
        self.db.update_status(job_id, JobStatus.CANCELED)
        return True

    def trigger_estop(self):
        msg = Bool()
        msg.data = True
        self.estop_pub.publish(msg)
        self.get_logger().error("E-STOP triggered via bridge")

        with self._goal_lock:
            for job_id, gh in list(self._active_goal_handles.items()):
                try:
                    asyncio.ensure_future(gh.cancel_goal_async())
                except Exception:
                    pass
                self.db.update_status(
                    job_id, JobStatus.FAILED,
                    result={"success": False, "message": "E-stop triggered"},
                )
            self._active_goal_handles.clear()

    def confirm_prompt(self):
        msg = Bool()
        msg.data = True
        self.confirm_pub.publish(msg)


def create_fastapi_app(bridge_node):
    api_key_env = os.environ.get("XARM_BRIDGE_API_KEY", "")
    require_auth = bool(api_key_env)

    app = FastAPI(
        title="xArm Skills Bridge",
        description="REST API for xArm chemistry lab skill execution. "
                    "Skills are auto-discovered from xarm_skills_msgs at startup.",
        version="1.0.0",
    )

    allowed_origins = os.environ.get("XARM_BRIDGE_CORS_ORIGINS", "")
    if allowed_origins:
        origins_list = [o.strip() for o in allowed_origins.split(",")]
    else:
        origins_list = ["*"]

    app.add_middleware(
        CORSMiddleware,
        allow_origins=origins_list,
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    api_key_header = APIKeyHeader(name="X-API-Key", auto_error=False)

    async def verify_api_key(key: str = Depends(api_key_header)):
        if not require_auth:
            return
        if key != api_key_env:
            raise HTTPException(status_code=403, detail="Invalid or missing API key")

    class JobResponse(BaseModel):
        id: str
        skill_name: str
        status: str
        goal: dict | None = None
        feedback: dict | None = None
        result: dict | None = None
        created_at: float | None = None
        updated_at: float | None = None

    class SkillInfo(BaseModel):
        name: str
        action_class: str
        goal_schema: dict
        result_fields: dict

    def _job_to_response(j):
        return JobResponse(
            id=j["id"],
            skill_name=j["skill_name"],
            status=j["status"],
            goal=json.loads(j["goal_json"]) if j.get("goal_json") else None,
            feedback=json.loads(j["feedback_json"]) if j.get("feedback_json") else None,
            result=json.loads(j["result_json"]) if j.get("result_json") else None,
            created_at=j.get("created_at"),
            updated_at=j.get("updated_at"),
        )

    @app.get("/skills", response_model=list[SkillInfo])
    async def list_skills():
        result = []
        for name, info in bridge_node._skill_registry.items():
            models = bridge_node._skill_models[name]
            result_fields = info["action_type"].Result.get_fields_and_field_types()
            result.append(SkillInfo(
                name=name,
                action_class=info["class_name"],
                goal_schema=models["schema"],
                result_fields=result_fields,
            ))
        return result

    for skill_name in list(bridge_node._skill_registry.keys()):
        info = bridge_node._skill_registry[skill_name]
        GoalModel = bridge_node._skill_models[skill_name]["goal_model"]

        def _make_run_endpoint(sname, model, cls_name):
            async def run_skill(body: model, _=Depends(verify_api_key)):
                try:
                    params = body.model_dump()
                    job_id = await bridge_node.start_skill(sname, params)
                    return {"job_id": job_id}
                except ValueError as e:
                    raise HTTPException(status_code=400, detail=str(e))
                except RuntimeError as e:
                    raise HTTPException(status_code=503, detail=str(e))

            run_skill.__name__ = "run_%s" % sname
            run_skill.__doc__ = "Execute the %s skill (auto-discovered from %s)" % (sname, cls_name)
            return run_skill

        endpoint = _make_run_endpoint(skill_name, GoalModel, info["class_name"])
        app.post(
            "/skills/%s/run" % skill_name,
            summary="Run %s" % skill_name.replace("_", " ").title(),
            tags=["skills"],
        )(endpoint)

    @app.get("/jobs/{job_id}", response_model=JobResponse)
    async def get_job(job_id: str):
        job = bridge_node.db.get_job(job_id)
        if job is None:
            raise HTTPException(status_code=404, detail="Job not found")
        return _job_to_response(job)

    @app.get("/jobs", response_model=list[JobResponse])
    async def list_jobs(limit: int = 50):
        jobs = bridge_node.db.list_jobs(limit=limit)
        return [_job_to_response(j) for j in jobs]

    @app.post("/jobs/{job_id}/cancel")
    async def cancel_job(job_id: str, _=Depends(verify_api_key)):
        success = await bridge_node.cancel_job(job_id)
        if not success:
            raise HTTPException(status_code=404, detail="No active job with that ID")
        return {"status": "canceled", "job_id": job_id}

    @app.post("/estop")
    async def estop(_=Depends(verify_api_key)):
        bridge_node.trigger_estop()
        return {"status": "estopped"}

    @app.post("/confirm")
    async def confirm(_=Depends(verify_api_key)):
        bridge_node.confirm_prompt()
        return {"status": "confirmed"}

    @app.get("/healthz")
    async def healthz():
        return {
            "status": "ok",
            "skills_discovered": list(bridge_node._skill_registry.keys()),
            "skills_count": len(bridge_node._skill_registry),
        }

    return app


def main(args=None):
    if not HAS_FASTAPI:
        print("ERROR: FastAPI not installed. Install with: pip3 install fastapi uvicorn")
        return

    rclpy.init(args=args)
    bridge_node = BridgeNode()

    app = create_fastapi_app(bridge_node)

    host = bridge_node.get_parameter("host").value
    port = bridge_node.get_parameter("port").value

    ros_thread = threading.Thread(
        target=lambda: rclpy.spin(bridge_node),
        daemon=True,
    )
    ros_thread.start()

    bridge_node.get_logger().info(
        "FastAPI bridge starting on %s:%d" % (host, port)
    )

    try:
        uvicorn.run(app, host=host, port=port, log_level="info")
    except KeyboardInterrupt:
        pass
    finally:
        bridge_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
