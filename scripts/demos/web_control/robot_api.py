"""HTTP transport for RobotServer; no simulator or image-library imports."""

import asyncio
import base64
import json

from fastapi import HTTPException, Request


def register_robot_routes(app, robot):
    def invoke(call, *args):
        try:
            return call(*args)
        except KeyError as exc:
            raise HTTPException(404, "Unknown or expired task (last 64 tasks retained)") from exc
        except ValueError as exc:
            raise HTTPException(400, str(exc)) from exc
        except RuntimeError as exc:
            raise HTTPException(409, str(exc)) from exc

    async def body(request):
        data = bytearray()
        async for part in request.stream():
            data.extend(part)
            if len(data) > 2 * 1024 * 1024:
                raise HTTPException(413, "Chunk request exceeds 2 MiB")
        try:
            result = json.loads(data)
        except (ValueError, UnicodeDecodeError) as exc:
            raise HTTPException(400, "Invalid JSON") from exc
        if not isinstance(result, dict):
            raise HTTPException(400, "Expected a JSON object")
        return result

    @app.get("/api/robot/joints")
    def joints():
        return invoke(robot.joint_state)

    @app.get("/api/robot/spec")
    def spec():
        return invoke(robot.robot_spec)

    @app.get("/api/robot/observation")
    def observation():
        frame = robot.frame()
        if frame is None or robot.status()["phase"] != "ready":
            raise HTTPException(503, "Observation is not ready")
        return {
            **{k: v for k, v in frame.items() if k not in ("images", "monotonic")},
            "image_encoding": "jpeg_base64",
            "images": {name: base64.b64encode(data).decode("ascii") for name, data in frame["images"].items()},
        }

    @app.post("/api/robot/chunks")
    async def execute(request: Request):
        data = await body(request)
        if set(data) != {"joint_names", "positions", "steps_per_action"}:
            raise HTTPException(400, "Expected joint_names, positions, steps_per_action")
        return invoke(robot.execute_joint_chunk, data["joint_names"], data["positions"], data["steps_per_action"])

    @app.get("/api/robot/chunks/{task_id}")
    def task(task_id: str):
        return invoke(robot.get_task, task_id)

    @app.post("/api/robot/chunks/{task_id}/cancel")
    async def cancel(task_id: str, request: Request):
        if await body(request) != {}:
            raise HTTPException(400, "Cancel takes an empty JSON object")
        return invoke(robot.cancel, task_id)

    @app.get("/api/robot/arms")
    def arms():
        return robot.get_arms()

    async def kinematic_request(request, fields, method):
        data = await body(request)
        if set(data) != set(fields):
            raise HTTPException(400, f"Expected fields: {fields}")
        return await asyncio.to_thread(invoke, method, *(data[key] for key in fields))

    @app.post("/api/robot/fk")
    async def fk(request: Request):
        return await kinematic_request(request, ["arm", "joint_positions"], robot.fk)

    @app.post("/api/robot/ik")
    async def ik(request: Request):
        return await kinematic_request(request, ["arm", "position", "quaternion", "seed"], robot.ik)

    @app.post("/api/robot/eef/chunks")
    async def eef(request: Request):
        return await kinematic_request(
            request, ["arm", "positions", "quaternions", "steps_per_action"], robot.execute_eef_chunk
        )

    @app.post("/api/robot/recording/start")
    async def start_recording(request: Request):
        return await kinematic_request(request, ["instruction"], robot.start_recording)

    @app.post("/api/robot/recording/stop")
    async def stop_recording(request: Request):
        return await kinematic_request(request, ["episode_id", "success", "reason"], robot.stop_recording)

    @app.get("/api/robot/recording")
    def recording_status():
        return robot.recording_status()
