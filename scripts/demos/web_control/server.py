"""Local HTTP interface. Simulator access is exclusively through ControlState."""

import asyncio
import json
import os
import re
import socket
import threading
import time
import uuid
import zipfile
from pathlib import Path

import uvicorn
from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import FileResponse, JSONResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles


class WebServer:
    def __init__(self, state, port=8080, output_dir="output/web_control"):
        self.state = state
        self.port = port
        self.output_dir = Path(output_dir).resolve()
        self.stopping = threading.Event()
        self.thread = None
        self.socket = None
        self.app = self._create_app()

    @property
    def url(self):
        return f"http://127.0.0.1:{self.port}"

    def _create_app(self):
        app = FastAPI(docs_url=None, redoc_url=None, openapi_url=None)
        static = Path(__file__).parent / "static"
        if hasattr(self.state, "execute_joint_chunk"):
            from robot_api import register_robot_routes

            register_robot_routes(app, self.state)

        @app.middleware("http")
        async def local_requests(request, call_next):
            host = request.headers.get("host", "")
            allowed = {f"127.0.0.1:{self.port}", f"localhost:{self.port}"}
            if host not in allowed:
                return JSONResponse({"detail": "Use the localhost SSH tunnel address"}, status_code=403)
            if request.method != "GET":
                origin = request.headers.get("origin")
                if origin and origin != f"http://{host}":
                    return JSONResponse({"detail": "Cross-origin control is not allowed"}, status_code=403)
                if request.headers.get("sec-fetch-site") == "cross-site":
                    return JSONResponse({"detail": "Cross-site control is not allowed"}, status_code=403)
                if request.headers.get("content-type", "").split(";")[0] != "application/json":
                    return JSONResponse({"detail": "JSON is required"}, status_code=415)
            response = await call_next(request)
            response.headers["Cache-Control"] = "no-store"
            response.headers["X-Content-Type-Options"] = "nosniff"
            return response

        @app.get("/")
        def index():
            return FileResponse(static / "index.html")

        @app.get("/api/status")
        def status():
            return self.state.status()

        async def command_body(request):
            body = await request.body()
            if len(body) > 4096:
                raise HTTPException(413, "Command too large")
            try:
                return json.loads(body)
            except (ValueError, UnicodeDecodeError) as exc:
                raise HTTPException(400, "Invalid JSON") from exc

        @app.post("/api/command")
        async def command(request: Request):
            body = await command_body(request)
            try:
                self.state.submit(body)
            except ValueError as exc:
                raise HTTPException(400, str(exc)) from exc
            return {"ok": True}

        def get_frame(camera):
            if camera not in {c["name"] for c in self.state.status()["cameras"]}:
                raise HTTPException(404, "Unknown camera")
            frame = self.state.frame()
            if frame is None or camera not in frame["images"]:
                raise HTTPException(503, "Camera is not ready")
            return frame

        @app.get("/api/camera/{camera}.jpg")
        def camera_image(camera: str):
            return Response(get_frame(camera)["images"][camera], media_type="image/jpeg")

        @app.get("/api/stream/{camera}")
        async def stream(camera: str, request: Request):
            get_frame(camera)

            async def frames():
                last = -1
                while not self.stopping.is_set() and not await request.is_disconnected():
                    frame = self.state.frame()
                    if frame is not None and frame["sequence"] != last:
                        last = frame["sequence"]
                        data = frame["images"][camera]
                        yield (
                            b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                            + str(len(data)).encode()
                            + b"\r\n\r\n"
                            + data
                            + b"\r\n"
                        )
                    await asyncio.sleep(0.03)

            return StreamingResponse(frames(), media_type="multipart/x-mixed-replace; boundary=frame")

        @app.post("/api/snapshot")
        async def snapshot(request: Request):
            if await command_body(request) != {}:
                raise HTTPException(400, "Snapshot takes an empty JSON object")
            frame = get_frame("scene")
            # File work runs off the event loop. A frame is one immutable paired capture.
            name = await asyncio.to_thread(self._save_snapshot, frame)
            return {"download": f"/api/download/{name}", "filename": name, "sim_time": frame["sim_time"]}

        @app.get("/api/download/{name}")
        def download(name: str):
            if not re.fullmatch(r"snapshot_[0-9a-f]{32}\.zip", name):
                raise HTTPException(404, "Unknown snapshot")
            path = self.output_dir / name
            if not path.is_file():
                raise HTTPException(404, "Snapshot not found")
            return FileResponse(path, media_type="application/zip", filename=name)

        app.mount("/static", StaticFiles(directory=static), name="static")
        return app

    def _save_snapshot(self, frame):
        self.output_dir.mkdir(parents=True, exist_ok=True)
        name = f"snapshot_{uuid.uuid4().hex}.zip"
        path = self.output_dir / name
        temporary = path.with_suffix(".tmp")
        metadata = {key: value for key, value in frame.items() if key not in ("images", "monotonic")}
        try:
            with zipfile.ZipFile(temporary, "w") as archive:
                for camera, data in frame["images"].items():
                    archive.writestr(f"{camera}.jpg", data)
                archive.writestr("metadata.json", json.dumps(metadata, indent=2))
            os.replace(temporary, path)
        finally:
            temporary.unlink(missing_ok=True)
        return name

    def start(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.socket.bind(("127.0.0.1", self.port))
            self.port = self.socket.getsockname()[1]
            self.socket.listen(32)
            self.server = uvicorn.Server(
                uvicorn.Config(self.app, log_level="warning", access_log=False, timeout_graceful_shutdown=2)
            )
            self.thread = threading.Thread(target=self.server.run, kwargs={"sockets": [self.socket]}, daemon=True)
            self.thread.start()
            deadline = time.monotonic() + 5
            while not self.server.started:
                if not self.thread.is_alive() or time.monotonic() > deadline:
                    raise RuntimeError("HTTP server failed to start")
                time.sleep(0.01)
        except BaseException:
            self.stop()
            raise

    def stop(self):
        self.stopping.set()
        if self.thread is not None:
            self.server.should_exit = True
            self.thread.join(timeout=5)
        if self.socket is not None:
            self.socket.close()
