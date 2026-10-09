"""FastAPI dashboard server."""
from __future__ import annotations

import asyncio
from pathlib import Path
import threading
from typing import Optional

from .models import DashboardStatus
from .ros_monitor import RosMonitor
from .supervisor import LaunchSupervisor

try:
    from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
    from fastapi.responses import FileResponse
    from pydantic import BaseModel, Field
except ImportError:  # Keep supervisor tests usable before system dependencies are installed.
    FastAPI = None
    class BaseModel:  # pragma: no cover - only used for dependency diagnostics
        pass
    def Field(default=None, **_kwargs):
        return default


class StartRequest(BaseModel):
    headless: bool = False
    rviz: bool = True
    world: Optional[str] = Field(default=None, max_length=512)


class TrajectoryRequest(BaseModel):
    execute: bool = False
    grasp_clearance: float = Field(default=0.02, ge=0.0, le=0.5)
    grasp_approach_height: float = Field(default=0.10, gt=0.0, le=0.5)
    arm_verification_tolerance: float = Field(default=0.05, gt=0.0, le=1.0)


def build_app(workspace: Optional[Path] = None):
    if FastAPI is None:
        raise RuntimeError("Phase 1 dashboard requires fastapi and uvicorn; install requirements.txt")
    workspace = (workspace or Path(__file__).resolve().parents[2]).resolve()
    monitor = RosMonitor()
    supervisor = LaunchSupervisor(workspace)
    subscribers: set[WebSocket] = set()
    loop_holder = {}

    app = FastAPI(title="UR5 Simulation Dashboard", version="0.1.0")
    app.state.supervisor = supervisor
    app.state.monitor = monitor
    app.state.trajectory_request_model = TrajectoryRequest

    def status():
        alive = supervisor.process is not None and supervisor.process.poll() is None
        supervisor.status.checks = monitor.checks(alive, supervisor._last_options["rviz"])
        if alive:
            checks = supervisor.status.checks
            required = ["launch", "clock", "joint_states", "arm_controller",
                        "gripper_controller", "moveit", "planning_scene"]
            if (all(checks[name].ok for name in required)
                    and supervisor.status.state not in {"Stopping", "Stopped", "Failed"}):
                if supervisor.status.state != "Ready":
                    supervisor.status.message = "Simulation is ready"
                supervisor.status.state = "Ready"
            elif supervisor.status.state == "Starting":
                supervisor.status.message = "Waiting for ROS readiness checks"
        value = supervisor.status.jsonable()
        value["telemetry"] = monitor.live_data()
        return value

    def require_ready():
        value = status()
        if not supervisor.status.launch_owned or supervisor.process is None or supervisor.process.poll() is not None:
            raise HTTPException(status_code=409, detail="Start the dashboard-owned simulation first")
        if supervisor.status.state != "Ready":
            raise HTTPException(status_code=409, detail="Wait for the simulation to reach Ready")
        required = ["clock", "joint_states", "arm_controller", "gripper_controller",
                    "moveit", "planning_scene"]
        unavailable = [name for name in required if not value["checks"].get(name, {}).get("ok")]
        if unavailable:
            raise HTTPException(
                status_code=409,
                detail="Simulation is not ready: " + ", ".join(unavailable))

    def notify():
        loop = loop_holder.get("loop")
        if loop and loop.is_running():
            asyncio.run_coroutine_threadsafe(broadcast(), loop)

    async def broadcast():
        message = status()
        dead = []
        for socket in list(subscribers):
            try:
                await socket.send_json(message)
            except Exception:
                dead.append(socket)
        for socket in dead:
            subscribers.discard(socket)

    supervisor.on_change = notify

    @app.on_event("startup")
    async def startup():
        loop_holder["loop"] = asyncio.get_running_loop()
        monitor.start()

    @app.on_event("shutdown")
    async def shutdown():
        monitor.stop()
        supervisor.close()

    @app.get("/api/status")
    def get_status():
        return status()

    @app.get("/api/config")
    def get_config():
        return {"workspace": str(workspace), "default_world": str(workspace / "ur5_bringup/worlds/ur5_pick_place.sdf"),
                "defaults": supervisor._last_options}

    @app.get("/api/health")
    def health():
        return {"ok": True, "service": "ur5_dashboard", "state": status()["state"]}

    @app.get("/api/logs")
    def logs():
        return {"lines": supervisor.status.logs}

    @app.post("/api/system/start")
    def start(request: StartRequest):
        try:
            values = request.model_dump() if hasattr(request, "model_dump") else request.dict()
            ok, message = supervisor.start(**values)
        except ValueError as error:
            raise HTTPException(status_code=400, detail=str(error)) from error
        if not ok:
            raise HTTPException(status_code=409, detail=message)
        return {"ok": True, "message": message, "status": status()}

    @app.post("/api/system/stop")
    def stop():
        ok, message = supervisor.stop()
        return {"ok": ok, "message": message, "status": status()}

    @app.post("/api/system/restart")
    def restart(request: StartRequest):
        try:
            values = request.model_dump() if hasattr(request, "model_dump") else request.dict()
            ok, message = supervisor.restart(**values)
        except ValueError as error:
            raise HTTPException(status_code=400, detail=str(error)) from error
        return {"ok": ok, "message": message, "status": status()}

    @app.post("/api/trajectory/start")
    def start_trajectory(request: TrajectoryRequest):
        require_ready()
        values = request.model_dump() if hasattr(request, "model_dump") else request.dict()
        ok, message = supervisor.start_trajectory(**values)
        if not ok:
            raise HTTPException(status_code=409, detail=message)
        return {"ok": True, "message": message, "status": status()}

    @app.post("/api/trajectory/stop")
    def stop_trajectory():
        ok, message = supervisor.stop_trajectory()
        if not ok:
            raise HTTPException(status_code=409, detail=message)
        return {"ok": True, "message": message, "status": status()}

    @app.post("/api/recording/start")
    def start_recording():
        require_ready()
        try:
            ok, message = supervisor.start_recording()
        except ValueError as error:
            raise HTTPException(status_code=500, detail=str(error)) from error
        if not ok:
            raise HTTPException(status_code=409, detail=message)
        return {"ok": True, "message": message, "path": supervisor.status.recording_path,
                "status": status()}

    @app.post("/api/recording/stop")
    def stop_recording():
        ok, message = supervisor.stop_recording()
        if not ok:
            raise HTTPException(status_code=409, detail=message)
        return {"ok": True, "message": message, "path": supervisor.status.recording_path,
                "status": status()}

    @app.websocket("/ws")
    async def websocket(socket: WebSocket):
        await socket.accept()
        subscribers.add(socket)
        try:
            while True:
                await socket.send_json(status())
                await asyncio.sleep(0.1)
        except WebSocketDisconnect:
            subscribers.discard(socket)
        except Exception:
            subscribers.discard(socket)

    web_root = Path(__file__).resolve().parents[1] / "web"
    if not web_root.exists():
        try:
            from ament_index_python.packages import get_package_share_directory
            web_root = Path(get_package_share_directory("ur5_dashboard")) / "web"
        except Exception:
            pass

    @app.get("/")
    def index():
        return FileResponse(web_root / "index.html")

    @app.get("/static/{filename:path}")
    def static_file(filename: str):
        target = (web_root / filename).resolve()
        if target.parent != web_root.resolve() or not target.is_file():
            raise HTTPException(status_code=404, detail="Not found")
        return FileResponse(target)

    return app


def main():
    import uvicorn
    workspace = Path(__file__).resolve().parents[2]
    uvicorn.run(build_app(workspace), host="127.0.0.1", port=8765, log_level="info")


if __name__ == "__main__":
    main()
