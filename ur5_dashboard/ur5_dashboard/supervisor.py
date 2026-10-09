"""Owned process supervision for the dashboard bringup launch."""
from __future__ import annotations

import os
from pathlib import Path
import signal
import shlex
import subprocess
import threading
import time
from datetime import datetime, timezone
from typing import Callable, Optional

from .models import DashboardStatus


class LaunchSupervisor:
    def __init__(self, workspace: Path, runtime: Optional[Path] = None,
                 on_change: Optional[Callable[[], None]] = None):
        self.workspace = workspace.resolve()
        self.runtime = (runtime or self.workspace / ".dashboard_runtime").resolve()
        self.runtime.mkdir(parents=True, exist_ok=True)
        (self.runtime / "ros_logs").mkdir(parents=True, exist_ok=True)
        self.on_change = on_change or (lambda: None)
        self._process: Optional[subprocess.Popen] = None
        self._trajectory_process: Optional[subprocess.Popen] = None
        self._recording_process: Optional[subprocess.Popen] = None
        self._launch_child_groups: set[int] = set()
        self._lock = threading.RLock()
        self._logs: list[str] = []
        self._shutdown_requested = False
        self._trajectory_stop_requested = False
        self._recording_stop_requested = False
        self._last_options = {"headless": False, "rviz": True, "world": None}
        self.status = DashboardStatus()

    @property
    def process(self):
        return self._process

    def _notify(self):
        self.on_change()

    def _append(self, line: str, source="bringup"):
        line = line.rstrip()
        if not line:
            return
        with self._lock:
            # ROS 2 launch forwards shutdown to children. Some Humble
            # components print error-looking teardown diagnostics (including
            # a MoveIt unload backtrace) even though the user requested a
            # normal stop. Keep normal runtime errors visible, but omit this
            # known teardown noise from the dashboard console.
            if source == "bringup" and self._shutdown_requested and (
                    "[ERROR]" in line or "Traceback" in line
                    or "Segmentation fault" in line):
                return
            stamp = datetime.now().strftime("%H:%M:%S")
            self._logs.append(f"[{stamp}] {line}")
            self._logs = self._logs[-500:]
            self.status.logs = self._logs.copy()
            with (self.runtime / f"{source}.log").open("a", encoding="utf-8") as stream:
                stream.write(self._logs[-1] + "\n")
        self._notify()

    def _read_output(self, stream, source="bringup"):
        try:
            for line in iter(stream.readline, ""):
                self._append(f"[{source}] {line}", source)
        finally:
            stream.close()

    def _command(self, args):
        overlay = self.workspace / "install" / "setup.bash"
        command = (
            f"source {shlex.quote('/opt/ros/humble/setup.bash')} && "
            f"source {shlex.quote(str(overlay))} && "
            f"exec {' '.join(shlex.quote(str(item)) for item in args)}"
        )
        return ["bash", "-lc", command]

    def _spawn_auxiliary(self, args, *, source, process_attr, state_attr,
                         active_state, stopping_flag):
        with self._lock:
            process = getattr(self, process_attr)
            if process and process.poll() is None:
                return False, f"{source.capitalize()} is already running"
            if not self._process or self._process.poll() is not None:
                return False, "Start the dashboard-owned simulation first"
            log_path = self.runtime / f"{source}.log"
            log_path.touch(exist_ok=True)
            process = subprocess.Popen(
                self._command(args), cwd=self.workspace, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, text=True, bufsize=1, start_new_session=True,
                env={**os.environ, "RCUTILS_COLORIZED_OUTPUT": "0",
                     "ROS_LOG_DIR": str(self.runtime / "ros_logs")})
            setattr(self, process_attr, process)
            setattr(self, stopping_flag, False)
            setattr(self.status, state_attr, active_state)
            if source == "trajectory":
                self.status.trajectory_pid = process.pid
            else:
                self.status.recording_pid = process.pid
            threading.Thread(
                target=self._read_output, args=(process.stdout, source), daemon=True).start()
            threading.Thread(
                target=self._watch_auxiliary,
                args=(process, source, process_attr, state_attr, stopping_flag),
                daemon=True).start()
        self._notify()
        return True, f"{source.capitalize()} started"

    def _watch_auxiliary(self, process, source, process_attr, state_attr, stopping_flag):
        return_code = process.wait()
        with self._lock:
            if getattr(self, process_attr) is process:
                setattr(self, process_attr, None)
                stopped = getattr(self, stopping_flag)
                if source == "trajectory":
                    self.status.trajectory_pid = None
                    result = "Stopped" if stopped else ("Completed" if return_code == 0 else "Failed")
                else:
                    self.status.recording_pid = None
                    result = "Stopped" if stopped or return_code == 0 else "Failed"
                setattr(self.status, state_attr, result)
                if return_code and not stopped:
                    self.status.message = f"{source.capitalize()} exited with code {return_code}"
        self._notify()

    def _stop_auxiliary(self, source, process_attr, state_attr, stopping_flag, timeout):
        with self._lock:
            process = getattr(self, process_attr)
            if not process or process.poll() is not None:
                setattr(self.status, state_attr, "Stopped")
                if source == "trajectory":
                    self.status.trajectory_pid = None
                else:
                    self.status.recording_pid = None
                self._notify()
                return False, f"No active {source} process"
            setattr(self, stopping_flag, True)
            setattr(self.status, state_attr, "Stopping")
        self._notify()
        try:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
        except ProcessLookupError:
            pass
        with self._lock:
            if getattr(self, process_attr) is process:
                setattr(self, process_attr, None)
            setattr(self.status, state_attr, "Stopped")
            if source == "trajectory":
                self.status.trajectory_pid = None
            else:
                self.status.recording_pid = None
        self._notify()
        return True, f"{source.capitalize()} stopped"

    def start_trajectory(self, *, execute=False, grasp_clearance=0.02,
                         grasp_approach_height=0.10, arm_verification_tolerance=0.05):
        run_id = datetime.now(timezone.utc).strftime("dashboard-%Y%m%dT%H%M%SZ")
        args = [
            "ros2", "launch", "ur5_pick_place", "pick_place.launch.py",
            f"execute:={'true' if execute else 'false'}",
            "experiment_logging:=true",
            f"experiment_run_id:={run_id}",
            f"grasp_clearance:={grasp_clearance}",
            f"grasp_approach_height:={grasp_approach_height}",
            f"arm_verification_tolerance:={arm_verification_tolerance}",
        ]
        return self._spawn_auxiliary(
            args, source="trajectory", process_attr="_trajectory_process",
            state_attr="trajectory_state", active_state="Running",
            stopping_flag="_trajectory_stop_requested")

    def stop_trajectory(self):
        return self._stop_auxiliary(
            "trajectory", "_trajectory_process", "trajectory_state",
            "_trajectory_stop_requested", timeout=30)

    def start_recording(self):
        qos_file = self.workspace / "ur5_pick_place/config/experiment_record_qos.yaml"
        if not qos_file.is_file():
            raise ValueError(f"Recording QoS configuration does not exist: {qos_file}")
        output_root = self.workspace / "experiment_bags"
        output_root.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime("dashboard_%Y%m%dT%H%M%SZ")
        output = output_root / stamp
        suffix = 1
        while output.exists():
            output = output_root / f"{stamp}_{suffix}"
            suffix += 1
        topics = [
            "/clock",
            "/joint_states",
            "/arm_controller/controller_state",
            "/robotiq_gripper_controller/controller_state",
            "/trajectory_experiment/events",
            "/pick_place_trajectory_node/display_planned_path",
            "/execute_trajectory/_action/status",
            "/world/ur5_pick_place/dynamic_pose/info",
        ]
        args = [
            "ros2", "bag", "record", "-o", str(output),
            "--qos-profile-overrides-path", str(qos_file), *topics,
        ]
        ok, message = self._spawn_auxiliary(
            args, source="recording", process_attr="_recording_process",
            state_attr="recording_state", active_state="Recording",
            stopping_flag="_recording_stop_requested")
        if ok:
            self.status.recording_path = str(output)
        return ok, message

    def stop_recording(self):
        return self._stop_auxiliary(
            "recording", "_recording_process", "recording_state",
            "_recording_stop_requested", timeout=30)

    def _watch(self, process):
        while process.poll() is None:
            self._launch_child_groups.update(
                self._child_process_groups(process.pid))
            time.sleep(0.25)
        return_code = process.returncode
        self._terminate_process_groups(self._launch_child_groups)
        with self._lock:
            if self._process is process:
                self._process = None
                if self.status.state not in {"Stopping", "Stopped"}:
                    self.status.state = "Failed" if return_code else "Stopped"
                    self.status.message = (
                        f"Bringup exited with code {return_code}" if return_code
                        else "Simulation stopped")
                self.status.launch_owned = False
                self.status.launch_pid = None
        self._notify()

    def _child_process_groups(self, root_pid: int) -> set[int]:
        """Find process groups owned by descendants of a supervised launch."""
        try:
            result = subprocess.run(
                ["ps", "-eo", "pid=,ppid=,pgid="], check=True,
                capture_output=True, text=True, timeout=1)
        except (OSError, subprocess.SubprocessError) as error:
            self._append(f"Unable to inspect launch child processes: {error}")
            return set()

        children = {}
        groups = {}
        for line in result.stdout.splitlines():
            try:
                pid, parent, group = (int(value) for value in line.split())
            except ValueError:
                continue
            children.setdefault(parent, []).append(pid)
            groups[pid] = group

        descendants = []
        pending = [root_pid]
        visited = {root_pid}
        while pending:
            parent = pending.pop()
            for child in children.get(parent, []):
                if child in visited:
                    continue
                visited.add(child)
                descendants.append(child)
                pending.append(child)
        return {
            groups[pid] for pid in descendants
            if pid in groups and groups[pid] != root_pid
        }

    def _terminate_process_groups(self, groups: set[int]):
        remaining = set()
        for group in groups:
            try:
                os.killpg(group, signal.SIGTERM)
                remaining.add(group)
            except ProcessLookupError:
                continue
        deadline = time.monotonic() + 1.0
        while remaining and time.monotonic() < deadline:
            for group in tuple(remaining):
                try:
                    os.killpg(group, 0)
                except ProcessLookupError:
                    remaining.discard(group)
            if remaining:
                time.sleep(0.1)
        for group in remaining:
            try:
                os.killpg(group, signal.SIGKILL)
            except ProcessLookupError:
                pass

    def _validate_world(self, world: Optional[str]) -> Optional[Path]:
        if not world:
            return None
        candidate = Path(world).expanduser().resolve()
        allowed = [self.workspace, Path("/opt/ros/humble/share").resolve()]
        if not candidate.is_file() or not any(candidate == root or root in candidate.parents for root in allowed):
            raise ValueError("world must be an existing file inside the workspace or ROS share")
        if candidate.suffix != ".sdf":
            raise ValueError("world must be an .sdf file")
        return candidate

    def external_bringup(self) -> bool:
        """Return true when a duplicate bringup or Ignition server is present."""
        try:
            result = subprocess.run(["ps", "-eo", "args"], check=False,
                                    capture_output=True, text=True, timeout=1)
            needle = "ros2 launch ur5_bringup ur5_bringup.launch.py"
            return any(
                ("ign gazebo server" in line)
                or (needle in line and "ur5_dashboard" not in line)
                for line in result.stdout.splitlines()
            )
        except (OSError, subprocess.SubprocessError):
            return False

    def start(self, *, headless=False, rviz=True, world=None):
        with self._lock:
            if self._process and self._process.poll() is None:
                return False, "Dashboard-owned bringup is already running"
            if self.external_bringup():
                self.status.state = "Degraded"
                self.status.message = (
                    "An external UR5 bringup or Ignition Gazebo server is already running; "
                    "stop it before starting another simulation")
                self.status.launch_owned = False
                self._notify()
                return False, self.status.message
            world_path = self._validate_world(world)
            args = ["ros2", "launch", "ur5_bringup", "ur5_bringup.launch.py",
                    f"headless:={'true' if headless else 'false'}",
                    f"rviz:={'true' if rviz else 'false'}"]
            if world_path:
                args.append(f"world:={world_path}")
            output = (self.runtime / "bringup.log").open("a", encoding="utf-8")
            self._process = subprocess.Popen(
                self._command(args), cwd=self.workspace, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, text=True, bufsize=1, start_new_session=True,
                env={**os.environ, "RCUTILS_COLORIZED_OUTPUT": "0",
                     "ROS_LOG_DIR": str(self.runtime / "ros_logs")})
            self._last_options = {"headless": bool(headless), "rviz": bool(rviz),
                                  "world": str(world_path) if world_path else None}
            self._shutdown_requested = False
            self._launch_child_groups.clear()
            self.status.state = "Starting"
            self.status.launch_owned = True
            self.status.launch_pid = self._process.pid
            self.status.started_at = datetime.now(timezone.utc).isoformat()
            self.status.message = "Starting ROS 2 bringup"
            threading.Thread(target=self._read_output, args=(self._process.stdout, "bringup"), daemon=True).start()
            threading.Thread(target=self._watch, args=(self._process,), daemon=True).start()
            output.close()
        self._notify()
        return True, "Bringup started"

    def stop(self, timeout=30.0):
        with self._lock:
            process = self._process
            if not process or process.poll() is not None:
                self.stop_trajectory()
                self.stop_recording()
                self.status.state = "Stopped"
                self.status.launch_owned = False
                self.status.launch_pid = None
                self.status.message = "Simulation is stopped"
                self._notify()
                return False, "No dashboard-owned bringup is running"
            self.status.state = "Stopping"
            self.status.message = "Stopping ROS 2 bringup"
            self._shutdown_requested = True
        self._notify()
        self.stop_trajectory()
        self.stop_recording()
        try:
            # Let ROS 2 launch propagate the first interrupt to its children.
            # Broadcasting SIGINT to the whole group at the same time causes
            # MoveIt/Gazebo to receive duplicate shutdown signals and can
            # produce false child crashes during an otherwise clean stop.
            process.send_signal(signal.SIGINT)
            deadline = time.monotonic() + timeout
            while process.poll() is None and time.monotonic() < deadline:
                time.sleep(0.1)
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    pass
            # ros2 launch may exit after handing shutdown to children while
            # Gazebo or a helper is still alive in the owned session. Clean
            # up that session explicitly so Stop never leaves orphan nodes.
            try:
                os.killpg(process.pid, signal.SIGTERM)
                time.sleep(0.5)
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        with self._lock:
            self._process = None
            self.status.state = "Stopped"
            self.status.launch_owned = False
            self.status.launch_pid = None
            self.status.message = "Simulation is stopped"
        self._notify()
        return True, "Bringup stopped"

    def restart(self, **options):
        self.stop()
        return self.start(**options)

    def close(self):
        self.stop()
