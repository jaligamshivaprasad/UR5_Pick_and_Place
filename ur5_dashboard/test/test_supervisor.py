from pathlib import Path
import subprocess
import time

import pytest

from ur5_dashboard.models import Check, DashboardStatus
from ur5_dashboard.supervisor import LaunchSupervisor


def test_status_is_jsonable():
    status = DashboardStatus(checks={"clock": Check("Clock", True, "fresh")})
    assert status.jsonable()["checks"]["clock"]["ok"] is True


def test_world_validation_rejects_outside_and_wrong_extension(tmp_path):
    supervisor = LaunchSupervisor(tmp_path, runtime=tmp_path / "runtime")
    with pytest.raises(ValueError, match="existing file"):
        supervisor._validate_world("/tmp/missing.sdf")
    world = tmp_path / "world.txt"
    world.write_text("world")
    with pytest.raises(ValueError, match=".sdf"):
        supervisor._validate_world(str(world))


def test_external_bringup_detects_leftover_ignition_server(tmp_path, monkeypatch):
    supervisor = LaunchSupervisor(tmp_path, runtime=tmp_path / "runtime")
    monkeypatch.setattr(
        "ur5_dashboard.supervisor.subprocess.run",
        lambda *args, **kwargs: subprocess.CompletedProcess(
            args[0], 0, stdout="ruby ign gazebo server\n", stderr=""),
    )

    assert supervisor.external_bringup() is True


def test_child_process_groups_excludes_the_launch_group(tmp_path, monkeypatch):
    supervisor = LaunchSupervisor(tmp_path, runtime=tmp_path / "runtime")
    monkeypatch.setattr(
        "ur5_dashboard.supervisor.subprocess.run",
        lambda *args, **kwargs: subprocess.CompletedProcess(
            args[0], 0,
            stdout="100 1 100\n101 100 101\n102 101 101\n103 100 103\n",
            stderr=""),
    )

    assert supervisor._child_process_groups(100) == {101, 103}


def test_owned_child_process_group_is_terminated(tmp_path):
    supervisor = LaunchSupervisor(tmp_path, runtime=tmp_path / "runtime")
    child = subprocess.Popen(["sleep", "30"], start_new_session=True)

    supervisor._terminate_process_groups({child.pid})

    assert child.wait(timeout=5) is not None


def test_stop_without_process_is_safe(tmp_path):
    supervisor = LaunchSupervisor(tmp_path, runtime=tmp_path / "runtime")
    ok, message = supervisor.stop()
    assert ok is False
    assert "No dashboard" in message


def test_trajectory_uses_existing_pick_place_launch_and_explicit_execute_flag(
        tmp_path, monkeypatch):
    supervisor = LaunchSupervisor(tmp_path, runtime=tmp_path / "runtime")
    captured = {}

    def capture(args, **kwargs):
        captured["args"] = args
        return True, "Trajectory started"

    monkeypatch.setattr(supervisor, "_spawn_auxiliary", capture)
    ok, _ = supervisor.start_trajectory(
        execute=True, grasp_clearance=0.03, grasp_approach_height=0.12,
        arm_verification_tolerance=0.04)

    assert ok is True
    assert captured["args"][:4] == [
        "ros2", "launch", "ur5_pick_place", "pick_place.launch.py"]
    assert "execute:=true" in captured["args"]
    assert "experiment_logging:=true" in captured["args"]
    assert "grasp_clearance:=0.03" in captured["args"]
    assert "grasp_approach_height:=0.12" in captured["args"]
    assert "arm_verification_tolerance:=0.04" in captured["args"]


def test_recording_uses_unique_output_and_execution_topics(tmp_path, monkeypatch):
    supervisor = LaunchSupervisor(tmp_path, runtime=tmp_path / "runtime")
    config = tmp_path / "ur5_pick_place/config/experiment_record_qos.yaml"
    config.parent.mkdir(parents=True)
    config.write_text("/clock: {}\n")
    captured = {}

    def capture(args, **kwargs):
        captured["args"] = args
        return True, "Recording started"

    monkeypatch.setattr(supervisor, "_spawn_auxiliary", capture)
    ok, _ = supervisor.start_recording()

    assert ok is True
    assert "--qos-profile-overrides-path" in captured["args"]
    assert "/trajectory_experiment/events" in captured["args"]
    assert "/pick_place_trajectory_node/display_planned_path" in captured["args"]
    output = Path(captured["args"][captured["args"].index("-o") + 1])
    assert output.parent == tmp_path / "experiment_bags"
    assert not output.exists()
    assert supervisor.status.recording_path == str(output)


def test_owned_process_is_stopped_as_a_process_group(tmp_path):
    supervisor = LaunchSupervisor(tmp_path, runtime=tmp_path / "runtime")
    process = subprocess.Popen(["bash", "-lc", "sleep 30"], start_new_session=True)
    supervisor._process = process
    supervisor.status.state = "Starting"
    supervisor.status.launch_owned = True
    ok, _ = supervisor.stop(timeout=0.5)
    assert ok is True
    assert process.poll() is not None
