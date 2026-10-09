from types import SimpleNamespace

import pytest

from ur5_dashboard.ros_monitor import RosMonitor


def test_stopped_monitor_reports_required_checks_without_claiming_ready():
    checks = RosMonitor().checks(False, True)
    assert checks["launch"].ok is False
    assert checks["clock"].ok is False
    assert checks["rviz"].ok is False


def test_live_telemetry_maps_arm_joints_and_calculates_tracking_error():
    monitor = RosMonitor()
    names = list(reversed(monitor.ARM_JOINTS))
    actual = {name: index * 0.1 for index, name in enumerate(monitor.ARM_JOINTS)}
    reference = {name: value + 0.02 for name, value in actual.items()}
    monitor._joint_state(SimpleNamespace(
        name=names,
        position=[actual[name] for name in names],
        velocity=[0.5] * len(names),
        effort=[1.5] * len(names),
    ))
    monitor._controller_state(SimpleNamespace(
        joint_names=names,
        reference=SimpleNamespace(positions=[reference[name] for name in names]),
        feedback=SimpleNamespace(
            positions=[actual[name] for name in names],
            velocities=[0.25] * len(names),
        ),
    ))

    telemetry = monitor.live_data()

    assert telemetry["ready"] is True
    assert telemetry["sample"]["position_actual"] == pytest.approx(
        [actual[name] for name in monitor.ARM_JOINTS])
    assert telemetry["sample"]["position_reference"] == pytest.approx(
        [reference[name] for name in monitor.ARM_JOINTS])
    assert telemetry["sample"]["velocity"] == pytest.approx([0.25] * 6)
    assert telemetry["sample"]["effort"] == pytest.approx([1.5] * 6)
    assert telemetry["sample"]["error"] == pytest.approx([0.02] * 6)
    assert telemetry["max_abs_error_rad"] == pytest.approx(0.02)
    assert telemetry["rms_error_rad"] == pytest.approx(0.02)


def test_live_telemetry_reports_missing_arm_feedback():
    monitor = RosMonitor()
    monitor._controller_state(SimpleNamespace(
        joint_names=["some_other_joint"],
        reference=SimpleNamespace(positions=[1.0]),
        feedback=SimpleNamespace(positions=[0.5], velocities=[0.0]),
    ))

    telemetry = monitor.live_data()

    assert telemetry["ready"] is False
    assert telemetry["sample"] is None
    assert telemetry["message"] == "Controller state is missing one or more UR5 arm joints"


def test_end_effector_telemetry_calculates_synchronized_gazebo_position_error():
    monitor = RosMonitor()
    monitor._gazebo_pose(SimpleNamespace(transforms=[SimpleNamespace(
        child_frame_id="ur5::wrist_3_link",
        header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0)),
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=1.003, y=2.004, z=3.0)),
    )]))
    received, _ = monitor._gazebo_tool_poses[-1]
    response = SimpleNamespace(
        error_code=SimpleNamespace(val=1),
        pose_stamped=[SimpleNamespace(pose=SimpleNamespace(
            position=SimpleNamespace(x=1.0, y=2.0, z=3.0)))],
    )
    monitor._tool_fk_completed(
        SimpleNamespace(result=lambda: response),
        received,
    )

    telemetry = monitor.live_data()["end_effector"]

    assert telemetry["ready"] is True
    assert telemetry["sample"]["position_reference_m"] == pytest.approx([1.0, 2.0, 3.0])
    assert telemetry["sample"]["position_actual_m"] == pytest.approx([1.003, 2.004, 3.0])
    assert telemetry["sample"]["error_xyz_m"] == pytest.approx([0.003, 0.004, 0.0])
    assert telemetry["sample"]["error_m"] == pytest.approx(0.005)


def test_end_effector_telemetry_rejects_unsynchronized_and_stale_pose_samples():
    monitor = RosMonitor()
    pose = (SimpleNamespace(
        child_frame_id="wrist_3_link",
        header=SimpleNamespace(stamp=SimpleNamespace(sec=20, nanosec=0)),
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=1.0, y=2.0, z=3.0)),
    ),)
    monitor._gazebo_pose(SimpleNamespace(transforms=pose))
    received, _ = monitor._gazebo_tool_poses[-1]
    response = SimpleNamespace(
        error_code=SimpleNamespace(val=1),
        pose_stamped=[SimpleNamespace(pose=SimpleNamespace(
            position=SimpleNamespace(x=1.0, y=2.0, z=3.0)))],
    )

    monitor._tool_fk_completed(
        SimpleNamespace(result=lambda: response), received + 0.6)
    telemetry = monitor.live_data()["end_effector"]

    assert telemetry["ready"] is False
    assert telemetry["sample"] is None
    assert "arrival time" in telemetry["message"]

    with monitor._telemetry_lock:
        monitor._gazebo_tool_poses.append((0.0, (1.0, 2.0, 3.0)))
    monitor._tool_fk_completed(
        SimpleNamespace(result=lambda: response), 0.0)
    telemetry = monitor.live_data()["end_effector"]

    assert telemetry["ready"] is False
    assert "stale" in telemetry["message"]


def test_gazebo_pose_callback_ignores_non_arm_frames():
    monitor = RosMonitor()
    monitor._gazebo_pose(SimpleNamespace(transforms=[SimpleNamespace(
        child_frame_id="tool0",
        header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0)),
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=1.0, y=2.0, z=3.0)),
    )]))

    assert not monitor._gazebo_tool_poses
