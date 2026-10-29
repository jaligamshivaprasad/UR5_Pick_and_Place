from types import SimpleNamespace

import math

from ur5_data_logger.effort_sync import EffortSynchronizer, extract_named_efforts


JOINTS = ("j1", "j2")


def message(stamp, entries):
    sec = int(stamp)
    nanosec = round((stamp - sec) * 1e9)
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec)),
        joint_names=[entry[0] for entry in entries],
        interface_values=[
            SimpleNamespace(interface_names=entry[1], values=entry[2])
            for entry in entries
        ],
    )


def test_extracts_named_effort_with_reordered_joints_and_interfaces():
    msg = message(1.0, [
        ("j2", ["effort", "position"], [2.0, 0.2]),
        ("ignored", ["effort"], [99.0]),
        ("j1", ["velocity", "effort"], [0.1, 1.0]),
    ])
    assert extract_named_efforts(msg, JOINTS) == {"j2": 2.0, "j1": 1.0}


def test_extraction_rejects_duplicate_names_and_omits_missing_effort():
    duplicate_joint = message(1.0, [
        ("j1", ["effort"], [1.0]),
        ("j1", ["effort"], [2.0]),
    ])
    assert extract_named_efforts(duplicate_joint, JOINTS) == {}

    missing = message(1.0, [
        ("j1", ["position"], [0.0]),
        ("j2", ["effort", "effort"], [2.0, 3.0]),
    ])
    assert extract_named_efforts(missing, JOINTS) == {}


def test_matches_future_sample_and_accepts_tolerance_boundary():
    sync = EffortSynchronizer(JOINTS, tolerance=0.015)
    assert sync.queue(1.0, "row", required=True) == []
    assert sync.add_message(message(1.014, [
        ("j1", ["effort"], [1.0]),
        ("j2", ["effort"], [2.0]),
    ])) == []
    matches = sync.add_message(message(1.016, [
        ("j1", ["effort"], [3.0]),
        ("j2", ["effort"], [4.0]),
    ]))
    assert len(matches) == 1
    assert matches[0].payload == "row"
    assert matches[0].efforts == {"j1": 1.0, "j2": 2.0}
    assert matches[0].error is None

    boundary = EffortSynchronizer(JOINTS, tolerance=0.015)
    boundary.queue(2.0, "boundary", required=True)
    finalized = boundary.add_message(message(2.015, [
        ("j1", ["effort"], [5.0]),
        ("j2", ["effort"], [6.0]),
    ]))
    match = finalized[0]
    assert match.efforts == {"j1": 5.0, "j2": 6.0}


def test_force_flush_uses_nearest_sample_and_rejects_stale_sample():
    sync = EffortSynchronizer(JOINTS, tolerance=0.015)
    sync.add_message(message(1.001, [
        ("j1", ["effort"], [1.0]),
        ("j2", ["effort"], [2.0]),
    ]))
    sync.queue(1.0, "near", required=True)
    near = sync.finalize_ready(force=True)[0]
    assert near.efforts == {"j1": 1.0, "j2": 2.0}

    sync.queue(2.0, "stale", required=True)
    stale = sync.finalize_ready(force=True)[0]
    assert stale.error == "timestamp_mismatch"


def test_missing_and_nonfinite_effort_are_distinct_failures():
    missing = EffortSynchronizer(JOINTS)
    missing.add_message(message(1.0, [("j1", ["effort"], [1.0])]))
    missing.queue(1.0, "row", required=True)
    assert missing.finalize_ready(force=True)[0].error == "missing_effort_fields"

    nonfinite = EffortSynchronizer(JOINTS)
    nonfinite.add_message(message(1.0, [
        ("j1", ["effort"], [math.nan]),
        ("j2", ["effort"], [2.0]),
    ]))
    nonfinite.queue(1.0, "row", required=True)
    assert nonfinite.finalize_ready(force=True)[0].error == "nonfinite_effort"


def test_strict_mode_reports_missing_topic_instead_of_silent_nan():
    sync = EffortSynchronizer(JOINTS)
    sync.queue(1.0, "row", required=True)
    match = sync.finalize_ready(force=True)[0]
    assert match.efforts is None
    assert sync.strict_error() == (
        "measured effort unavailable: /dynamic_joint_states was not received"
    )
    assert sync.metadata()["post_startup_coverage"] == 0.0
