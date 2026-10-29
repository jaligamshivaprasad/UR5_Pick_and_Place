"""Timestamp synchronization for simulated joint-effort telemetry."""

from collections import deque
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class EffortMatch:
    """A finalized controller sample and its measured-effort match."""

    payload: object
    efforts: dict | None
    error: str | None
    required: bool


def extract_named_efforts(message, joints):
    """Extract joint effort values from a DynamicJointState-like message.

    Duplicate joint or interface names are rejected because their mapping is
    ambiguous. Missing and non-finite values are retained as distinct errors by
    :class:`EffortSynchronizer`.
    """
    if len(set(message.joint_names)) != len(message.joint_names):
        return {}

    wanted = set(joints)
    efforts = {}
    for index, joint in enumerate(message.joint_names):
        if joint not in wanted or index >= len(message.interface_values):
            continue
        interface = message.interface_values[index]
        names = list(interface.interface_names)
        if len(set(names)) != len(names):
            continue
        try:
            effort_index = names.index("effort")
        except ValueError:
            continue
        if effort_index < len(interface.values):
            efforts[joint] = float(interface.values[effort_index])
    return efforts


class EffortSynchronizer:
    """Match controller rows to nearest DynamicJointState effort samples."""

    def __init__(self, joints, tolerance=0.015, max_samples=1000):
        if tolerance < 0.0:
            raise ValueError("effort synchronization tolerance must be non-negative")
        self.joints = tuple(joints)
        self.tolerance = float(tolerance)
        self.samples = deque(maxlen=max_samples)
        self.pending = deque()
        self.dynamic_messages = 0
        self.required_rows = 0
        self.matched_required_rows = 0
        self.failure_counts = {
            "missing_effort_fields": 0,
            "nonfinite_effort": 0,
            "timestamp_mismatch": 0,
        }

    @staticmethod
    def stamp_seconds(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def add_message(self, message):
        self.dynamic_messages += 1
        timestamp = self.stamp_seconds(message.header.stamp)
        self.samples.append((timestamp, extract_named_efforts(message, self.joints)))
        return self.finalize_ready()

    def queue(self, timestamp, payload, required=False):
        required = bool(required)
        self.pending.append((float(timestamp), payload, required))
        if required:
            self.required_rows += 1
        return self.finalize_ready()

    def finalize_ready(self, force=False):
        finalized = []
        while self.pending:
            timestamp, payload, required = self.pending[0]
            window_closed = (
                self.samples
                and self.samples[-1][0] + 1e-12 >= timestamp + self.tolerance
            )
            if not force and not window_closed:
                break
            self.pending.popleft()
            finalized.append(self._match(timestamp, payload, required))
        return finalized

    def _match(self, timestamp, payload, required):
        candidates = [
            (abs(sample_time - timestamp), efforts)
            for sample_time, efforts in self.samples
            if abs(sample_time - timestamp) <= self.tolerance + 1e-12
        ]
        if not candidates:
            error = "timestamp_mismatch"
            efforts = None
        else:
            _, efforts = min(candidates, key=lambda item: item[0])
            if not all(joint in efforts for joint in self.joints):
                error = "missing_effort_fields"
                efforts = None
            elif not all(math.isfinite(efforts[joint]) for joint in self.joints):
                error = "nonfinite_effort"
                efforts = None
            else:
                error = None
                efforts = {joint: efforts[joint] for joint in self.joints}

        if required:
            if error is None:
                self.matched_required_rows += 1
            else:
                self.failure_counts[error] += 1
        return EffortMatch(payload, efforts, error, required)

    def metadata(self):
        coverage = (
            self.matched_required_rows / self.required_rows
            if self.required_rows else None
        )
        return {
            "source": "/dynamic_joint_states named effort interface; gz_ros2_control/Gazebo JointTransmittedWrench feedback",
            "meaning": "simulated transmitted joint effort, not a physical torque-sensor measurement",
            "alignment": f"nearest simulation timestamp within +/- {self.tolerance:.3f} s; no stale or commanded-effort fallback",
            "dynamic_messages_received": self.dynamic_messages,
            "required_post_startup_rows": self.required_rows,
            "matched_post_startup_rows": self.matched_required_rows,
            "post_startup_coverage": coverage,
            "failure_counts": dict(self.failure_counts),
        }

    def strict_error(self):
        if not self.required_rows:
            return "measured effort unavailable: no post-startup controller rows"
        if self.dynamic_messages == 0:
            return "measured effort unavailable: /dynamic_joint_states was not received"
        if self.failure_counts["missing_effort_fields"]:
            return "measured effort unavailable: named effort fields are missing for one or more arm joints"
        if self.failure_counts["nonfinite_effort"]:
            return "measured effort unavailable: non-finite joint effort feedback was received"
        if self.failure_counts["timestamp_mismatch"]:
            return "measured effort unavailable: controller and dynamic-state timestamps did not synchronize within 15 ms"
        if self.matched_required_rows != self.required_rows:
            return "measured effort unavailable: post-startup telemetry coverage is incomplete"
        return None
