import argparse
from collections import deque
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import sys
import time

from control_msgs.msg import DynamicJointState, JointTrajectoryControllerState
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

from .fk import VerifiedFK
from .report import write_summary, write_workbook
from .schema import CSV_COLUMNS, JOINTS


NAN = float("nan")
CONTROLLER_TOPIC = "/arm_controller/controller_state"
DYNAMIC_TOPIC = "/dynamic_joint_states"
ROBOT_DESCRIPTION_TOPIC = "/robot_description"
CLOCK_TOPIC = "/clock"


def _stamp_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _safe_id(text):
    cleaned = re.sub(r"[^A-Za-z0-9_.-]+", "_", text.strip()).strip("._")
    if not cleaned:
        raise ValueError("trajectory ID must contain at least one safe filename character")
    return cleaned


def _named_values(names, values):
    """Return a name/value mapping only for present, non-duplicate entries."""
    if len(set(names)) != len(names):
        return {}
    return {name: float(values[index]) for index, name in enumerate(names) if index < len(values)}


def _point_values(names, point, field):
    return _named_values(names, getattr(point, field))


def _finite_or_nan(mapping, name):
    value = mapping.get(name, NAN)
    return value if math.isfinite(value) else NAN


class ExperimentLogger(Node):
    def __init__(self, args):
        super().__init__(
            "ur5_experiment_logger",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.args = args
        self.trajectory_id = args.trajectory_id
        self.output_dir = self._make_output_dir(args.output_root, self.trajectory_id)
        self.csv_path = self.output_dir / "joint_trajectory_log.csv"
        self.info_path = self.output_dir / "experiment_info.json"
        self.summary_path = self.output_dir / "summary_metrics.csv"
        self.workbook_path = self.output_dir / "UR5_Experiment_Report.xlsx"
        self.csv_handle = self.csv_path.open("w", newline="", encoding="utf-8")
        self.csv_writer = csv.DictWriter(self.csv_handle, fieldnames=CSV_COLUMNS)
        self.csv_writer.writeheader()
        self.csv_handle.flush()

        self.records = []
        self.pending = deque()
        self.dynamic_samples = deque(maxlen=1000)
        self.controller_received = 0
        self.dynamic_received = 0
        self.velocity_matches = 0
        self.measured_effort_available_rows = 0
        self.controller_error_mismatch_count = 0
        self.clock_received = 0
        self.reset_events = []
        self.epoch = 0
        self.last_clock_time = None
        self.last_controller_time = None
        self.last_dynamic_time = None
        self.controller_times = []
        self.fk = None
        self.fk_error = "runtime robot_description has not been received"
        self.robot_description_sha256 = None
        self.finalized = False

        stream_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        description_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(JointTrajectoryControllerState, CONTROLLER_TOPIC, self._on_controller, stream_qos)
        self.create_subscription(DynamicJointState, DYNAMIC_TOPIC, self._on_dynamic, stream_qos)
        self.create_subscription(Clock, CLOCK_TOPIC, self._on_clock, stream_qos)
        self.create_subscription(String, ROBOT_DESCRIPTION_TOPIC, self._on_robot_description, description_qos)
        self.create_timer(0.2, self._timer_flush_pending)

        self.get_logger().info(f"Passive logger started; output: {self.output_dir}")
        self.get_logger().info("Press Ctrl+C to stop, finalize metrics, and write the workbook.")

    @staticmethod
    def _make_output_dir(root, trajectory_id):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        base = Path(root).expanduser().resolve()
        base.mkdir(parents=True, exist_ok=True)
        candidate = base / f"{timestamp}_{_safe_id(trajectory_id)}"
        suffix = 1
        while candidate.exists():
            candidate = base / f"{timestamp}_{_safe_id(trajectory_id)}_{suffix:02d}"
            suffix += 1
        candidate.mkdir()
        return candidate

    def _on_robot_description(self, msg):
        digest = hashlib.sha256(msg.data.encode("utf-8")).hexdigest()
        if self.robot_description_sha256 == digest and self.fk is not None:
            return
        self.robot_description_sha256 = digest
        try:
            self.fk = VerifiedFK(msg.data, self.args.base_link, self.args.tcp_link, JOINTS)
            self.fk_error = None
            self.get_logger().info(
                f"Verified FK chain {self.args.base_link} -> {self.args.tcp_link} "
                f"with joints {list(JOINTS)}"
            )
        except Exception as exc:
            self.fk = None
            self.fk_error = str(exc)
            self.get_logger().warning(f"EE tracking disabled: {exc}")

    def _on_clock(self, msg):
        value = _stamp_sec(msg.clock)
        self.clock_received += 1
        if self.last_clock_time is not None and value < self.last_clock_time - 1e-9:
            self._handle_time_reset(value, "clock")
        self.last_clock_time = value

    def _handle_time_reset(self, new_time, source):
        old_time = self.last_clock_time if source == "clock" else (
            self.last_controller_time if source == "controller_state" else self.last_dynamic_time
        )
        self._process_pending(force=True)
        self.epoch += 1
        self.reset_events.append({
            "source": source,
            "previous_time_sec": old_time,
            "new_time_sec": new_time,
            "new_epoch": self.epoch,
        })
        self.dynamic_samples.clear()
        self.last_clock_time = None
        self.last_controller_time = None
        self.last_dynamic_time = None
        self.get_logger().warning(f"Simulation time reset detected from {source}; starting epoch {self.epoch}")

    def _on_dynamic(self, msg):
        timestamp = _stamp_sec(msg.header.stamp)
        if self.last_dynamic_time is not None and timestamp < self.last_dynamic_time - 1e-9:
            self._handle_time_reset(timestamp, "dynamic_joint_states")
        self.last_dynamic_time = timestamp
        self.dynamic_received += 1
        velocity = {}
        effort = {}
        if len(set(msg.joint_names)) == len(msg.joint_names):
            for index, joint in enumerate(msg.joint_names):
                if index >= len(msg.interface_values):
                    continue
                interface = msg.interface_values[index]
                values = _named_values(interface.interface_names, interface.values)
                if "velocity" in values:
                    velocity[joint] = values["velocity"]
                if "effort" in values:
                    effort[joint] = values["effort"]
        self.dynamic_samples.append({
            "time": timestamp,
            "epoch": self.epoch,
            "velocity": velocity,
            "effort": effort,
        })
        self._process_pending(force=False)

    def _on_controller(self, msg):
        timestamp = _stamp_sec(msg.header.stamp)
        if self.last_controller_time is not None and timestamp < self.last_controller_time - 1e-9:
            self._handle_time_reset(timestamp, "controller_state")
        self.last_controller_time = timestamp
        self.controller_received += 1
        self.controller_times.append((self.epoch, timestamp))
        self.pending.append({
            "message": msg,
            "time": timestamp,
            "epoch": self.epoch,
            "wall_received": time.monotonic(),
        })
        self._process_pending(force=False)

    def _timer_flush_pending(self):
        # If dynamic states stop while controller states continue, do not retain
        # rows indefinitely. Two wall seconds allows future samples to arrive,
        # while the timestamp tolerance still prevents stale matches.
        if self.pending and time.monotonic() - self.pending[0]["wall_received"] >= 2.0:
            self._finalize_one(self.pending.popleft())

    def _process_pending(self, force):
        while self.pending:
            item = self.pending[0]
            same_epoch = [sample for sample in self.dynamic_samples if sample["epoch"] == item["epoch"]]
            future_window_closed = same_epoch and same_epoch[-1]["time"] >= item["time"] + self.args.velocity_tolerance
            if not force and not future_window_closed:
                break
            self.pending.popleft()
            self._finalize_one(item)

    def _nearest_dynamic(self, timestamp, epoch):
        candidates = [
            sample for sample in self.dynamic_samples
            if sample["epoch"] == epoch and abs(sample["time"] - timestamp) <= self.args.velocity_tolerance
        ]
        return min(candidates, key=lambda sample: abs(sample["time"] - timestamp)) if candidates else None

    def _gain(self, gain_name, joint_index):
        values = getattr(self.args, gain_name)
        return float(values[joint_index]) if values is not None else NAN

    def _finalize_one(self, item):
        msg = item["message"]
        names = list(msg.joint_names)
        reference_position = _point_values(names, msg.reference, "positions")
        actual_position = _point_values(names, msg.feedback, "positions")
        reference_velocity = _point_values(names, msg.reference, "velocities")
        reported_error = _point_values(names, msg.error, "positions")
        dynamic = self._nearest_dynamic(item["time"], item["epoch"])
        actual_velocity = dynamic["velocity"] if dynamic else {}
        measured_effort = dynamic["effort"] if dynamic else {}
        if dynamic and all(joint in actual_velocity for joint in JOINTS):
            self.velocity_matches += 1
        if dynamic and any(joint in measured_effort for joint in JOINTS):
            self.measured_effort_available_rows += 1

        record = {
            "time_sec": item["time"],
            "trajectory_id": self.trajectory_id,
            "controller_mode": self.args.controller_mode,
            "payload_kg": self.args.payload_kg,
            "_epoch": item["epoch"],
        }
        for index, joint in enumerate(JOINTS):
            reference = _finite_or_nan(reference_position, joint)
            actual = _finite_or_nan(actual_position, joint)
            error = reference - actual if math.isfinite(reference) and math.isfinite(actual) else NAN
            message_error = _finite_or_nan(reported_error, joint)
            if math.isfinite(error) and math.isfinite(message_error) and not math.isclose(error, message_error, abs_tol=1e-8):
                self.controller_error_mismatch_count += 1
            prefix = joint + "__"
            record[prefix + "reference_position_rad"] = reference
            record[prefix + "actual_position_rad"] = actual
            record[prefix + "position_error_rad"] = error
            record[prefix + "reference_velocity_rad_s"] = _finite_or_nan(reference_velocity, joint)
            record[prefix + "actual_velocity_rad_s"] = _finite_or_nan(actual_velocity, joint)
            # Position command interfaces are verified. Neither reference.effort
            # nor output.effort is relabelled as commanded torque.
            record[prefix + "commanded_effort_Nm"] = NAN
            record[prefix + "measured_effort_Nm"] = _finite_or_nan(measured_effort, joint)
            record[prefix + "Kp"] = self._gain("kp", index)
            record[prefix + "Ki"] = self._gain("ki", index)
            record[prefix + "Kd"] = self._gain("kd", index)

        for name in (
            "reference_ee_x_m", "reference_ee_y_m", "reference_ee_z_m",
            "actual_ee_x_m", "actual_ee_y_m", "actual_ee_z_m",
            "ee_error_x_mm", "ee_error_y_mm", "ee_error_z_mm", "ee_error_3d_mm",
        ):
            record[name] = NAN
        reference_vector = [record[joint + "__reference_position_rad"] for joint in JOINTS]
        actual_vector = [record[joint + "__actual_position_rad"] for joint in JOINTS]
        if self.fk is not None and all(math.isfinite(value) for value in reference_vector + actual_vector):
            desired_xyz = self.fk.xyz(reference_vector)
            actual_xyz = self.fk.xyz(actual_vector)
            error_mm = (desired_xyz - actual_xyz) * 1000.0
            for axis, value in zip("xyz", desired_xyz):
                record[f"reference_ee_{axis}_m"] = float(value)
            for axis, value in zip("xyz", actual_xyz):
                record[f"actual_ee_{axis}_m"] = float(value)
            for axis, value in zip("xyz", error_mm):
                record[f"ee_error_{axis}_mm"] = float(value)
            record["ee_error_3d_mm"] = float(np.linalg.norm(error_mm))

        self.csv_writer.writerow({name: record.get(name, NAN) for name in CSV_COLUMNS})
        self.csv_handle.flush()
        self.records.append(record)

    def _metrics(self):
        rows = []
        for joint in JOINTS:
            key = joint + "__position_error_rad"
            finite = [record[key] for record in self.records if math.isfinite(record[key])]
            rmse = math.sqrt(sum(value * value for value in finite) / len(finite)) if finite else NAN
            maximum = max((abs(value) for value in finite), default=NAN)
            iae = 0.0 if finite else NAN
            ise = 0.0 if finite else NAN
            previous = None
            for record in self.records:
                value = record[key]
                if previous is not None:
                    dt = record["time_sec"] - previous["time_sec"]
                    if record["_epoch"] == previous["_epoch"] and dt > 0.0 and math.isfinite(value) and math.isfinite(previous[key]):
                        iae += 0.5 * (abs(previous[key]) + abs(value)) * dt
                        ise += 0.5 * (previous[key] ** 2 + value ** 2) * dt
                previous = record
            rows.append({
                "joint": joint,
                "sample_count": len(finite),
                "position_rmse_rad": rmse,
                "max_abs_position_error_rad": maximum,
                "iae_rad_s": iae,
                "ise_rad2_s": ise,
                # Manual logger start/stop does not identify the pick/place cycle boundaries.
                "cycle_duration_sec": NAN,
            })
        return rows

    def _observed_rate(self):
        positive_intervals = []
        previous = None
        for epoch, timestamp in self.controller_times:
            if previous is not None and epoch == previous[0] and timestamp > previous[1]:
                positive_intervals.append(timestamp - previous[1])
            previous = (epoch, timestamp)
        return 1.0 / (sum(positive_intervals) / len(positive_intervals)) if positive_intervals else NAN

    def _info(self):
        times = [record["time_sec"] for record in self.records]
        observed_rate = self._observed_rate()
        return {
            "trajectory_id": self.trajectory_id,
            "controller_mode": self.args.controller_mode,
            "controller_mode_source": "default from verified position-command configuration" if not self.args.controller_mode_supplied else "user supplied; not runtime-verified by this read-only logger",
            "payload_kg": None if math.isnan(self.args.payload_kg) else self.args.payload_kg,
            "payload_metadata_source": "not supplied; physical attachment not inferred" if math.isnan(self.args.payload_kg) else "user supplied; physical attachment not verified",
            "joint_order": list(JOINTS),
            "topics": {
                "controller_state": CONTROLLER_TOPIC,
                "dynamic_joint_states": DYNAMIC_TOPIC,
                "clock": CLOCK_TOPIC,
                "robot_description": ROBOT_DESCRIPTION_TOPIC,
            },
            "qos": {
                "data_and_clock": "KEEP_LAST(200), BEST_EFFORT, VOLATILE",
                "robot_description": "KEEP_LAST(1), RELIABLE, TRANSIENT_LOCAL",
            },
            "timestamp_policy": "time_sec is the controller-state header stamp in simulation time; one row is written for every received controller-state callback",
            "velocity_alignment_policy": f"nearest DynamicJointState in the same simulation-time epoch within +/- {self.args.velocity_tolerance:.9g} s; otherwise blank/NaN; future samples are allowed before finalization and stale samples are never substituted",
            "simulation_time_reset_policy": "pending rows are finalized, the dynamic-state cache is cleared, and integration starts a new epoch; raw time_sec is not fabricated or offset",
            "sample_counts": {
                "controller_state_received": self.controller_received,
                "csv_rows_written": len(self.records),
                "dynamic_joint_states_received": self.dynamic_received,
                "rows_with_all_actual_velocities_aligned": self.velocity_matches,
                "rows_with_any_measured_effort": self.measured_effort_available_rows,
                "clock_messages_received": self.clock_received,
            },
            "observed_controller_state_rate_hz": observed_rate if math.isfinite(observed_rate) else None,
            "start_simulation_time_sec": times[0] if times else None,
            "end_simulation_time_sec": times[-1] if times else None,
            "simulation_time_resets": self.reset_events,
            "position_error_definition": "reference position minus actual/feedback position; controller error field is checked but not copied",
            "controller_error_value_mismatch_count": self.controller_error_mismatch_count,
            "effort_availability": {
                "commanded_effort_Nm": "unavailable: position command interfaces are configured; reference/output effort is not assumed to be commanded torque",
                "measured_effort_Nm": "copied only from a named 'effort' interface in DynamicJointState; blank/NaN when absent",
                "effort_error": "not computed because commanded and measured effort semantics are not verified comparable",
            },
            "gain_metadata_source": "not supplied" if self.args.kp is None and self.args.ki is None and self.args.kd is None else "user supplied; not read from or verified against a running controller",
            "ee_tracking": {
                "available": self.fk is not None,
                "base_frame": self.args.base_link,
                "tcp_link": self.args.tcp_link,
                "model_source": "runtime /robot_description, SHA-256 recorded below",
                "validation": "chain must contain exactly the six canonical movable joints in canonical order",
                "unavailable_reason": self.fk_error,
            },
            "robot_description_sha256": self.robot_description_sha256,
            "configuration_provenance": {
                "controller_configuration": "ur5_moveit_config/config/ros2_controllers.yaml inspected in this workspace; runtime parameters are not modified",
                "moveit_chain": "ur5_moveit_config/config/ur5.srdf declares base_link -> tool0",
                "logger_version": "ur5_data_logger 0.1.0",
                "created_utc": datetime.now(timezone.utc).isoformat(),
            },
            "cycle_duration_sec": None,
            "cycle_duration_note": "not reported: manual logger boundaries do not prove pick-and-place cycle start/finish",
            "csv_columns": list(CSV_COLUMNS),
        }

    def finalize(self):
        if self.finalized:
            return
        self.finalized = True
        self._process_pending(force=True)
        self.csv_handle.flush()
        self.csv_handle.close()
        metrics = self._metrics()
        info = self._info()
        write_summary(self.summary_path, metrics)
        with self.info_path.open("w", encoding="utf-8") as handle:
            json.dump(info, handle, indent=2, allow_nan=False)
            handle.write("\n")
        write_workbook(self.workbook_path, self.records, metrics, info, self.fk is not None)
        if rclpy.ok():
            self.get_logger().info(
                f"Finalized {len(self.records)} rows from {self.controller_received} "
                f"controller messages in {self.output_dir}"
            )


def _parser():
    parser = argparse.ArgumentParser(description="Passive UR5 controller-state experiment logger")
    parser.add_argument("--trajectory-id", default=None, help="Run/trajectory ID (default: generated UTC timestamp)")
    parser.add_argument("--controller-mode", choices=("baseline_position", "fixed_pid", "adaptive_pid"), default=None)
    parser.add_argument("--payload-kg", type=float, default=NAN, help="User-supplied payload metadata; not physical verification")
    parser.add_argument("--output-root", default="experiment_logs")
    parser.add_argument("--velocity-tolerance", type=float, default=0.015, help="Maximum absolute timestamp difference in seconds")
    parser.add_argument("--base-link", default="base_link")
    parser.add_argument("--tcp-link", default="tool0")
    parser.add_argument("--kp", type=float, nargs=6, metavar=("J1", "J2", "J3", "J4", "J5", "J6"))
    parser.add_argument("--ki", type=float, nargs=6, metavar=("J1", "J2", "J3", "J4", "J5", "J6"))
    parser.add_argument("--kd", type=float, nargs=6, metavar=("J1", "J2", "J3", "J4", "J5", "J6"))
    return parser


def main(argv=None):
    cli = rclpy.utilities.remove_ros_args(args=sys.argv if argv is None else argv)[1:]
    args = _parser().parse_args(cli)
    if args.velocity_tolerance < 0.0:
        raise SystemExit("--velocity-tolerance must be non-negative")
    if math.isfinite(args.payload_kg) and args.payload_kg < 0.0:
        raise SystemExit("--payload-kg must be non-negative")
    args.controller_mode_supplied = args.controller_mode is not None
    args.controller_mode = args.controller_mode or "baseline_position"
    args.trajectory_id = args.trajectory_id or datetime.now(timezone.utc).strftime("run_%Y%m%dT%H%M%SZ")

    rclpy.init(args=sys.argv if argv is None else argv)
    node = None
    try:
        node = ExperimentLogger(args)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.finalize()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
