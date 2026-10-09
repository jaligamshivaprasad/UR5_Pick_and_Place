"""Extract UR5 trajectory execution metrics from ROS 2 bags.

The bag reader intentionally imports ROS 2 packages only when a bag is read,
so the metric and filename parsing helpers remain usable in unit tests.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import logging
from pathlib import Path
import re
from typing import Any, Iterable, Sequence

import numpy as np
import pandas as pd

JOINT_COUNT = 6
JOINT_STATES_TOPIC = "/joint_states"
COMMAND_TOPICS = (
    "/joint_trajectory_controller/joint_trajectory",
    "/joint_trajectory_controller/controller_state",
    # The repository's Humble controller is named arm_controller.
    "/arm_controller/joint_trajectory",
    "/arm_controller/controller_state",
)
RUN_NAME_RE = re.compile(
    r"^(?P<method>.+)_(?P<payload>[0-9]+(?:\.[0-9]+)?)kg_run(?P<index>[0-9]+)$"
)
LOGGER = logging.getLogger(__name__)


@dataclass(frozen=True)
class RunInfo:
    """Metadata parsed from a bag directory name."""

    run_id: str
    method: str
    payload_kg: float


@dataclass
class SignalSeries:
    """Timestamped six-joint signal collected from a bag."""

    times: list[float]
    values: list[np.ndarray]
    names: list[str]


def parse_run_name(name: str) -> RunInfo:
    """Parse ``{Method}_{Payload}kg_run{idx}`` and reject ambiguous names."""
    match = RUN_NAME_RE.fullmatch(name)
    if match is None:
        raise ValueError(
            f"Bag directory {name!r} must match "
            "'{Method}_{Payload}kg_run{idx}'"
        )
    return RunInfo(
        run_id=name,
        method=match.group("method"),
        payload_kg=float(match.group("payload")),
    )


def _message_time(message: Any, bag_time_ns: int) -> float:
    """Return a message header time when present, otherwise bag time."""
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    if stamp is not None and (stamp.sec != 0 or stamp.nanosec != 0):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9
    return bag_time_ns * 1e-9


def _six_joint_values(
    names: Sequence[str], values: Sequence[float], joint_names: Sequence[str]
) -> np.ndarray | None:
    """Map a message's named values to the requested six UR5 joints."""
    by_name = dict(zip(names, values))
    if not all(name in by_name for name in joint_names):
        return None
    return np.asarray([by_name[name] for name in joint_names], dtype=float)


def _normalise_series(series: SignalSeries) -> SignalSeries:
    """Sort samples and collapse duplicate timestamps by keeping the last."""
    if not series.times:
        return series
    order = np.argsort(np.asarray(series.times))
    times = np.asarray(series.times)[order]
    values = np.asarray(series.values)[order]
    unique_times, unique_indices = np.unique(times, return_index=True)
    # The last sample at a duplicate timestamp is the most recent bag sample.
    last_indices = np.r_[unique_indices[1:] - 1, len(times) - 1]
    return SignalSeries(
        times=unique_times.tolist(),
        values=values[last_indices].tolist(),
        names=series.names,
    )


def _trajectory_samples(message: Any, bag_time_ns: int) -> Iterable[tuple[float, list[str], Sequence[float]]]:
    """Yield absolute-time samples from JointTrajectory or controller state."""
    message_time = _message_time(message, bag_time_ns)
    joint_names = list(message.joint_names)
    for point in message.points:
        offset = float(point.time_from_start.sec) + point.time_from_start.nanosec * 1e-9
        yield message_time + offset, joint_names, point.positions


def _read_bag(
    bag_path: Path,
    joint_names: Sequence[str],
) -> tuple[SignalSeries, SignalSeries, SignalSeries]:
    """Read actual positions/efforts and commanded positions from one bag."""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise RuntimeError(
            "ROS 2 Humble Python packages are required to read bags. "
            "Source /opt/ros/humble/setup.bash before running the analyzer."
        ) from exc

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_path), storage_id="sqlite3"
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader.open(storage_options, converter_options)
    topic_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    selected_topics = {
        topic: topic_types[topic]
        for topic in (JOINT_STATES_TOPIC, *COMMAND_TOPICS)
        if topic in topic_types
    }
    if JOINT_STATES_TOPIC not in selected_topics:
        raise ValueError(f"{bag_path} does not contain {JOINT_STATES_TOPIC}")
    if not any(topic in selected_topics for topic in COMMAND_TOPICS):
        raise ValueError(
            f"{bag_path} contains neither command topic: {', '.join(COMMAND_TOPICS)}"
        )

    message_types = {
        topic: get_message(message_type)
        for topic, message_type in selected_topics.items()
    }
    actual_positions = SignalSeries([], [], list(joint_names))
    efforts = SignalSeries([], [], list(joint_names))
    commands = SignalSeries([], [], list(joint_names))

    while reader.has_next():
        topic, raw_data, bag_time_ns = reader.read_next()
        if topic not in selected_topics:
            continue
        message = deserialize_message(raw_data, message_types[topic])
        if topic == JOINT_STATES_TOPIC:
            values = _six_joint_values(message.name, message.position, joint_names)
            if values is not None:
                timestamp = _message_time(message, bag_time_ns)
                actual_positions.times.append(timestamp)
                actual_positions.values.append(values)
                effort = _six_joint_values(message.name, message.effort, joint_names)
                if effort is not None:
                    efforts.times.append(timestamp)
                    efforts.values.append(effort)
            continue

        if hasattr(message, "points"):  # trajectory_msgs/msg/JointTrajectory
            samples = _trajectory_samples(message, bag_time_ns)
        elif hasattr(message, "desired"):  # control_msgs controller state
            desired = message.desired
            samples = (
                (_message_time(message, bag_time_ns), message.joint_names, desired.positions),
            )
        else:
            LOGGER.warning("Ignoring unsupported command message on %s", topic)
            continue
        for timestamp, names, positions in samples:
            values = _six_joint_values(names, positions, joint_names)
            if values is not None:
                commands.times.append(timestamp)
                commands.values.append(values)

    return (
        _normalise_series(actual_positions),
        _normalise_series(commands),
        _normalise_series(efforts),
    )


def _integral_squared(times: np.ndarray, values: np.ndarray) -> float:
    squared = np.sum(np.square(values), axis=1)
    if len(times) < 2:
        return 0.0
    # ``trapz`` is available in the NumPy versions shipped with Humble.
    return float(np.trapz(squared, times))


def calculate_metrics(
    actual: SignalSeries,
    commanded: SignalSeries,
    efforts: SignalSeries,
    *,
    velocity_scaling: float = 1.0,
    acceleration_scaling: float = 1.0,
) -> dict[str, float]:
    """Calculate execution metrics for one run."""
    if len(actual.times) < 2:
        raise ValueError("At least two actual joint-state samples are required")
    if len(commanded.times) == 0:
        raise ValueError("At least one commanded joint-position sample is required")

    actual_t = np.asarray(actual.times, dtype=float)
    actual_q = np.asarray(actual.values, dtype=float)
    command_t = np.asarray(commanded.times, dtype=float)
    command_q = np.asarray(commanded.values, dtype=float)
    overlap_start = max(actual_t[0], command_t[0])
    overlap_end = min(actual_t[-1], command_t[-1])
    if overlap_end <= overlap_start:
        raise ValueError("Actual and commanded trajectories have no time overlap")
    mask = (actual_t >= overlap_start) & (actual_t <= overlap_end)
    if not np.any(mask):
        raise ValueError("No actual samples fall within the command time range")
    sampled_actual_t = actual_t[mask]
    sampled_actual_q = actual_q[mask]
    sampled_command_q = np.column_stack(
        [np.interp(sampled_actual_t, command_t, command_q[:, joint]) for joint in range(JOINT_COUNT)]
    )
    error = sampled_command_q - sampled_actual_q
    rmse = np.sqrt(np.mean(np.square(error), axis=0))

    effort_t = np.asarray(efforts.times, dtype=float)
    effort_q = np.asarray(efforts.values, dtype=float)
    peak_torque = (
        np.max(np.abs(effort_q), axis=0) if len(effort_q) else np.full(JOINT_COUNT, np.nan)
    )
    total_ist = _integral_squared(effort_t, effort_q) if len(effort_q) else np.nan

    metrics: dict[str, float] = {
        "Cycle_Time_s": float(actual_t[-1] - actual_t[0]),
        "Max_Joint_RMSE": float(np.max(rmse)),
        "Peak_Torque_J2": float(peak_torque[1]),
        "Peak_Torque_J3": float(peak_torque[2]),
        "Total_IST": total_ist,
        "S_v": float(velocity_scaling),
        "S_a": float(acceleration_scaling),
    }
    metrics.update({f"Joint{index + 1}_RMSE": float(value) for index, value in enumerate(rmse)})
    return metrics


def analyse_run(
    bag_path: Path,
    *,
    velocity_scaling: float = 1.0,
    acceleration_scaling: float = 1.0,
    joint_names: Sequence[str] = (
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ),
) -> dict[str, Any]:
    """Read and calculate metrics for one bag directory."""
    info = parse_run_name(bag_path.name)
    actual, commanded, efforts = _read_bag(bag_path, joint_names)
    result: dict[str, Any] = {
        "Run_ID": info.run_id,
        "Method": info.method,
        "Payload_kg": info.payload_kg,
    }
    result.update(
        calculate_metrics(
            actual,
            commanded,
            efforts,
            velocity_scaling=velocity_scaling,
            acceleration_scaling=acceleration_scaling,
        )
    )
    return result


def export_results(rows: Sequence[dict[str, Any]], output_path: Path) -> None:
    """Write run-level and grouped statistical results to an Excel workbook."""
    if not rows:
        raise ValueError("No valid runs were found")
    summary_columns = [
        "Run_ID", "Method", "Payload_kg", "S_v", "S_a", "Cycle_Time_s",
        *(f"Joint{index}_RMSE" for index in range(1, 7)),
        "Max_Joint_RMSE", "Peak_Torque_J2", "Peak_Torque_J3", "Total_IST",
    ]
    summary = pd.DataFrame(rows).reindex(columns=summary_columns)
    grouped = summary.groupby(["Method", "Payload_kg"], dropna=False)[
        ["Cycle_Time_s", "Max_Joint_RMSE", "Peak_Torque_J2", "Peak_Torque_J3"]
    ].agg(["mean", "std"]).reset_index()
    grouped.columns = [
        "_".join(str(part) for part in column if part).rstrip("_")
        if isinstance(column, tuple)
        else str(column)
        for column in grouped.columns
    ]
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with pd.ExcelWriter(output_path, engine="openpyxl") as writer:
        summary.to_excel(writer, sheet_name="Run_Summary", index=False)
        grouped.to_excel(writer, sheet_name="Statistical_Summary", index=False)


def _bag_directories(input_dir: Path) -> list[Path]:
    return sorted(
        path for path in input_dir.iterdir()
        if path.is_dir() and RUN_NAME_RE.fullmatch(path.name)
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_dir", type=Path, help="Directory containing ROS 2 bag directories")
    parser.add_argument(
        "-o", "--output", type=Path, default=Path("UR5_Experiment_Results.xlsx"),
        help="Excel output path (default: UR5_Experiment_Results.xlsx)",
    )
    parser.add_argument("--velocity-scaling", type=float, default=1.0, dest="velocity_scaling")
    parser.add_argument("--acceleration-scaling", type=float, default=1.0, dest="acceleration_scaling")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    args = build_parser().parse_args(argv)
    if not args.input_dir.is_dir():
        raise SystemExit(f"Input directory does not exist: {args.input_dir}")
    rows = []
    for bag_path in _bag_directories(args.input_dir):
        try:
            rows.append(
                analyse_run(
                    bag_path,
                    velocity_scaling=args.velocity_scaling,
                    acceleration_scaling=args.acceleration_scaling,
                )
            )
        except (OSError, RuntimeError, ValueError) as exc:
            LOGGER.error("Skipping %s: %s", bag_path.name, exc)
    if not rows:
        raise SystemExit("No valid bag runs were found")
    export_results(rows, args.output)
    LOGGER.info("Wrote %d runs to %s", len(rows), args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
