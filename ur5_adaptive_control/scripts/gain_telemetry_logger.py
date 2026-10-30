#!/usr/bin/env python3
"""
gain_telemetry_logger.py
Subscribes to /adaptive_arm_controller/gain_telemetry and logs joint-by-joint
gain adaptation telemetry to CSV during experiments.
"""

import argparse
import csv
from pathlib import Path
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray
from rosgraph_msgs.msg import Clock

JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

STATE_MAP = {
    0: "DISABLED",
    1: "ACTIVE",
    2: "FROZEN_SATURATION",
    3: "FROZEN_OSCILLATION",
    4: "FROZEN_FEEDBACK",
    5: "FALLBACK",
}

CSV_HEADER = [
    "timestamp_sec",
    "clock_sec",
    "joint_name",
    "joint_index",
    "kp",
    "ki",
    "kd",
    "position_error_rad",
    "filtered_velocity_error_rad_s",
    "reference_velocity_rad_s",
    "composite_error",
    "commanded_effort_Nm",
    "saturated",
    "adaptation_state_code",
    "adaptation_state_name",
]

class GainTelemetryLogger(Node):
    def __init__(self, output_file: Path):
        super().__init__("gain_telemetry_logger")
        self.output_file = output_file
        self.output_file.parent.mkdir(parents=True, exist_ok=True)
        self.file_handle = self.output_file.open("w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.file_handle)
        self.writer.writerow(CSV_HEADER)
        self.file_handle.flush()

        self.latest_clock_sec = 0.0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(Clock, "/clock", self.clock_cb, 10)
        self.create_subscription(
            Float64MultiArray,
            "/adaptive_arm_controller/gain_telemetry",
            self.telemetry_cb,
            qos,
        )
        self.get_logger().info(f"Logging gain adaptation telemetry to: {self.output_file}")

    def clock_cb(self, msg: Clock):
        self.latest_clock_sec = float(msg.clock.sec) + float(msg.clock.nanosec) * 1e-9

    def telemetry_cb(self, msg: Float64MultiArray):
        # 6 joints * 8 values = 48 floats
        # Per joint: [kp, ki, kd, pos_err, vel_err, ref_vel, comp_err, effort, sat_flag, state_code]
        # Or 10 floats per joint -> 60 floats
        stride = 10
        data = msg.data
        if len(data) < 6 * stride:
            return

        now = self.get_clock().now()
        stamp_sec = now.nanoseconds * 1e-9

        for j_idx, j_name in enumerate(JOINTS):
            base = j_idx * stride
            kp = data[base + 0]
            ki = data[base + 1]
            kd = data[base + 2]
            pos_err = data[base + 3]
            vel_err = data[base + 4]
            ref_vel = data[base + 5]
            comp_err = data[base + 6]
            effort = data[base + 7]
            sat = bool(int(round(data[base + 8])))
            state_code = int(round(data[base + 9]))
            state_name = STATE_MAP.get(state_code, "UNKNOWN")

            self.writer.writerow([
                f"{stamp_sec:.6f}",
                f"{self.latest_clock_sec:.6f}",
                j_name,
                j_idx,
                f"{kp:.6f}",
                f"{ki:.6f}",
                f"{kd:.6f}",
                f"{pos_err:.6f}",
                f"{vel_err:.6f}",
                f"{ref_vel:.6f}",
                f"{comp_err:.6f}",
                f"{effort:.6f}",
                sat,
                state_code,
                state_name,
            ])
        self.file_handle.flush()

    def destroy_node(self):
        try:
            self.file_handle.flush()
            self.file_handle.close()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description="Log UR5 gain adaptation telemetry.")
    parser.add_argument("--output", type=str, required=True, help="Output CSV path")
    parsed_args, _ = parser.parse_known_args()

    node = GainTelemetryLogger(Path(parsed_args.output))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

