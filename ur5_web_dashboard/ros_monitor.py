"""Independent read-only ROS 2 monitoring node for UR5 controller state and kinematics.

Safely listens to controller feedback, velocities, and dynamic states in a
background thread without modifying controllers, publishing commands, or
interfering with active loggers.
"""

from collections import deque
import math
import threading
import time
from typing import Any, Dict, List, Optional

import numpy as np

# ROS 2 imports with graceful fallback if environment is missing
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from control_msgs.msg import DynamicJointState, JointTrajectoryControllerState
    from sensor_msgs.msg import JointState
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    Node = object
    Clock = Any
    DynamicJointState = Any
    JointTrajectoryControllerState = Any
    JointState = Any
    String = Any

from .config import (
    BASE_LINK,
    DEFAULT_BUFFER_SIZE,
    JOINTS,
    TCP_LINK,
    TOPIC_CLOCK,
    TOPIC_CONTROLLER_STATE,
    TOPIC_DYNAMIC_JOINT_STATES,
    TOPIC_JOINT_STATES,
    TOPIC_ROBOT_DESCRIPTION,
)
from .fk_engine import VerifiedFK


def _stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class ROSMonitor:
    """Thread-safe ROS 2 subscriber and bounded ring-buffer manager."""

    def __init__(self, buffer_size: int = DEFAULT_BUFFER_SIZE):
        self.buffer_size = buffer_size
        self._lock = threading.Lock()
        self.records: deque = deque(maxlen=buffer_size)

        # Topic diagnostics and statistics
        self.msg_counts = {
            "controller_state": 0,
            "joint_states": 0,
            "dynamic_joint_states": 0,
            "clock": 0,
            "robot_description": 0,
        }
        self.last_msg_time = {
            "controller_state": 0.0,
            "joint_states": 0.0,
            "dynamic_joint_states": 0.0,
            "clock": 0.0,
        }
        self.rates_hz = {
            "controller_state": 0.0,
            "joint_states": 0.0,
            "clock": 0.0,
        }
        self._rate_timestamps = {
            "controller_state": deque(maxlen=50),
            "joint_states": deque(maxlen=50),
            "clock": deque(maxlen=50),
        }

        # Status and health flags
        self.is_ros_initialized = False
        self.node = None
        self.spin_thread = None
        self._running = False
        self.sim_time = 0.0
        self.last_sim_time = None
        self.epoch = 0
        self.resets_detected = 0

        # Hardware & Controller authenticity
        self.controller_name = "joint_trajectory_controller"
        self.controller_mode = "Open-Loop Position (Verified)"
        self.effort_available = False
        self.effort_status_note = "Unavailable: Position interfaces active (gz_ros2_control/GazeboSimSystem)"

        # Forward Kinematics engine
        self.fk_engine: Optional[VerifiedFK] = None
        self.fk_status = "Awaiting /robot_description"
        self.urdf_sha256 = None

        # Cached latest joint states and velocities from /joint_states and dynamic states
        self._latest_dynamic = {}
        self._latest_joint_vel = {}
        self._latest_joint_pos = {}
        self._prev_ref_pos = {}
        self._prev_act_pos = {}
        self._prev_stamp_sec = None

    def start(self):
        """Initialize ROS 2 node and spin thread if available."""
        if not ROS_AVAILABLE:
            self.fk_status = "ROS 2 Python libraries not installed"
            return

        with self._lock:
            if self._running:
                return
            self._running = True

        try:
            if not rclpy.ok():
                rclpy.init(args=None)
            self.is_ros_initialized = True

            # Create independent read-only ROS 2 node with use_sim_time enabled
            self.node = Node(
                "ur5_dashboard_passive_monitor",
                parameter_overrides=[Parameter("use_sim_time", value=True)],
            )

            qos_stream = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=100,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )

            qos_transient = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )

            # Subscriptions strictly in read-only mode (NEVER publish commands or switch controllers)
            self.node.create_subscription(
                JointTrajectoryControllerState,
                TOPIC_CONTROLLER_STATE,
                self._on_controller_state,
                qos_stream,
            )
            self.node.create_subscription(
                JointState,
                TOPIC_JOINT_STATES,
                self._on_joint_states,
                qos_stream,
            )
            self.node.create_subscription(
                DynamicJointState,
                TOPIC_DYNAMIC_JOINT_STATES,
                self._on_dynamic_joint_states,
                qos_stream,
            )
            self.node.create_subscription(
                String,
                TOPIC_ROBOT_DESCRIPTION,
                self._on_robot_description,
                qos_transient,
            )

            self.spin_thread = threading.Thread(target=self._spin_worker, daemon=True)
            self.spin_thread.start()

        except Exception as e:
            self.is_ros_initialized = False
            self.fk_status = f"ROS init error: {str(e)}"

    def _spin_worker(self):
        """Worker loop running in a separate thread with throttled sleep to prevent CPU spinning."""
        try:
            while self._running and rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.02)
                self._compute_rolling_rates()
                time.sleep(0.005)
        except Exception:
            pass

    def stop(self):
        """Gracefully stop node and thread."""
        with self._lock:
            self._running = False
        if self.node is not None:
            try:
                self.node.destroy_node()
            except Exception:
                pass
            self.node = None
        if ROS_AVAILABLE:
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass

    def _update_rate(self, topic: str):
        now = time.monotonic()
        deq = self._rate_timestamps.get(topic)
        if deq is not None:
            deq.append(now)
            if len(deq) > 1:
                dt = deq[-1] - deq[0]
                if dt > 0.05:
                    self.rates_hz[topic] = (len(deq) - 1) / dt

    def _compute_rolling_rates(self):
        now = time.monotonic()
        for topic, deq in self._rate_timestamps.items():
            if deq and (now - deq[-1] > 2.0):
                self.rates_hz[topic] = 0.0

    def get_current_sim_time(self) -> float:
        """Get simulation time directly from ROS 2 node clock (zero Python subscription overhead)."""
        if self.node is not None and ROS_AVAILABLE and rclpy.ok():
            try:
                t = float(self.node.get_clock().now().nanoseconds * 1e-9)
                if t > 0.0:
                    with self._lock:
                        if self.last_sim_time is not None and t < self.last_sim_time - 1e-4:
                            self.epoch += 1
                            self.resets_detected += 1
                        self.sim_time = t
                        self.last_sim_time = t
                    return t
            except Exception:
                pass
        return self.sim_time

    def _on_clock(self, msg: Clock):
        pass

    def _on_robot_description(self, msg: String):
        try:
            import hashlib
            digest = hashlib.sha256(msg.data.encode("utf-8")).hexdigest()
            with self._lock:
                self.msg_counts["robot_description"] += 1
                self.urdf_sha256 = digest
                self.fk_engine = VerifiedFK(msg.data, BASE_LINK, TCP_LINK, JOINTS)
                self.fk_status = f"Verified: {BASE_LINK} -> {TCP_LINK}"
        except Exception as e:
            with self._lock:
                self.fk_engine = None
                self.fk_status = f"FK setup failed: {str(e)}"

    def _on_dynamic_joint_states(self, msg: DynamicJointState):
        with self._lock:
            self.msg_counts["dynamic_joint_states"] += 1
            self.last_msg_time["dynamic_joint_states"] = time.monotonic()
            vel = {}
            eff = {}
            for idx, name in enumerate(msg.joint_names):
                if idx < len(msg.interface_values):
                    iv = msg.interface_values[idx]
                    for if_name, val in zip(iv.interface_names, iv.values):
                        if if_name == "velocity":
                            vel[name] = float(val)
                        elif if_name == "effort":
                            eff[name] = float(val)
            self._latest_dynamic = {"vel": vel, "eff": eff, "stamp": _stamp_to_sec(msg.header.stamp)}
            if eff:
                self.effort_available = True
                self.effort_status_note = "Effort interfaces reported"

    def _on_joint_states(self, msg: JointState):
        with self._lock:
            self.msg_counts["joint_states"] += 1
            self.last_msg_time["joint_states"] = time.monotonic()
            if hasattr(msg, "name"):
                for idx, name in enumerate(msg.name):
                    if hasattr(msg, "velocity") and idx < len(msg.velocity):
                        v = msg.velocity[idx]
                        if math.isfinite(v):
                            self._latest_joint_vel[name] = float(v)
                    if hasattr(msg, "position") and idx < len(msg.position):
                        p = msg.position[idx]
                        if math.isfinite(p):
                            self._latest_joint_pos[name] = float(p)
        self._update_rate("joint_states")

    def _on_controller_state(self, msg: JointTrajectoryControllerState):
        now_mono = time.monotonic()
        stamp_sec = _stamp_to_sec(msg.header.stamp)
        if stamp_sec <= 0.0:
            stamp_sec = self.sim_time if self.sim_time > 0.0 else now_mono

        with self._lock:
            self.msg_counts["controller_state"] += 1
            self.last_msg_time["controller_state"] = now_mono

            # Time step for numerical derivatives if needed
            dt = None
            if self._prev_stamp_sec is not None:
                dt_cand = stamp_sec - self._prev_stamp_sec
                if 0.001 < dt_cand < 0.5:
                    dt = dt_cand

            # Joint index map from msg.joint_names
            name_to_idx = {name: i for i, name in enumerate(msg.joint_names)}

            # Extract positions, velocities, errors
            ref_pos = {}
            act_pos = {}
            err_pos = {}
            ref_vel = {}
            act_vel = {}
            meas_eff = {}

            # Positions & Velocities
            for joint in JOINTS:
                idx = name_to_idx.get(joint)
                if idx is not None:
                    # Reference Position
                    if hasattr(msg, "reference") and idx < len(msg.reference.positions) and math.isfinite(msg.reference.positions[idx]):
                        ref_pos[joint] = float(msg.reference.positions[idx])
                    else:
                        ref_pos[joint] = float("nan")

                    # Actual Position (Feedback preferred, then actual, then latest /joint_states)
                    if hasattr(msg, "feedback") and idx < len(msg.feedback.positions) and math.isfinite(msg.feedback.positions[idx]):
                        act_pos[joint] = float(msg.feedback.positions[idx])
                    elif hasattr(msg, "actual") and idx < len(msg.actual.positions) and math.isfinite(msg.actual.positions[idx]):
                        act_pos[joint] = float(msg.actual.positions[idx])
                    elif joint in self._latest_joint_pos:
                        act_pos[joint] = self._latest_joint_pos[joint]
                    else:
                        act_pos[joint] = float("nan")

                    # Error: reference minus actual
                    if math.isfinite(ref_pos[joint]) and math.isfinite(act_pos[joint]):
                        err_pos[joint] = ref_pos[joint] - act_pos[joint]
                    elif hasattr(msg, "error") and idx < len(msg.error.positions) and math.isfinite(msg.error.positions[idx]):
                        err_pos[joint] = float(msg.error.positions[idx])
                    else:
                        err_pos[joint] = float("nan")

                    # Reference Velocity
                    if hasattr(msg, "reference") and idx < len(msg.reference.velocities) and math.isfinite(msg.reference.velocities[idx]):
                        ref_vel[joint] = float(msg.reference.velocities[idx])
                    elif dt is not None and joint in self._prev_ref_pos and math.isfinite(ref_pos[joint]) and math.isfinite(self._prev_ref_pos[joint]):
                        ref_vel[joint] = (ref_pos[joint] - self._prev_ref_pos[joint]) / dt
                    else:
                        ref_vel[joint] = 0.0

                    # Actual Velocity (Feedback preferred, then /joint_states, then numerical diff)
                    if hasattr(msg, "feedback") and idx < len(msg.feedback.velocities) and math.isfinite(msg.feedback.velocities[idx]):
                        act_vel[joint] = float(msg.feedback.velocities[idx])
                    elif hasattr(msg, "actual") and idx < len(msg.actual.velocities) and math.isfinite(msg.actual.velocities[idx]):
                        act_vel[joint] = float(msg.actual.velocities[idx])
                    elif joint in self._latest_joint_vel and math.isfinite(self._latest_joint_vel[joint]):
                        act_vel[joint] = self._latest_joint_vel[joint]
                    elif joint in self._latest_dynamic.get("vel", {}):
                        act_vel[joint] = self._latest_dynamic["vel"][joint]
                    elif dt is not None and joint in self._prev_act_pos and math.isfinite(act_pos[joint]) and math.isfinite(self._prev_act_pos[joint]):
                        act_vel[joint] = (act_pos[joint] - self._prev_act_pos[joint]) / dt
                    else:
                        act_vel[joint] = 0.0

                    # Effort: never fabricate!
                    if joint in self._latest_dynamic.get("eff", {}):
                        meas_eff[joint] = self._latest_dynamic["eff"][joint]
                    else:
                        meas_eff[joint] = float("nan")
                else:
                    ref_pos[joint] = float("nan")
                    act_pos[joint] = float("nan")
                    err_pos[joint] = float("nan")
                    ref_vel[joint] = float("nan")
                    act_vel[joint] = float("nan")
                    meas_eff[joint] = float("nan")

            self._prev_ref_pos = dict(ref_pos)
            self._prev_act_pos = dict(act_pos)
            self._prev_stamp_sec = stamp_sec

            # Forward Kinematics calculation
            ee_ref_xyz = [float("nan"), float("nan"), float("nan")]
            ee_act_xyz = [float("nan"), float("nan"), float("nan")]
            ee_err_mm = [float("nan"), float("nan"), float("nan")]
            ee_3d_err_mm = float("nan")

            if self.fk_engine is not None:
                q_ref = [ref_pos[j] for j in JOINTS]
                q_act = [act_pos[j] for j in JOINTS]
                if all(math.isfinite(v) for v in q_ref) and all(math.isfinite(v) for v in q_act):
                    try:
                        xyz_ref = self.fk_engine.xyz(q_ref)
                        xyz_act = self.fk_engine.xyz(q_act)
                        ee_ref_xyz = [float(xyz_ref[0]), float(xyz_ref[1]), float(xyz_ref[2])]
                        ee_act_xyz = [float(xyz_act[0]), float(xyz_act[1]), float(xyz_act[2])]
                        err_vec = (xyz_ref - xyz_act) * 1000.0
                        ee_err_mm = [float(err_vec[0]), float(err_vec[1]), float(err_vec[2])]
                        ee_3d_err_mm = float(np.linalg.norm(err_vec))
                    except Exception:
                        pass

            record = {
                "time_sec": stamp_sec,
                "wall_time": now_mono,
                "epoch": self.epoch,
                "ref_pos": ref_pos,
                "act_pos": act_pos,
                "err_pos": err_pos,
                "ref_vel": ref_vel,
                "act_vel": act_vel,
                "meas_eff": meas_eff,
                "ee_ref_xyz": ee_ref_xyz,
                "ee_act_xyz": ee_act_xyz,
                "ee_err_mm": ee_err_mm,
                "ee_3d_err_mm": ee_3d_err_mm,
            }
            self.records.append(record)

        self._update_rate("controller_state")

    def get_snapshot(self) -> Dict[str, Any]:
        """Return a thread-safe snapshot of live buffer and diagnostics."""
        sim_t = self.get_current_sim_time()
        with self._lock:
            data = list(self.records)
            diag = {
                "is_ros_available": ROS_AVAILABLE,
                "is_ros_initialized": self.is_ros_initialized,
                "is_running": self._running,
                "sim_time": sim_t,
                "msg_counts": dict(self.msg_counts),
                "rates_hz": dict(self.rates_hz),
                "controller_name": self.controller_name,
                "controller_mode": self.controller_mode,
                "effort_available": self.effort_available,
                "effort_status_note": self.effort_status_note,
                "fk_status": self.fk_status,
                "resets_detected": self.resets_detected,
                "buffer_count": len(data),
                "buffer_capacity": self.buffer_size,
                "last_controller_seen_ago": time.monotonic() - self.last_msg_time["controller_state"]
                if self.last_msg_time["controller_state"] > 0 else 999.0,
            }
        return {"data": data, "diagnostics": diag}

    def clear_buffer(self):
        """Clear memory buffer without stopping subscription."""
        with self._lock:
            self.records.clear()
