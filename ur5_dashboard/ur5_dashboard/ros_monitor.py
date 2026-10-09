"""Non-blocking ROS graph and topic freshness monitor."""
from __future__ import annotations

import math
import threading
import time
from collections import deque

from .models import Check


class RosMonitor:
    ARM_JOINTS = (
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
    )
    POSITION_WINDOW_S = 30.0
    TOPICS = {
        "clock": "/clock",
        "joint_states": "/joint_states",
    }

    def __init__(self, freshness_s=3.0):
        self.freshness_s = freshness_s
        self.position_freshness_s = freshness_s
        self.position_sync_tolerance_s = 0.5
        self._lock = threading.Lock()
        self._last = {}
        self._node = None
        self._thread = None
        self._running = False
        self._error = None
        self._telemetry_lock = threading.Lock()
        self._joint_effort = {}
        self._joint_velocity = {}
        self._last_joint_state_monotonic = None
        self._samples = deque(maxlen=300)
        self._last_controller_monotonic = None
        self._telemetry_error = None
        self._fk_client = None
        self._last_fk_request_monotonic = 0.0
        self._gazebo_tool_poses = deque(maxlen=600)
        self._position_samples = deque(maxlen=300)
        self._position_error = "Waiting for Gazebo tool pose and MoveIt FK"
        self._last_position_monotonic = None

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._node:
            try:
                import rclpy
                self._node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass

    def _run(self):
        try:
            import rclpy
            from rclpy.node import Node
            from rosgraph_msgs.msg import Clock
            from sensor_msgs.msg import JointState
            from control_msgs.msg import JointTrajectoryControllerState
            from moveit_msgs.srv import GetPositionFK
            from tf2_msgs.msg import TFMessage
            rclpy.init(args=None)
            node = Node("ur5_dashboard_monitor")
            self._node = node
            self._fk_client = node.create_client(GetPositionFK, "/compute_fk")
            node.create_subscription(Clock, "/clock", lambda _: self._seen("clock"), 10)
            node.create_subscription(
                JointState, "/joint_states", self._joint_state, 10)
            node.create_subscription(
                JointTrajectoryControllerState,
                "/arm_controller/controller_state", self._controller_state, 10)
            node.create_subscription(
                TFMessage,
                "/world/ur5_pick_place/dynamic_pose/info",
                self._gazebo_pose,
                10,
            )
            while self._running and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.25)
        except Exception as error:
            self._error = str(error)

    @staticmethod
    def _named_values(names, values):
        if len(names) != len(values):
            return {}
        by_name = dict(zip(names, values))
        result = {}
        for name in RosMonitor.ARM_JOINTS:
            value = by_name.get(name)
            try:
                number = float(value)
            except (TypeError, ValueError):
                continue
            if math.isfinite(number):
                result[name] = number
        return result

    def _joint_state(self, message):
        self._seen("joint_states")
        velocity = self._named_values(message.name, message.velocity)
        effort = self._named_values(message.name, message.effort)
        with self._telemetry_lock:
            self._joint_velocity = velocity
            self._joint_effort = effort
            self._last_joint_state_monotonic = time.monotonic()

    @staticmethod
    def _frame_leaf(frame_id):
        return str(frame_id).replace("::", "/").strip("/").split("/")[-1]

    def _gazebo_pose(self, message):
        poses = []
        for transform in message.transforms:
            if self._frame_leaf(transform.child_frame_id) != "wrist_3_link":
                continue
            translation = transform.transform.translation
            position = (float(translation.x), float(translation.y), float(translation.z))
            if not all(math.isfinite(value) for value in position):
                continue
            poses.append((
                time.monotonic(),
                position,
            ))
        if poses:
            with self._telemetry_lock:
                self._gazebo_tool_poses.extend(poses)

    def _request_tool_fk(self, names, reference):
        client = self._fk_client
        now = time.monotonic()
        if client is None or not client.service_is_ready():
            with self._telemetry_lock:
                self._position_error = "MoveIt /compute_fk service is unavailable"
            return
        if now - self._last_fk_request_monotonic < 0.1:
            return
        self._last_fk_request_monotonic = now

        from moveit_msgs.srv import GetPositionFK

        request = GetPositionFK.Request()
        request.header.frame_id = "world"
        request.robot_state.joint_state.name = list(names)
        request.robot_state.joint_state.position = list(reference)
        request.fk_link_names = ["tool0"]
        try:
            future = client.call_async(request)
        except Exception as error:
            with self._telemetry_lock:
                self._position_error = f"MoveIt FK request failed: {error}"
            return
        future.add_done_callback(
            lambda completed: self._tool_fk_completed(completed, now))

    def _tool_fk_completed(self, future, reference_received_monotonic):
        try:
            response = future.result()
            if response is None or response.error_code.val != 1 or not response.pose_stamped:
                raise RuntimeError("MoveIt could not calculate tool0 FK")
            pose = response.pose_stamped[0].pose.position
            desired = (float(pose.x), float(pose.y), float(pose.z))
            if not all(math.isfinite(value) for value in desired):
                raise RuntimeError("MoveIt returned a non-finite tool0 position")
        except Exception as error:
            with self._telemetry_lock:
                self._position_error = str(error)
            return

        with self._telemetry_lock:
            if not self._gazebo_tool_poses:
                self._position_error = "Waiting for a Gazebo wrist_3_link pose"
                return
            actual_received, actual = min(
                self._gazebo_tool_poses,
                key=lambda item: abs(item[0] - reference_received_monotonic),
            )
            time_delta = abs(actual_received - reference_received_monotonic)
            now = time.monotonic()
            if now - actual_received > self.position_freshness_s:
                self._position_error = "Gazebo wrist_3_link pose is stale"
                return
            if time_delta > self.position_sync_tolerance_s:
                self._position_error = (
                    "No Gazebo wrist_3_link pose close enough in arrival time "
                    f"({time_delta:.3f}s apart)"
                )
                return
            # Gazebo reduces the fixed tool0/flange links; their URDF origins
            # from wrist_3_link are both zero, so tool0 and wrist_3_link share XYZ.
            error = tuple(actual_value - desired_value
                          for actual_value, desired_value in zip(actual, desired))
            position_error = math.sqrt(sum(value * value for value in error))
            self._position_samples.append({
                "t": now,
                "position_actual_m": list(actual),
                "position_reference_m": list(desired),
                "error_xyz_m": list(error),
                "error_m": position_error,
            })
            self._last_position_monotonic = now
            self._position_error = None

    def _controller_state(self, message):
        names = list(message.joint_names)
        reference = self._named_values(names, message.reference.positions)
        feedback = self._named_values(names, message.feedback.positions)
        velocity = self._named_values(names, message.feedback.velocities)
        if len(reference) != len(self.ARM_JOINTS) or len(feedback) != len(self.ARM_JOINTS):
            with self._telemetry_lock:
                self._telemetry_error = "Controller state is missing one or more UR5 arm joints"
            return
        now = time.monotonic()
        with self._telemetry_lock:
            self._last_controller_monotonic = now
            self._telemetry_error = None
            actual = [feedback[name] for name in self.ARM_JOINTS]
            desired = [reference[name] for name in self.ARM_JOINTS]
            error = [desired_value - actual_value
                     for desired_value, actual_value in zip(desired, actual)]
            joint_state_fresh = (
                self._last_joint_state_monotonic is not None
                and now - self._last_joint_state_monotonic <= self.freshness_s
            )
            current_velocity = [
                velocity.get(name, self._joint_velocity.get(name) if joint_state_fresh else None)
                for name in self.ARM_JOINTS
            ]
            effort = [
                self._joint_effort.get(name) if joint_state_fresh else None
                for name in self.ARM_JOINTS
            ]
            self._samples.append({
                "t": now,
                "position_actual": actual,
                "position_reference": desired,
                "velocity": current_velocity,
                "effort": effort,
                "error": error,
            })
        self._request_tool_fk(names, [reference[name] for name in names])

    def live_data(self):
        with self._telemetry_lock:
            sample = dict(self._samples[-1]) if self._samples else None
            if sample:
                sample["t"] = round(sample["t"], 3)
            last_seen = self._last_controller_monotonic
            error = self._telemetry_error
        age = None if last_seen is None else max(0.0, time.monotonic() - last_seen)
        fresh = age is not None and age <= self.freshness_s
        if error is None and not fresh:
            error = "Waiting for fresh arm controller feedback"
        errors = sample["error"] if sample and fresh else None
        if errors is not None:
            absolute = [abs(value) for value in errors]
            rms = math.sqrt(sum(value * value for value in errors) / len(errors))
            maximum = max(absolute)
        else:
            rms = maximum = None
        with self._telemetry_lock:
            position_sample = (
                dict(self._position_samples[-1]) if self._position_samples else None)
            if position_sample:
                position_sample["t"] = round(position_sample["t"], 3)
            position_samples = list(self._position_samples)
            position_last_seen = self._last_position_monotonic
            position_error = self._position_error
        position_age = (
            None if position_last_seen is None
            else max(0.0, time.monotonic() - position_last_seen)
        )
        position_fresh = (
            position_age is not None and position_age <= self.position_freshness_s)
        position_errors = [
            sample["error_m"] for sample in position_samples
            if position_last_seen is not None
            and time.monotonic() - sample["t"] <= self.POSITION_WINDOW_S
        ]
        if position_errors and position_fresh:
            position_rms = math.sqrt(
                sum(value * value for value in position_errors) / len(position_errors))
            position_maximum = max(position_errors)
        else:
            position_rms = position_maximum = None
        return {
            "joint_names": list(self.ARM_JOINTS),
            "sample": sample if fresh else None,
            "age_s": age,
            "ready": fresh,
            "message": "Live arm feedback" if fresh else error,
            "max_abs_error_rad": maximum,
            "rms_error_rad": rms,
            "end_effector": {
                "sample": position_sample if position_fresh else None,
                "age_s": position_age,
                "ready": position_fresh,
                "message": (
                    "Synchronized MoveIt reference and Gazebo tool0 pose"
                    if position_fresh else
                    position_error or "Waiting for fresh synchronized position data"
                ),
                "rms_error_mm": (
                    None if position_rms is None else position_rms * 1000.0),
                "max_error_mm": (
                    None if position_maximum is None else position_maximum * 1000.0),
            },
        }

    def _seen(self, key):
        with self._lock:
            self._last[key] = time.monotonic()

    def checks(self, launch_alive: bool, rviz_requested: bool):
        now = time.monotonic()
        def topic(name, label):
            with self._lock:
                seen = self._last.get(name)
            age = None if seen is None else now - seen
            ok = seen is not None and age <= self.freshness_s
            return Check(label, ok, "fresh" if ok else "no fresh messages", age)
        result = {
            "clock": topic("clock", "Simulation clock"),
            "joint_states": topic("joint_states", "Joint-state feedback"),
            "launch": Check("Bringup process", launch_alive,
                             "running" if launch_alive else "not running"),
        }
        # Graph checks are filled by the backend's rclpy node when available.
        node_names, service_names, topic_names = set(), set(), set()
        if self._node is not None:
            try:
                node_names = {name for name, _ in self._node.get_node_names_and_namespaces()}
                service_names = {name for name, _ in self._node.get_service_names_and_types()}
                topic_names = {name for name, _ in self._node.get_topic_names_and_types()}
            except Exception:
                pass
        controller_manager = "/controller_manager/list_controllers" in service_names
        arm_seen = controller_manager and any("arm_controller" in name for name in topic_names | service_names)
        gripper_seen = controller_manager and any("robotiq_gripper_controller" in name for name in topic_names | service_names)
        move_action = ("/move_action" in topic_names
                       or any(name.startswith("/move_action/") for name in service_names))
        result.update({
            "arm_controller": Check("Arm controller", arm_seen,
                                     "available" if arm_seen else "not discovered"),
            "gripper_controller": Check("Gripper controller", gripper_seen,
                                         "available" if gripper_seen else "not discovered"),
            "moveit": Check("MoveIt", "move_group" in node_names and move_action,
                             "available" if "move_group" in node_names else "not discovered"),
            "planning_scene": Check("Planning scene", "ur5_scene_publisher" in node_names,
                                     "available" if "ur5_scene_publisher" in node_names else "not discovered"),
            "rviz": Check("RViz", not rviz_requested, "disabled" if not rviz_requested else "waiting"),
        })
        if self._error:
            result["ros_monitor"] = Check("ROS monitor", False, self._error)
        return result
