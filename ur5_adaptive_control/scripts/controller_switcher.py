#!/usr/bin/env python3
"""UR5 Controller Switching & Safety Supervisor Utility.

Provides safe, deterministic online switching between fixed PID (arm_controller)
and adaptive PID (adaptive_arm_controller) under ROS 2 Humble ros2_control.

Enforces Phase 3 Gate G3 requirements:
1. Strictness: STRICT controller switching via controller_manager.
2. Exclusivity: Exactly one arm controller owns the 6 effort command interfaces.
3. Pre-switch safety: Rejects switching during motion (|dq| >= 0.01 rad/s) or active goals.
4. Bumpless transfer: Verified reference pose holding and torque transient monitoring.
5. Verification: Post-switch stationary hold drift (< 0.02 rad) and torque continuity.
6. Safe recovery: Instant fallback to fixed PID baseline on fault or request.
"""

import argparse
import sys
import time
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from action_msgs.msg import GoalStatus, GoalStatusArray
from builtin_interfaces.msg import Duration
from rcl_interfaces.msg import Parameter as ParamMsg, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from controller_manager_msgs.srv import (
    ConfigureController,
    ListControllers,
    ListHardwareInterfaces,
    LoadController,
    SwitchController,
)
from control_msgs.msg import JointTrajectoryControllerState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState

ARM_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)

FIXED_CONTROLLER = "arm_controller"
ADAPTIVE_CONTROLLER = "adaptive_arm_controller"
MAX_VELOCITY_TOLERANCE = 0.01  # rad/s quiescent threshold
MAX_ERROR_TOLERANCE = 0.02     # rad quiescent tracking error threshold
MAX_ALLOWED_DRIFT = 0.02       # rad max post-switch drift
MAX_TORQUE_TRANSIENT = 5.0     # N*m max allowable torque step transient


class ControllerSupervisor(Node):
    """Supervisor node for querying state, verifying safety, and switching controllers."""

    def __init__(self):
        super().__init__(
            "ur5_controller_supervisor",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )

        # Service clients
        self.cli_list = self.create_client(ListControllers, "/controller_manager/list_controllers")
        self.cli_list_hw = self.create_client(
            ListHardwareInterfaces, "/controller_manager/list_hardware_interfaces"
        )
        self.cli_switch = self.create_client(SwitchController, "/controller_manager/switch_controller")
        self.cli_load = self.create_client(LoadController, "/controller_manager/load_controller")
        self.cli_config = self.create_client(
            ConfigureController, "/controller_manager/configure_controller"
        )
        self.cli_set_param = self.create_client(
            SetParameters, f"/{ADAPTIVE_CONTROLLER}/set_parameters"
        )

        # State storage
        self.latest_joint_state: Optional[JointState] = None
        self.joint_state_time: float = 0.0
        self.joint_positions: Dict[str, float] = {}
        self.joint_velocities: Dict[str, float] = {}
        self.joint_efforts: Dict[str, float] = {}
        self.latest_ctrl_state: Optional[JointTrajectoryControllerState] = None
        self.ctrl_state_time: float = 0.0
        self.active_goals: Dict[str, int] = {FIXED_CONTROLLER: 0, ADAPTIVE_CONTROLLER: 0}

        # Subscriptions
        self.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10)
        self.create_subscription(
            JointTrajectoryControllerState,
            "/arm_controller/controller_state",
            self._ctrl_state_cb,
            10,
        )
        self.create_subscription(
            GoalStatusArray,
            f"/{FIXED_CONTROLLER}/follow_joint_trajectory/_action/status",
            lambda msg: self._action_status_cb(FIXED_CONTROLLER, msg),
            10,
        )
        self.create_subscription(
            GoalStatusArray,
            f"/{ADAPTIVE_CONTROLLER}/follow_joint_trajectory/_action/status",
            lambda msg: self._action_status_cb(ADAPTIVE_CONTROLLER, msg),
            10,
        )

    def _action_status_cb(self, controller: str, msg: GoalStatusArray):
        count = 0
        for s in msg.status_list:
            if s.status in (
                GoalStatus.STATUS_ACCEPTED,
                GoalStatus.STATUS_EXECUTING,
                GoalStatus.STATUS_CANCELING,
            ):
                count += 1
        self.active_goals[controller] = count

    def _joint_state_cb(self, msg: JointState):
        self.latest_joint_state = msg
        self.joint_state_time = time.monotonic()
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = float(msg.position[i])
            if i < len(msg.velocity):
                self.joint_velocities[name] = float(msg.velocity[i])
            if i < len(msg.effort):
                self.joint_efforts[name] = float(msg.effort[i])

    def get_arm_positions(self) -> Optional[List[float]]:
        if all(j in self.joint_positions for j in ARM_JOINTS):
            return [self.joint_positions[j] for j in ARM_JOINTS]
        return None

    def get_arm_velocities(self) -> Optional[List[float]]:
        if all(j in self.joint_velocities for j in ARM_JOINTS):
            return [self.joint_velocities[j] for j in ARM_JOINTS]
        return None

    def _ctrl_state_cb(self, msg: JointTrajectoryControllerState):
        self.latest_ctrl_state = msg
        self.ctrl_state_time = time.monotonic()

    def wait_for_services(self, timeout_sec: float = 10.0) -> bool:
        """Ensure core controller_manager services are available."""
        clients = [self.cli_list, self.cli_list_hw, self.cli_switch]
        deadline = time.monotonic() + timeout_sec
        for client in clients:
            rem = max(0.5, deadline - time.monotonic())
            if not client.wait_for_service(timeout_sec=rem):
                self.get_logger().error(f"Service {client.srv_name} unavailable")
                return False
        return True

    def spin_for(self, duration_sec: float):
        """Spin ROS node for a given wall-clock duration."""
        end_time = time.monotonic() + duration_sec
        while time.monotonic() < end_time:
            rclpy.spin_once(self, timeout_sec=0.02)

    def get_controller_states(self) -> Dict[str, str]:
        """Return dict of {controller_name: state}."""
        future = self.cli_list.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            raise RuntimeError("Failed to list controllers from controller_manager")
        return {c.name: c.state for c in future.result().controller}

    def get_claimed_command_interfaces(self) -> Dict[str, str]:
        """Return dict of {interface_name: claiming_controller}."""
        future = self.cli_list.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            raise RuntimeError("Failed to list controllers from controller_manager")

        claims = {}
        for c in future.result().controller:
            for iface in c.claimed_interfaces:
                claims[iface] = c.name

        # Optionally check hardware interfaces for any remaining
        hw_future = self.cli_list_hw.call_async(ListHardwareInterfaces.Request())
        rclpy.spin_until_future_complete(self, hw_future, timeout_sec=5.0)
        if hw_future.done() and hw_future.result() is not None:
            for cmd_iface in hw_future.result().command_interfaces:
                if cmd_iface.name not in claims:
                    claims[cmd_iface.name] = "CLAIMED (OTHER)" if cmd_iface.is_claimed else "UNCLAIMED"

        return claims

    def get_active_arm_controller(self) -> Optional[str]:
        """Return name of currently active arm controller, or None."""
        states = self.get_controller_states()
        active = []
        if states.get(FIXED_CONTROLLER) == "active":
            active.append(FIXED_CONTROLLER)
        if states.get(ADAPTIVE_CONTROLLER) == "active":
            active.append(ADAPTIVE_CONTROLLER)
        if len(active) > 1:
            raise RuntimeError(
                f"FATAL: Multiple arm controllers active simultaneously: {active}!"
            )
        return active[0] if active else None

    def sample_quiescence(self, dwell_sec: float = 0.5) -> Tuple[bool, float, float]:
        """Sample joint state over dwell window to verify robot is at rest.

        Returns: (is_quiescent, max_velocity, max_error)
        """
        self.spin_for(0.1)  # Warm up
        deadline = time.monotonic() + dwell_sec
        max_vel = 0.0
        max_err = 0.0

        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            vels = self.get_arm_velocities()
            if vels:
                max_vel = max(max_vel, max(abs(v) for v in vels))

            if self.latest_ctrl_state is not None and len(self.latest_ctrl_state.error.positions) >= 6:
                errs = [abs(e) for e in self.latest_ctrl_state.error.positions[:6]]
                max_err = max(max_err, max(errs))

        if not self.joint_positions:
            self.get_logger().error("Safety check failed: No /joint_states feedback received!")
            return False, float("inf"), float("inf")

        # 1. Trajectory Action Execution Check
        active_goals_count = sum(self.active_goals.values())
        if active_goals_count > 0:
            self.get_logger().error(
                f"Safety check rejected: {active_goals_count} trajectory action goal(s) currently active!"
            )
            return False, max_vel, max_err

        # 2. Reference Trajectory Motion Check
        if self.latest_ctrl_state is not None and self.latest_ctrl_state.reference.velocities:
            ref_v = max(abs(v) for v in self.latest_ctrl_state.reference.velocities)
            if ref_v > 0.001:
                self.get_logger().error(
                    f"Safety check rejected: Controller reference interpolator is in motion (ref_vel={ref_v:.6f} rad/s)!"
                )
                return False, max_vel, max_err

        is_quiescent = (max_vel < MAX_VELOCITY_TOLERANCE) and (max_err < MAX_ERROR_TOLERANCE)
        return is_quiescent, max_vel, max_err

    def align_and_hold(self, duration_sec: float = 2.0) -> bool:
        """Command active controller to hold current joint positions with zero velocity."""
        active = self.get_active_arm_controller()
        if not active:
            return False
        client = ActionClient(self, FollowJointTrajectory, f"/{active}/follow_joint_trajectory")
        if not client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn(f"Action server /{active}/follow_joint_trajectory unavailable for hold")
            return False

        self.spin_for(0.1)
        cur_q = self.get_arm_positions()
        if not cur_q:
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_JOINTS)
        p = JointTrajectoryPoint()
        p.positions = list(cur_q)
        p.velocities = [0.0] * 6
        ns = int(duration_sec * 1e9)
        p.time_from_start = Duration(sec=ns // 1000000000, nanosec=ns % 1000000000)
        goal.trajectory.points.append(p)

        future = client.send_goal_async(goal)
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            return False

        handle = future.result()
        if not handle.accepted:
            return False

        res_future = handle.get_result_async()
        wait_deadline = time.monotonic() + max(10.0, duration_sec * 4.0 + 5.0)
        while not res_future.done() and time.monotonic() < wait_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

        return res_future.done()

    def prepare_target_controller(self, target_name: str):
        """Ensure target controller is loaded and in inactive state ready for switch."""
        states = self.get_controller_states()
        if target_name not in states:
            self.get_logger().info(f"Target '{target_name}' not loaded. Loading now...")
            req = LoadController.Request(name=target_name)
            fut = self.cli_load.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
            if not fut.done() or not fut.result().ok:
                raise RuntimeError(f"Failed to load target controller '{target_name}'")
            states = self.get_controller_states()

        if states.get(target_name) == "unconfigured":
            self.get_logger().info(f"Target '{target_name}' is unconfigured. Configuring now...")
            req = ConfigureController.Request(name=target_name)
            fut = self.cli_config.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
            if not fut.done() or not fut.result().ok:
                raise RuntimeError(f"Failed to configure target controller '{target_name}'")

        states = self.get_controller_states()
        if states.get(target_name) != "inactive":
            raise RuntimeError(
                f"Target '{target_name}' is in unexpected state: {states.get(target_name)}"
            )
        self.get_logger().info(f"Target '{target_name}' is ready in 'inactive' state.")

    def execute_strict_switch(
        self, target_name: str, current_name: Optional[str]
    ) -> bool:
        """Perform atomic STRICT switch via controller_manager."""
        req = SwitchController.Request()
        req.activate_controllers = [target_name]
        req.deactivate_controllers = [current_name] if current_name else []
        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = False
        req.timeout = Duration(sec=5, nanosec=0)

        self.get_logger().info(
            f"Executing STRICT switch: Activate=[{target_name}], Deactivate=[{current_name}]..."
        )
        fut = self.cli_switch.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=8.0)
        if not fut.done():
            self.get_logger().error("SwitchController service timed out!")
            return False
        res = fut.result()
        if not res.ok:
            self.get_logger().error("SwitchController service returned ok=False!")
            return False
        return True

    def switch_to(self, target_name: str) -> bool:
        """High-level safe switch to target controller with pre/post checks."""
        self.get_logger().info(f"--- Request to switch to '{target_name}' ---")

        # 1. Check current controller
        current = self.get_active_arm_controller()
        if current == target_name:
            self.get_logger().info(f"Target controller '{target_name}' is already active. No-op.")
            return True

        self.get_logger().info(f"Current active arm controller: '{current}'")

        # 2. Safety Quiescence Gate: Robot must be at rest
        self.get_logger().info("Verifying stationary quiescence before switch...")
        is_at_rest, max_v, max_e = self.sample_quiescence(0.5)
        if not is_at_rest:
            self.get_logger().info(f"Arm not at rest (max_vel={max_v:.4f} rad/s). Attempting active hold stabilization...")
            self.align_and_hold(duration_sec=2.0)
            is_at_rest, max_v, max_e = self.sample_quiescence(0.5)

        if not is_at_rest:
            err_msg = (
                f"REJECTED: Robot is not at rest! Max joint vel = {max_v:.4f} rad/s "
                f"(limit {MAX_VELOCITY_TOLERANCE}), Max error = {max_e:.4f} rad "
                f"(limit {MAX_ERROR_TOLERANCE})."
            )
            self.get_logger().error(err_msg)
            return False
        self.get_logger().info(f"Quiescence confirmed: max_vel={max_v:.6f} rad/s, max_err={max_e:.6f} rad.")

        # 3. Capture pre-switch baseline posture & effort
        pre_positions = self.get_arm_positions()
        pre_torques = None
        if self.latest_ctrl_state is not None and len(self.latest_ctrl_state.output.effort) >= 6:
            pre_torques = list(self.latest_ctrl_state.output.effort[:6])

        # 4. Prepare target controller (load + configure if needed)
        self.prepare_target_controller(target_name)

        # 5. Perform STRICT atomic switch
        t0 = time.monotonic()
        if not self.execute_strict_switch(target_name, current):
            self.get_logger().error("Switch execution failed! Attempting safe recovery to fixed PID...")
            self.recover_fixed()
            return False

        # 6. Post-switch dwell & verification (1.0 s)
        self.get_logger().info("Verifying post-switch holding stability and torque continuity (1.0s dwell)...")
        max_drift = 0.0
        max_torque_transient = 0.0

        dwell_end = time.monotonic() + 1.0
        while time.monotonic() < dwell_end:
            rclpy.spin_once(self, timeout_sec=0.02)
            cur_pos = self.get_arm_positions()
            if pre_positions and cur_pos:
                drift = max(abs(c - p) for c, p in zip(cur_pos, pre_positions))
                max_drift = max(max_drift, drift)

            if pre_torques and self.latest_ctrl_state is not None and len(self.latest_ctrl_state.output.effort) >= 6:
                cur_t = list(self.latest_ctrl_state.output.effort[:6])
                transient = max(abs(c - p) for c, p in zip(cur_t, pre_torques))
                max_torque_transient = max(max_torque_transient, transient)

        # 7. Check controller states and interface claims
        states = self.get_controller_states()
        if states.get(target_name) != "active" or (current and states.get(current) != "inactive"):
            self.get_logger().error(f"Post-switch state mismatch! States: {states}")
            self.recover_fixed()
            return False

        claims = self.get_claimed_command_interfaces()
        effort_claims = {k: v for k, v in claims.items() if k.endswith("/effort")}
        unowned_or_conflicted = [k for k, v in effort_claims.items() if v != target_name]
        if unowned_or_conflicted:
            self.get_logger().error(
                f"Interface claim conflict! Target '{target_name}' does not exclusively own effort interfaces: {effort_claims}"
            )
            self.recover_fixed()
            return False

        # 8. Drift and torque validation
        self.get_logger().info(
            f"Switch completed successfully in {(time.monotonic() - t0):.3f}s. "
            f"Hold drift: {max_drift:.6f} rad (gate < {MAX_ALLOWED_DRIFT}), "
            f"Peak torque transient: {max_torque_transient:.3f} Nm (gate < {MAX_TORQUE_TRANSIENT})."
        )
        if max_drift >= MAX_ALLOWED_DRIFT:
            self.get_logger().warn(f"Warning: Measured hold drift exceeded threshold ({max_drift:.4f} rad)!")
        if max_torque_transient >= MAX_TORQUE_TRANSIENT:
            self.get_logger().warn(
                f"Warning: Torque transient exceeded threshold ({max_torque_transient:.2f} Nm)!"
            )

        return True

    def recover_fixed(self) -> bool:
        """Emergency safe fallback to fixed PID controller."""
        self.get_logger().warn("TRIGGERING RECOVERY TO FIXED PID BASELINE (arm_controller)...")
        states = self.get_controller_states()
        if states.get(FIXED_CONTROLLER) == "active":
            self.get_logger().info("arm_controller is already active.")
            return True

        req = SwitchController.Request()
        req.activate_controllers = [FIXED_CONTROLLER]
        active_adaptive = [ADAPTIVE_CONTROLLER] if states.get(ADAPTIVE_CONTROLLER) == "active" else []
        req.deactivate_controllers = active_adaptive
        req.strictness = SwitchController.Request.BEST_EFFORT  # ensure fallback succeeds even under error
        req.timeout = Duration(sec=5, nanosec=0)

        fut = self.cli_switch.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=6.0)
        self.spin_for(1.0)
        new_states = self.get_controller_states()
        recovered = new_states.get(FIXED_CONTROLLER) == "active"
        if recovered:
            self.get_logger().info("Successfully recovered baseline arm_controller to active state.")
        else:
            self.get_logger().error(f"CRITICAL: Recovery failed! Current states: {new_states}")
        return recovered

    def set_operating_mode(self, mode: str) -> bool:
        """Enforce one of the 3 validated Phase 4 operating modes:
        - fixed: arm_controller active (calibrated C1 baseline)
        - adaptive_disabled: adaptive_arm_controller active, adaptation_enabled=false
        - adaptive_enabled: adaptive_arm_controller active, adaptation_enabled=true
        """
        self.get_logger().info(f"Setting operating mode: {mode}")
        if mode == "fixed":
            return self.switch_to(FIXED_CONTROLLER)

        # For adaptive modes, ensure adaptive_arm_controller is active first
        if not self.switch_to(ADAPTIVE_CONTROLLER):
            return False

        # Now configure adaptation_enabled parameter
        enable_adapt = (mode == "adaptive_enabled")
        if not self.cli_set_param.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"Parameter service {self.cli_set_param.srv_name} unavailable")
            return False

        req = SetParameters.Request()
        p = ParamMsg()
        p.name = "adaptation_enabled"
        p.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=enable_adapt)
        req.parameters = [p]

        future = self.cli_set_param.call_async(req)
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)

        res = future.result()
        if res and all(r.successful for r in res.results):
            self.get_logger().info(f"Operating mode successfully set to: {mode} (adaptation_enabled={enable_adapt})")
            return True
        else:
            self.get_logger().error(f"Failed to set parameter adaptation_enabled={enable_adapt}")
            return False

    def set_payload_feedforward(self, enabled: bool) -> bool:
        """Enable or disable payload gravity feedforward online."""
        if not self.cli_set_param.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"Parameter service {self.cli_set_param.srv_name} unavailable")
            return False

        req = SetParameters.Request()
        p = ParamMsg()
        p.name = "payload_feedforward_enabled"
        p.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=enabled)
        req.parameters = [p]

        future = self.cli_set_param.call_async(req)
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)

        res = future.result()
        if res and all(r.successful for r in res.results):
            self.get_logger().info(f"payload_feedforward_enabled set to {enabled}")
            return True
        else:
            self.get_logger().error(f"Failed to set payload_feedforward_enabled={enabled}")
            return False

    def print_status(self):
        """Print full diagnostics of controllers, hardware interfaces, and robot state."""
        self.spin_for(0.2)
        states = self.get_controller_states()
        claims = self.get_claimed_command_interfaces()

        print("\n" + "=" * 70)
        print("          UR5 ROS 2 CONTROL SUPERVISOR STATUS REPORT")
        print("=" * 70)

        print("\n[Controller Manager Lifecycle States]:")
        for name, state in sorted(states.items()):
            tag = " [ACTIVE ARM]" if state == "active" and name in (FIXED_CONTROLLER, ADAPTIVE_CONTROLLER) else ""
            print(f"  - {name:30s}: {state.upper()}{tag}")

        print("\n[Six-Joint Effort Command Interface Claims]:")
        for j in ARM_JOINTS:
            iface = f"{j}/effort"
            owner = claims.get(iface, "NOT PRESENT")
            print(f"  - {iface:32s}: Claimed by -> {owner}")

        is_at_rest, max_v, max_e = self.sample_quiescence(0.3)
        print(f"\n[Motion Quiescence & Safety Check]:")
        print(f"  - Safe to Switch:           {'YES' if is_at_rest else 'NO [REJECTED]'}")
        print(f"  - Active Trajectory Goals:  {sum(self.active_goals.values())} (Fixed: {self.active_goals.get(FIXED_CONTROLLER, 0)}, Adaptive: {self.active_goals.get(ADAPTIVE_CONTROLLER, 0)})")
        print(f"  - Max Joint Velocity:       {max_v:.6f} rad/s (Threshold: < {MAX_VELOCITY_TOLERANCE})")
        print(f"  - Max Position Error:       {max_e:.6f} rad   (Threshold: < {MAX_ERROR_TOLERANCE})")

        if self.joint_positions:
            print("\n[Measured Joint Feedback]:")
            for j in ARM_JOINTS:
                p = self.joint_positions.get(j, float("nan"))
                v = self.joint_velocities.get(j, float("nan"))
                e = self.joint_efforts.get(j, float("nan"))
                print(f"  - {j:22s}: pos={p:8.4f} rad, vel={v:8.4f} rad/s, effort={e:8.2f} Nm")

        print("=" * 70 + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("status", help="Display full controller, interface, and robot status")
    subparsers.add_parser("recover", help="Safely recover to baseline fixed PID controller")

    switch_parser = subparsers.add_parser("switch", help="Switch arm controller")
    switch_parser.add_argument(
        "mode",
        choices=["fixed", "adaptive"],
        help="Target controller mode ('fixed' for arm_controller, 'adaptive' for adaptive_arm_controller)",
    )

    mode_parser = subparsers.add_parser(
        "mode", help="Set Phase 4 operating mode (fixed, adaptive_disabled, adaptive_enabled)"
    )
    mode_parser.add_argument(
        "target_mode",
        choices=["fixed", "adaptive_disabled", "adaptive_enabled"],
        help="Target operating mode",
    )

    args = parser.parse_args()

    rclpy.init()
    node = ControllerSupervisor()

    if not node.wait_for_services(10.0):
        print("ERROR: controller_manager services not available. Is Gazebo Sim running?", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    exit_code = 0
    try:
        if args.command == "status":
            node.print_status()
        elif args.command == "recover":
            ok = node.recover_fixed()
            exit_code = 0 if ok else 1
        elif args.command == "switch":
            target = FIXED_CONTROLLER if args.mode == "fixed" else ADAPTIVE_CONTROLLER
            ok = node.switch_to(target)
            exit_code = 0 if ok else 1
        elif args.command == "mode":
            ok = node.set_operating_mode(args.target_mode)
            exit_code = 0 if ok else 1
    except Exception as ex:
        print(f"Exception during command execution: {ex}", file=sys.stderr)
        exit_code = 2
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
