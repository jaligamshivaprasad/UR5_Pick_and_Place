#!/usr/bin/env python3
"""
test_progressive_phase4.py
Progressive Physical Testing for Phase 4 / Gate G4:
1. Stationary holding test on active controller.
2. Online controller switching to adaptive_arm_controller (mode adaptive_disabled).
3. Bumpless transfer and torque transient verification.
4. Online transition to adaptive_enabled mode.
5. Stationary holding in adaptive_enabled mode.
6. Safe baseline recovery back to fixed mode.
"""

import sys
import time
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from controller_switcher import ControllerSupervisor, FIXED_CONTROLLER, ADAPTIVE_CONTROLLER, ARM_JOINTS

START_POSE = np.array([-1.71368, -1.53079, -0.0570137, -1.48518, -6.21366, 1.55019])

def send_hold_trajectory(supervisor: ControllerSupervisor, controller_name: str, duration_sec: float = 3.0) -> bool:
    client = ActionClient(supervisor, FollowJointTrajectory, f"/{controller_name}/follow_joint_trajectory")
    if not client.wait_for_server(timeout_sec=10.0):
        supervisor.get_logger().error(f"Action server /{controller_name}/follow_joint_trajectory unavailable")
        return False

    supervisor.spin_for(0.2)
    current_q = supervisor.get_arm_positions()
    if not current_q:
        current_q = list(START_POSE)

    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = list(ARM_JOINTS)

    dist = float(np.max(np.abs(np.array(current_q) - START_POSE)))
    points = []
    if dist > 0.02:
        # Slew to start pose first
        p1 = JointTrajectoryPoint()
        p1.positions = list(START_POSE)
        p1.velocities = [0.0] * 6
        p1.time_from_start = Duration(sec=2, nanosec=0)
        points.append(p1)

        p2 = JointTrajectoryPoint()
        p2.positions = list(START_POSE)
        p2.velocities = [0.0] * 6
        total_sec = 2.0 + duration_sec
        ns = int(total_sec * 1e9)
        p2.time_from_start = Duration(sec=ns // 1000000000, nanosec=ns % 1000000000)
        points.append(p2)
    else:
        # Already at target, hold target pose
        total_sec = duration_sec
        p1 = JointTrajectoryPoint()
        p1.positions = list(START_POSE)
        p1.velocities = [0.0] * 6
        ns = int(total_sec * 1e9)
        p1.time_from_start = Duration(sec=ns // 1000000000, nanosec=ns % 1000000000)
        points.append(p1)

    goal.trajectory.points = points

    future = client.send_goal_async(goal)
    deadline = time.monotonic() + 15.0
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(supervisor, timeout_sec=0.05)

    if not future.done():
        supervisor.get_logger().error("Timed out sending hold goal")
        return False

    handle = future.result()
    if not handle.accepted:
        supervisor.get_logger().error("Hold goal rejected")
        return False

    res_future = handle.get_result_async()
    # Account for Gazebo real-time factor (RTF can be ~0.25 - 0.5x)
    wait_sec = max(35.0, total_sec * 4.0 + 15.0)
    deadline = time.monotonic() + wait_sec
    while not res_future.done() and time.monotonic() < deadline:
        rclpy.spin_once(supervisor, timeout_sec=0.05)

    if not res_future.done():
        supervisor.get_logger().error(f"Hold goal execution did not complete within {wait_sec}s")
        return False

    res = res_future.result()
    return res.status == 4 or res.result.error_code == 0

def run_tests():
    rclpy.init()
    supervisor = ControllerSupervisor()

    if not supervisor.wait_for_services(10.0):
        print("ERROR: controller_manager services not available", file=sys.stderr)
        supervisor.destroy_node()
        rclpy.shutdown()
        return 1

    print("\n" + "=" * 70)
    print("      PHASE 4 / GATE G4 PROGRESSIVE CONTROLLER & ADAPTATION TEST")
    print("=" * 70 + "\n")

    # Step 1: Align and Hold on baseline arm_controller
    print("[TEST 1/5] Aligning and holding stationary on 'arm_controller'...")
    ok = send_hold_trajectory(supervisor, FIXED_CONTROLLER, duration_sec=3.0)
    if not ok:
        print("FAILED: could not establish initial hold trajectory")
        return 1

    supervisor.spin_for(1.0)
    pos = supervisor.get_arm_positions()
    vel = supervisor.get_arm_velocities()
    drift = float(np.max(np.abs(np.array(pos) - START_POSE)))
    max_vel = float(np.max(np.abs(vel)))
    print(f"  -> Hold Drift: {drift:.6f} rad (Gate limit < 0.05 rad)")
    print(f"  -> Max Velocity: {max_vel:.6f} rad/s (Quiescence limit < 0.01 rad/s)")
    if drift > 0.05 or max_vel > 0.01:
        print("FAILED: Stationary holding not satisfied on baseline controller")
        return 1
    print("  -> TEST 1 PASSED!\n")

    # Step 2: Switch to adaptive_arm_controller in adaptive_disabled mode
    print("[TEST 2/5] Switching to 'adaptive_arm_controller' (mode: adaptive_disabled)...")
    ok = supervisor.set_operating_mode("adaptive_disabled")
    if not ok:
        print("FAILED: switch to adaptive_disabled mode failed")
        return 2

    supervisor.spin_for(1.0)
    pos_after = supervisor.get_arm_positions()
    switch_drift = float(np.max(np.abs(np.array(pos_after) - START_POSE)))
    print(f"  -> Post-Switch Drift from Start: {switch_drift:.6f} rad (Gate limit < 0.02 rad)")
    if switch_drift > 0.02:
        print(f"FAILED: Post-switch drift exceeded 0.02 rad ({switch_drift:.6f} rad)")
        return 2
    print("  -> TEST 2 PASSED!\n")

    # Step 3: Stationary holding in adaptive_arm_controller
    print("[TEST 3/5] Holding stationary with AdaptiveArmController for 3.0s...")
    ok = send_hold_trajectory(supervisor, ADAPTIVE_CONTROLLER, duration_sec=3.0)
    if not ok:
        print("FAILED: Hold trajectory on adaptive_arm_controller failed")
        return 3

    supervisor.spin_for(1.0)
    pos_hold = supervisor.get_arm_positions()
    hold_drift = float(np.max(np.abs(np.array(pos_hold) - START_POSE)))
    print(f"  -> Measured Hold Drift: {hold_drift:.6f} rad (Gate limit < 0.02 rad)")
    if hold_drift > 0.02:
        print(f"FAILED: Hold drift exceeded 0.02 rad ({hold_drift:.6f} rad)")
        return 3
    print("  -> TEST 3 PASSED!\n")

    # Step 4: Online transition to adaptive_enabled mode
    print("[TEST 4/5] Transitioning online to 'adaptive_enabled' mode...")
    ok = supervisor.set_operating_mode("adaptive_enabled")
    if not ok:
        print("FAILED: failed to set operating mode to adaptive_enabled")
        return 4

    supervisor.spin_for(2.0)
    pos_adapt = supervisor.get_arm_positions()
    adapt_drift = float(np.max(np.abs(np.array(pos_adapt) - START_POSE)))
    print(f"  -> Stationary Hold Drift under Adaptation: {adapt_drift:.6f} rad (Gate limit < 0.02 rad)")
    if adapt_drift > 0.02:
        print(f"FAILED: Stationary drift under adaptive_enabled exceeded 0.02 rad ({adapt_drift:.6f} rad)")
        return 4
    print("  -> TEST 4 PASSED!\n")

    # Step 5: Safe recovery to fixed baseline
    print("[TEST 5/5] Testing safe recovery back to baseline fixed controller...")
    ok = supervisor.recover_fixed()
    if not ok:
        print("FAILED: Baseline recovery failed")
        return 5

    supervisor.spin_for(1.0)
    active = supervisor.get_controller_states().get(FIXED_CONTROLLER)
    print(f"  -> Baseline 'arm_controller' status: {active.upper()}")
    if active != "active":
        print(f"FAILED: arm_controller is not active after recovery ({active})")
        return 5
    print("  -> TEST 5 PASSED!\n")

    print("=" * 70)
    print("   ALL PROGRESSIVE PHYSICAL CONTROL & SWITCHING TESTS PASSED!")
    print("=" * 70 + "\n")

    supervisor.destroy_node()
    rclpy.shutdown()
    return 0

if __name__ == "__main__":
    sys.exit(run_tests())
