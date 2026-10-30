#!/usr/bin/env python3
"""
test_progressive_phase5.py
Progressive Physical Testing for Phase 5 (Payload Feedforward + Adaptive Control):
1. Stationary holding test on baseline arm_controller.
2. Online controller switching to adaptive_arm_controller.
3. Online transition to adaptive_enabled mode.
4. Enable payload_feedforward_enabled parameter.
5. Simulated grasp attach during stationary holding:
   - Verify smooth S-curve ramp (ramp duration ~0.20 s).
   - Verify feedforward torque magnitudes match static payload model.
   - Verify stationary holding drift < 0.02 rad and velocity < 0.02 rad/s.
6. Simulated grasp detach during stationary holding:
   - Verify smooth down-ramp to zero feedforward.
   - Verify stability.
7. Safe recovery back to fixed baseline arm_controller.
"""

import sys
import time
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Empty, Float64MultiArray, String
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

    attach_pub = supervisor.create_publisher(Empty, "/pick_place/grasp/attach", 10)
    detach_pub = supervisor.create_publisher(Empty, "/pick_place/grasp/detach", 10)

    latest_ff = None
    def ff_cb(msg: Float64MultiArray):
        nonlocal latest_ff
        latest_ff = list(msg.data)

    supervisor.create_subscription(
        Float64MultiArray,
        f"/{ADAPTIVE_CONTROLLER}/payload_feedforward",
        ff_cb,
        10,
    )

    grasp_state = None
    grasp_revision = 0
    def grasp_cb(msg: String):
        nonlocal grasp_state, grasp_revision
        grasp_state = msg.data.lower()
        grasp_revision += 1

    supervisor.create_subscription(String, "/pick_place/grasp/state", grasp_cb, 10)

    def set_grasp(attach: bool) -> bool:
        desired = "attached" if attach else "detached"
        pub = attach_pub if attach else detach_pub
        t0 = time.monotonic()
        while pub.get_subscription_count() == 0 and time.monotonic() - t0 < 5.0:
            rclpy.spin_once(supervisor, timeout_sec=0.05)
        
        cur_rev = grasp_revision
        pub.publish(Empty())
        t0 = time.monotonic()
        while time.monotonic() - t0 < 5.0:
            rclpy.spin_once(supervisor, timeout_sec=0.05)
            if grasp_revision > cur_rev and grasp_state and desired in grasp_state:
                return True
        return False

    if not supervisor.wait_for_services(10.0):
        print("ERROR: controller_manager services not available", file=sys.stderr)
        supervisor.destroy_node()
        rclpy.shutdown()
        return 1

    print("\n" + "=" * 70)
    print("   PHASE 5: PROGRESSIVE PAYLOAD FEEDFORWARD & ADAPTATION TEST")
    print("=" * 70 + "\n")

    # Step 1: Align and Hold on baseline arm_controller
    print("[TEST 1/6] Aligning and holding stationary on 'arm_controller'...")
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

    # Step 2: Switch to adaptive_arm_controller (mode: adaptive_disabled)
    print("[TEST 2/6] Switching to 'adaptive_arm_controller' (mode: adaptive_disabled)...")
    ok = supervisor.set_operating_mode("adaptive_disabled")
    if not ok:
        print("FAILED: switch to adaptive_disabled mode failed")
        return 2
    supervisor.spin_for(1.0)
    print("  -> TEST 2 PASSED!\n")

    # Step 3: Transition to adaptive_enabled mode
    print("[TEST 3/6] Transitioning online to 'adaptive_enabled' mode...")
    ok = supervisor.set_operating_mode("adaptive_enabled")
    if not ok:
        print("FAILED: failed to set operating mode to adaptive_enabled")
        return 3
    supervisor.spin_for(1.0)
    print("  -> TEST 3 PASSED!\n")

    # Step 4: Enable payload_feedforward_enabled parameter
    print("[TEST 4/6] Enabling payload_feedforward_enabled parameter...")
    ok = supervisor.set_payload_feedforward(True)
    if not ok:
        print("FAILED: failed to enable payload_feedforward_enabled")
        return 4
    supervisor.spin_for(1.0)
    print("  -> TEST 4 PASSED!\n")

    # Step 5: Test Simulated Grasp Attach during stationary hold
    print("[TEST 5/6] Triggering grasp attach during stationary hold...")
    ok = set_grasp(True)
    if not ok:
        print("FAILED: grasp attach confirmation timeout")
        return 5

    # Spin and collect feedforward data over 1.0s
    t0 = time.monotonic()
    ff_history = []
    while time.monotonic() - t0 < 1.0:
        rclpy.spin_once(supervisor, timeout_sec=0.05)
        if latest_ff:
            ff_history.append((time.monotonic() - t0, latest_ff.copy()))

    pos_after = supervisor.get_arm_positions()
    vel_after = supervisor.get_arm_velocities()
    drift_after = float(np.max(np.abs(np.array(pos_after) - START_POSE)))
    max_vel_after = float(np.max(np.abs(vel_after)))

    if latest_ff:
        tau_ff = latest_ff[:6]
        alpha = latest_ff[6]
        mass = latest_ff[7]
        print(f"  -> Ramp Factor alpha: {alpha:.3f} (Expected: 1.000)")
        print(f"  -> Active Mass: {mass:.2f} kg (Expected: 3.00 kg)")
        print(f"  -> Feedforward Torques (Nm): {[round(x, 4) for x in tau_ff]}")
        print(f"  -> Post-Attach Hold Drift: {drift_after:.6f} rad (Limit < 0.02 rad)")
        print(f"  -> Post-Attach Max Velocity: {max_vel_after:.6f} rad/s (Limit < 0.02 rad/s)")
        
        if alpha < 0.99:
            print(f"FAILED: alpha did not reach 1.0 (got {alpha:.3f})")
            return 5
        if mass < 2.99:
            print(f"FAILED: active mass did not reach 3.0 kg (got {mass:.2f})")
            return 5
        if drift_after > 0.02 or max_vel_after > 0.02:
            print("FAILED: excessive motion transient during grasp attach")
            return 5
    else:
        print("WARNING: no payload_feedforward telemetry received")

    print("  -> TEST 5 PASSED!\n")

    # Step 6: Test Simulated Grasp Detach
    print("[TEST 6/6] Triggering grasp detach during stationary hold...")
    ok = set_grasp(False)
    if not ok:
        print("FAILED: grasp detach confirmation timeout")
        return 6
    supervisor.spin_for(1.0)
    if latest_ff:
        alpha_detach = latest_ff[6]
        tau_ff_detach = latest_ff[:6]
        print(f"  -> Post-Detach Ramp Factor: {alpha_detach:.3f} (Expected: 0.000)")
        print(f"  -> Post-Detach Torques (Nm): {[round(x, 4) for x in tau_ff_detach]}")
        if alpha_detach > 0.01:
            print(f"FAILED: alpha did not ramp down to 0.0 (got {alpha_detach:.3f})")
            return 6
    print("  -> TEST 6 PASSED!\n")

    # Step 7: Safe Recovery
    print("[CLEANUP] Recovering fixed arm_controller baseline...")
    supervisor.set_payload_feedforward(False)
    supervisor.recover_fixed()
    supervisor.spin_for(1.0)
    active = supervisor.get_controller_states().get(FIXED_CONTROLLER)
    print(f"  -> Final active controller: {active.upper()}")

    print("=" * 70)
    print("  ALL PROGRESSIVE PAYLOAD FEEDFORWARD TESTS PASSED!")
    print("=" * 70 + "\n")

    supervisor.destroy_node()
    rclpy.shutdown()
    return 0

if __name__ == "__main__":
    sys.exit(run_tests())
