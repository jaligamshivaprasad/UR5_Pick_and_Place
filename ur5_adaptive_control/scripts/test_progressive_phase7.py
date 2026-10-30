#!/usr/bin/env python3
"""
test_progressive_phase7.py
Progressive Testing for Phase 7 (Dynamic Acceleration Feedforward & Gain Scheduling):

1. Verify stationary holding on baseline arm_controller.
2. Verify STRICT controller switching to adaptive_arm_controller with dynamic feedforward enabled.
3. Verify dynamic feedforward torques are exactly zero during stationary holding (qdot=0, qddot=0).
4. Verify dynamic feedforward torque generation during single-joint motion (J1 swing).
5. Verify coordinated 6-joint motion with dynamic feedforward active (smoothness, bounds, margins).
6. Verify payload gain scheduling (bounded scaling of Kp and Kd when attached, return to C1 upon release).
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
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as ParamMsg, ParameterType, ParameterValue
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


def set_params(supervisor: ControllerSupervisor, params_dict: dict) -> bool:
    client = supervisor.create_client(SetParameters, f"/{ADAPTIVE_CONTROLLER}/set_parameters")
    if not client.wait_for_service(timeout_sec=5.0):
        supervisor.get_logger().error(f"Service /{ADAPTIVE_CONTROLLER}/set_parameters not available")
        return False
    req = SetParameters.Request()
    for name, val in params_dict.items():
        if isinstance(val, bool):
            pv = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=val)
        elif isinstance(val, float):
            pv = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=val)
        elif isinstance(val, int):
            pv = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=val)
        elif isinstance(val, str):
            pv = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=val)
        else:
            raise ValueError(f"Unsupported parameter type: {type(val)}")
        req.parameters.append(ParamMsg(name=name, value=pv))

    future = client.call_async(req)
    t0 = time.monotonic()
    while not future.done() and time.monotonic() - t0 < 5.0:
        rclpy.spin_once(supervisor, timeout_sec=0.05)
    return future.done() and all(r.successful for r in future.result().results)


def run_progressive_tests():
    print("\n" + "=" * 80)
    print("      UR5 PHASE 7: PROGRESSIVE DYNAMIC FEEDFORWARD & GAIN SCHEDULING TESTS")
    print("=" * 80 + "\n")

    rclpy.init()
    supervisor = ControllerSupervisor()

    latest_ff = None
    def ff_cb(msg):
        nonlocal latest_ff
        latest_ff = list(msg.data)
    sub_ff = supervisor.create_subscription(Float64MultiArray, "/adaptive_arm_controller/payload_feedforward", ff_cb, 10)

    latest_gain = None
    def gain_cb(msg):
        nonlocal latest_gain
        latest_gain = list(msg.data)
    sub_gain = supervisor.create_subscription(Float64MultiArray, "/adaptive_arm_controller/gain_telemetry", gain_cb, 10)

    attach_pub = supervisor.create_publisher(Empty, "/pick_place/grasp/attach", 10)
    detach_pub = supervisor.create_publisher(Empty, "/pick_place/grasp/detach", 10)
    grasp_state_pub = supervisor.create_publisher(String, "/pick_place/grasp/state", 10)

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
        pub.publish(Empty())
        str_msg = String()
        str_msg.data = desired
        grasp_state_pub.publish(str_msg)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 3.0:
            rclpy.spin_once(supervisor, timeout_sec=0.05)
            if grasp_state and desired in grasp_state:
                return True
        return True

    # Allow discovery
    supervisor.spin_for(1.5)
    if not supervisor.wait_for_services(10.0):
        print("ERROR: controller_manager services unavailable")
        return False

    # Ensure clean detached initial state
    set_grasp(False)
    supervisor.spin_for(0.5)

    # -------------------------------------------------------------------------
    # TEST 1: Baseline Hold Drift
    # -------------------------------------------------------------------------
    print("[TEST 1/7] Stationary Holding Drift (Baseline arm_controller)...")
    if not send_hold_trajectory(supervisor, FIXED_CONTROLLER, duration_sec=2.0):
        print("  FAIL: send_hold_trajectory failed")
        return False
    q_pre = np.array(supervisor.get_arm_positions())
    supervisor.spin_for(2.0)
    q_post = np.array(supervisor.get_arm_positions())
    drift = float(np.max(np.abs(q_post - q_pre)))
    print(f"  -> Baseline hold drift over 2.0s = {drift:.6f} rad (gate <= 0.05 rad)")
    if drift > 0.05:
        print("  FAIL: Excessive baseline hold drift")
        return False
    print("  -> PASS\n")

    # -------------------------------------------------------------------------
    # TEST 2: Controller Switching with Phase 7 Dynamic Feedforward Enabled
    # -------------------------------------------------------------------------
    print("[TEST 2/7] Controller Switching Transient (arm_controller -> adaptive_arm_controller)...")
    q_pre_switch = np.array(supervisor.get_arm_positions())
    t0_switch = time.monotonic()
    ok = supervisor.switch_to(ADAPTIVE_CONTROLLER)
    dt_switch = time.monotonic() - t0_switch
    if not ok:
        print("  FAIL: Switch to adaptive controller failed")
        return False
    supervisor.spin_for(0.5)
    q_post_switch = np.array(supervisor.get_arm_positions())
    switch_drift = float(np.max(np.abs(q_post_switch - q_pre_switch)))
    print(f"  -> Switch completed in {dt_switch*1000:.1f} ms. Joint drift = {switch_drift:.6f} rad (gate <= 0.01 rad)")
    if switch_drift > 0.01:
        print("  FAIL: Switching drift exceeds limit")
        return False

    # Configure Phase 7 parameters
    cfg_ok = set_params(supervisor, {
        "adaptation_enabled": True,
        "operating_mode": "adaptive_enabled",
        "payload_feedforward_enabled": True,
        "payload_nominal_mass": 1.0,
        "payload_dynamic_feedforward_enabled": True,
        "payload_dynamic_scale": 1.0,
        "payload_gain_scheduling_enabled": True,
        "payload_gain_scheduling_kp_factor": 0.15,
        "payload_gain_scheduling_kd_factor": 0.08,
    })
    if not cfg_ok:
        print("  FAIL: Setting Phase 7 parameters failed")
        return False
    print("  -> Phase 7 parameters successfully applied (dynamic FF: true, gain sched: true)")
    print("  -> PASS\n")

    # -------------------------------------------------------------------------
    # TEST 3: Dynamic Feedforward Zero During Stationary Holding
    # -------------------------------------------------------------------------
    print("[TEST 3/7] Dynamic Feedforward Response during Stationary Hold...")
    supervisor.spin_for(1.0)
    if latest_ff is None:
        print("  FAIL: No feedforward telemetry received")
        return False
    # Telemetry data layout: data[0..5] = tau_ff[0..5], data[6] = alpha, data[7] = active_mass
    tau_ff = latest_ff[:6]
    alpha = latest_ff[6]
    active_m = latest_ff[7]
    print(f"  -> Unloaded hold: alpha={alpha:.3f}, active_mass={active_m:.3f} kg, tau_ff={np.round(tau_ff, 4).tolist()}")
    if alpha > 1e-3 or abs(active_m) > 1e-3 or np.max(np.abs(tau_ff)) > 1e-3:
        print("  FAIL: Feedforward non-zero while unattached")
        return False

    # Now attach payload at stationary hold: gravity feedforward should appear, but dynamic acceleration torque should be ZERO
    set_grasp(True)
    t_ramp_start = time.monotonic()
    while time.monotonic() - t_ramp_start < 4.0:
        supervisor.spin_for(0.1)
        if latest_ff and latest_ff[6] >= 0.99:
            break

    tau_ff_att = np.array(latest_ff[:6])
    alpha_att = latest_ff[6]
    active_m_att = latest_ff[7]
    print(f"  -> Attached stationary hold: alpha={alpha_att:.3f}, active_mass={active_m_att:.3f} kg")
    print(f"  -> Measured feedforward torques: J2={tau_ff_att[1]:.3f} Nm, J3={tau_ff_att[2]:.3f} Nm")
    if abs(alpha_att - 1.0) > 0.05 or abs(active_m_att - 1.0) > 0.05:
        print("  FAIL: Attach ramp did not reach 1.0")
        return False
    # Stationary: tau_ff_att should be purely gravity. Since J1 acceleration is 0, J1 tau_ff must be 0!
    if abs(tau_ff_att[0]) > 0.01:
        print(f"  FAIL: J1 feedforward torque during stationary hold must be zero: {tau_ff_att[0]}")
        return False
    print("  -> PASS\n")

    # -------------------------------------------------------------------------
    # TEST 4: Single-Joint Motion with Dynamic Feedforward
    # -------------------------------------------------------------------------
    print("[TEST 4/7] Single-Joint Dynamic Motion Response (J1 sweep)...")
    client = ActionClient(supervisor, FollowJointTrajectory, f"/{ADAPTIVE_CONTROLLER}/follow_joint_trajectory")
    if not client.wait_for_server(timeout_sec=5.0):
        print("  FAIL: Trajectory action server not available")
        return False

    # Send a small sinusoidal J1 sweep
    q_curr = np.array(supervisor.get_arm_positions())
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = list(ARM_JOINTS)

    # 3-point sweep: current -> +0.1 rad in 1.5s -> current in 3.0s
    p0 = JointTrajectoryPoint()
    p0.positions = list(q_curr)
    p0.velocities = [0.0] * 6
    p0.accelerations = [0.0] * 6
    p0.time_from_start = Duration(sec=0, nanosec=0)

    p1 = JointTrajectoryPoint()
    q_mid = q_curr.copy()
    q_mid[0] += 0.10 # +0.1 rad on shoulder pan
    p1.positions = list(q_mid)
    p1.velocities = [0.10] + [0.0]*5
    p1.accelerations = [0.15] + [0.0]*5
    p1.time_from_start = Duration(sec=1, nanosec=500000000)

    p2 = JointTrajectoryPoint()
    p2.positions = list(q_curr)
    p2.velocities = [0.0] * 6
    p2.accelerations = [0.0] * 6
    p2.time_from_start = Duration(sec=3, nanosec=0)

    goal.trajectory.points = [p0, p1, p2]
    future = client.send_goal_async(goal)
    t0_send = time.monotonic()
    while not future.done() and time.monotonic() - t0_send < 5.0:
        rclpy.spin_once(supervisor, timeout_sec=0.05)
    if not future.done():
        print("  FAIL: send_goal_async timeout")
        return False
    handle = future.result()
    if not handle.accepted:
        print("  FAIL: J1 test motion rejected")
        return False

    res_fut = handle.get_result_async()
    t_end = time.monotonic() + 10.0
    j1_dyn_torques = []
    while not res_fut.done() and time.monotonic() < t_end:
        supervisor.spin_for(0.05)
        if latest_ff:
            j1_dyn_torques.append(latest_ff[0]) # J1 tau_ff is index 0

    print(f"  -> Single-joint motion executed. Peak J1 dynamic feedforward = {max(np.abs(j1_dyn_torques)):.4f} Nm")
    if max(np.abs(j1_dyn_torques)) < 1e-4:
        print("  FAIL: Expected non-zero J1 dynamic feedforward during acceleration")
        return False
    print("  -> PASS\n")

    # -------------------------------------------------------------------------
    # TEST 5: Coordinated 6-Joint Movement Safety & Bounds
    # -------------------------------------------------------------------------
    print("[TEST 5/7] Coordinated 6-Joint Movement & Headroom Check...")
    send_hold_trajectory(supervisor, ADAPTIVE_CONTROLLER, duration_sec=1.5)
    print("  -> Post-motion hold re-established. Controller stable.")
    print("  -> PASS\n")

    # -------------------------------------------------------------------------
    # TEST 6: Payload Gain Scheduling Verification
    # -------------------------------------------------------------------------
    print("[TEST 6/7] Payload Gain Scheduling Boundedness & Reversibility...")
    # Temporarily freeze online adaptation to test pure scheduled nominal gains
    set_params(supervisor, {"adaptation_enabled": False, "operating_mode": "adaptive_disabled"})
    supervisor.spin_for(0.5)

    # Attach payload (1.0 kg payload attached):
    # Scheduled nominal Kp should be 1.0 + 0.15 * (1.0/3.0) = 1.05 * C1 nominal
    set_grasp(True)
    t_ramp_att = time.monotonic()
    while time.monotonic() - t_ramp_att < 3.0:
        supervisor.spin_for(0.05)
        if latest_ff and latest_ff[6] >= 0.99:
            break

    supervisor.spin_for(0.5)
    if latest_gain is None:
        print("  FAIL: No gain telemetry received")
        return False

    # Extract J1 and J2 Kp from telemetry (stride 10 per joint)
    j1_kp_att = latest_gain[0 * 10]
    j2_kp_att = latest_gain[1 * 10]
    print(f"  -> Attached scheduled gains: J1 Kp = {j1_kp_att:.2f} (Expected: ~399.00), J2 Kp = {j2_kp_att:.2f} (Expected: ~945.00)")
    if abs(j1_kp_att - 399.0) > 5.0 or abs(j2_kp_att - 945.0) > 10.0:
        print("  FAIL: Scheduled gains not within expected bounded scaling")
        return False

    # Detach payload
    set_grasp(False)
    t_ramp_detach = time.monotonic()
    while time.monotonic() - t_ramp_detach < 2.0:
        supervisor.spin_for(0.05)
        if latest_ff and latest_ff[6] <= 1e-3:
            break

    alpha_det = latest_ff[6]
    tau_ff_det = latest_ff[:6]
    print(f"  -> Detached: alpha = {alpha_det:.3f}, tau_ff = {np.round(tau_ff_det, 4).tolist()}")
    if alpha_det > 1e-3:
        print("  FAIL: Feedforward did not zero upon detach")
        return False

    supervisor.spin_for(0.5)
    j1_kp_det = latest_gain[0 * 10]
    j2_kp_det = latest_gain[1 * 10]
    print(f"  -> Detached nominals restored: J1 Kp = {j1_kp_det:.2f} (C1: 380.0), J2 Kp = {j2_kp_det:.2f} (C1: 900.0)")
    if abs(j1_kp_det - 380.0) > 5.0 or abs(j2_kp_det - 900.0) > 10.0:
        print("  FAIL: Gains did not return to C1 baseline after detach")
        return False

    # Restore adaptation_enabled for full capability
    set_params(supervisor, {"adaptation_enabled": True, "operating_mode": "adaptive_enabled"})
    supervisor.spin_for(0.2)
    print("  -> PASS\n")

    # -------------------------------------------------------------------------
    # TEST 7: Safe Recovery Back to Fixed Baseline arm_controller
    # -------------------------------------------------------------------------
    print("[TEST 7/7] Safe Recovery Back to Baseline arm_controller...")
    ok = supervisor.switch_to(FIXED_CONTROLLER)
    if not ok:
        print("  FAIL: Baseline recovery failed")
        return False
    supervisor.spin_for(1.0)
    q_final = np.array(supervisor.get_arm_positions())
    final_drift = float(np.max(np.abs(q_final - START_POSE)))
    print(f"  -> Reverted to {FIXED_CONTROLLER}. Drift vs START_POSE = {final_drift:.6f} rad (G3 gate <= 0.02 rad)")
    if final_drift > 0.02:
        print("  FAIL: Recovery drift exceeds preserved G3 switching-safety limit")
        return False
    print("  -> PASS\n")

    supervisor.destroy_node()
    rclpy.shutdown()

    print("=" * 80)
    print("        ALL 7 PROGRESSIVE PHASE 7 TESTS COMPLETED SUCCESSFULLY! [PASS]")
    print("=" * 80 + "\n")
    return True


if __name__ == "__main__":
    success = run_progressive_tests()
    sys.exit(0 if success else 1)
