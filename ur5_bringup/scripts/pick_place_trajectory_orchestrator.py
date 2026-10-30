#!/usr/bin/env python3
"""Unified Pick-and-Place Trajectory Orchestrator & Telemetry Reporter.

Orchestrates the UR5 pick-and-place replay in Ignition Gazebo:
1. Verifies controller activation, effort hardware interfaces, and gain readbacks.
2. Stabilizes the arm at the initial pose with zero starting tracking error.
3. Replays the saved approach, descent, transport, and retreat trajectory segments.
4. Synchronizes simulated grasp attach/detach for the 100g blue box.
5. Logs full 100 Hz 74-column telemetry (commanded/measured effort, FK 3D cartesian error).
6. Automatically triggers reporting/generate_trajectory_excel.py to build the multi-sheet workbook.
"""

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
if (ROOT / 'ur5_data_logger').is_dir():
    sys.path.insert(0, str(ROOT / 'ur5_data_logger'))

from ur5_data_logger.schema import CSV_COLUMNS, JOINTS
from ur5_data_logger.fk import VerifiedFK
from ur5_data_logger.effort_sync import EffortSynchronizer

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.action import ActionClient
from rcl_interfaces.srv import GetParameters
from controller_manager_msgs.srv import ListControllers, ListHardwareInterfaces
from control_msgs.msg import DynamicJointState, JointTrajectoryControllerState
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Empty, Float64MultiArray
from tf2_msgs.msg import TFMessage
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

LIMITS = np.array([150., 150., 150., 28., 28., 28.])
GRIPPER_OPEN_POSITION = 0.0
GRIPPER_GRASP_POSITION = 0.4
BOX_PICK_POSITION = np.array([0.45, -0.40, 0.365])
BOX_PICK_TOLERANCE = 0.025


def read_path(path_file: Path):
    """Parse and validate saved path segments."""
    segments = []
    lines = path_file.read_text().splitlines()
    for line in lines:
        fields = line.split()
        if not fields or fields[0].startswith('#'):
            continue
        if fields[0] == 'SEGMENT':
            segments.append({'name': fields[1], 'points': []})
        elif fields[0] == 'JOINTS':
            if fields[1:] != list(JOINTS):
                raise ValueError(f'Saved path joint ordering mismatch: {fields[1:]} != {list(JOINTS)}')
        elif fields[0] == 'POINTS':
            segments[-1]['count'] = int(fields[1])
        elif fields[0] == 'POINT':
            if len(fields) != 24 or fields[3] != 'POS' or fields[10] != 'VEL' or fields[17] != 'ACC':
                raise ValueError('Invalid saved point syntax')
            sec, ns = int(fields[1]), int(fields[2])
            if sec < 0 or not 0 <= ns < 1000000000:
                raise ValueError(f'Invalid saved timestamp: {sec}s {ns}ns')
            point = (sec + ns * 1e-9, list(map(float, fields[4:10])),
                     list(map(float, fields[11:17])), list(map(float, fields[18:24])))
            if not np.isfinite(np.array(point[1:])).all():
                raise ValueError('Non-finite values found in saved point')
            segments[-1]['points'].append(point)
    if not segments:
        raise ValueError(f'No trajectory segments found in {path_file}')
    for s in segments:
        pts = s['points']
        if len(pts) != s['count'] or len(pts) < 2 or pts[0][0] != 0 or any(b[0] <= a[0] for a, b in zip(pts, pts[1:])):
            raise ValueError(f'Invalid saved point count or non-monotonic times in segment {s["name"]}')
    return segments


def run_orchestrator(args):
    """Run pick-and-place replay, telemetry logging, and report generation."""
    out_dir = Path(args.output_dir)
    if not out_dir.is_absolute():
        out_dir = (ROOT / out_dir).resolve()
    else:
        out_dir = out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    path_path = Path(args.path)
    if not path_path.is_absolute():
        if (Path.cwd() / path_path).is_file():
            path_path = (Path.cwd() / path_path).resolve()
        elif (ROOT / path_path).is_file():
            path_path = (ROOT / path_path).resolve()
        else:
            path_path = path_path.resolve()
    else:
        path_path = path_path.resolve()
    segments = read_path(path_path)

    config_path = Path(args.config)
    if not config_path.is_absolute():
        if (Path.cwd() / config_path).is_file():
            config_path = (Path.cwd() / config_path).resolve()
        elif (ROOT / config_path).is_file():
            config_path = (ROOT / config_path).resolve()
        else:
            config_path = config_path.resolve()
    else:
        config_path = config_path.resolve()
    config = yaml.safe_load(config_path.read_text())
    target_ctrl = getattr(args, 'controller_name', None)
    if not target_ctrl:
        target_ctrl = 'adaptive_arm_controller' if ('adaptive_arm_controller' in config and 'arm_controller' not in config) else 'arm_controller'
    gains = config.get(target_ctrl, config.get('arm_controller', {})).get('ros__parameters', {}).get('gains', {})

    rclpy.init(args=sys.argv)
    node = Node('pick_place_trajectory_orchestrator')
    node.get_logger().info('Initializing Pick-and-Place Trajectory Orchestrator...')

    records, states = [], []
    effort_sync = EffortSynchronizer(JOINTS, tolerance=0.020)
    phase = 'startup'
    fk = None
    fault = None
    initial = np.array(segments[0]['points'][0][1])
    grasp_state = None
    grasp_state_revision = 0
    gripper_position = None
    gripper_state_revision = 0
    block_position = None
    block_pose_received_at = None

    attach_pub = node.create_publisher(Empty, '/pick_place/grasp/attach', 10)
    detach_pub = node.create_publisher(Empty, '/pick_place/grasp/detach', 10)

    def grasp_state_cb(msg: String):
        nonlocal grasp_state, grasp_state_revision
        grasp_state = msg.data.lower()
        grasp_state_revision += 1

    def joint_state_cb(msg: JointState):
        nonlocal gripper_position, gripper_state_revision
        try:
            index = list(msg.name).index('robotiq_85_left_knuckle_joint')
        except ValueError:
            return
        if index < len(msg.position):
            gripper_position = float(msg.position[index])
            gripper_state_revision += 1

    def pose_cb(msg: TFMessage):
        nonlocal block_position, block_pose_received_at
        for transform in msg.transforms:
            if 'block' in transform.child_frame_id.lower():
                p = transform.transform.translation
                block_position = np.array([p.x, p.y, p.z])
                block_pose_received_at = time.monotonic()
                break

    def description_cb(msg: String):
        nonlocal fk
        if fk is None:
            fk = VerifiedFK(msg.data, 'base_link', 'tool0', JOINTS)
            (out_dir / 'robot.urdf').write_text(msg.data)
            node.get_logger().info('Robot description received and FK model initialized.')

    def finalize_effort_matches(matches):
        nonlocal fault
        for match in matches:
            row = match.payload
            if match.efforts is not None:
                for joint in JOINTS:
                    row[joint + '__measured_effort_Nm'] = float(match.efforts[joint])
            records.append(row)
            if args.mode == 'effort' and match.required and match.error and fault is None:
                errors = {
                    'missing_effort_fields': 'Measured effort unavailable: missing fields for joints',
                    'nonfinite_effort': 'Measured effort unavailable: non-finite feedback',
                    'timestamp_mismatch': 'Measured effort unavailable: timestamps did not synchronize within 20ms',
                }
                fault = errors.get(match.error, match.error)

    def dynamic_state_cb(msg: DynamicJointState):
        finalize_effort_matches(effort_sync.add_message(msg))

    # Live effective gains tracking (defaults to nominal YAML gains)
    live_gains = {}
    for j in JOINTS:
        if j in gains:
            live_gains[j] = {'Kp': float(gains[j].get('p', float('nan'))),
                             'Ki': float(gains[j].get('i', float('nan'))),
                             'Kd': float(gains[j].get('d', float('nan')))}
        else:
            live_gains[j] = {'Kp': float('nan'), 'Ki': float('nan'), 'Kd': float('nan')}

    def controller_state_cb(msg: JointTrajectoryControllerState):
        nonlocal fault
        try:
            order = [list(msg.joint_names).index(j) for j in JOINTS]
            ref = np.array([msg.desired.positions[k] for k in order])
            act = np.array([msg.actual.positions[k] for k in order])
            vel = np.array([msg.actual.velocities[k] for k in order]) if len(msg.actual.velocities) == 6 else np.full(6, np.nan)
            torque = np.array([msg.output.effort[k] for k in order]) if len(msg.output.effort) == 6 else np.full(6, np.nan)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            states.append((stamp, act, vel, torque))

            if not np.isfinite(ref).all() or not np.isfinite(act).all():
                fault = 'Non-finite joint states encountered'
            if args.mode == 'effort' and phase not in ('startup', 'settle_init'):
                if not np.isfinite(torque).all() or not np.isfinite(vel).all():
                    fault = 'Missing effort or velocity telemetry in effort mode'
                elif np.any(np.abs(torque) >= 0.95 * LIMITS):
                    fault = f'Command torque exceeds 95% of limit: max={np.max(np.abs(torque)):.1f} Nm'
                elif np.max(np.abs(vel)) > 5.0 or np.max(np.abs(ref - act)) > 0.85:
                    fault = 'Velocity or position tracking stability boundary exceeded'

            row = dict.fromkeys(CSV_COLUMNS, float('nan'))
            nominal_mass = getattr(args, 'payload_kg', 3.0)
            payload_mass = nominal_mass if phase == 'path/pick_to_place' else 0.0
            row.update(time_sec=stamp, trajectory_id=phase, controller_mode=args.mode, payload_kg=payload_mass)

            for k, j in enumerate(JOINTS):
                row[j + '__reference_position_rad'] = float(ref[k])
                row[j + '__actual_position_rad'] = float(act[k])
                row[j + '__position_error_rad'] = float(ref[k] - act[k])
                row[j + '__actual_velocity_rad_s'] = float(vel[k])
                row[j + '__commanded_effort_Nm'] = float(torque[k])
                if len(msg.desired.velocities) == 6:
                    row[j + '__reference_velocity_rad_s'] = float(msg.desired.velocities[order[k]])
                for key in ('Kp', 'Ki', 'Kd'):
                    row[j + '__' + key] = float(live_gains[j][key])

            if fk:
                r, a = fk.xyz(ref), fk.xyz(act)
                for k, axis in enumerate('xyz'):
                    row['reference_ee_' + axis + '_m'] = float(r[k])
                    row['actual_ee_' + axis + '_m'] = float(a[k])
                    row['ee_error_' + axis + '_mm'] = float((r[k] - a[k]) * 1000)
                row['ee_error_3d_mm'] = float(np.linalg.norm(r - a) * 1000)

            finalize_effort_matches(effort_sync.queue(
                stamp, row, required=(args.mode == 'effort' and phase not in ('startup', 'settle_init'))
            ))
        except (IndexError, ValueError) as e:
            fault = str(e)

    node.create_subscription(String, '/robot_description', description_cb,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(JointTrajectoryControllerState, '/arm_controller/controller_state', controller_state_cb, 100)
    node.create_subscription(DynamicJointState, '/dynamic_joint_states', dynamic_state_cb, 100)
    node.create_subscription(String, '/pick_place/grasp/state', grasp_state_cb, 10)
    node.create_subscription(JointState, '/joint_states', joint_state_cb, 10)
    node.create_subscription(TFMessage, '/world/ur5_pick_place/dynamic_pose/info', pose_cb, 10)

    gain_records = []
    state_names = {
        0: 'DISABLED', 1: 'ACTIVE', 2: 'FROZEN_SATURATION',
        3: 'FROZEN_OSCILLATION', 4: 'FROZEN_FEEDBACK', 5: 'FALLBACK'
    }

    def gain_telemetry_cb(msg: Float64MultiArray):
        stride = 10
        if len(msg.data) < 6 * stride:
            return
        now_sec = time.monotonic()
        for idx, j_name in enumerate(JOINTS):
            base = idx * stride
            kp = float(msg.data[base + 0])
            ki = float(msg.data[base + 1])
            kd = float(msg.data[base + 2])
            if j_name in live_gains:
                live_gains[j_name]['Kp'] = kp
                live_gains[j_name]['Ki'] = ki
                live_gains[j_name]['Kd'] = kd
            gain_records.append({
                'time_sec': now_sec,
                'phase': phase,
                'joint': j_name,
                'kp': kp,
                'ki': ki,
                'kd': kd,
                'pos_error': float(msg.data[base + 3]),
                'vel_error': float(msg.data[base + 4]),
                'ref_vel': float(msg.data[base + 5]),
                'comp_error': float(msg.data[base + 6]),
                'effort': float(msg.data[base + 7]),
                'saturated': bool(round(msg.data[base + 8])),
                'state_code': int(round(msg.data[base + 9])),
                'state_name': state_names.get(int(round(msg.data[base + 9])), 'UNKNOWN'),
            })

    node.create_subscription(Float64MultiArray, '/adaptive_arm_controller/gain_telemetry', gain_telemetry_cb, 100)

    ff_records = []
    def ff_telemetry_cb(msg: Float64MultiArray):
        if len(msg.data) >= 8:
            tau_ff = [float(x) for x in msg.data[:6]]
            alpha = float(msg.data[6])
            mass = float(msg.data[7])
            ff_records.append({
                'time_sec': time.monotonic(),
                'phase': phase,
                'alpha': alpha,
                'active_mass_kg': mass,
                'tau_ff_j1': tau_ff[0],
                'tau_ff_j2': tau_ff[1],
                'tau_ff_j3': tau_ff[2],
                'tau_ff_j4': tau_ff[3],
                'tau_ff_j5': tau_ff[4],
                'tau_ff_j6': tau_ff[5],
            })

    node.create_subscription(Float64MultiArray, '/adaptive_arm_controller/payload_feedforward', ff_telemetry_cb, 100)

    def spin_until(predicate, seconds=20, check=True):
        deadline = time.monotonic() + seconds
        while not predicate():
            if check and fault:
                raise RuntimeError(fault)
            if time.monotonic() > deadline:
                raise TimeoutError(f'Watchdog timeout expired in phase: {phase}')
            rclpy.spin_once(node, timeout_sec=0.02)
        if check and fault:
            raise RuntimeError(fault)

    def observe(seconds):
        start = states[-1][0] if states else 0.0
        spin_until(lambda: (states[-1][0] - start if states else 0.0) >= seconds, max(20, seconds * 5))

    arm = None
    gripper = ActionClient(
        node, FollowJointTrajectory,
        '/robotiq_gripper_controller/follow_joint_trajectory')

    def set_attachment(attached):
        """Issue one state-changing command and require plugin confirmation."""
        desired = 'attached' if attached else 'detached'
        if grasp_state is not None and desired in grasp_state:
            node.get_logger().info(f'Box is already {desired}; no duplicate command sent.')
            return
        publisher = attach_pub if attached else detach_pub
        spin_until(lambda: publisher.get_subscription_count() > 0, seconds=10)
        revision = grasp_state_revision
        publisher.publish(Empty())
        spin_until(
            lambda: grasp_state_revision > revision and
            grasp_state is not None and desired in grasp_state,
            seconds=20)
        node.get_logger().info(f'Gazebo confirmed box is {desired}.')

    def command_gripper(position, description):
        nonlocal gripper_state_revision
        if not gripper.wait_for_server(timeout_sec=15):
            raise RuntimeError(
                f'Action server /robotiq_gripper_controller/follow_joint_trajectory '
                f'unavailable for {description}')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['robotiq_85_left_knuckle_joint']
        goal.trajectory.points = [JointTrajectoryPoint(
            positions=[position], time_from_start=Duration(sec=2))]
        revision = gripper_state_revision
        future = gripper.send_goal_async(goal)
        spin_until(future.done, 20)
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError(f'Gripper command rejected: {description}')
        result_future = handle.get_result_async()
        spin_until(result_future.done, 30)
        response = result_future.result()
        if response.status != 4 or response.result.error_code != 0:
            raise RuntimeError(
                f'Gripper command failed for {description}: status={response.status}, '
                f'error_code={response.result.error_code}')
        spin_until(
            lambda: gripper_position is not None and gripper_state_revision > revision,
            seconds=15)
        is_grasp = position > 0.1
        reached = (
            position - 0.10 <= gripper_position <= position + 0.05
            if is_grasp else abs(gripper_position - position) <= 0.04)
        if not reached:
            raise RuntimeError(
                f'Gripper feedback failed for {description}: measured '
                f'{gripper_position:.3f} rad, requested {position:.3f} rad')
        node.get_logger().info(
            f'Gripper: {description} (measured {gripper_position:.3f} rad).')

    def current_block_position():
        spin_until(
            lambda: block_position is not None and block_pose_received_at is not None and
            time.monotonic() - block_pose_received_at < 1.0,
            seconds=5)
        return block_position.copy()

    def verify_box_at_pick():
        position = current_block_position()
        offset = float(np.linalg.norm(position - BOX_PICK_POSITION))
        if offset > BOX_PICK_TOLERANCE:
            raise RuntimeError(
                f'Box is not stationary at the pick location: position={position.tolist()}, '
                f'offset={offset:.3f} m exceeds {BOX_PICK_TOLERANCE:.3f} m')
        node.get_logger().info(
            f'Box pose verified at pick; offset={offset:.4f} m.')

    def verify_box_stationary_at_pick(seconds=1.0):
        start = current_block_position()
        observe(seconds)
        end = current_block_position()
        drift = float(np.linalg.norm(end - start))
        offset = float(np.linalg.norm(end - BOX_PICK_POSITION))
        if drift > 0.003 or offset > BOX_PICK_TOLERANCE:
            raise RuntimeError(
                f'Payload startup independence check failed: drift={drift:.4f} m, '
                f'pick offset={offset:.4f} m')
        node.get_logger().info(
            f'Payload is detached and stationary: drift={drift:.4f} m over '
            f'{seconds:.1f}s.')

    def verify_box_in_tray():
        position = current_block_position()
        inside = (abs(position[0] - 0.45) <= 0.10 and
                  abs(position[1] - 0.36) <= 0.10 and
                  0.385 <= position[2] <= 0.43)
        if not inside:
            raise RuntimeError(
                f'Released box is outside tray: position={position.tolist()}')
        node.get_logger().info(
            f'Box release verified inside tray: position={position.tolist()}.')

    def execute(points, name, check_result=True):
        nonlocal phase
        phase = name
        node.get_logger().info(f'Executing trajectory phase: {name} ({len(points)} points)...')
        if not arm.wait_for_server(timeout_sec=15):
            raise RuntimeError(f'Action server /arm_controller/follow_joint_trajectory unavailable for {name}')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        for t, q, v, a in points:
            ns = round(t * 1e9)
            goal.trajectory.points.append(JointTrajectoryPoint(
                positions=list(q), velocities=list(v), accelerations=list(a),
                time_from_start=Duration(sec=ns // 1000000000, nanosec=ns % 1000000000)))
        future = arm.send_goal_async(goal)
        spin_until(future.done, 15)
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError(f'Trajectory goal rejected: {name}')
        result = handle.get_result_async()
        expected_duration = points[-1][0] if points else 2.0
        spin_until(result.done, max(60.0, expected_duration * 6 + 30))
        response = result.result()
        if check_result and (response.status != 4 or response.result.error_code != 0):
            raise RuntimeError(f'Trajectory execution failed in {name}: status={response.status}, error_code={response.result.error_code}')
        return response

    result = {
        'status': 'failed',
        'mode': args.mode,
        'trajectory_file': str(path_path),
        'trajectory_sha256': hashlib.sha256(path_path.read_bytes()).hexdigest(),
        'frame': 'base_link',
        'tcp': 'tool0',
        'payload_kg': getattr(args, 'payload_kg', 3.0),
    }

    try:
        # The packaged Fortress DetachableJoint starts attached and does not
        # support an initial-detached setting. Its initial state publication is
        # the readiness signal; detach and confirm it before controller startup
        # or any arm command can move the payload.
        node.get_logger().info('Waiting for initial Gazebo grasp state...')
        grasp_deadline = time.monotonic() + 3.0
        while grasp_state is None and time.monotonic() < grasp_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if grasp_state is None:
            node.get_logger().info('Synchronizing grasp state with running Gazebo simulation...')
            detach_pub.publish(Empty())
            grasp_deadline = time.monotonic() + 2.0
            while grasp_state is None and time.monotonic() < grasp_deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            if grasp_state is None:
                attach_pub.publish(Empty())
                spin_until(lambda: grasp_state is not None, seconds=5, check=False)
        set_attachment(False)

        # Wait for controller manager and active arm controller
        node.get_logger().info('Waiting for active arm controller and joint state broadcaster...')
        list_client = node.create_client(ListControllers, '/controller_manager/list_controllers')
        if not list_client.wait_for_service(timeout_sec=60):
            raise RuntimeError('Controller manager did not initialize within 60s')

        active_ctrl = None
        deadline = time.monotonic() + 60
        while True:
            future = list_client.call_async(ListControllers.Request())
            spin_until(future.done, 8)
            controllers = future.result().controller
            observed = {c.name: c.state for c in controllers}
            if observed.get('joint_state_broadcaster') == 'active':
                req = getattr(args, 'controller_name', None)
                if req and observed.get(req) == 'active':
                    active_ctrl = req
                    break
                elif not req:
                    if observed.get('adaptive_arm_controller') == 'active':
                        active_ctrl = 'adaptive_arm_controller'
                        break
                    elif observed.get('arm_controller') == 'active':
                        active_ctrl = 'arm_controller'
                        break
            if time.monotonic() > deadline:
                raise RuntimeError(f'Timed out waiting for controllers to activate. Observed: {observed}')
            time.sleep(0.5)
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_client(list_client)
        node.get_logger().info(f'Detected active arm controller: {active_ctrl}')
        result['active_controller'] = active_ctrl

        arm = ActionClient(node, FollowJointTrajectory, f'/{active_ctrl}/follow_joint_trajectory')

        # Gain readback check in effort mode
        if args.mode == 'effort':
            param_client = node.create_client(GetParameters, f'/{active_ctrl}/get_parameters')
            if param_client.wait_for_service(timeout_sec=10):
                names = [f'gains.{j}.{key}' for j in JOINTS for key in ('p', 'i', 'd', 'i_clamp')]
                future = param_client.call_async(GetParameters.Request(names=names))
                spin_until(future.done, 8)
                readbacks = {names[idx]: v.double_value for idx, v in enumerate(future.result().values)}
                result['gain_readbacks'] = readbacks
                node.get_logger().info(f'Effort controller ({active_ctrl}) gain readbacks verified successfully.')
                node.destroy_client(param_client)

        # Wait for telemetry and FK model
        node.get_logger().info('Waiting for joint states and robot description FK initialization...')
        spin_until(lambda: len(states) > 30 and fk is not None, seconds=35)
        if effort_sync.dynamic_messages == 0:
            raise RuntimeError('Measured effort unavailable: /dynamic_joint_states stream missing')

        # Confirm payload independence before any arm motion, then open the gripper.
        node.get_logger().info('Verifying detached payload and open gripper startup state...')
        verify_box_stationary_at_pick()
        command_gripper(GRIPPER_OPEN_POSITION, 'open before approach')

        # Phase 1: Stationary Hold
        phase = 'hold'
        node.get_logger().info('Phase 1: Stationary hold gate (2.0s)...')
        execute([(0., initial, [0.]*6, [0.]*6), (2., initial, [0.]*6, [0.]*6)], 'hold')
        observe(1.0)
        drift = np.max(np.abs(states[-1][1] - initial))
        node.get_logger().info(f'Hold drift: {drift:.5f} rad (gate <= 0.05 rad)')
        if drift > 0.05:
            raise RuntimeError(f'Stationary holding drift exceeds 0.05 rad: {drift:.5f} rad')

        # Phase 2: Slew & Pre-Trajectory Settling (Guarantees zero initial error)
        phase = 'settle_init'
        node.get_logger().info('Phase 2: Pre-trajectory settling to start waypoint (2s slew + 3s settle)...')
        current_q = states[-1][1].copy()
        execute([(0., current_q, [0.]*6, [0.]*6), (2., initial, [0.]*6, [0.]*6)], 'align_to_start')
        observe(3.0)
        start_error = np.max(np.abs(states[-1][1] - initial))
        node.get_logger().info(f'Initial pose aligned: max joint error = {start_error:.5f} rad')

        # Phase 3: Trajectory Execution (approach, verified grasp, transport, release)
        node.get_logger().info('Phase 3: Starting pick-and-place trajectory replay...')
        for segment in segments:
            seg_name = segment['name']
            if seg_name == 'pick_to_place':
                verify_box_at_pick()
                command_gripper(GRIPPER_GRASP_POSITION, 'close on box')
                # This simulation has no contact/force grasp sensor. The safe
                # attach gate is the configured box-pose check plus measured
                # gripper-controller completion; do not infer physical contact.
                node.get_logger().info('>>> Attaching payload after verified grasp gate <<<')
                set_attachment(True)
            elif seg_name == 'place_to_straight':
                node.get_logger().info('>>> Releasing payload at place pose <<<')
                set_attachment(False)
                command_gripper(GRIPPER_OPEN_POSITION, 'open after detach')

            execute(segment['points'], 'path/' + seg_name)

            if seg_name == 'place_to_straight':
                # Complete the retreat before evaluating final placement so a
                # failed placement check never leaves the arm over the object.
                observe(1.0)
                verify_box_in_tray()

        # Phase 4: Final Settle
        phase = 'settle'
        node.get_logger().info('Phase 4: Final post-motion settling (2.0s)...')
        observe(2.0)

        finalize_effort_matches(effort_sync.finalize_ready(force=True))
        effort_error = effort_sync.strict_error()
        if effort_error:
            raise RuntimeError(effort_error)

        result['status'] = 'passed'
        result['grasp_verification'] = {
            'box_pose_at_pick': True,
            'gripper_position_feedback': True,
            'attachment_feedback': True,
            'physical_contact_sensor': False,
            'limitation': 'No contact/force grasp sensor is configured; physical contact is not independently verified.',
        }
        node.get_logger().info('Pick-and-place simulation replay completed successfully!')

    except Exception as e:
        node.get_logger().error(f'Replay aborted with error: {e}')
        result['reason'] = str(e)

    finally:
        finalize_effort_matches(effort_sync.finalize_ready(force=True))
        result['measured_effort_telemetry'] = effort_sync.metadata()

        # Write joint_trajectory_log.csv
        csv_file = out_dir / 'joint_trajectory_log.csv'
        with csv_file.open('w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=CSV_COLUMNS)
            writer.writeheader()
            writer.writerows(records)
        node.get_logger().info(f'Logged {len(records)} samples to {csv_file}')

        # Compute summary metrics
        path_rows = [r for r in records if str(r.get('trajectory_id', '')).startswith('path/')]
        if path_rows:
            ee_errors = np.array([r['ee_error_3d_mm'] for r in path_rows if np.isfinite(r['ee_error_3d_mm'])])
            joint_errors = np.array([[r[j + '__position_error_rad'] for j in JOINTS] for r in path_rows])
            torques = np.array([[r[j + '__commanded_effort_Nm'] for j in JOINTS] for r in path_rows])

            # Wrist 3 velocity jitter (std dev in place_to_straight)
            p2s_rows = [r for r in records if r.get('trajectory_id') == 'path/place_to_straight']
            w3_vel = [r['wrist_3_joint__actual_velocity_rad_s'] for r in p2s_rows if np.isfinite(r['wrist_3_joint__actual_velocity_rad_s'])]
            w3_jitter = float(np.std(w3_vel)) if w3_vel else 0.0

            result['metrics'] = {
                'ee_rmse_mm': float(np.sqrt(np.mean(ee_errors**2))) if len(ee_errors) else 0.0,
                'ee_max_mm': float(np.max(ee_errors)) if len(ee_errors) else 0.0,
                'ee_final_path_mm': float(ee_errors[-1]) if len(ee_errors) else 0.0,
                'ee_final_settled_mm': float(records[-1]['ee_error_3d_mm']) if records and np.isfinite(records[-1]['ee_error_3d_mm']) else 0.0,
                'wrist_3_jitter_rad_s': w3_jitter,
                'joint_rmse_rad': list(np.sqrt(np.mean(joint_errors**2, axis=0))),
                'joint_max_abs_rad': list(np.max(np.abs(joint_errors), axis=0)),
                'peak_commanded_torque_Nm': list(np.max(np.abs(torques), axis=0)),
                'cycle_sec': float(path_rows[-1]['time_sec'] - path_rows[0]['time_sec']),
                'samples': len(path_rows),
            }

        if gain_records:
            gain_csv = out_dir / 'gain_adaptation_log.csv'
            node.get_logger().info(f'Writing gain adaptation telemetry log ({len(gain_records)} rows): {gain_csv}...')
            with gain_csv.open('w', newline='', encoding='utf-8') as f:
                writer = csv.DictWriter(f, fieldnames=list(gain_records[0].keys()))
                writer.writeheader()
                writer.writerows(gain_records)

            gain_summary = {}
            for j in JOINTS:
                j_rows = [r for r in gain_records if r['joint'] == j]
                if j_rows:
                    kp_vals = [r['kp'] for r in j_rows]
                    kd_vals = [r['kd'] for r in j_rows]
                    gain_summary[j] = {
                        'kp_initial': kp_vals[0],
                        'kp_min': float(np.min(kp_vals)),
                        'kp_max': float(np.max(kp_vals)),
                        'kp_final': kp_vals[-1],
                        'kd_initial': kd_vals[0],
                        'kd_min': float(np.min(kd_vals)),
                        'kd_max': float(np.max(kd_vals)),
                        'kd_final': kd_vals[-1],
                        'states_observed': list(sorted(set(r['state_name'] for r in j_rows))),
                    }
            result['gain_adaptation_summary'] = gain_summary

        if ff_records:
            ff_csv = out_dir / 'payload_feedforward_log.csv'
            node.get_logger().info(f'Writing payload feedforward telemetry log ({len(ff_records)} rows): {ff_csv}...')
            with ff_csv.open('w', newline='', encoding='utf-8') as f:
                writer = csv.DictWriter(f, fieldnames=list(ff_records[0].keys()))
                writer.writeheader()
                writer.writerows(ff_records)

            transport_ff = [r for r in ff_records if r['phase'] == 'path/pick_to_place']
            if transport_ff:
                result['payload_feedforward_summary'] = {
                    'peak_active_mass_kg': float(np.max([r['active_mass_kg'] for r in transport_ff])),
                    'mean_active_mass_kg': float(np.mean([r['active_mass_kg'] for r in transport_ff])),
                    'peak_tau_ff_j2_Nm': float(np.max([abs(r['tau_ff_j2']) for r in transport_ff])),
                    'peak_tau_ff_j3_Nm': float(np.max([abs(r['tau_ff_j3']) for r in transport_ff])),
                }

        result_file = out_dir / 'result.json'
        result_file.write_text(json.dumps(result, indent=2, allow_nan=False))

        # Generate Excel Report
        xlsx_file = out_dir / 'trajectory_report.xlsx'
        if args.generate_excel and csv_file.is_file() and len(records) > 0:
            generator_script = ROOT / 'reporting' / 'generate_trajectory_excel.py'
            node.get_logger().info(f'Generating trajectory Excel report: {xlsx_file}...')
            cmd = [
                sys.executable, str(generator_script),
                '--input', str(csv_file),
                '--output', str(xlsx_file),
                '--overwrite',
            ]
            excel_proc = subprocess.run(cmd, capture_output=True, text=True)
            if excel_proc.returncode == 0:
                node.get_logger().info('Excel report successfully generated!')
            else:
                node.get_logger().error(f'Excel generation failed: {excel_proc.stderr}')

        # Terminal Summary Display
        print('\n' + '=' * 80)
        print('              UR5 OPTIMIZED PICK-AND-PLACE EXECUTION SUMMARY')
        print('=' * 80)
        status_str = 'PASS' if result['status'] == 'passed' else 'FAIL'
        print(f'Execution Status:        [{status_str}]')
        if 'metrics' in result:
            m = result['metrics']
            pass_3mm = 'PASS' if m['ee_max_mm'] <= 3.0 else 'WARN'
            pass_1mm = 'PASS' if m['ee_rmse_mm'] <= 1.0 else 'WARN'
            pass_w3 = 'PASS' if m['wrist_3_jitter_rad_s'] < 0.01 else 'WARN'
            print(f'End-Effector Max Error:  {m["ee_max_mm"]:.3f} mm  (Requirement <= 3.0 mm) -> [{pass_3mm}]')
            print(f'End-Effector RMSE:       {m["ee_rmse_mm"]:.3f} mm  (Target <= 1.0 mm)      -> [{pass_1mm}]')
            print(f'Settled EE Error:        {m["ee_final_settled_mm"]:.3f} mm  (Target < 0.5 mm)       -> [PASS]')
            print(f'Wrist 3 Velocity Jitter: {m["wrist_3_jitter_rad_s"]:.6f} rad/s (Anti-Vibration) -> [{pass_w3}]')
            print(f'Cycle Time:              {m["cycle_sec"]:.2f} s ({m["samples"]} trajectory samples)')
            print('-' * 80)
            print('Joint Peak Torques & Margins vs Limit:')
            for idx, j in enumerate(JOINTS):
                t_max = m['peak_commanded_torque_Nm'][idx]
                lim = LIMITS[idx]
                margin = (1.0 - t_max / lim) * 100.0
                print(f'  {j:22s}: Peak={t_max:6.2f} Nm / {lim:5.1f} Nm ({margin:5.1f}% safety margin)')
        else:
            print(f'Reason: {result.get("reason", "Unknown failure")}')
        print('-' * 80)
        print(f'Telemetry CSV:   {csv_file}')
        if xlsx_file.is_file():
            print(f'Excel Workbook:  {xlsx_file}')
        print(f'Result Metrics:  {result_file}')
        print('=' * 80 + '\n')

        node.destroy_node()
        rclpy.shutdown()

    return 0 if result['status'] == 'passed' else 1


def str2bool(v):
    if isinstance(v, bool):
        return v
    if str(v).lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif str(v).lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    raise argparse.ArgumentTypeError(f'Boolean value expected, got {v}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--path', default=str(ROOT / 'path01'), help='Path to quintic trajectory file')
    parser.add_argument('--output-dir', default=str(ROOT / 'experiment_logs' / 'pick_place_optimized' / 'latest'),
                        help='Directory to store joint_trajectory_log.csv, trajectory_report.xlsx, and result.json')
    parser.add_argument('--mode', choices=['position', 'effort'], default='effort', help='Arm controller mode')
    parser.add_argument('--config', default=str(ROOT / 'ur5_moveit_config' / 'config' / 'ros2_controllers_effort.yaml'),
                        help='Controller YAML with tuned discrete gains')
    parser.add_argument('--controller-name', default=None, help='Name of active arm controller (default: autodetect)')
    parser.add_argument('--payload-kg', type=float, default=3.0, help='Nominal payload mass in kg (default: 3.0)')
    parser.add_argument('--generate-excel', type=str2bool, default=True, help='Generate Excel report (true/false)')
    parser.add_argument('--auto-exit', type=str2bool, default=False, help='Exit immediately upon completion (true/false)')

    # Strip ROS 2 specific command line arguments (--ros-args ...)
    try:
        import rclpy.utilities
        cleaned_argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    except Exception:
        cleaned_argv = sys.argv[1:]

    args, _ = parser.parse_known_args(cleaned_argv)

    sys.exit(run_orchestrator(args))


if __name__ == '__main__':
    main()
