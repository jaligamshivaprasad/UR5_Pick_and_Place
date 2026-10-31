#!/usr/bin/env python3
"""Release diagnostics for an already running, disposable UR5 simulation.

Run R3 and R4 from separate fresh Gazebo sessions with the adaptive arm active.
"""

import argparse
import csv
import json
import time
from pathlib import Path
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import Empty, Float64MultiArray, String
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

JOINTS = ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint')
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'ur5_bringup' / 'scripts'))
from pick_place_trajectory_orchestrator import read_path


class Recorder(Node):
    def __init__(self):
        super().__init__('diagnose_release_isolates')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.rows = []
        self.events = []
        self.last = {}
        self.create_subscription(JointTrajectoryControllerState,
                                 '/arm_controller/controller_state', self.arm_cb, 100)
        self.create_subscription(JointState, '/joint_states', self.grip_cb, 100)
        self.create_subscription(Float64MultiArray,
                                 '/adaptive_arm_controller/payload_feedforward', self.ff_cb, 100)
        self.create_subscription(Float64MultiArray,
                                 '/adaptive_arm_controller/gain_telemetry', self.gain_cb, 100)
        self.create_subscription(Float64MultiArray,
                                 '/adaptive_arm_controller/pid_telemetry', self.pid_cb, 100)
        try:
            from ros_gz_interfaces.msg import Contacts
            self.create_subscription(Contacts, '/tray/contact', self.tray_contact_cb, 10)
        except ImportError:
            pass
        self.create_subscription(String, '/pick_place/grasp/state', self.state_cb, 10)
        self.create_subscription(TFMessage,
                                 '/world/ur5_pick_place/dynamic_pose/info', self.pose_cb, 10)
        self.state_pub = self.create_publisher(String, '/pick_place/grasp/state', 10)
        self.attach_pub = self.create_publisher(Empty, '/pick_place/grasp/attach', 10)
        self.detach_pub = self.create_publisher(Empty, '/pick_place/grasp/detach', 10)
        self.gripper = ActionClient(self, FollowJointTrajectory,
                                    '/robotiq_gripper_controller/follow_joint_trajectory')
        self.arm = ActionClient(self, FollowJointTrajectory,
                                '/adaptive_arm_controller/follow_joint_trajectory')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def event(self, name):
        self.events.append({'time_sec': self.now(), 'event': name})

    def arm_cb(self, msg):
        try:
            indices = [msg.joint_names.index(j) for j in JOINTS]
            row = {'time_sec': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9}
            for j, i in zip(JOINTS, indices):
                row[j + '_ref_rad'] = msg.desired.positions[i]
                row[j + '_pos_rad'] = msg.actual.positions[i]
                row[j + '_vel_rad_s'] = msg.actual.velocities[i]
                row[j + '_cmd_Nm'] = msg.output.effort[i]
            row.update(self.last)
            self.rows.append(row)
        except (ValueError, IndexError):
            pass

    def grip_cb(self, msg):
        if 'robotiq_85_left_knuckle_joint' in msg.name:
            i = msg.name.index('robotiq_85_left_knuckle_joint')
            if i < len(msg.position):
                self.last['gripper_rad'] = msg.position[i]

    def ff_cb(self, msg):
        if len(msg.data) >= 8:
            self.last['payload_alpha'] = msg.data[6]
            self.last['active_mass_kg'] = msg.data[7]
            for j, v in zip(JOINTS, msg.data[:6]):
                self.last[j + '_ff_Nm'] = v

    def gain_cb(self, msg):
        if len(msg.data) >= 60:
            for k, j in enumerate(JOINTS):
                self.last[j + '_Kp'] = msg.data[10 * k]
                self.last[j + '_Ki'] = msg.data[10 * k + 1]
                self.last[j + '_Kd'] = msg.data[10 * k + 2]

    def pid_cb(self, msg):
        if len(msg.data) >= 49:
            for k, j in enumerate(JOINTS):
                off = 1 + k * 8
                self.last[j + '_p_Nm'] = msg.data[off + 0]
                self.last[j + '_i_Nm'] = msg.data[off + 1]
                self.last[j + '_d_Nm'] = msg.data[off + 2]
                self.last[j + '_ff_grav_Nm'] = msg.data[off + 3]
                self.last[j + '_ff_dyn_Nm'] = msg.data[off + 4]
                self.last[j + '_ff_total_Nm'] = msg.data[off + 5]
                self.last[j + '_cmd_effort_Nm'] = msg.data[off + 6]
                self.last[j + '_integral_state'] = msg.data[off + 7]

    def tray_contact_cb(self, msg):
        self.last['tray_contacts_count'] = len(msg.contacts)
        self.last['tray_contact_active'] = 1.0 if msg.contacts else 0.0

    def state_cb(self, msg):
        self.last['attachment_state'] = msg.data
        self.event('state_' + msg.data)

    def pose_cb(self, msg):
        for t in msg.transforms:
            if 'block' in t.child_frame_id.lower():
                p = t.transform.translation
                self.last.update(block_x_m=p.x, block_y_m=p.y, block_z_m=p.z)

    def spin_for(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def spin_sim_for(self, duration, wall_limit=60):
        start = self.rows[-1]['time_sec']
        end = time.monotonic() + wall_limit
        while self.rows[-1]['time_sec'] - start < duration:
            if time.monotonic() >= end:
                self.event('sim_time_window_timeout')
                break
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_future(self, future, timeout):
        end = time.monotonic() + timeout
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
        if not future.done():
            raise TimeoutError('ROS action timed out')
        return future.result()

    def command_gripper(self, position):
        if not self.gripper.wait_for_server(timeout_sec=10):
            raise RuntimeError('Gripper action server unavailable')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['robotiq_85_left_knuckle_joint']
        goal.trajectory.points = [JointTrajectoryPoint(
            positions=[position], time_from_start=Duration(sec=2))]
        self.event('gripper_command_' + str(position))
        handle = self.wait_future(self.gripper.send_goal_async(goal), 10)
        if not handle.accepted:
            raise RuntimeError('Gripper action rejected')
        result = self.wait_future(handle.get_result_async(), 20)
        if result.result.error_code != 0:
            raise RuntimeError('Gripper action failed: ' + str(result.result.error_code))
        self.event('gripper_complete_' + str(position))

    def command_arm(self, points, label):
        if not self.arm.wait_for_server(timeout_sec=10):
            raise RuntimeError('Adaptive arm action server unavailable')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        for t, positions, velocities, accelerations in points:
            ns = round(t * 1e9)
            goal.trajectory.points.append(JointTrajectoryPoint(
                positions=list(positions), velocities=list(velocities),
                accelerations=list(accelerations),
                time_from_start=Duration(sec=ns // 1000000000,
                                         nanosec=ns % 1000000000)))
        self.event('arm_start_' + label)
        handle = self.wait_future(self.arm.send_goal_async(goal), 15)
        if not handle.accepted:
            raise RuntimeError('Arm action rejected: ' + label)
        duration = points[-1][0]
        result = self.wait_future(handle.get_result_async(), max(30, duration * 5))
        if result.result.error_code != 0:
            raise RuntimeError('Arm action failed for ' + label + ': ' +
                               str(result.result.error_code))
        self.event('arm_complete_' + label)

    def attachment(self, attached):
        desired = 'attached' if attached else 'detached'
        pub = self.attach_pub if attached else self.detach_pub
        if pub.get_subscription_count() == 0:
            raise RuntimeError('Gazebo grasp bridge unavailable')
        self.event('detach_command' if not attached else 'attach_command')
        pub.publish(Empty())
        end = time.monotonic() + 10
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.last.get('attachment_state') == desired:
                return
        raise TimeoutError('No Gazebo attachment confirmation: ' + desired)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('case', choices=['R1', 'R2', 'R3', 'R4'])
    parser.add_argument('--out', required=True, type=Path)
    parser.add_argument('--at-place', action='store_true',
                        help='For R1/R2, move the unloaded arm to the saved place pose first')
    args = parser.parse_args()
    rclpy.init()
    node = Recorder()
    try:
        ready_by = time.monotonic() + 15.0
        while not node.rows and time.monotonic() < ready_by:
            rclpy.spin_once(node, timeout_sec=0.05)
        if not node.rows:
            raise RuntimeError('No arm controller state received')
        if args.at_place and args.case in ('R1', 'R2'):
            segments = read_path(ROOT / 'path01')
            place = next(s for s in segments if s['name'] == 'pick_to_place')['points'][-1][1]
            current = [node.rows[-1][j + '_pos_rad'] for j in JOINTS]
            zeros = [0.0] * 6
            node.command_arm([(0.0, current, zeros, zeros),
                              (3.0, place, zeros, zeros)], 'unloaded_place_pose')
            node.spin_for(1.0)
        if args.case == 'R1':
            node.command_gripper(0.4)
            node.spin_for(1.0)
            node.command_gripper(0.0)
        elif args.case == 'R2':
            node.state_pub.publish(String(data='attached'))
            node.event('synthetic_payload_attach')
            node.spin_for(1.5)
            if node.last.get('payload_alpha', 0) < 0.9:
                raise RuntimeError('Adaptive feedforward did not engage')
            node.event('synthetic_payload_detach')
            node.state_pub.publish(String(data='detached'))
        else:
            segments = read_path(ROOT / 'path01')
            start = segments[0]['points'][0][1]
            current = [node.rows[-1][j + '_pos_rad'] for j in JOINTS]
            zeros = [0.0] * 6
            # Fortress DetachableJoint may initialize attached and its initial
            # state publication is not latched. Establish a known free block.
            node.attachment(False)
            node.command_gripper(0.0)
            node.command_arm([(0.0, current, zeros, zeros),
                              (3.0, start, zeros, zeros)], 'align')
            node.spin_for(1.0)
            for segment in segments:
                label = segment['name']
                if label == 'pick_to_place':
                    node.command_gripper(0.4)
                    node.attachment(True)
                    node.spin_for(0.5)
                if label == 'place_to_straight':
                    node.spin_for(0.5)
                    # Safe Release State Machine
                    # 1. APPROACH: Lower block smoothly down to tray contact surface
                    q_approach = [node.rows[-1][j + '_pos_rad'] for j in JOINTS]
                    q_support = [-1.917477, -2.386708, -0.971846, -1.428074, -4.713392, 1.15695]
                    node.event('release_approach_start')
                    node.command_arm([(0.0, q_approach, zeros, zeros),
                                      (1.5, q_support, zeros, zeros)], 'release_approach')
                    node.event('release_approach_complete')

                    # 2. SETTLE: 0.5s pause to ensure arm is completely stationary
                    node.event('release_settle_start')
                    node.spin_for(0.5)
                    node.event('release_settle_complete')

                    # 3. SUPPORT_CONFIRMED: Verify block is rested on tray
                    bz = node.last.get('block_z_m', 0.0)
                    node.event(f'support_confirmed_z_{bz:.4f}')

                    # 4. CONTROLLED_UNLOAD: Smoothly ramp down feedforward while resting on tray
                    node.event('release_unload_start')
                    node.state_pub.publish(String(data='unload'))
                    unload_deadline = time.monotonic() + 3.0
                    while time.monotonic() < unload_deadline:
                        node.spin_for(0.05)
                        if node.last.get('payload_alpha', 1.0) < 0.02:
                            break
                    node.event('release_unload_complete')

                    # 5. DETACH_CONFIRMED: Break rigid joint with 0 Nm torque discontinuity
                    node.attachment(False)

                    # 6. OPEN: Gripper opens away from resting block
                    if args.case == 'R4':
                        node.command_gripper(0.0)

                    # 2.0s sim observation window with stationary arm reference
                    node.spin_sim_for(2.0)

                    # 7. RETRACT: Return arm smoothly back to clearance / approach posture
                    if args.case == 'R4':
                        node.event('release_retract_start')
                        q_curr = [node.rows[-1][j + '_pos_rad'] for j in JOINTS]
                        node.command_arm([(0.0, q_curr, zeros, zeros),
                                          (1.5, q_approach, zeros, zeros)], 'release_retract')
                        node.event('release_retract_complete')
                        node.command_arm(segment['points'], label)
                    break
                node.command_arm(segment['points'], label)
        if args.case in ('R1', 'R2'):
            node.spin_for(2.0)
        args.out.mkdir(parents=True, exist_ok=True)
        columns = sorted(set().union(*(row.keys() for row in node.rows)))
        with (args.out / 'samples.csv').open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=columns)
            writer.writeheader()
            writer.writerows(node.rows)
        (args.out / 'events.json').write_text(json.dumps(node.events, indent=2))
        print(json.dumps({'case': args.case, 'samples': len(node.rows),
                          'events': node.events, 'output': str(args.out)}, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
