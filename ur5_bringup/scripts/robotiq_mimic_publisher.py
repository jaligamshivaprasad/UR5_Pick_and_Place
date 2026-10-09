#!/usr/bin/env python3
import math
from typing import Optional, Sequence

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import JointState


MIMIC_JOINTS = (
    'robotiq_85_right_knuckle_joint',
    'robotiq_85_left_inner_knuckle_joint',
    'robotiq_85_right_inner_knuckle_joint',
    'robotiq_85_left_finger_tip_joint',
    'robotiq_85_right_finger_tip_joint',
)


def measured_mimic_positions(
    names: Sequence[str],
    positions: Sequence[float],
    mimic_joints: Sequence[str] = MIMIC_JOINTS,
) -> Optional[list[float]]:
    """Return measured Gazebo mimic-joint positions, or None if incomplete."""
    if len(names) != len(positions):
        return None

    positions_by_name = dict(zip(names, positions))
    measured = []
    for joint in mimic_joints:
        # gz_ros2_control exposes the physical follower state through this name.
        state_name = f'{joint}_mimic'
        if state_name not in positions_by_name or not math.isfinite(positions_by_name[state_name]):
            return None
        measured.append(positions_by_name[state_name])
    return measured


class MimicPublisher(Node):
    def __init__(self):
        super().__init__('robotiq_mimic_publisher')
        self.sub = self.create_subscription(JointState, '/joint_states', self.callback, 10)
        self.pub = self.create_publisher(JointState, '/joint_states', 10)

        self.mimics = MIMIC_JOINTS
        self.master = 'robotiq_85_left_knuckle_joint'
        self._reported_missing_feedback = False
        self.get_logger().info("Robotiq Mimic Publisher Started")

    def callback(self, msg):
        if self.master not in msg.name:
            return

        # Ignore this node's own canonical-name messages to prevent a feedback loop.
        if any(joint in msg.name for joint in self.mimics):
            return

        positions = measured_mimic_positions(msg.name, msg.position, self.mimics)
        if positions is None:
            if not self._reported_missing_feedback:
                self.get_logger().error(
                    'Gazebo mimic-joint feedback is incomplete; not publishing '
                    'fabricated follower positions. Check joint_state_broadcaster joints.'
                )
                self._reported_missing_feedback = True
            return

        out_msg = JointState()
        out_msg.header = msg.header
        out_msg.name = list(self.mimics)
        out_msg.position = positions
        if len(msg.velocity) == len(msg.name):
            velocities_by_name = dict(zip(msg.name, msg.velocity))
            out_msg.velocity = [
                velocities_by_name[f'{joint}_mimic'] for joint in self.mimics
            ]

        self.pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MimicPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
