#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class MimicPublisher(Node):
    def __init__(self):
        super().__init__('robotiq_mimic_publisher')
        # We subscribe to joint_states and publish back to joint_states
        # to ensure MoveIt gets the full state.
        self.sub = self.create_subscription(JointState, '/joint_states', self.callback, 10)
        self.pub = self.create_publisher(JointState, '/joint_states', 10)

        self.mimics = {
            'robotiq_85_right_knuckle_joint': -1.0,
            'robotiq_85_left_inner_knuckle_joint': 1.0,
            'robotiq_85_right_inner_knuckle_joint': -1.0,
            'robotiq_85_left_finger_tip_joint': -1.0,
            'robotiq_85_right_finger_tip_joint': 1.0,
        }
        self.master = 'robotiq_85_left_knuckle_joint'
        self.get_logger().info("Robotiq Mimic Publisher Started")

    def callback(self, msg):
        if self.master in msg.name:
            # Check if this msg already has mimics to avoid loop
            if 'robotiq_85_right_knuckle_joint' in msg.name:
                return

            idx = msg.name.index(self.master)
            pos = msg.position[idx]

            out_msg = JointState()
            out_msg.header.stamp = self.get_clock().now().to_msg()
            out_msg.header.frame_id = msg.header.frame_id

            for joint, mult in self.mimics.items():
                out_msg.name.append(joint)
                out_msg.position.append(pos * mult)

            self.pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MimicPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
