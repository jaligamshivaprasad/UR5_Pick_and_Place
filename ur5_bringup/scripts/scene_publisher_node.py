#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

class UR5ScenePublisher(Node):
    def __init__(self):
        super().__init__('ur5_scene_publisher')
        self.publisher = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self.timer = self.create_timer(1.0, self.publish_scene)
        self.get_logger().info('UR5 Planning Scene Publisher started.')

    def publish_scene(self):
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True

        # 1. Table
        table = CollisionObject()
        table.header.frame_id = 'world'
        table.id = 'table'
        table_shape = SolidPrimitive()
        table_shape.type = SolidPrimitive.BOX
        table_shape.dimensions = [1.0, 1.2, 0.34]
        table_pose = Pose()
        table_pose.position.x = 0.75
        table_pose.position.y = 0.0
        table_pose.position.z = 0.17
        table.primitives.append(table_shape)
        table.primitive_poses.append(table_pose)
        table.operation = CollisionObject.ADD
        scene.world.collision_objects.append(table)

        # 2. Block
        block = CollisionObject()
        block.header.frame_id = 'world'
        block.id = 'block'
        block_shape = SolidPrimitive()
        block_shape.type = SolidPrimitive.BOX
        block_shape.dimensions = [0.05, 0.05, 0.05]
        block_pose = Pose()
        block_pose.position.x = 0.35
        block_pose.position.y = -0.40
        block_pose.position.z = 0.365
        block.primitives.append(block_shape)
        block.primitive_poses.append(block_pose)
        block.operation = CollisionObject.ADD
        scene.world.collision_objects.append(block)

        # 3. Tray
        tray = CollisionObject()
        tray.header.frame_id = 'world'
        tray.id = 'tray'
        tray_shape = SolidPrimitive()
        tray_shape.type = SolidPrimitive.BOX
        tray_shape.dimensions = [0.25, 0.25, 0.04]
        tray_pose = Pose()
        tray_pose.position.x = 1.11
        tray_pose.position.y = 0.36
        tray_pose.position.z = 0.36
        tray.primitives.append(tray_shape)
        tray.primitive_poses.append(tray_pose)
        tray.operation = CollisionObject.ADD
        scene.world.collision_objects.append(tray)

        self.publisher.publish(scene)

def main(args=None):
    rclpy.init(args=args)
    node = UR5ScenePublisher()
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
