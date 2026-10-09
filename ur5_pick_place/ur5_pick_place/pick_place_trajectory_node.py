"""Plan the complete named-state task before optionally executing it."""

import argparse
from copy import deepcopy
from datetime import datetime, timezone
import json
from pathlib import Path
import math
import signal
import sys
import time
import xml.etree.ElementTree as ET

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    AttachedCollisionObject, CollisionObject, DisplayTrajectory, MoveItErrorCodes,
    PlanningScene, PlanningSceneComponents,
    Constraints, PositionConstraint, OrientationConstraint, RobotTrajectory,
)
from moveit_msgs.srv import GetPlanningScene, GetPositionFK
import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint

from ur5_pick_place.task import (
    allow_contacts, compose, goal_constraints, gripper_links, inverse,
    named_states, sequence, update_robot_state,
)

GRASP_ORIENTATION = (1.0, 0.0, 0.0, 0.0)


class ActionFailure(RuntimeError):
    def __init__(self, label, status, code):
        self.code = code
        super().__init__(f'{label} failed: action status={status}, MoveIt code={code}')


class PickPlaceTrajectoryNode(Node):
    def __init__(self, srdf_file=None):
        super().__init__('pick_place_trajectory_node')
        defaults = {
            'execute': False,
            'arm_group': 'arm',
            'gripper_group': 'robotiq_gripper',
            'object_id': 'block',
            'attach_link': 'robotiq_85_base_link',
            'grasp_clearance': 0.03,
            'grasp_approach_height': 0.10,
            'release_retreat_height': 0.10,
            'cartesian_link': 'tool0',
            'support_surfaces': ['table', 'tray'],
            'move_group_node': '/move_group',
            'move_action': '/move_action',
            'execute_action': '/execute_trajectory',
            'planning_scene_service': '/get_planning_scene',
            'fk_service': '/compute_fk',
            'planning_pipeline': 'ompl',
            'linear_planning_pipeline': 'pilz_industrial_motion_planner',
            'linear_planner_id': 'LIN',
            'place_planning_pipeline': 'pilz_industrial_motion_planner',
            'place_planner_id': 'PTP',
            'planning_time': 5.0,
            'planning_attempts': 10,
            'planning_retry_attempts': 5,
            'arm_velocity_scaling': 0.2,
            'arm_acceleration_scaling': 0.2,
            'gripper_velocity_scaling': 0.3,
            'gripper_acceleration_scaling': 0.3,
            'joint_goal_tolerance': 0.0001,
            'verification_tolerance': 0.01,
            # The simulated arm controller is open loop, so measured joints
            # can settle slightly away from the commanded trajectory endpoint.
            'arm_verification_tolerance': 0.05,
            'verification_timeout': 10.0,
            'grasp_min_closure_fraction': 0.4,
            'grasp_contact_margin': 0.05,
            'grasp_stability_tolerance': 0.01,
            'grasp_confirmation_time': 0.5,
            'grasp_retention_tolerance': 0.10,
            'server_timeout': 15.0,
            # Gazebo can run below real time while MoveIt and collision checks
            # are active; keep the wall-clock timeout above a full transfer.
            'execution_timeout': 300.0,
            # Opt-in, observation-only experiment annotations.  Publishing
            # these messages does not add sleeps or alter trajectory timing.
            'experiment_logging': False,
            'experiment_run_id': '',
            'controller_mode': 'Position_OpenLoop',
            'trajectory_id': '',
            'payload_condition': 'block_0.1kg_contact_grasp',
            'block_mass_kg': 0.1,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        for name in ('arm_velocity_scaling', 'arm_acceleration_scaling',
                     'gripper_velocity_scaling', 'gripper_acceleration_scaling'):
            if not 0.0 < self.value(name) <= 1.0:
                raise ValueError(name + ' must be greater than 0 and at most 1')
        for name in ('planning_time', 'joint_goal_tolerance', 'verification_tolerance',
                     'arm_verification_tolerance', 'verification_timeout',
                     'grasp_contact_margin', 'grasp_stability_tolerance', 'grasp_confirmation_time',
                     'grasp_retention_tolerance', 'server_timeout', 'execution_timeout',
                     'planning_attempts', 'planning_retry_attempts'):
            if self.value(name) <= 0:
                raise ValueError(name + ' must be positive')
        if not 0.0 < self.value('grasp_min_closure_fraction') < 1.0:
            raise ValueError('grasp_min_closure_fraction must be greater than 0 and less than 1')
        if self.value('grasp_confirmation_time') >= self.value('verification_timeout'):
            raise ValueError('grasp_confirmation_time must be less than verification_timeout')
        self.srdf_file = srdf_file
        if self.value('grasp_clearance') < 0.0:
            raise ValueError('grasp_clearance must not be negative')
        for name in ('grasp_approach_height', 'release_retreat_height'):
            if self.value(name) <= 0.0:
                raise ValueError(name + ' must be positive')
        self.stages = sequence(self.value('arm_group'), self.value('gripper_group'))
        self.cartesian_stages = {
            'move_to_pre_grasp', 'descend_to_grasp', 'retreat_after_place'}
        self.periodic_joints = set()
        self.parameters_client = self.create_client(
            GetParameters, self.value('move_group_node').rstrip('/') + '/get_parameters')
        self.scene_client = self.create_client(GetPlanningScene, self.value('planning_scene_service'))
        self.fk_client = self.create_client(GetPositionFK, self.value('fk_service'))
        self.planner = ActionClient(self, MoveGroup, self.value('move_action'))
        self.executor_client = ActionClient(self, ExecuteTrajectory, self.value('execute_action'))
        self.preview = self.create_publisher(DisplayTrajectory, '~/display_planned_path', 10)
        self.experiment_events = self.create_publisher(
            String, '/trajectory_experiment/events', 10)
        self.active_goal = None
        self.active_stage_name = None

    def value(self, name):
        return self.get_parameter(name).value

    def emit_experiment_event(self, event, **fields):
        """Publish a JSON annotation with independent simulation/wall clocks."""
        try:
            enabled = bool(self.value('experiment_logging'))
        except Exception:
            enabled = False
        if not enabled:
            return
        stamp = self.get_clock().now().nanoseconds * 1e-9
        payload = {
            'schema_version': 1,
            'event': event,
            'sim_time_s': stamp,
            'recorded_at_utc': datetime.now(timezone.utc).isoformat(),
            'run_id': self.value('experiment_run_id'),
            'controller_mode': self.value('controller_mode'),
            'trajectory_id': self.value('trajectory_id'),
            'payload_condition': self.value('payload_condition'),
            'block_mass_kg': self.value('block_mass_kg'),
        }
        payload.update(fields)
        self.experiment_events.publish(String(data=json.dumps(payload, allow_nan=False)))

    def wait(self, future, timeout, operation):
        deadline = time.monotonic() + timeout
        while not future.done():
            if not rclpy.ok() or time.monotonic() >= deadline:
                raise TimeoutError(operation + ' timed out')
            rclpy.spin_once(self, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
        if future.exception() is not None:
            raise future.exception()
        return future.result()

    def cancel_active(self):
        if self.active_goal is not None and rclpy.ok():
            self.get_logger().warning('Cancelling the active action; no subsequent stage will run.')
            self.wait(self.active_goal.cancel_goal_async(), 5.0, 'Action cancellation')

    def action(self, client, goal, timeout, label):
        handle = self.wait(client.send_goal_async(goal), self.value('server_timeout'), label + ' acceptance')
        if not handle.accepted:
            raise RuntimeError(label + ' was rejected')
        if self.active_stage_name is not None:
            self.emit_experiment_event(
                'stage_accepted', stage=self.active_stage_name, action_label=label)
        self.active_goal = handle
        try:
            result = self.wait(handle.get_result_async(), timeout, label)
            self.active_goal = None
            code = result.result.error_code.val
            if result.status != GoalStatus.STATUS_SUCCEEDED or code != MoveItErrorCodes.SUCCESS:
                raise ActionFailure(label, result.status, code)
            if self.active_stage_name is not None:
                self.emit_experiment_event(
                    'controller_completed', stage=self.active_stage_name,
                    action_label=label, action_status=int(result.status),
                    moveit_error_code=int(code))
            return result.result
        except BaseException:
            try:
                self.cancel_active()
            except Exception as error:
                self.get_logger().error('Action cancellation failed: ' + str(error))
            self.active_goal = None
            raise

    def connect(self):
        timeout = self.value('server_timeout')
        if not self.parameters_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError('MoveIt robot-description parameter services are unavailable')
        for client, label in ((self.scene_client, 'planning scene'), (self.fk_client, 'forward kinematics')):
            if not client.wait_for_service(timeout_sec=timeout):
                raise RuntimeError(label + ' service is unavailable; start the existing bringup first')
        if not self.planner.wait_for_server(timeout_sec=timeout):
            raise RuntimeError('MoveGroup planning action is unavailable')
        if self.value('execute') and not self.executor_client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError('Trajectory execution action is unavailable')
        remote = self.value('move_group_node').strip('/')
        matches = [name for name, namespace in self.get_node_names_and_namespaces()
                   if (namespace.rstrip('/') + '/' + name).strip('/') == remote]
        if len(matches) > 1 or self.count_publishers('/clock') > 1:
            raise RuntimeError('Multiple bringup instances detected (duplicate MoveGroup or clocks). '
                               'Keep only one existing bringup instance before starting the task.')
        response = self.wait(self.parameters_client.call_async(GetParameters.Request(
            names=['robot_description', 'robot_description_semantic'])), timeout, 'Read robot model')
        self.urdf, srdf = (p.string_value for p in response.values)
        if self.srdf_file:
            srdf = Path(self.srdf_file).read_text()
        if not self.urdf or not srdf:
            raise RuntimeError('MoveIt did not provide the robot model and named states')
        self.periodic_joints = {
            joint.attrib['name'] for joint in ET.fromstring(self.urdf).findall('joint')
            if joint.attrib.get('type') in ('revolute', 'continuous')
        }
        self.states = named_states(srdf)
        self.touch_links = gripper_links(self.urdf, self.value('attach_link'))
        for stage in self.stages:
            if stage.name not in self.cartesian_stages and (stage.group, stage.state) not in self.states:
                raise ValueError(f'Missing SRDF group state: {stage.group}/{stage.state}')

    def get_scene(self):
        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.ROBOT_STATE
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
            | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
            | PlanningSceneComponents.TRANSFORMS
        )
        return self.wait(self.scene_client.call_async(request), self.value('server_timeout'),
                         'Read planning scene').scene

    def fk(self, state, frame, link=None):
        request = GetPositionFK.Request()
        request.header.frame_id = frame
        request.fk_link_names = [link or self.value('attach_link')]
        request.robot_state = deepcopy(state)
        response = self.wait(self.fk_client.call_async(request), self.value('server_timeout'),
                             'Compute gripper pose')
        if response.error_code.val != MoveItErrorCodes.SUCCESS or len(response.pose_stamped) != 1:
            raise RuntimeError('Could not compute the attachment-link pose')
        return response.pose_stamped[0].pose

    def make_plan_goal(self, stage, state, scene_diff):
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = stage.group
        request.pipeline_id = self.value('planning_pipeline')
        request.num_planning_attempts = self.value('planning_attempts')
        request.allowed_planning_time = self.value('planning_time')
        prefix = 'arm' if stage.group == self.value('arm_group') else 'gripper'
        request.max_velocity_scaling_factor = self.value(prefix + '_velocity_scaling')
        request.max_acceleration_scaling_factor = self.value(prefix + '_acceleration_scaling')
        request.start_state = deepcopy(state)
        request.start_state.is_diff = False
        if stage.name in self.cartesian_stages:
            target = self.cartesian_targets[stage.name]
            position = PositionConstraint(link_name=self.value('cartesian_link'), weight=1.0)
            position.header.frame_id = self.cartesian_frame
            region_pose = Pose()
            region_pose.position = deepcopy(target.position)
            region_pose.orientation.w = 1.0
            position.constraint_region.primitives = [SolidPrimitive(
                type=SolidPrimitive.SPHERE, dimensions=[0.001])]
            position.constraint_region.primitive_poses = [region_pose]
            orientation = OrientationConstraint(
                link_name=self.value('cartesian_link'), orientation=target.orientation,
                absolute_x_axis_tolerance=0.001, absolute_y_axis_tolerance=0.001,
                absolute_z_axis_tolerance=0.001, weight=1.0)
            orientation.header.frame_id = self.cartesian_frame
            request.goal_constraints = [Constraints(
                name=stage.state, position_constraints=[position],
                orientation_constraints=[orientation])]
            if stage.name in ('descend_to_grasp', 'retreat_after_place'):
                request.pipeline_id = self.value('linear_planning_pipeline')
                request.planner_id = self.value('linear_planner_id')
            elif stage.name == 'move_to_place':
                request.pipeline_id = self.value('place_planning_pipeline')
                request.planner_id = self.value('place_planner_id')
        else:
            request.goal_constraints = [goal_constraints(
                stage.state, self.states[(stage.group, stage.state)], self.value('joint_goal_tolerance'))]
        goal.planning_options.plan_only = True
        goal.planning_options.replan = False
        goal.planning_options.planning_scene_diff = deepcopy(scene_diff)
        return goal

    def plan_all(self, scene):
        object_id = self.value('object_id')
        required = {joint for stage in self.stages if stage.name not in self.cartesian_stages
                    for joint in self.states[(stage.group, stage.state)]}
        present = set(scene.robot_state.joint_state.name)
        if not required <= present:
            raise RuntimeError('Robot joint feedback is incomplete: ' + str(sorted(required - present)))
        if any(a.object.id == object_id for a in scene.robot_state.attached_collision_objects):
            raise RuntimeError(object_id + ' is already attached; the task requires an ungrasped box')
        original = next((deepcopy(o) for o in scene.world.collision_objects if o.id == object_id), None)
        if original is None:
            raise RuntimeError('The planning scene has no object named ' + object_id)
        if not original.header.frame_id:
            raise RuntimeError('The box has no reference frame')
        state = deepcopy(scene.robot_state)
        state.is_diff = False
        pick_state = deepcopy(state)
        pick_positions = dict(zip(pick_state.joint_state.name,
                                  pick_state.joint_state.position))
        pick_positions.update(self.states[(self.value('arm_group'), 'pick')])
        pick_state.joint_state.name = list(pick_positions)
        pick_state.joint_state.position = list(pick_positions.values())
        self.cartesian_frame = original.header.frame_id
        grasp_pose = self.fk(pick_state, self.cartesian_frame, self.value('cartesian_link'))
        grasp_pose.position.z += self.value('grasp_clearance')
        (grasp_pose.orientation.x, grasp_pose.orientation.y,
         grasp_pose.orientation.z, grasp_pose.orientation.w) = GRASP_ORIENTATION
        pre_grasp_pose = deepcopy(grasp_pose)
        pre_grasp_pose.position.z += self.value('grasp_approach_height')
        place_state = deepcopy(state)
        place_positions = dict(zip(
            place_state.joint_state.name, place_state.joint_state.position))
        place_positions.update(self.states[(self.value('arm_group'), 'place')])
        place_state.joint_state.name = list(place_positions)
        place_state.joint_state.position = list(place_positions.values())
        retreat_pose = self.fk(
            place_state, self.cartesian_frame, self.value('cartesian_link'))
        retreat_pose.position.z += self.value('release_retreat_height')
        self.cartesian_targets = {
            'move_to_pre_grasp': pre_grasp_pose,
            'descend_to_grasp': grasp_pose,
            'retreat_after_place': retreat_pose,
        }
        attachment = None
        released = None
        plans = []
        for index, stage in enumerate(self.stages, start=1):
            diff = PlanningScene(is_diff=True)
            diff.robot_state.is_diff = True
            if stage.name == 'grasp':
                held = deepcopy(original)
                held.header.frame_id = self.value('attach_link')
                held.pose = compose(inverse(self.fk(state, original.header.frame_id)), original.pose)
                held.operation = CollisionObject.ADD
                attachment = AttachedCollisionObject(
                    link_name=self.value('attach_link'), object=held, touch_links=self.touch_links)
                state.attached_collision_objects.append(deepcopy(attachment))
            elif stage.name == 'release':
                released = deepcopy(original)
                released.pose = compose(self.fk(state, original.header.frame_id), attachment.object.pose)
                released.operation = CollisionObject.ADD
                state.attached_collision_objects = [
                    a for a in state.attached_collision_objects if a.object.id != object_id]
                attachment = None
            if attachment is not None:
                # Attaching the same ID removes its world representation in
                # this request's scene; a second REMOVE is redundant.
                diff.robot_state.attached_collision_objects = [deepcopy(attachment)]
            elif released is not None:
                diff.world.collision_objects = [deepcopy(released)]
            # Keep the long move to pre-grasp collision-free. Contact is only
            # permitted for the controlled vertical descent and grasp.
            contacts = []
            if stage.name == 'descend_to_grasp':
                contacts.extend(self.touch_links)
            if stage.name in ('grasp', 'move_to_place'):
                contacts.extend(self.touch_links)
                contacts.extend(self.value('support_surfaces'))
            if released is not None:
                contacts.extend(self.value('support_surfaces'))
                if stage.name in ('release', 'retreat_after_place'):
                    contacts.extend(self.touch_links)
            diff.allowed_collision_matrix = allow_contacts(
                scene.allowed_collision_matrix, object_id, contacts)
            # Collision checking must use the hypothetical gripper state from
            # the preceding plan, rather than the still-unmoved live gripper.
            diff.robot_state.joint_state = deepcopy(state.joint_state)
            self.get_logger().info(f'Planning {index}/{len(self.stages)}: {stage.group}/{stage.state}')
            start = deepcopy(state)
            for attempt in range(self.value('planning_retry_attempts')):
                try:
                    result = self.action(self.planner, self.make_plan_goal(stage, state, diff),
                                         self.value('planning_time') + self.value('server_timeout'),
                                         'Plan ' + stage.name)
                    break
                except ActionFailure as error:
                    if (error.code not in (MoveItErrorCodes.PLANNING_FAILED,
                                           MoveItErrorCodes.INVALID_MOTION_PLAN)
                            or attempt + 1 == self.value('planning_retry_attempts')):
                        raise
                    self.get_logger().warning(
                        f'Replanning {stage.name}: candidate {attempt + 1} was rejected by MoveIt.')
            trajectory = result.planned_trajectory
            state = update_robot_state(state, trajectory, self.urdf)
            if stage.name not in self.cartesian_stages:
                self.verify_positions(
                    state, self.states[(stage.group, stage.state)],
                    self.value('joint_goal_tolerance') + 1e-6, 'Planned ' + stage.name,
                    getattr(self, 'periodic_joints', set()))
            plans.append((stage, trajectory, result.trajectory_start, start))
        return plans

    @staticmethod
    def verify_positions(state, targets, tolerance, label, periodic_joints=None):
        measured = dict(zip(state.joint_state.name, state.joint_state.position))
        periodic_joints = periodic_joints or set()
        errors = {
            joint: (abs(math.atan2(math.sin(measured[joint] - target),
                                   math.cos(measured[joint] - target)))
                    if joint in periodic_joints else abs(measured[joint] - target))
            if joint in measured else float('inf')
                  for joint, target in targets.items()}
        wrong = {joint: error for joint, error in errors.items()
                 if not math.isfinite(error) or error > tolerance}
        if wrong:
            raise RuntimeError(f'{label} joint positions differ from the target: {wrong}')

    @staticmethod
    def contact_grasp_positions(state, open_targets, close_targets,
                                min_closure_fraction, contact_margin):
        """Return measured contact positions or reject an absent/invalid grasp.

        A grasp must close far enough to reach the object, but remain far enough
        from the fully closed target to demonstrate that something stopped the
        fingers. This is suitable for the open-loop simulated gripper, whose
        trajectory controller cannot report contact itself.
        """
        measured = dict(zip(state.joint_state.name, state.joint_state.position))
        positions = {}
        progress = {}
        problems = {}
        for joint, closed in close_targets.items():
            opened = open_targets.get(joint)
            actual = measured.get(joint)
            if opened is None:
                problems[joint] = 'missing from the open state'
                continue
            if actual is None or not all(math.isfinite(value) for value in (opened, closed, actual)):
                problems[joint] = 'missing or non-finite feedback'
                continue
            travel = closed - opened
            if abs(travel) <= 1e-9:
                problems[joint] = 'open and close targets are identical'
                continue
            closure = (actual - opened) / travel
            remaining = abs(closed - actual)
            if closure < min_closure_fraction:
                problems[joint] = f'only {closure:.1%} closed'
            elif closure > 1.0 + 1e-3:
                problems[joint] = f'past the close target ({closure:.1%})'
            elif remaining < contact_margin:
                problems[joint] = 'reached the fully closed position; no object contact detected'
            positions[joint] = actual
            progress[joint] = closure
        if problems:
            raise RuntimeError('Grasp was not confirmed: ' + str(problems))
        return positions, progress

    def verify_live_grasp(self):
        open_targets = self.states[(self.value('gripper_group'), 'open')]
        close_targets = self.states[(self.value('gripper_group'), 'close')]
        confirmation_time = self.value('grasp_confirmation_time')
        deadline = time.monotonic() + self.value('verification_timeout')
        samples = []
        last_error = None
        while True:
            now = time.monotonic()
            try:
                positions, progress = self.contact_grasp_positions(
                    self.get_scene().robot_state, open_targets, close_targets,
                    self.value('grasp_min_closure_fraction'), self.value('grasp_contact_margin'))
                samples.append((now, positions))
                samples = [sample for sample in samples if now - sample[0] <= confirmation_time]
                if samples and now - samples[0][0] >= confirmation_time * 0.95:
                    movement = {
                        joint: max(sample[1][joint] for sample in samples)
                        - min(sample[1][joint] for sample in samples)
                        for joint in positions
                    }
                    if all(delta <= self.value('grasp_stability_tolerance')
                           for delta in movement.values()):
                        self.grasp_positions = positions
                        summary = ', '.join(
                            f'{joint}={positions[joint]:.3f} ({progress[joint]:.0%} closed)'
                            for joint in positions)
                        self.get_logger().info('Stable object contact confirmed: ' + summary)
                        return
                    last_error = RuntimeError('Grasp contact is not stable: ' + str(movement))
            except RuntimeError as error:
                samples = []
                last_error = error
            if time.monotonic() >= deadline:
                raise last_error or RuntimeError('Grasp was not confirmed before the timeout')
            rclpy.spin_once(self, timeout_sec=0.05)

    def hold_grasp_contact(self):
        if not getattr(self, 'grasp_positions', None):
            raise RuntimeError('Cannot hold the grasp: no measured contact position is available')
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.joint_names = list(self.grasp_positions)
        point = JointTrajectoryPoint()
        point.positions = list(self.grasp_positions.values())
        point.time_from_start = Duration(sec=1)
        trajectory.joint_trajectory.points = [point]
        self.get_logger().info(
            'Holding the gripper at the measured contact position before carrying.')
        self.action(
            self.executor_client, ExecuteTrajectory.Goal(trajectory=trajectory),
            self.value('execution_timeout'), 'Hold grasp contact')
        self.verify_live_targets(
            self.grasp_positions, 'Held grasp contact',
            self.value('verification_tolerance'))

    @staticmethod
    def release_from_contact(trajectory, contact_positions, open_targets, close_targets):
        """Scale a planned fully-closed-to-open path to start at contact.

        The planned path is collision checked from the configured close state.
        A contact-limited grasp never reaches that state, so execution uses the
        monotonic subset between the measured contact position and open.
        """
        result = deepcopy(trajectory)
        joint_names = result.joint_trajectory.joint_names
        if not result.joint_trajectory.points:
            raise RuntimeError('Release trajectory is empty')
        factors = {}
        for joint in joint_names:
            if joint not in contact_positions or joint not in open_targets or joint not in close_targets:
                raise RuntimeError('Release trajectory has no grasp state for joint ' + joint)
            travel = close_targets[joint] - open_targets[joint]
            if abs(travel) <= 1e-9:
                raise RuntimeError('Release joint has identical open and close targets: ' + joint)
            factor = (contact_positions[joint] - open_targets[joint]) / travel
            if not 0.0 <= factor <= 1.0:
                raise RuntimeError(f'Release contact position is outside the planned path: {joint}')
            factors[joint] = factor
        for point in result.joint_trajectory.points:
            for index, joint in enumerate(joint_names):
                opened = open_targets[joint]
                closed = close_targets[joint]
                fraction = (point.positions[index] - opened) / (closed - opened)
                if not -1e-3 <= fraction <= 1.0 + 1e-3:
                    raise RuntimeError(f'Release trajectory leaves the open/close interval: {joint}')
                point.positions[index] = opened + fraction * (
                    contact_positions[joint] - opened)
                if point.velocities:
                    point.velocities[index] *= factors[joint]
                if point.accelerations:
                    point.accelerations[index] *= factors[joint]
        return result

    def verify_live_targets(self, targets, label, tolerance=None):
        deadline = time.monotonic() + self.value('verification_timeout')
        tolerance = self.value('verification_tolerance') if tolerance is None else tolerance
        while True:
            try:
                state = self.get_scene().robot_state
                self.verify_positions(
                    state, targets, tolerance, label,
                    getattr(self, 'periodic_joints', set()))
                measured = dict(zip(state.joint_state.name, state.joint_state.position))
                return {joint: measured[joint] for joint in targets}
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise
                rclpy.spin_once(self, timeout_sec=0.05)

    def stage_verification_tolerance(self, stage):
        if stage.group == self.value('arm_group'):
            return self.value('arm_verification_tolerance')
        return self.value('verification_tolerance')

    def publish_plan_preview(self, plans, initial_scene):
        pass

    def prepare_plans(self, initial_scene):
        return self.plan_all(initial_scene)

    def run(self):
        self.connect()
        initial_scene = self.get_scene()
        plans = self.prepare_plans(initial_scene)
        self.get_logger().info(f'All {len(plans)} trajectories planned successfully.')
        self.publish_plan_preview(plans, initial_scene)
        if not self.value('execute'):
            self.get_logger().info('Planning only: no trajectory execution requests were sent. '
                                   'Use --ros-args -p execute:=true to run the task.')
            return
        self.emit_experiment_event(
            'task_started', stage_count=len(plans),
            arm_velocity_scaling=self.value('arm_velocity_scaling'),
            arm_acceleration_scaling=self.value('arm_acceleration_scaling'))
        try:
            # A concurrent RViz command must not invalidate the reviewed
            # trajectory starts.
            initial_targets = dict(zip(
                initial_scene.robot_state.joint_state.name,
                initial_scene.robot_state.joint_state.position))
            self.verify_live_targets(initial_targets, 'Robot changed during planning')
            for index, (stage, trajectory, trajectory_start, start) in enumerate(
                    plans, start=1):
                self.active_stage_name = stage.name
                try:
                    stage_tolerance = self.stage_verification_tolerance(stage)
                    execution_trajectory = trajectory
                    preview_start = trajectory_start
                    if stage.name == 'release':
                        current_grasp_positions = self.verify_live_targets(
                            self.grasp_positions, 'Start of release',
                            self.value('grasp_retention_tolerance'))
                        self.grasp_positions = current_grasp_positions
                        execution_trajectory = self.release_from_contact(
                            trajectory, current_grasp_positions,
                            self.states[(self.value('gripper_group'), 'open')],
                            self.states[(self.value('gripper_group'), 'close')])
                        preview_start = deepcopy(trajectory_start)
                        preview_positions = dict(zip(
                            preview_start.joint_state.name,
                            preview_start.joint_state.position))
                        preview_positions.update(current_grasp_positions)
                        preview_start.joint_state.name = list(preview_positions)
                        preview_start.joint_state.position = list(preview_positions.values())
                    joints = execution_trajectory.joint_trajectory.joint_names
                    start_positions = dict(zip(
                        start.joint_state.name, start.joint_state.position))
                    if stage.name != 'release':
                        self.verify_live_targets(
                            {joint: start_positions[joint] for joint in joints},
                            'Start of ' + stage.name, stage_tolerance)
                    points = execution_trajectory.joint_trajectory.points
                    duration = 0.0
                    if points:
                        duration = (points[-1].time_from_start.sec
                                    + points[-1].time_from_start.nanosec * 1e-9)
                    self.emit_experiment_event(
                        'stage_submitted', stage=stage.name, stage_index=index,
                        group=stage.group, state=stage.state,
                        planned_duration_s=duration,
                        joint_names=list(joints), waypoint_count=len(points))
                    self.get_logger().info(
                        f'Executing {index}/{len(plans)}: {stage.group}/{stage.state}')
                    self.preview.publish(DisplayTrajectory(
                        model_id=initial_scene.robot_model_name,
                        trajectory_start=preview_start,
                        trajectory=[execution_trajectory]))
                    self.action(
                        self.executor_client,
                        ExecuteTrajectory.Goal(trajectory=execution_trajectory),
                        self.value('execution_timeout'), 'Execute ' + stage.name)
                    targets = (
                        dict(zip(joints, execution_trajectory.joint_trajectory.points[-1].positions))
                        if stage.name in self.cartesian_stages
                        else self.states[(stage.group, stage.state)])
                    if stage.name == 'grasp':
                        self.verify_live_grasp()
                        self.hold_grasp_contact()
                    else:
                        self.verify_live_targets(
                            targets, 'Executed ' + stage.name, stage_tolerance)
                    if stage.name == 'move_to_place':
                        self.grasp_positions = self.verify_live_targets(
                            self.grasp_positions, 'Grasp retained at place',
                            self.value('grasp_retention_tolerance'))
                    self.emit_experiment_event(
                        'stage_completed', stage=stage.name, stage_index=index,
                        verification='passed')
                except BaseException as error:
                    self.emit_experiment_event(
                        'stage_failed', stage=stage.name, stage_index=index,
                        error_type=type(error).__name__, error=str(error))
                    raise
                finally:
                    self.active_stage_name = None
        except BaseException as error:
            self.emit_experiment_event(
                'task_failed', error_type=type(error).__name__, error=str(error))
            raise
        self.emit_experiment_event('task_completed', execution_status='success')
        self.get_logger().info(
            'Pick and place trajectory sequence completed; gripper is open and arm is straight.')


def main(args=None, node_factory=PickPlaceTrajectoryNode):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dry-run', action='store_true',
                        help='Print the existing named-state targets without connecting to ROS.')
    parser.add_argument('--srdf', type=Path, help='Read named states from this SRDF file.')
    cli = parser.parse_args(remove_ros_args(args=sys.argv if args is None else ['pick_place_node', *args])[1:])
    if cli.dry_run:
        path = cli.srdf or Path(get_package_share_directory('ur5_moveit_config')) / 'config/ur5.srdf'
        states = named_states(path.read_text())
        for index, stage in enumerate(sequence(), start=1):
            if stage.name == 'move_to_pre_grasp':
                print(f'{index}. {stage.name}: arm/tool0: pick +0.13 m in Z, vertical-down gripper')
                continue
            if stage.name == 'descend_to_grasp':
                print(f'{index}. {stage.name}: arm/tool0: linear descent to pick +0.03 m in Z')
                continue
            if stage.name == 'retreat_after_place':
                print(f'{index}. {stage.name}: arm/tool0: linear vertical retreat +0.10 m')
                continue
            targets = states[(stage.group, stage.state)]
            print(f'{index}. {stage.name}: {stage.group}/{stage.state}: {targets}')
        return 0
    # Keep the ROS context alive while cancelling an action on Ctrl-C/SIGTERM.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    def interrupt(signum, frame):
        raise KeyboardInterrupt

    previous_handlers = {number: signal.signal(number, interrupt)
                         for number in (signal.SIGINT, signal.SIGTERM)}
    node = None
    try:
        node = node_factory(cli.srdf)
        node.run()
        return 0
    except (KeyboardInterrupt, ExternalShutdownException):
        if node is not None and rclpy.ok():
            node.cancel_active()
        return 130
    except Exception as error:
        if node is not None:
            node.get_logger().error(str(error) + '; stopping the sequence.')
        else:
            print(str(error), file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for number, handler in previous_handlers.items():
            signal.signal(number, handler)


if __name__ == '__main__':
    raise SystemExit(main())
