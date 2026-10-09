from copy import deepcopy
import math
from types import SimpleNamespace
from unittest.mock import Mock

from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    AllowedCollisionEntry, AllowedCollisionMatrix, CollisionObject, PlanningScene,
    RobotState, RobotTrajectory,
)
import pytest
from trajectory_msgs.msg import JointTrajectoryPoint

from ur5_pick_place.pick_place_trajectory_node import PickPlaceTrajectoryNode, ActionFailure
from ur5_pick_place.pick_place_trajectory_node import GRASP_ORIENTATION
from ur5_pick_place.task import (
    allow_contacts, compose, inverse, named_states, sequence, update_robot_state,
)


SRDF = '''<robot name="test">
<group_state group="arm" name="straight"><joint name="arm_joint" value="0.5"/></group_state>
<group_state group="arm" name="pick"><joint name="arm_joint" value="1"/></group_state>
<group_state group="arm" name="place"><joint name="arm_joint" value="2"/></group_state>
<group_state group="robotiq_gripper" name="open"><joint name="finger_joint" value="0"/></group_state>
<group_state group="robotiq_gripper" name="close"><joint name="finger_joint" value="0.8"/></group_state>
</robot>'''
URDF = '''<robot name="test">
<link name="gripper"/><link name="left"/><link name="right"/>
<joint name="finger_joint" type="revolute"><parent link="gripper"/><child link="left"/></joint>
<joint name="right_joint" type="revolute"><parent link="gripper"/><child link="right"/>
<mimic joint="finger_joint" multiplier="-1"/></joint></robot>'''


def scene():
    result = PlanningScene()
    result.robot_state = RobotState()
    result.robot_state.joint_state.name = ['arm_joint', 'finger_joint', 'right_joint']
    result.robot_state.joint_state.position = [0.0, 0.4, -0.4]
    box = CollisionObject(id='block')
    box.header.frame_id = 'world'
    box.pose.position.x = 1.0
    box.pose.position.z = 1.0
    box.pose.orientation.w = 1.0
    result.world.collision_objects = [box]
    return result


class FakeTask(PickPlaceTrajectoryNode):
    """No ROS initialization, action servers, controller commands, or scene writes."""

    def __init__(self, execute=False, fail_stage=None, fail_execution=None, grasp_position=0.4):
        self.config = {
            'execute': execute, 'object_id': 'block', 'attach_link': 'gripper',
            'cartesian_link': 'tool0', 'grasp_clearance': 0.03,
            'grasp_approach_height': 0.1, 'release_retreat_height': 0.1,
            'support_surfaces': ['table', 'tray'], 'arm_group': 'arm',
            'gripper_group': 'robotiq_gripper', 'planning_pipeline': 'ompl',
            'linear_planning_pipeline': 'pilz_industrial_motion_planner',
            'linear_planner_id': 'LIN',
            'place_planning_pipeline': 'pilz_industrial_motion_planner',
            'place_planner_id': 'PTP',
            'planning_attempts': 10, 'planning_time': 5.0, 'server_timeout': 15.0,
            'planning_retry_attempts': 1,
            'joint_goal_tolerance': 0.0001, 'verification_tolerance': 0.01,
            'arm_verification_tolerance': 0.05,
            'verification_timeout': 10.0,
            'grasp_min_closure_fraction': 0.4, 'grasp_contact_margin': 0.05,
            'grasp_stability_tolerance': 0.01, 'grasp_confirmation_time': 0.5,
            'grasp_retention_tolerance': 0.10,
            'execution_timeout': 120.0,
            'arm_velocity_scaling': 0.2, 'arm_acceleration_scaling': 0.2,
            'gripper_velocity_scaling': 0.3, 'gripper_acceleration_scaling': 0.3,
        }
        self.urdf = URDF
        self.states = named_states(SRDF)
        self.touch_links = ['gripper', 'left', 'right']
        self.stages = sequence()
        self.cartesian_stages = {
            'move_to_pre_grasp', 'descend_to_grasp', 'retreat_after_place'}
        self.planner = object()
        self.executor_client = object()
        self.preview = Mock()
        self.logger = Mock()
        self.live_scene = scene()
        self.plan_goals = []
        self.executed = []
        self.fail_stage = fail_stage
        self.fail_execution = fail_execution
        self.grasp_position = grasp_position

    def value(self, name):
        return self.config[name]

    def get_logger(self):
        return self.logger

    def connect(self):
        pass

    def get_scene(self):
        return deepcopy(self.live_scene)

    def fk(self, state, frame, link=None):
        pose = Pose()
        pose.position.x = dict(zip(state.joint_state.name, state.joint_state.position))['arm_joint']
        pose.position.z = 1.0
        pose.orientation.w = 1.0
        return pose

    def verify_live_grasp(self):
        positions, _ = self.contact_grasp_positions(
            self.live_scene.robot_state,
            self.states[(self.value('gripper_group'), 'open')],
            self.states[(self.value('gripper_group'), 'close')],
            self.value('grasp_min_closure_fraction'), self.value('grasp_contact_margin'))
        self.grasp_positions = positions

    def verify_live_targets(self, targets, label, tolerance=None):
        self.verify_positions(
            self.live_scene.robot_state, targets,
            self.value('verification_tolerance') if tolerance is None else tolerance, label)
        measured = dict(zip(self.live_scene.robot_state.joint_state.name,
                            self.live_scene.robot_state.joint_state.position))
        return {joint: measured[joint] for joint in targets}

    def action(self, client, goal, timeout, label):
        if client is self.executor_client:
            self.executed.append(goal)
            if len(self.executed) == self.fail_execution:
                raise RuntimeError('Controller failure')
            self.live_scene.robot_state = update_robot_state(
                self.live_scene.robot_state, goal.trajectory, self.urdf)
            if label == 'Execute grasp':
                positions = dict(zip(self.live_scene.robot_state.joint_state.name,
                                     self.live_scene.robot_state.joint_state.position))
                positions['finger_joint'] = self.grasp_position
                positions['right_joint'] = -self.grasp_position
                self.live_scene.robot_state.joint_state.name = list(positions)
                self.live_scene.robot_state.joint_state.position = list(positions.values())
            return SimpleNamespace()
        self.plan_goals.append(deepcopy(goal))
        if len(self.plan_goals) == self.fail_stage:
            raise RuntimeError('Planner failure')
        constraints = goal.request.goal_constraints[0]
        joints = constraints.joint_constraints
        trajectory = RobotTrajectory()
        if joints:
            trajectory.joint_trajectory.joint_names = [j.joint_name for j in joints]
            positions = [j.position for j in joints]
        else:
            trajectory.joint_trajectory.joint_names = ['arm_joint']
            positions = [constraints.position_constraints[0].constraint_region.primitive_poses[0].position.x]
        trajectory.joint_trajectory.points = [JointTrajectoryPoint(positions=positions)]
        return SimpleNamespace(planned_trajectory=trajectory,
                               trajectory_start=deepcopy(goal.request.start_state))


def test_periodic_joint_verification_accepts_equivalent_revolutions():
    state = RobotState()
    state.joint_state.name = ['joint']
    state.joint_state.position = [0.0]
    PickPlaceTrajectoryNode.verify_positions(
        state, {'joint': 2 * math.pi}, 1e-6, 'Test', {'joint'})


def test_sequence_starts_and_ends_straight_and_keeps_gripper_open_for_pick():
    assert [(s.group, s.state) for s in sequence()] == [
        ('arm', 'straight'), ('robotiq_gripper', 'open'),
        ('arm', 'pre_grasp'), ('arm', 'grasp_pose'),
        ('robotiq_gripper', 'close'), ('arm', 'place'),
        ('robotiq_gripper', 'open'), ('arm', 'retreat_pose'), ('arm', 'straight')]


def test_plan_only_never_executes_and_does_not_change_live_scene():
    task = FakeTask()
    before = deepcopy(task.live_scene)
    task.run()
    assert len(task.plan_goals) == 9
    assert all(g.planning_options.plan_only for g in task.plan_goals)
    assert task.executed == []
    assert task.live_scene == before


def test_attachment_is_local_and_release_pose_follows_the_gripper():
    task = FakeTask()
    original = scene()
    task.plan_all(original)
    grasp, carry, release = task.plan_goals[4:7]
    attached = carry.request.start_state.attached_collision_objects[0]
    assert attached.link_name == 'gripper'
    assert attached.object.pose.position.x == pytest.approx(0.0)
    assert grasp.planning_options.planning_scene_diff.robot_state.attached_collision_objects[0].object.id == 'block'
    assert grasp.planning_options.planning_scene_diff.world.collision_objects == []
    released = release.planning_options.planning_scene_diff.world.collision_objects[0]
    assert released.id == 'block'
    assert released.pose.position.x == pytest.approx(2.0)
    assert release.request.start_state.attached_collision_objects == []
    assert original == scene()


def test_pre_grasp_avoids_box_and_descent_allows_only_finger_contact():
    task = FakeTask()
    task.plan_all(scene())
    pre_grasp = task.plan_goals[2].planning_options.planning_scene_diff.allowed_collision_matrix
    box = pre_grasp.entry_names.index('block')
    for link in task.touch_links:
        if link in pre_grasp.entry_names:
            index = pre_grasp.entry_names.index(link)
            assert not pre_grasp.entry_values[box].enabled[index]

    descent = task.plan_goals[3].planning_options.planning_scene_diff
    matrix = descent.allowed_collision_matrix
    box = matrix.entry_names.index('block')
    for link in task.touch_links:
        index = matrix.entry_names.index(link)
        assert matrix.entry_values[box].enabled[index]
        assert matrix.entry_values[index].enabled[box]
    assert 'table' not in matrix.entry_names
    assert descent.world.collision_objects == []
    assert descent.robot_state.attached_collision_objects == []
    positions = dict(zip(descent.robot_state.joint_state.name,
                         descent.robot_state.joint_state.position))
    assert positions['finger_joint'] == pytest.approx(0.0)
    assert task.plan_goals[3].request.pipeline_id == 'pilz_industrial_motion_planner'
    assert task.plan_goals[3].request.planner_id == 'LIN'


def test_cartesian_approach_keeps_orientation_and_saved_pick_state_unchanged():
    task = FakeTask()
    states_before = deepcopy(task.states)
    task.cartesian_frame = 'world'
    target = Pose()
    target.position.z = 1.12
    target.orientation.w = 1.0
    task.cartesian_targets = {'move_to_pre_grasp': target, 'descend_to_grasp': target}
    approach = sequence()[2]
    goal = task.make_plan_goal(approach, scene().robot_state, PlanningScene(is_diff=True))
    constraints = goal.request.goal_constraints[0]
    assert constraints.position_constraints[0].constraint_region.primitive_poses[0].position.z == 1.12
    assert constraints.orientation_constraints[0].orientation.w == 1.0
    assert goal.planning_options.plan_only
    assert task.states == states_before


def test_grasp_target_uses_the_collision_free_table_clearance():
    task = FakeTask()
    task.plan_all(scene())

    assert task.cartesian_targets['descend_to_grasp'].position.z == pytest.approx(1.03)
    orientation = task.cartesian_targets['descend_to_grasp'].orientation
    assert (orientation.x, orientation.y, orientation.z, orientation.w) == GRASP_ORIENTATION
    assert task.cartesian_targets['move_to_pre_grasp'].orientation == orientation


def test_release_retreat_is_linear_and_keeps_only_safe_release_contacts():
    task = FakeTask()
    task.plan_all(scene())

    retreat = task.plan_goals[7]
    retreat_matrix = retreat.planning_options.planning_scene_diff.allowed_collision_matrix
    block = retreat_matrix.entry_names.index('block')
    for link in task.touch_links:
        index = retreat_matrix.entry_names.index(link)
        assert retreat_matrix.entry_values[block].enabled[index]
        assert retreat_matrix.entry_values[index].enabled[block]
    assert retreat.request.pipeline_id == 'pilz_industrial_motion_planner'
    assert retreat.request.planner_id == 'LIN'

    return_goal = task.plan_goals[8]
    return_matrix = return_goal.planning_options.planning_scene_diff.allowed_collision_matrix
    block = return_matrix.entry_names.index('block')
    tray = return_matrix.entry_names.index('tray')
    assert return_matrix.entry_values[block].enabled[tray]
    assert return_matrix.entry_values[tray].enabled[block]
    for link in task.touch_links:
        if link in return_matrix.entry_names:
            index = return_matrix.entry_names.index(link)
            assert not return_matrix.entry_values[block].enabled[index]
            assert not return_matrix.entry_values[index].enabled[block]


def test_rejected_motion_plan_is_replanned_before_any_execution():
    task = FakeTask()
    task.config['planning_retry_attempts'] = 2
    original_action = task.action
    rejected = []

    def transient_failure(client, goal, timeout, label):
        if label == 'Plan move_to_pre_grasp' and not rejected:
            rejected.append(label)
            raise ActionFailure(label, 6, -2)
        return original_action(client, goal, timeout, label)

    task.action = transient_failure
    task.run()
    assert rejected == ['Plan move_to_pre_grasp']
    assert len(task.plan_goals) == 9
    assert not task.executed


def test_all_plans_finish_before_execution_and_final_state_is_straight_open():
    task = FakeTask(execute=True)
    task.run()
    assert len(task.plan_goals) == 9
    assert len(task.executed) == 10
    held = task.executed[5].trajectory.joint_trajectory
    assert held.joint_names == list(task.grasp_positions)
    assert held.points[-1].positions == pytest.approx(list(task.grasp_positions.values()))
    positions = dict(zip(task.live_scene.robot_state.joint_state.name,
                         task.live_scene.robot_state.joint_state.position))
    assert positions == {'arm_joint': 0.5, 'finger_joint': 0.0, 'right_joint': 0.0}


def test_arm_execution_allows_simulation_tracking_error_without_relaxing_gripper():
    task = FakeTask()
    assert task.stage_verification_tolerance(sequence()[0]) == pytest.approx(0.05)
    assert task.stage_verification_tolerance(sequence()[1]) == pytest.approx(0.01)


def test_planning_failure_sends_no_execution_commands():
    task = FakeTask(execute=True, fail_stage=3)
    with pytest.raises(RuntimeError, match='Planner failure'):
        task.run()
    assert task.executed == []


def test_execution_failure_stops_before_the_next_stage():
    task = FakeTask(execute=True, fail_execution=3)
    with pytest.raises(RuntimeError, match='Controller failure'):
        task.run()
    assert len(task.executed) == 3


def test_contact_grasp_rejects_early_obstruction_and_fully_closed_gripper():
    task = FakeTask(execute=True, grasp_position=0.2)
    with pytest.raises(RuntimeError, match='only 25.0% closed'):
        task.run()
    assert len(task.executed) == 5

    task = FakeTask(execute=True, grasp_position=0.8)
    with pytest.raises(RuntimeError, match='no object contact detected'):
        task.run()
    assert len(task.executed) == 5


def test_contact_grasp_accepts_stable_partial_closure():
    state = scene().robot_state
    state.joint_state.position[1] = 0.36
    positions, progress = PickPlaceTrajectoryNode.contact_grasp_positions(
        state, {'finger_joint': 0.0}, {'finger_joint': 0.8}, 0.4, 0.05)
    assert positions == {'finger_joint': pytest.approx(0.36)}
    assert progress == {'finger_joint': pytest.approx(0.45)}


def test_confirmed_contact_is_commanded_as_the_gripper_hold_target():
    task = FakeTask(execute=True, grasp_position=0.38)
    task.live_scene.robot_state.joint_state.position[1] = 0.38
    task.grasp_positions = {'finger_joint': 0.38}

    task.hold_grasp_contact()

    hold = task.executed[-1].trajectory.joint_trajectory
    assert hold.joint_names == ['finger_joint']
    assert hold.points[-1].positions == pytest.approx([0.38])
    assert hold.points[-1].time_from_start.sec == 1
    task.verify_live_targets({'finger_joint': 0.38}, 'Held grasp contact', 0.01)


def test_release_path_is_scaled_from_measured_contact_to_open():
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.joint_names = ['finger_joint']
    trajectory.joint_trajectory.points = [
        JointTrajectoryPoint(positions=[0.8], velocities=[-0.4], accelerations=[-0.2]),
        JointTrajectoryPoint(positions=[0.4], velocities=[-0.2], accelerations=[0.0]),
        JointTrajectoryPoint(positions=[0.0], velocities=[0.0], accelerations=[0.2]),
    ]
    release = PickPlaceTrajectoryNode.release_from_contact(
        trajectory, {'finger_joint': 0.36}, {'finger_joint': 0.0}, {'finger_joint': 0.8})
    assert [point.positions[0] for point in release.joint_trajectory.points] == pytest.approx(
        [0.36, 0.18, 0.0])
    assert release.joint_trajectory.points[0].velocities[0] == pytest.approx(-0.18)
    assert release.joint_trajectory.points[0].accelerations[0] == pytest.approx(-0.09)
    assert list(trajectory.joint_trajectory.points[0].positions) == [0.8]


def test_collision_contacts_preserve_other_pairs_and_original_matrix():
    matrix = AllowedCollisionMatrix(entry_names=['table', 'arm_link'], entry_values=[
        AllowedCollisionEntry(enabled=[False, False]),
        AllowedCollisionEntry(enabled=[False, True]),
    ])
    before = deepcopy(matrix)
    allowed = allow_contacts(matrix, 'block', ['left', 'table'])
    b, t, a = [allowed.entry_names.index(n) for n in ['block', 'table', 'arm_link']]
    assert allowed.entry_values[b].enabled[t]
    assert not allowed.entry_values[a].enabled[t]
    assert allowed.entry_values[a].enabled[a]
    assert matrix == before


def test_pose_conversion_preserves_world_geometry_under_rotation():
    gripper = Pose()
    gripper.position.x = 1.0
    gripper.orientation.z = math.sin(math.pi / 4)
    gripper.orientation.w = math.cos(math.pi / 4)
    box = Pose()
    box.position.x, box.position.y = 1.0, 2.0
    local = compose(inverse(gripper), box)
    restored = compose(gripper, local)
    assert local.position.x == pytest.approx(2.0)
    assert restored.position.x == pytest.approx(1.0)
    assert restored.position.y == pytest.approx(2.0)
    assert restored.orientation.w == pytest.approx(1.0)


def test_empty_planner_trajectory_and_missing_feedback_fail():
    with pytest.raises(ValueError, match='empty'):
        update_robot_state(RobotState(), RobotTrajectory(), URDF)
    with pytest.raises(RuntimeError, match='target'):
        PickPlaceTrajectoryNode.verify_positions(RobotState(), {'finger_joint': 0.8}, 0.01, 'Grasp')
    invalid = RobotState()
    invalid.joint_state.name = ['finger_joint']
    invalid.joint_state.position = [float('nan')]
    with pytest.raises(RuntimeError, match='target'):
        PickPlaceTrajectoryNode.verify_positions(invalid, {'finger_joint': 0.8}, 0.01, 'Grasp')
