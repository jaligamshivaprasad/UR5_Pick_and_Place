"""No services, controllers, or scene writes: optimizer regression tests."""

from copy import deepcopy
import json
import math

from builtin_interfaces.msg import Duration
from moveit_msgs.msg import MoveItErrorCodes, RobotTrajectory
import pytest
from trajectory_msgs.msg import JointTrajectoryPoint

from test_task import FakeTask, URDF, scene
from ur5_pick_place.pick_place_trajectory_node import ActionFailure
from ur5_pick_place.trajectory_metrics import (
    OBJECTIVES, ToolKinematics, measure_trajectory, select_candidate,
)
from ur5_pick_place.trajectory_optimizer_node import TrajectoryOptimizerNode
import ur5_pick_place.trajectory_optimizer_node as optimizer_module


def trajectory(positions=(0.0, 1.0, 0.0), times=(0, 1, 2)):
    result = RobotTrajectory()
    result.joint_trajectory.joint_names = ['joint']
    result.joint_trajectory.points = [
        JointTrajectoryPoint(positions=[position], velocities=[0.0],
                             accelerations=[acceleration],
                             time_from_start=Duration(sec=seconds))
        for position, acceleration, seconds in zip(positions, (0.0, 2.0, 0.0), times)]
    return result


def test_metrics_measure_detours_duration_and_acceleration_without_mutating():
    path = trajectory()
    before = deepcopy(path)
    result = measure_trajectory(path, {'joint': 0.0}, lambda q: (q['joint'], 0, 0))
    assert result == {'duration_s': 2.0, 'joint_distance_rad': 2.0,
                      'tool_distance_m': pytest.approx(2.0),
                      'acceleration_variation_rad_s2': 4.0, 'points': 3}
    assert path == before


def test_full_revolutions_count_as_real_commanded_travel():
    path = trajectory((0.0, math.pi, 2 * math.pi))
    assert measure_trajectory(path, {'joint': 0.0})['joint_distance_rad'] == pytest.approx(2 * math.pi)


@pytest.mark.parametrize('change, message', [
    (lambda p: setattr(p.points[1], 'positions', [float('nan')]), 'non-finite'),
    (lambda p: setattr(p.points[1], 'accelerations', []), 'Missing'),
    (lambda p: setattr(p.points[1], 'velocities', [float('inf')]), 'non-finite'),
    (lambda p: setattr(p.points[1], 'time_from_start', Duration()), 'increase'),
    (lambda p: setattr(p.points[0], 'positions', [0.02]), 'start differs'),
    (lambda p: setattr(p.points[-1], 'velocities', [1.0]), 'at rest'),
    (lambda p: setattr(p, 'joint_names', ['joint', 'joint']), 'duplicate'),
])
def test_invalid_candidates_are_rejected(change, message):
    path = trajectory()
    change(path.joint_trajectory)
    with pytest.raises(ValueError, match=message):
        measure_trajectory(path, {'joint': 0.0})


def test_stationary_zero_duration_is_allowed_but_moving_is_not():
    path = trajectory((0.0,), (0,))
    assert measure_trajectory(path, {'joint': 0.0})['duration_s'] == 0
    path.joint_trajectory.points[0].positions = [0.005]
    with pytest.raises(ValueError, match='zero duration'):
        measure_trajectory(path, {'joint': 0.0})


def test_tool_kinematics_combines_fixed_rotated_and_prismatic_joints():
    urdf = '''<robot name="test"><link name="world"/><link name="arm"/>
    <link name="slider"/><link name="tool"/>
    <joint name="turn" type="revolute"><parent link="world"/><child link="arm"/>
      <origin xyz="1 2 3" rpy="0 0 1.5707963267948966"/><axis xyz="0 0 1"/></joint>
    <joint name="slide" type="prismatic"><parent link="arm"/><child link="slider"/>
      <axis xyz="1 0 0"/></joint>
    <joint name="tip" type="fixed"><parent link="slider"/><child link="tool"/>
      <origin xyz="1 0 0"/></joint></robot>'''
    fk = ToolKinematics(urdf, 'tool')
    assert fk.position({'turn': 0.0, 'slide': 1.0}) == pytest.approx((1, 4, 3))
    assert fk.position({'turn': math.pi / 2, 'slide': 1.0}) == pytest.approx((-1, 2, 3))


def record(values):
    return {'totals': dict(zip(OBJECTIVES, values))}


def test_selection_rejects_shortcut_with_worse_smoothness_or_time():
    records = [record([10, 10, 10, 10]), record([9, 9, 9, 11]),
               record([11, 1, 1, 1]), {'error': 'collision'}, record([9, 8, 9, 8])]
    assert select_candidate(records) == 4
    assert [r['eligible'] for r in records] == [True, False, False, False, True]


def test_selection_falls_back_to_baseline_for_ties_or_all_failures():
    records = [record([0, 10, 0, 10]), record([0, 10, 0, 10]),
               record([0, 9, 0.1, 9]), {'error': 'failed'}]
    assert select_candidate(records) == 0


def test_selection_never_accepts_a_worsening_when_baseline_is_zero():
    records = [record([0, 10, 0, 10]), record([0, 10, 1e-10, 9])]
    assert select_candidate(records) == 0
    assert records[1]['eligible'] is False


def test_selection_accepts_a_strictly_improved_score_without_a_deadband():
    records = [record([1, 1, 1, 1]), record([1 - 1e-8, 1, 1, 1])]
    assert select_candidate(records) == 1


class FakeOptimizer(TrajectoryOptimizerNode, FakeTask):
    def __init__(self, tmp_path, execute=False, reject_alternatives=False):
        FakeTask.__init__(self, execute=execute)
        self.config.update(optimization_candidates=2,
                           optimization_try_ptp=True,
                           preview_duration=0.0,
                           optimization_min_improvement=0.01,
                           selected_trajectory=str(tmp_path / 'selected.trajectory.json'),
                           optimization_report=str(tmp_path / 'report.json'))
        self.strategy = 'baseline'
        self.reject_alternatives = reject_alternatives
        self.urdf = URDF.replace('</robot>', '''<link name="world"/><link name="tool0"/>
          <joint name="arm_joint" type="prismatic"><parent link="world"/>
          <child link="gripper"/><axis xyz="1 0 0"/></joint>
          <joint name="tool" type="fixed"><parent link="gripper"/>
          <child link="tool0"/></joint></robot>''')

    def action(self, client, goal, timeout, label):
        if client is self.executor_client:
            assert len(self.plan_goals) >= 9  # Complete task before any execution.
            return FakeTask.action(self, client, goal, timeout, label)
        if self.reject_alternatives and self.strategy != 'baseline':
            raise ActionFailure(label, 6, MoveItErrorCodes.INVALID_MOTION_PLAN)
        result = FakeTask.action(self, client, goal, timeout, label)
        path = result.planned_trajectory.joint_trajectory
        joint = path.joint_names[0]
        start = dict(zip(goal.request.start_state.joint_state.name,
                         goal.request.start_state.joint_state.position))[joint]
        end = path.points[-1].positions[0]
        # Deterministic arm alternatives improve duration and remove a detour.
        arm = goal.request.group_name == 'arm'
        improved = arm and self.strategy != 'baseline'
        middle = (start + end) / 2 + (1.0 if arm and not improved else 0.0)
        path.points = [
            JointTrajectoryPoint(positions=[position], velocities=[0.0],
                                 accelerations=[acceleration], time_from_start=Duration(sec=seconds))
            for position, acceleration, seconds in zip(
                (start, middle, end), (0.0, 1.0 if improved else 2.0, 0.0),
                (0, 1, 2 if improved else 3))]
        return result


def test_optimizer_plan_only_preserves_live_scene_and_contacts(tmp_path):
    task = FakeOptimizer(tmp_path)
    before = deepcopy(task.live_scene)
    task.run()
    report = json.loads((tmp_path / 'report.json').read_text())
    assert report['selected'] == 'per_stage'
    assert set(report['optimized_stages']) <= {
        'move_to_straight_before_pick', 'move_to_place', 'return_to_straight'}
    assert task.executed == []
    assert task.live_scene == before
    assert len(task.plan_goals) == 21
    assert all(goal.planning_options.plan_only for goal in task.plan_goals)
    baseline = task.plan_goals[:9]
    candidates = task.plan_goals[9:]
    ompl = [goal for goal in candidates if goal.request.planner_id != 'PTP']
    ptp = [goal for goal in candidates if goal.request.planner_id == 'PTP']
    assert baseline[5].request.pipeline_id == 'ompl'
    assert all(goal.request.planner_id == '' for goal in ompl)
    assert all(goal.request.planner_id == 'PTP' for goal in ptp)
    for candidate in ompl + ptp:
        assert candidate.request.max_velocity_scaling_factor in (0.2, 0.3)
        assert candidate.request.max_acceleration_scaling_factor in (0.2, 0.3)
        assert candidate.request.goal_constraints
    assert any(candidate.request.start_state.attached_collision_objects
               for candidate in ompl + ptp)
    assert [row['stage'] for row in report['candidates']] == [
        stage.name for stage in task.stages]
    assert all(row['candidates'][0]['name'] == 'baseline'
               for row in report['candidates'])
    for row in report['candidates']:
        if row['stage'] in ('move_to_pre_grasp', 'descend_to_grasp',
                            'open_before_pick', 'grasp', 'release'):
            assert row['selected'] == 'baseline'
            assert row['optimized'] is False
            assert len(row['candidates']) == 1


def test_place_selection_prioritizes_joint_travel_and_rejects_worsening(tmp_path):
    records = [
        record([10, 10, 10, 10]),
        record([8, 9, 9, 9]),
        record([9, 5, 10, 10]),
        record([1, 1, 11, 1]),
    ]
    assert TrajectoryOptimizerNode.select_stage_candidate(
        records, arm=True, min_improvement=0.01) == 2
    assert records[3]['eligible'] is False


def test_gripper_selection_uses_duration_only():
    records = [
        record([10, 0, None, None]),
        record([8, 999, None, None]),
    ]
    assert TrajectoryOptimizerNode.select_stage_candidate(
        records, arm=False, min_improvement=0.01) == 1


def test_stage_selection_rejects_planner_noise_without_meaningful_improvement():
    records = [record([10, 10, 10, 10]), record([9.99, 9.99, 9.99, 9.99])]
    assert TrajectoryOptimizerNode.select_stage_candidate(
        records, arm=True, min_improvement=0.01) == 0


def test_selected_place_preview_includes_attached_box_start_state(tmp_path, monkeypatch):
    task = FakeOptimizer(tmp_path)
    plans = task.plan_all(scene())
    task.config['preview_duration'] = 1.0
    ticks = iter((0.0, 0.0, 0.0, 1.0))
    monkeypatch.setattr(optimizer_module.time, 'monotonic', lambda: next(ticks))
    monkeypatch.setattr(optimizer_module.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(optimizer_module.rclpy, 'spin_once', lambda *_args, **_kwargs: None)

    task.publish_plan_preview(plans, task.live_scene)

    message = task.preview.publish.call_args.args[0]
    assert len(message.trajectory) == 1
    assert message.trajectory[0] == plans[5][1]
    assert message.trajectory_start.attached_collision_objects


def test_failed_optional_candidates_fall_back_to_complete_baseline(tmp_path):
    task = FakeOptimizer(tmp_path, reject_alternatives=True)
    plans = task.plan_all(scene())
    report = json.loads((tmp_path / 'report.json').read_text())
    assert report['selected'] == 'per_stage'
    assert report['optimized_stages'] == []
    assert len(plans) == 9
    assert not task.executed
    assert all(candidate['status'] == 'failed' and 'error' in candidate
               for stage in report['candidates']
               for candidate in stage['candidates'][1:])


def test_selected_task_uses_existing_execution_and_grasp_checks(tmp_path):
    task = FakeOptimizer(tmp_path, execute=True)
    task.run()
    assert len(task.executed) == 10
    task.verify_positions(task.live_scene.robot_state,
                          {'arm_joint': 0.5, 'finger_joint': 0.0}, 1e-6, 'Final')


def test_baseline_failure_aborts_before_execution(tmp_path):
    task = FakeOptimizer(tmp_path, execute=True)
    task.fail_stage = 3
    with pytest.raises(RuntimeError, match='Planner failure'):
        task.run()
    assert not task.executed
    assert not (tmp_path / 'report.json').exists()


def test_default_search_uses_existing_pipelines_only(tmp_path):
    task = FakeOptimizer(tmp_path)
    task.config['optimization_try_ptp'] = False
    task.run()
    assert all(goal.request.planner_id != 'PTP' for goal in task.plan_goals)
    assert json.loads((tmp_path / 'report.json').read_text())['optimized_stages']


def test_optional_planning_timeout_aborts_without_execution(tmp_path):
    task = FakeOptimizer(tmp_path, execute=True)
    original = task.action

    def timeout(client, goal, duration, label):
        if task.strategy != 'baseline':
            raise TimeoutError('Planning timed out')
        return original(client, goal, duration, label)

    task.action = timeout
    with pytest.raises(TimeoutError):
        task.run()
    assert not task.executed


def test_failed_grasp_still_stops_the_optimized_task(tmp_path):
    task = FakeOptimizer(tmp_path, execute=True)
    task.grasp_position = 0.8
    with pytest.raises(RuntimeError, match='no object contact'):
        task.run()
    assert len(task.executed) == 5


def test_report_write_failure_prevents_execution(tmp_path):
    task = FakeOptimizer(tmp_path, execute=True)
    task.config['optimization_report'] = str(tmp_path)  # A directory cannot be replaced.
    with pytest.raises(OSError):
        task.run()
    assert not task.executed


@pytest.mark.parametrize('replay', ['', 'none'])
def test_explicit_replanning_uses_current_start_state(tmp_path, replay):
    task = FakeOptimizer(tmp_path)
    task.config['replay_trajectory'] = replay
    task.live_scene.robot_state.joint_state.position = [0.0, 0.0, 0.0]

    task.run()

    assert task.plan_goals[0].request.start_state == task.live_scene.robot_state
    saved = json.loads((tmp_path / 'selected.trajectory.json').read_text())
    assert saved['initial_joint_positions'] == {
        'arm_joint': 0.0, 'finger_joint': 0.0, 'right_joint': 0.0}
    assert not task.executed


@pytest.mark.parametrize('replay', ['path', 'latest'])
def test_saved_bundle_replays_without_planning(tmp_path, monkeypatch, replay):
    monkeypatch.setenv('ROS_HOME', str(tmp_path))
    bundle = tmp_path / 'trajectory_reports' / 'selected.trajectory.json'
    original = FakeOptimizer(tmp_path)
    original.config['selected_trajectory'] = str(bundle)
    expected = original.plan_all(original.live_scene)
    task = FakeOptimizer(tmp_path)
    task.config['replay_trajectory'] = str(bundle) if replay == 'path' else replay

    plans = task.prepare_plans(task.live_scene)

    assert plans == expected


def test_replay_rejects_a_bundle_that_does_not_match_required_hash(tmp_path):
    original = FakeOptimizer(tmp_path)
    original.plan_all(original.live_scene)
    task = FakeOptimizer(tmp_path)
    task.config['replay_trajectory'] = str(tmp_path / 'selected.trajectory.json')
    task.config['required_trajectory_sha256'] = '0' * 64

    with pytest.raises(RuntimeError, match='trajectory SHA-256'):
        task.prepare_plans(task.live_scene)
    assert not task.plan_goals
    assert not task.executed


@pytest.mark.parametrize('joint, position', [
    ('finger_joint', 0.0), ('right_joint', 0.0), ('arm_joint', 0.8),
])
def test_replay_start_mismatch_explains_recovery_and_stops_before_motion(
        tmp_path, joint, position):
    original = FakeOptimizer(tmp_path)
    original.plan_all(original.live_scene)
    bundle = tmp_path / 'selected.trajectory.json'
    before = bundle.read_bytes()
    task = FakeOptimizer(tmp_path, execute=True)
    task.config['replay_trajectory'] = str(bundle)
    state = task.live_scene.robot_state.joint_state
    state.position[state.name.index(joint)] = position
    current_scene = deepcopy(task.live_scene)

    with pytest.raises(RuntimeError, match='Cannot replay saved trajectory') as failure:
        task.run()

    message = str(failure.value)
    assert str(bundle) in message
    assert joint in message
    assert "'saved':" in message and "'current':" in message
    assert 'replay_trajectory:=none execute:=false' in message
    assert not task.plan_goals
    assert not task.executed
    assert task.live_scene == current_scene
    assert bundle.read_bytes() == before


def test_replay_preserves_periodic_joint_comparison(tmp_path):
    original = FakeOptimizer(tmp_path)
    expected = original.plan_all(original.live_scene)
    task = FakeOptimizer(tmp_path)
    task.periodic_joints = {'arm_joint'}
    task.config['replay_trajectory'] = str(tmp_path / 'selected.trajectory.json')
    task.live_scene.robot_state.joint_state.position[0] += 2 * math.pi

    assert task.prepare_plans(task.live_scene) == expected
    assert not task.plan_goals


def test_explicit_replay_still_rejects_missing_joint_feedback(tmp_path):
    original = FakeOptimizer(tmp_path)
    original.plan_all(original.live_scene)
    task = FakeOptimizer(tmp_path, execute=True)
    task.config['replay_trajectory'] = str(tmp_path / 'selected.trajectory.json')
    task.live_scene.robot_state.joint_state.name.pop()
    task.live_scene.robot_state.joint_state.position.pop()

    with pytest.raises(RuntimeError, match='not present in current robot feedback'):
        task.run()

    assert not task.plan_goals
    assert not task.executed
