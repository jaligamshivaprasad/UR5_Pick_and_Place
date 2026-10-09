"""Opt-in per-stage trajectory search for the complete pick-and-place task."""

from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
import time
import base64

import rclpy
from moveit_msgs.msg import DisplayTrajectory, MoveItErrorCodes, RobotState, RobotTrajectory
from rclpy.serialization import deserialize_message, serialize_message
from ur5_pick_place.pick_place_trajectory_node import (
    ActionFailure, PickPlaceTrajectoryNode, main as task_main,
)
from ur5_pick_place.trajectory_metrics import (
    OBJECTIVES, ToolKinematics, measure_trajectory,
)


class TrajectoryOptimizerNode(PickPlaceTrajectoryNode):
    def __init__(self, srdf_file=None):
        super().__init__(srdf_file)
        self.declare_parameter('optimization_candidates', 4)
        self.declare_parameter('optimization_try_ptp', False)
        self.declare_parameter('optimization_report', '')
        self.declare_parameter('selected_trajectory', '')
        self.declare_parameter('replay_trajectory', '')
        self.declare_parameter('required_trajectory_sha256', '')
        self.declare_parameter('preview_duration', 5.0)
        self.declare_parameter('optimization_min_improvement', 0.01)
        count = self.value('optimization_candidates')
        if not 0 <= count <= 20:
            raise ValueError('optimization_candidates must be between 0 and 20')
        if not math.isfinite(self.value('preview_duration')) or self.value('preview_duration') < 0:
            raise ValueError('preview_duration must be finite and non-negative')
        if not 0.0 <= self.value('optimization_min_improvement') < 1.0:
            raise ValueError('optimization_min_improvement must be between 0 and 1')
        for name in ('arm_velocity_scaling', 'arm_acceleration_scaling',
                     'gripper_velocity_scaling', 'gripper_acceleration_scaling',
                     'grasp_clearance', 'grasp_approach_height'):
            if not math.isfinite(self.value(name)):
                raise ValueError(name + ' must be finite')
        self.strategy = 'baseline'
        self.baseline_goals = {}

    def make_plan_goal(self, stage, state, scene_diff):
        goal = super().make_plan_goal(stage, state, scene_diff)
        if self.strategy == 'baseline':
            self.baseline_goals[stage.name] = deepcopy(goal)
        return goal

    def measure_stage(self, plan, kinematics):
        stage, trajectory, _, start = plan
        arm = stage.group == self.value('arm_group')
        positions = dict(zip(start.joint_state.name, start.joint_state.position))
        return measure_trajectory(
            trajectory, positions, kinematics.position if arm else None)

    @staticmethod
    def select_stage_candidate(records, arm, min_improvement=0.01):
        """Select a stage candidate without worsening applicable metrics."""
        objectives = OBJECTIVES if arm else ('duration_s',)
        baseline = records[0]['totals']
        if any(not math.isfinite(baseline[key]) or baseline[key] < 0 for key in objectives):
            raise ValueError('Invalid baseline stage metrics')
        best = 0
        ranking = ('joint_distance_rad', 'duration_s', 'tool_distance_m',
                   'acceleration_variation_rad_s2') if arm else ('duration_s',)
        best_key = tuple(baseline[key] for key in ranking)
        for index, record in enumerate(records):
            totals = record.get('totals')
            eligible = totals is not None and all(
                math.isfinite(totals[key]) and 0 <= totals[key] <= baseline[key]
                for key in objectives)
            record['eligible'] = eligible
            record['score'] = None
            if not eligible:
                continue
            key = tuple(totals[name] for name in ranking)
            record['score'] = key[0]
            primary = 'joint_distance_rad' if arm else 'duration_s'
            baseline_primary = baseline[primary]
            meaningful = (
                baseline_primary <= 1e-9
                or totals[primary] <= baseline_primary * (1.0 - min_improvement))
            if meaningful and key < best_key:
                best, best_key = index, key
        return best

    def plan_stage_candidate(self, stage, name, strategy):
        if stage.name not in self.baseline_goals:
            raise RuntimeError('Baseline goal was not captured for ' + stage.name)
        goal = deepcopy(self.baseline_goals[stage.name])
        if (strategy == 'ptp' and stage.group == self.value('arm_group')
                and stage.name not in self.cartesian_stages):
            goal.request.pipeline_id = self.value('place_planning_pipeline')
            goal.request.planner_id = self.value('place_planner_id')
            goal.request.num_planning_attempts = 1
        elif strategy == 'ompl':
            goal.request.pipeline_id = self.value('planning_pipeline')
            goal.request.planner_id = ''
            goal.request.num_planning_attempts = self.value('planning_attempts')
        previous_strategy = self.strategy
        self.strategy = strategy
        try:
            for attempt in range(self.value('planning_retry_attempts')):
                try:
                    return self.action(
                        self.planner, goal,
                        self.value('planning_time') + self.value('server_timeout'),
                        'Plan ' + stage.name + ' (' + name + ')')
                except ActionFailure as error:
                    if (error.code not in (MoveItErrorCodes.PLANNING_FAILED,
                                           MoveItErrorCodes.INVALID_MOTION_PLAN)
                            or attempt + 1 == self.value('planning_retry_attempts')):
                        raise
                    self.get_logger().warning(
                        f'Replanning {stage.name} for {name}: candidate {attempt + 1} '
                        'was rejected by MoveIt.')
        finally:
            self.strategy = previous_strategy

    def plan_all(self, scene):
        kinematics = ToolKinematics(self.urdf, self.value('cartesian_link'))
        stage_records, selected_plans = [], None
        self.strategy = 'baseline'
        self.baseline_goals = {}
        planning_started = time.monotonic()
        baseline_plans = super().plan_all(scene)
        selected_plans = list(baseline_plans)
        report_stages = []
        total_planning_wall = time.monotonic() - planning_started
        total_measurement_wall = 0.0
        for stage_index, baseline_plan in enumerate(baseline_plans):
            stage = baseline_plan[0]
            arm = stage.group == self.value('arm_group')
            measurement_started = time.monotonic()
            baseline_totals = self.measure_stage(baseline_plan, kinematics)
            total_measurement_wall += time.monotonic() - measurement_started
            records = [{'name': 'baseline', 'status': 'planned', 'totals': baseline_totals,
                        'planning_wall_s': 0.0, 'measurement_wall_s': 0.0}]
            if stage.name in self.cartesian_stages or not arm:
                selected_plans[stage_index] = baseline_plan
                stage_records.append({
                    'stage': stage.name, 'group': stage.group, 'selected': 'baseline',
                    'optimized': False,
                    'reason': ('Cartesian safety-critical stage kept on configured baseline planner'
                               if stage.name in self.cartesian_stages
                               else 'Gripper timing kept on the proven baseline trajectory'),
                    'candidates': records,
                })
                continue
            candidates = [('ompl_' + str(index), 'ompl')
                          for index in range(1, self.value('optimization_candidates') + 1)]
            if self.value('optimization_try_ptp') and arm and stage.name not in self.cartesian_stages:
                candidates.extend(
                    ('ptp_' + str(index), 'ptp')
                    for index in range(1, self.value('optimization_candidates') + 1))
            for name, strategy in candidates:
                self.get_logger().info(f'Evaluating {stage.name} candidate: {name}')
                planning_started = time.monotonic()
                try:
                    result = self.plan_stage_candidate(stage, name, strategy)
                    candidate_plan = (stage, result.planned_trajectory,
                                      result.trajectory_start, baseline_plan[3])
                    measurement_started = time.monotonic()
                    totals = self.measure_stage(candidate_plan, kinematics)
                    measurement_wall = time.monotonic() - measurement_started
                    total_measurement_wall += measurement_wall
                    records.append({
                        'name': name, 'status': 'planned', 'totals': totals,
                        'planning_wall_s': time.monotonic() - planning_started,
                        'measurement_wall_s': measurement_wall})
                    records[-1]['plan'] = candidate_plan
                except (ActionFailure, ValueError) as error:
                    records.append({'name': name, 'status': 'failed', 'error': str(error),
                                    'planning_wall_s': time.monotonic() - planning_started,
                                    'measurement_wall_s': 0.0})
                    self.get_logger().warning(f'Candidate {name} rejected: {error}')
            selected = self.select_stage_candidate(
                records, arm, self.value('optimization_min_improvement'))
            chosen_record = records[selected]
            if selected:
                selected_plans[stage_index] = chosen_record['plan']
            stage_records.append({
                'stage': stage.name, 'group': stage.group, 'selected': records[selected]['name'],
                'candidates': [{key: value for key, value in record.items() if key != 'plan'}
                               for record in records],
            })
            total_planning_wall += sum(record['planning_wall_s'] for record in records)
            self.get_logger().info(
                f'{stage.name}: selected {records[selected]["name"]}; '
                f'duration {chosen_record["totals"]["duration_s"]:.2f} s')
        selected = stage_records
        report = {
            'schema_version': 1,
            'created_at_utc': datetime.now(timezone.utc).isoformat(),
            'selected': 'per_stage',
            'optimized_stages': [row['stage'] for row in stage_records
                                 if row['selected'] != 'baseline'],
            'selection': ('Arm candidates must not worsen any arm metric and are ranked by '
                          'joint travel. Gripper candidates are ranked by duration.'),
            'measurement_notes': [
                'Duration sums trajectory timestamps, excluding planning, controller overhead '
                'and grasp checks.',
                'Joint travel sums Euclidean segment lengths in commanded arm coordinates; '
                'angles are not wrapped.',
                'Tool travel samples linear joint segments at most 0.05 rad apart; '
                'not continuous controller interpolation.',
                'Acceleration variation sums absolute sampled changes per arm joint, '
                'including boundaries to/from rest.',
                'Acceleration variation is a smoothness proxy, '
                'not a continuous jerk measurement or jerk limit.',
                'Every stage is searched independently; selected alternatives preserve the '
                'same stage endpoint and planning-scene state as baseline.',
                'All candidates use the same attached-box start state, planning scene, target '
                'and speed scaling.',
                'This report describes planned motion, not measured simulation or hardware performance.',
            ],
            'settings': {name: self.value(name) for name in (
                'execute', 'optimization_candidates', 'optimization_try_ptp', 'arm_velocity_scaling',
                'arm_acceleration_scaling', 'gripper_velocity_scaling',
                'gripper_acceleration_scaling', 'grasp_clearance', 'grasp_approach_height',
                'planning_time', 'planning_attempts', 'planning_retry_attempts',
                'planning_pipeline', 'place_planning_pipeline', 'place_planner_id',
                'linear_planning_pipeline', 'linear_planner_id', 'arm_group', 'gripper_group',
                'object_id', 'attach_link', 'cartesian_link', 'support_surfaces',
                'joint_goal_tolerance', 'verification_tolerance', 'arm_verification_tolerance',
                'grasp_min_closure_fraction', 'grasp_contact_margin',
                'grasp_stability_tolerance', 'grasp_confirmation_time',
                'grasp_retention_tolerance', 'preview_duration',
                'optimization_min_improvement')},
            'planning_wall_s': total_planning_wall,
            'measurement_wall_s': total_measurement_wall,
            'initial_joint_positions': dict(zip(scene.robot_state.joint_state.name,
                                                scene.robot_state.joint_state.position)),
            'candidates': stage_records,
        }
        destination = self.write_report(report)
        self.write_selected_trajectory(selected_plans, scene)
        self.get_logger().info(f'Selected per-stage trajectories; report: {destination}')
        self.strategy = 'baseline'
        return selected_plans

    def prepare_plans(self, initial_scene):
        try:
            filename = self.value('replay_trajectory')
        except KeyError:
            filename = ''
        if filename and filename != 'none':
            if filename == 'latest':
                filename = self.latest_trajectory(initial_scene)
            return self.load_selected_trajectory(filename, initial_scene)
        return self.plan_all(initial_scene)

    def latest_trajectory(self, scene):
        directory = Path(os.environ.get('ROS_HOME', str(Path.home() / '.ros'))) / 'trajectory_reports'
        candidates = sorted(directory.glob('*.trajectory.json'), key=lambda path: path.stat().st_mtime)
        current_joints = set(scene.robot_state.joint_state.name)
        current_model = scene.robot_model_name
        for candidate in reversed(candidates):
            try:
                payload = json.loads(candidate.read_text())
                saved_joints = set(payload['initial_joint_positions'])
                saved_model = payload.get('robot_model_name', '')
            except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError):
                continue
            if saved_joints <= current_joints and (not current_model or not saved_model
                                                   or saved_model == current_model):
                return candidate
        if not candidates:
            raise RuntimeError(f'No saved trajectories found in {directory}')
        raise RuntimeError(
            f'No compatible saved trajectory found in {directory}; '
            'run planning again with the current robot bringup')

    @staticmethod
    def _encode(message):
        return base64.b64encode(serialize_message(message)).decode('ascii')

    @staticmethod
    def _decode(encoded, message_type):
        try:
            return deserialize_message(base64.b64decode(encoded), message_type)
        except (TypeError, ValueError) as error:
            raise ValueError('Invalid saved trajectory message') from error

    def write_selected_trajectory(self, plans, initial_scene):
        try:
            filename = self.value('selected_trajectory')
        except KeyError:
            filename = ''
        if filename:
            destination = Path(filename).expanduser().resolve()
        else:
            stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
            ros_directory = Path(os.environ.get('ROS_HOME', str(Path.home() / '.ros')))
            destination = ros_directory / 'trajectory_reports' / f'{stamp}.trajectory.json'
        payload = {
            'schema_version': 1,
            'created_at_utc': datetime.now(timezone.utc).isoformat(),
            'robot_model_name': initial_scene.robot_model_name,
            'initial_joint_positions': dict(zip(
                initial_scene.robot_state.joint_state.name,
                initial_scene.robot_state.joint_state.position)),
            'stages': [{
                'name': stage.name,
                'group': stage.group,
                'state': stage.state,
                'trajectory': self._encode(trajectory),
                'trajectory_start': self._encode(trajectory_start),
                'start': self._encode(start),
            } for stage, trajectory, trajectory_start, start in plans],
        }
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', dir=destination.parent, delete=False) as stream:
                temporary = Path(stream.name)
                json.dump(payload, stream, indent=2, allow_nan=False)
                stream.write('\n')
            temporary.replace(destination)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)
        self.get_logger().info(f'Saved selected trajectory: {destination}')
        return destination

    def load_selected_trajectory(self, filename, scene):
        path = Path(filename).expanduser().resolve()
        try:
            try:
                required_hash = self.value('required_trajectory_sha256').strip().lower()
            except KeyError:  # Test doubles and older subclasses omit the opt-in guard.
                required_hash = ''
            if required_hash:
                actual_hash = hashlib.sha256(path.read_bytes()).hexdigest()
                if actual_hash != required_hash:
                    raise ValueError(
                        f'trajectory SHA-256 is {actual_hash}, expected {required_hash}')
            payload = json.loads(path.read_text())
            if payload.get('schema_version') != 1:
                raise ValueError('unsupported schema version')
            saved = payload['stages']
            if len(saved) != len(self.stages):
                raise ValueError('stage count does not match the current task')
            plans = []
            for expected, item in zip(self.stages, saved):
                if (item['name'], item['group'], item['state']) != (
                        expected.name, expected.group, expected.state):
                    raise ValueError('stage sequence does not match the current task')
                plans.append((
                    expected,
                    self._decode(item['trajectory'], RobotTrajectory),
                    self._decode(item['trajectory_start'], RobotState),
                    self._decode(item['start'], RobotState),
                ))
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            raise RuntimeError(f'Could not load saved trajectory {path}: {error}') from error
        saved_model = payload.get('robot_model_name', '')
        if saved_model and scene.robot_model_name and saved_model != scene.robot_model_name:
            raise RuntimeError(
                f'Could not load saved trajectory {path}: robot model differs '
                f'(saved {saved_model!r}, current {scene.robot_model_name!r})')
        saved_initial = payload.get('initial_joint_positions', {})
        if saved_initial:
            available = set(scene.robot_state.joint_state.name)
            missing = sorted(set(saved_initial) - available)
            if missing:
                raise RuntimeError(
                    f'Could not load saved trajectory {path}: saved joint names are '
                    f'not present in current robot feedback: {missing}')
            try:
                self.verify_positions(
                    scene.robot_state, saved_initial, self.value('arm_verification_tolerance'),
                    'Robot differs from saved trajectory start',
                    getattr(self, 'periodic_joints', set()))
            except RuntimeError as error:
                measured = dict(zip(scene.robot_state.joint_state.name,
                                    scene.robot_state.joint_state.position))
                positions = {joint: {'saved': target, 'current': measured[joint]}
                             for joint, target in saved_initial.items()}
                raise RuntimeError(
                    f'Cannot replay saved trajectory {path}: {error}. '
                    f'Start positions (radians): {positions}. '
                    'To plan from the current robot state, run '
                    'ros2 launch ur5_pick_place optimized_pick_place.launch.py '
                    'replay_trajectory:=none execute:=false, review the new plan, '
                    'then replay it with execute:=true while the robot and scene are unchanged. '
                    'Replaying this existing bundle requires its saved starting pose, '
                    'including the gripper.') from error
        self.get_logger().info(f'Loaded saved trajectory: {path}; no planning requested')
        return plans

    def publish_plan_preview(self, plans, initial_scene):
        if self.value('execute') or self.value('preview_duration') <= 0:
            return
        preview = next(
            (entry for entry in plans if entry[0].name == 'move_to_place'), None)
        if preview is None:
            raise RuntimeError('Selected plan has no move_to_place trajectory to preview')
        _, trajectory, trajectory_start, _ = preview
        message = DisplayTrajectory(
            model_id=initial_scene.robot_model_name,
            trajectory_start=trajectory_start,
            trajectory=[trajectory])
        deadline = time.monotonic() + self.value('preview_duration')
        self.get_logger().info(
            'Publishing the attached-box move_to_place path on '
            '~/display_planned_path; select it in RViz MotionPlanning > Planned Path.')
        while rclpy.ok() and time.monotonic() < deadline:
            self.preview.publish(message)
            rclpy.spin_once(self, timeout_sec=min(0.5, deadline - time.monotonic()))

    def write_report(self, report):
        filename = self.value('optimization_report')
        if filename:
            destination = Path(filename).expanduser().resolve()
        else:
            stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
            ros_directory = Path(os.environ.get('ROS_HOME', str(Path.home() / '.ros')))
            destination = ros_directory / 'trajectory_reports' / f'{stamp}.json'
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', dir=destination.parent, delete=False) as stream:
                temporary = Path(stream.name)
                json.dump(report, stream, indent=2, allow_nan=False)
                stream.write('\n')
            temporary.replace(destination)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)
        return destination


def main(args=None):
    return task_main(args, node_factory=TrajectoryOptimizerNode)


if __name__ == '__main__':
    raise SystemExit(main())
