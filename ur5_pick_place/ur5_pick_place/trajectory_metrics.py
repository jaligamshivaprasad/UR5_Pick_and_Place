"""Read-only trajectory measurements and conservative whole-task selection.

Distances use the commanded joint coordinates (no angle wrapping). Tool travel
is a sampled polyline, not a continuous collision or dynamics check. Smoothness
is a sampled acceleration-change metric, not a jerk-limit guarantee.
"""

import math
import xml.etree.ElementTree as ET

from geometry_msgs.msg import Pose

from ur5_pick_place.task import compose


class ToolKinematics:
    """URDF forward kinematics for a fixed-base, non-mimic tool chain."""

    def __init__(self, urdf, tool_link):
        robot = ET.fromstring(urdf)
        if tool_link not in {link.get('name') for link in robot.findall('link')}:
            raise ValueError('Unknown tool link: ' + tool_link)
        parents = {joint.find('child').get('link'): joint for joint in robot.findall('joint')}
        chain, visited = [], set()
        while tool_link in parents:
            if tool_link in visited:
                raise ValueError('Cycle in the tool chain')
            visited.add(tool_link)
            joint = parents[tool_link]
            kind = joint.get('type')
            if (kind not in ('fixed', 'revolute', 'continuous', 'prismatic')
                    or joint.find('mimic') is not None):
                raise ValueError('Unsupported tool-chain joint: ' + joint.get('name'))
            origin = joint.find('origin')
            xyz = [0.0] * 3 if origin is None else list(map(float, origin.get('xyz', '0 0 0').split()))
            rpy = [0.0] * 3 if origin is None else list(map(float, origin.get('rpy', '0 0 0').split()))
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = xyz
            r, p, y = (angle / 2 for angle in rpy)
            cr, cp, cy = math.cos(r), math.cos(p), math.cos(y)
            sr, sp, sy = math.sin(r), math.sin(p), math.sin(y)
            pose.orientation.x = sr * cp * cy - cr * sp * sy
            pose.orientation.y = cr * sp * cy + sr * cp * sy
            pose.orientation.z = cr * cp * sy - sr * sp * cy
            pose.orientation.w = cr * cp * cy + sr * sp * sy
            axis_element = joint.find('axis')
            axis = ([1.0, 0.0, 0.0] if axis_element is None
                    else list(map(float, axis_element.get('xyz').split())))
            length = math.sqrt(sum(value * value for value in axis))
            if not math.isfinite(length) or length == 0:
                raise ValueError('Invalid joint axis')
            chain.append((joint.get('name'), kind, pose, [value / length for value in axis]))
            tool_link = joint.find('parent').get('link')
        self.chain = list(reversed(chain))

    def position(self, joints):
        pose = Pose()
        pose.orientation.w = 1.0
        for name, kind, origin, axis in self.chain:
            pose = compose(pose, origin)
            if kind == 'fixed':
                continue
            value = joints[name]
            if not math.isfinite(value):
                raise ValueError('Non-finite joint position')
            motion = Pose()
            motion.orientation.w = 1.0
            if kind == 'prismatic':
                motion.position.x, motion.position.y, motion.position.z = [value * a for a in axis]
            else:
                rotation = [math.sin(value / 2) * a for a in axis]
                motion.orientation.x, motion.orientation.y, motion.orientation.z = rotation
                motion.orientation.w = math.cos(value / 2)
            pose = compose(pose, motion)
        return (pose.position.x, pose.position.y, pose.position.z)


def measure_trajectory(trajectory, start_positions, tool_position=None, joint_sample_step=0.05):
    """Measure a returned path without editing any positions or timestamps."""
    joint_path = trajectory.joint_trajectory
    names, points = list(joint_path.joint_names), joint_path.points
    if trajectory.multi_dof_joint_trajectory.points:
        raise ValueError('Multi-DOF trajectories are unsupported')
    if not names or len(set(names)) != len(names) or not points:
        raise ValueError('Empty trajectory or duplicate joint names')
    if not math.isfinite(joint_sample_step) or joint_sample_step <= 0:
        raise ValueError('joint_sample_step must be finite and positive')
    times = []
    for point in points:
        for field in ('positions', 'velocities', 'accelerations'):
            values = getattr(point, field)
            if len(values) != len(names) or not all(math.isfinite(value) for value in values):
                raise ValueError('Missing or non-finite trajectory ' + field)
        stamp = point.time_from_start
        if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
            raise ValueError('Invalid trajectory timestamp')
        seconds = stamp.sec + stamp.nanosec * 1e-9
        if times and seconds <= times[-1]:
            raise ValueError('Trajectory timestamps must increase strictly')
        times.append(seconds)
    initial = [start_positions[name] for name in names]
    if not all(math.isfinite(value) for value in initial):
        raise ValueError('Non-finite start state')
    if max(abs(a - b) for a, b in zip(initial, points[0].positions)) > 0.01:
        raise ValueError('Trajectory start differs from requested start by more than 0.01 rad')
    if any(abs(value) > 1e-6 for point in (points[0], points[-1]) for value in point.velocities):
        raise ValueError('Task stages must start and end at rest')
    joint_distance = 0.0
    tool_distance = 0.0
    previous = initial
    tool_previous = tool_position(start_positions) if tool_position else None
    for point in points:
        delta = [b - a for a, b in zip(previous, point.positions)]
        distance = math.sqrt(sum(value * value for value in delta))
        joint_distance += distance
        if tool_position:
            subdivisions = max(1, math.ceil(distance / joint_sample_step))
            for step in range(1, subdivisions + 1):
                joints = dict(start_positions)
                joints.update({name: a + d * step / subdivisions
                               for name, a, d in zip(names, previous, delta)})
                current = tool_position(joints)
                if len(current) != 3 or not all(math.isfinite(value) for value in current):
                    raise ValueError('Invalid tool position')
                tool_distance += math.dist(tool_previous, current)
                tool_previous = current
        previous = point.positions
    if times[-1] <= 0 and joint_distance > 1e-9:
        raise ValueError('Moving trajectory has zero duration')
    # Include acceleration changes to/from rest at the stage boundaries.
    acceleration_variation = sum(abs(value) for value in points[0].accelerations)
    acceleration_variation += sum(abs(value) for value in points[-1].accelerations)
    for before, after in zip(points, points[1:]):
        acceleration_variation += sum(
            abs(b - a) for a, b in zip(before.accelerations, after.accelerations))
    return {
        'duration_s': times[-1],
        'joint_distance_rad': joint_distance,
        'tool_distance_m': tool_distance if tool_position else None,
        'acceleration_variation_rad_s2': acceleration_variation,
        'points': len(points),
    }


OBJECTIVES = ('duration_s', 'joint_distance_rad', 'tool_distance_m', 'acceleration_variation_rad_s2')


def select_candidate(records):
    """Choose a complete plan only if all four totals are no worse than baseline.

    Equal weights on baseline-normalized metrics break ties among eligible
    improvements. A baseline zero must stay zero; unavailable/invalid metrics
    fail closed. The baseline wins numerical ties.
    """
    baseline = records[0]['totals']
    if any(not math.isfinite(baseline[key]) or baseline[key] < 0 for key in OBJECTIVES):
        raise ValueError('Invalid baseline metrics')
    best, best_score = 0, 4.0
    for index, record in enumerate(records):
        totals = record.get('totals')
        eligible = totals is not None and all(
            math.isfinite(totals[key]) and 0 <= totals[key] <= baseline[key]
            for key in OBJECTIVES)
        record['eligible'] = eligible
        record['score'] = None
        if not eligible:
            continue
        score = sum(totals[key] / baseline[key] if baseline[key] > 1e-9 else 1.0 for key in OBJECTIVES)
        record['score'] = score
        if score < best_score:
            best, best_score = index, score
    return best
