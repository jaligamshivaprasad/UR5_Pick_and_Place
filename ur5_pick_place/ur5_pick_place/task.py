"""Named-state sequence and request-local object geometry helpers."""

from copy import deepcopy
from dataclasses import dataclass
import math
import xml.etree.ElementTree as ET

from geometry_msgs.msg import Pose
from moveit_msgs.msg import AllowedCollisionEntry, JointConstraint, Constraints


@dataclass(frozen=True)
class Stage:
    name: str
    group: str
    state: str


def sequence(arm_group='arm', gripper_group='robotiq_gripper'):
    return (
        Stage('move_to_straight_before_pick', arm_group, 'straight'),
        Stage('open_before_pick', gripper_group, 'open'),
        Stage('move_to_pre_grasp', arm_group, 'pre_grasp'),
        Stage('descend_to_grasp', arm_group, 'grasp_pose'),
        Stage('grasp', gripper_group, 'close'),
        Stage('move_to_place', arm_group, 'place'),
        Stage('release', gripper_group, 'open'),
        Stage('retreat_after_place', arm_group, 'retreat_pose'),
        Stage('return_to_straight', arm_group, 'straight'),
    )


def named_states(srdf):
    result = {}
    for state in ET.fromstring(srdf).findall('group_state'):
        values = {j.attrib['name']: float(j.attrib['value']) for j in state.findall('joint')}
        if not values or not all(math.isfinite(v) for v in values.values()):
            raise ValueError('Empty or invalid named state: ' + state.attrib['name'])
        key = (state.attrib['group'], state.attrib['name'])
        if key in result:
            raise ValueError('Duplicate named state: ' + str(key))
        result[key] = values
    return result


def goal_constraints(name, joints, tolerance):
    return Constraints(
        name=name,
        joint_constraints=[
            JointConstraint(
                joint_name=joint, position=value, tolerance_above=tolerance,
                tolerance_below=tolerance, weight=1.0,
            )
            for joint, value in joints.items()
        ],
    )


def gripper_links(urdf, root_link):
    robot = ET.fromstring(urdf)
    if root_link not in {link.attrib['name'] for link in robot.findall('link')}:
        raise ValueError('Attachment link is absent from the robot model: ' + root_link)
    children = {}
    for joint in robot.findall('joint'):
        children.setdefault(joint.find('parent').attrib['link'], []).append(
            joint.find('child').attrib['link'])
    links, pending = [], [root_link]
    while pending:
        link = pending.pop()
        links.append(link)
        pending.extend(children.get(link, []))
    return links


def update_robot_state(state, trajectory, urdf):
    """Advance a hypothetical state, including the URDF's mimic joints."""
    if not trajectory.joint_trajectory.points:
        raise ValueError('Planner returned an empty joint trajectory')
    result = deepcopy(state)
    positions = dict(zip(result.joint_state.name, result.joint_state.position))
    positions.update(zip(trajectory.joint_trajectory.joint_names,
                         trajectory.joint_trajectory.points[-1].positions))
    mimics = [j for j in ET.fromstring(urdf).findall('joint') if j.find('mimic') is not None]
    for _ in range(len(mimics) + 1):
        for joint in mimics:
            mimic = joint.find('mimic')
            if mimic.attrib['joint'] in positions:
                positions[joint.attrib['name']] = (
                    positions[mimic.attrib['joint']] * float(mimic.get('multiplier', '1'))
                    + float(mimic.get('offset', '0')))
    result.joint_state.name = list(positions)
    result.joint_state.position = list(positions.values())
    result.joint_state.velocity = []
    result.joint_state.effort = []
    result.is_diff = False
    return result


def allow_contacts(matrix, object_id, links):
    """Copy the ACM and allow only the requested object's contact pairs."""
    result = deepcopy(matrix)
    for name in [object_id, *links]:
        if name not in result.entry_names:
            result.entry_names.append(name)
            for row in result.entry_values:
                row.enabled.append(False)
            result.entry_values.append(AllowedCollisionEntry(
                enabled=[False] * len(result.entry_names)))
    object_index = result.entry_names.index(object_id)
    for name in links:
        index = result.entry_names.index(name)
        result.entry_values[object_index].enabled[index] = True
        result.entry_values[index].enabled[object_index] = True
    return result


def _quaternion(pose):
    q = pose.orientation
    values = (q.x, q.y, q.z, q.w)
    length = math.sqrt(sum(v * v for v in values))
    # Existing CollisionObject messages sometimes leave the root pose unset.
    return tuple(v / length for v in values) if length else (0.0, 0.0, 0.0, 1.0)


def _multiply(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return (w * X + x * W + y * Z - z * Y,
            w * Y - x * Z + y * W + z * X,
            w * Z + x * Y - y * X + z * W,
            w * W - x * X - y * Y - z * Z)


def _rotate(q, vector):
    inverse = (-q[0], -q[1], -q[2], q[3])
    return _multiply(_multiply(q, (*vector, 0.0)), inverse)[:3]


def compose(a, b):
    """Pose multiplication: frame -> a -> b."""
    result = Pose()
    qa, qb = _quaternion(a), _quaternion(b)
    rotated = _rotate(qa, (b.position.x, b.position.y, b.position.z))
    result.position.x = a.position.x + rotated[0]
    result.position.y = a.position.y + rotated[1]
    result.position.z = a.position.z + rotated[2]
    q = _multiply(qa, qb)
    result.orientation.x, result.orientation.y, result.orientation.z, result.orientation.w = q
    return result


def inverse(pose):
    result = Pose()
    q = _quaternion(pose)
    q = (-q[0], -q[1], -q[2], q[3])
    p = _rotate(q, (-pose.position.x, -pose.position.y, -pose.position.z))
    result.position.x, result.position.y, result.position.z = p
    result.orientation.x, result.orientation.y, result.orientation.z, result.orientation.w = q
    return result
