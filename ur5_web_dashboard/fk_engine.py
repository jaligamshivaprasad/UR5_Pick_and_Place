"""Forward Kinematics computation engine for UR5 base_link -> tool0 kinematic chain.

Validates URDF structure before computing Cartesian coordinates to ensure
never fabricating end-effector positions.
"""

from dataclasses import dataclass
import math
from typing import Dict, List, Optional, Tuple
import xml.etree.ElementTree as ET

import numpy as np


def _parse_numbers(text: Optional[str], default: str) -> List[float]:
    return [float(val) for val in (text or default).split()]


def _origin_transform(element: Optional[ET.Element]) -> np.ndarray:
    xyz = _parse_numbers(element.get("xyz") if element is not None else None, "0 0 0")
    r, p, y = _parse_numbers(element.get("rpy") if element is not None else None, "0 0 0")
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    rotation = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=float)
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = rotation
    transform[:3, 3] = xyz
    return transform


def _motion_transform(joint_type: str, axis: List[float], val: float) -> np.ndarray:
    transform = np.eye(4, dtype=float)
    axis_arr = np.asarray(axis, dtype=float)
    if joint_type == "prismatic":
        transform[:3, 3] = axis_arr * val
        return transform
    norm = np.linalg.norm(axis_arr)
    if norm == 0.0:
        raise ValueError("movable joint has a zero-length axis")
    x, y, z = axis_arr / norm
    c = math.cos(val)
    s = math.sin(val)
    one_c = 1.0 - c
    transform[:3, :3] = np.array([
        [c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s],
        [y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s],
        [z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c],
    ], dtype=float)
    return transform


@dataclass(frozen=True)
class KinematicJoint:
    name: str
    joint_type: str
    parent: str
    child: str
    origin: np.ndarray
    axis: List[float]


class VerifiedFK:
    """Read-only Forward Kinematics engine verified against URDF."""

    def __init__(self, urdf_xml: str, base_link: str, tcp_link: str, expected_joints: List[str]):
        root = ET.fromstring(urdf_xml)
        by_child: Dict[str, KinematicJoint] = {}
        for elem in root.findall("joint"):
            name = elem.get("name", "")
            j_type = elem.get("type", "fixed")
            parent_elem = elem.find("parent")
            child_elem = elem.find("child")
            if parent_elem is None or child_elem is None:
                continue
            parent = parent_elem.get("link", "")
            child = child_elem.get("link", "")
            origin = _origin_transform(elem.find("origin"))
            axis_elem = elem.find("axis")
            axis = _parse_numbers(axis_elem.get("xyz") if axis_elem is not None else None, "1 0 0")
            by_child[child] = KinematicJoint(name, j_type, parent, child, origin, axis)

        reversed_chain = []
        current = tcp_link
        visited = set()
        while current != base_link:
            if current in visited or current not in by_child:
                raise ValueError(f"No unambiguous kinematic chain from {base_link} to {tcp_link}")
            visited.add(current)
            joint = by_child[current]
            reversed_chain.append(joint)
            current = joint.parent

        self.chain = list(reversed(reversed_chain))
        movable = [j.name for j in self.chain if j.joint_type != "fixed"]
        if movable != list(expected_joints):
            raise ValueError(
                f"FK chain movable joints {movable} do not match canonical order {list(expected_joints)}"
            )

        self.base_link = base_link
        self.tcp_link = tcp_link
        self.joint_names = tuple(expected_joints)

    def xyz(self, positions: List[float]) -> np.ndarray:
        """Compute Cartesian [x, y, z] in meters given canonical joint positions in radians."""
        if len(positions) != len(self.joint_names):
            raise ValueError(f"Expected {len(self.joint_names)} joint positions, got {len(positions)}")
        pos_map = dict(zip(self.joint_names, positions))
        transform = np.eye(4, dtype=float)
        for joint in self.chain:
            transform = transform @ joint.origin
            if joint.joint_type != "fixed":
                if joint.joint_type not in ("revolute", "continuous", "prismatic"):
                    raise ValueError(f"Unsupported joint type: {joint.joint_type}")
                transform = transform @ _motion_transform(joint.joint_type, joint.axis, pos_map[joint.name])
        return transform[:3, 3].copy()

