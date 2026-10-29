"""Small, read-only URDF FK implementation used only after chain validation."""

from dataclasses import dataclass
import math
from typing import Dict, List
import xml.etree.ElementTree as ET

import numpy as np


def _numbers(text, default):
    return [float(value) for value in (text or default).split()]


def _origin(element):
    xyz = _numbers(element.get("xyz") if element is not None else None, "0 0 0")
    r, p, y = _numbers(element.get("rpy") if element is not None else None, "0 0 0")
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    rotation = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = xyz
    return transform


def _motion(joint_type, axis, value):
    transform = np.eye(4)
    axis = np.asarray(axis, dtype=float)
    if joint_type == "prismatic":
        transform[:3, 3] = axis * value
        return transform
    norm = np.linalg.norm(axis)
    if norm == 0.0:
        raise ValueError("movable joint has a zero-length axis")
    x, y, z = axis / norm
    c, s, one_c = math.cos(value), math.sin(value), 1.0 - math.cos(value)
    transform[:3, :3] = np.array([
        [c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s],
        [y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s],
        [z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c],
    ])
    return transform


@dataclass(frozen=True)
class Joint:
    name: str
    joint_type: str
    parent: str
    child: str
    origin: np.ndarray
    axis: List[float]


class VerifiedFK:
    def __init__(self, urdf_xml: str, base_link: str, tcp_link: str, expected_joints):
        root = ET.fromstring(urdf_xml)
        by_child: Dict[str, Joint] = {}
        for element in root.findall("joint"):
            joint = Joint(
                name=element.get("name", ""),
                joint_type=element.get("type", "fixed"),
                parent=element.find("parent").get("link"),
                child=element.find("child").get("link"),
                origin=_origin(element.find("origin")),
                axis=_numbers(element.find("axis").get("xyz") if element.find("axis") is not None else None, "1 0 0"),
            )
            if joint.child in by_child:
                raise ValueError(f"URDF child link has multiple parents: {joint.child}")
            by_child[joint.child] = joint

        reversed_chain = []
        current = tcp_link
        visited = set()
        while current != base_link:
            if current in visited or current not in by_child:
                raise ValueError(f"no unambiguous URDF chain from {base_link} to {tcp_link}")
            visited.add(current)
            joint = by_child[current]
            reversed_chain.append(joint)
            current = joint.parent
        self.chain = list(reversed(reversed_chain))
        movable = [joint.name for joint in self.chain if joint.joint_type != "fixed"]
        if movable != list(expected_joints):
            raise ValueError(f"FK chain joints {movable!r} do not match canonical order {list(expected_joints)!r}")
        self.base_link = base_link
        self.tcp_link = tcp_link
        self.joint_names = tuple(expected_joints)

    def xyz(self, positions):
        values = dict(zip(self.joint_names, positions))
        transform = np.eye(4)
        for joint in self.chain:
            transform = transform @ joint.origin
            if joint.joint_type != "fixed":
                if joint.joint_type not in ("revolute", "continuous", "prismatic"):
                    raise ValueError(f"unsupported joint type {joint.joint_type}")
                transform = transform @ _motion(joint.joint_type, joint.axis, values[joint.name])
        return transform[:3, 3].copy()
