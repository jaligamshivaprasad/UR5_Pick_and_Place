"""Check startup assembly and usable endpoints of the Robotiq linkage."""

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import xacro


MACRO = Path(__file__).resolve().parents[1] / 'urdf/robotiq_gripper_macro.xacro'


def gripper(*, ignition, prefix='', closed_position=0.7929):
    document = xacro.parse(f'''
      <robot xmlns:xacro="http://wiki.ros.org/xacro" name="gripper_test">
        <xacro:include filename="{MACRO}"/>
        <link name="world"/>
        <xacro:robotiq_gripper name="gripper" prefix="{prefix}" parent="world"
            sim_ignition="{str(ignition).lower()}"
            gripper_closed_position="{closed_position}">
          <origin xyz="0 0 0" rpy="0 0 0"/>
        </xacro:robotiq_gripper>
      </robot>
    ''')
    xacro.process_doc(document)
    return ET.fromstring(document.toxml())


@pytest.mark.parametrize('prefix', ['', 'test_'])
@pytest.mark.parametrize('closed_position', [0.7929, 0.6])
def test_ignition_starts_with_an_assembled_linkage(prefix, closed_position):
    robot = gripper(ignition=True, prefix=prefix, closed_position=closed_position)
    positions = {
        joint.get('name'): float(joint.findtext(
            "state_interface[@name='position']/param[@name='initial_value']", '0'))
        for joint in robot.findall('ros2_control/joint')
    }
    mimics = [joint for joint in robot.findall('joint') if joint.find('mimic') is not None]
    assert len(mimics) == 5
    for joint in mimics:
        mimic = joint.find('mimic')
        expected = (positions[mimic.get('joint')] * float(mimic.get('multiplier', '1'))
                    + float(mimic.get('offset', '0')))
        assert positions[joint.get('name')] == pytest.approx(expected), joint.get('name')


@pytest.mark.parametrize('position', [0.0, 0.4, 0.8])
def test_commanded_endpoints_do_not_touch_ignition_stops(position):
    robot = gripper(ignition=True)
    for side, sign in [('left', 1), ('right', -1)]:
        limit = robot.find(f"joint[@name='robotiq_85_{side}_knuckle_joint']/limit")
        assert float(limit.get('lower')) < sign * position < float(limit.get('upper'))


def test_hardware_keeps_its_original_stops_and_closed_calibration():
    robot = gripper(ignition=False, closed_position=0.7)
    for side, lower, upper in [('left', 0.0, 0.8), ('right', -0.8, 0.0)]:
        limit = robot.find(f"joint[@name='robotiq_85_{side}_knuckle_joint']/limit")
        assert float(limit.get('lower')) == lower
        assert float(limit.get('upper')) == upper
    initial = robot.findtext(
        "ros2_control/joint[@name='robotiq_85_left_knuckle_joint']/"
        "state_interface[@name='position']/param[@name='initial_value']")
    assert float(initial) == 0.7
