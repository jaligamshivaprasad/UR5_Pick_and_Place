from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))

from robotiq_mimic_publisher import MIMIC_JOINTS, measured_mimic_positions


def test_returns_measured_follower_positions_in_urdf_order():
    names = [
        'robotiq_85_left_knuckle_joint',
        'robotiq_85_right_knuckle_joint_mimic',
        'robotiq_85_left_inner_knuckle_joint_mimic',
        'robotiq_85_right_inner_knuckle_joint_mimic',
        'robotiq_85_left_finger_tip_joint_mimic',
        'robotiq_85_right_finger_tip_joint_mimic',
    ]
    positions = [0.4, -0.39, 0.38, -0.37, -0.36, 0.35]

    assert measured_mimic_positions(names, positions) == positions[1:]


def test_missing_follower_feedback_does_not_fabricate_position():
    names = ['robotiq_85_left_knuckle_joint']
    positions = [0.4]

    assert measured_mimic_positions(names, positions) is None


def test_mismatched_joint_and_position_arrays_are_rejected():
    names = ['robotiq_85_right_knuckle_joint_mimic']
    positions = []

    assert measured_mimic_positions(names, positions, MIMIC_JOINTS[:1]) is None


def test_non_finite_follower_feedback_is_rejected():
    names = ['robotiq_85_right_knuckle_joint_mimic']
    positions = [float('nan')]

    assert measured_mimic_positions(names, positions, MIMIC_JOINTS[:1]) is None
