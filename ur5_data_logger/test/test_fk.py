import math

import numpy as np
import pytest

from ur5_data_logger.fk import VerifiedFK


URDF = """
<robot name="test">
  <link name="base"/><link name="l1"/><link name="tcp"/>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="l1"/>
    <origin xyz="1 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
  </joint>
  <joint name="tip" type="fixed">
    <parent link="l1"/><child link="tcp"/><origin xyz="1 0 0" rpy="0 0 0"/>
  </joint>
</robot>
"""


def test_fk_uses_joint_origin_then_motion_and_fixed_tip():
    model = VerifiedFK(URDF, "base", "tcp", ("j1",))
    assert np.allclose(model.xyz([math.pi / 2.0]), [1.0, 1.0, 0.0], atol=1e-12)


def test_fk_rejects_unverified_joint_chain():
    with pytest.raises(ValueError, match="canonical order"):
        VerifiedFK(URDF, "base", "tcp", ("wrong_joint",))
