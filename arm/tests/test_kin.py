import math
from pathlib import Path

import numpy as np
import pytest

from maknae_arm.kin import Kinematics

URDF = Path('~/dev/mobile-robot/hardware/mobile_base/models/sim/mobile_robot.urdf').expanduser()
pytestmark = pytest.mark.skipif(not URDF.exists(), reason='mobile-robot URDF 없음')


@pytest.fixture(scope='module')
def kin():
    return Kinematics(URDF)


def test_zero_pose_points_forward(kin):
    p = kin.tip(np.zeros(5))
    assert p[0] > 0.3 and abs(p[1]) < 0.01          # 영점에서 팔은 앞으로 뻗는다
    a = kin.approach(np.zeros(5))
    assert a[0] > 0.95                              # 그리퍼도 앞을 본다


def test_ik_roundtrip(kin):
    rng = np.random.default_rng(1)
    solved = 0
    for _ in range(60):
        q = rng.uniform(kin.lower * 0.8, kin.upper * 0.8)
        q[4] = 0.0
        pos, app = kin.tip(q), kin.approach(q)
        s = kin.ik(pos, app, roll=0.0)
        if s is None:
            continue
        solved += 1
        assert np.linalg.norm(kin.tip(s) - pos) < 0.002
    assert solved > 50


def test_ik_top_down_grasp(kin):
    down = np.array([0.0, 0.0, -1.0])
    for x, y in ((0.2, 0.0), (0.26, 0.10), (0.24, -0.14), (0.29, 0.0)):
        s = kin.ik([x, y, 0.02], down, roll=0.0)
        assert s is not None, (x, y)
        assert np.linalg.norm(kin.tip(s) - [x, y, 0.02]) < 0.002
        assert kin.approach(s)[2] < -0.9
        assert np.all(s >= kin.lower) and np.all(s <= kin.upper)


def test_unreachable_returns_none(kin):
    assert kin.ik([0.8, 0.0, 0.0], [0, 0, -1.0]) is None


def test_offsets_roundtrip():
    k = Kinematics(URDF, offset_deg={'elbow_flex': 3.0})
    deg = {'shoulder_pan': 1.0, 'shoulder_lift': -20.0, 'elbow_flex': 30.0, 'wrist_flex': 40.0, 'wrist_roll': 5.0}
    back = k.to_lerobot(k.to_urdf(deg))
    assert all(math.isclose(back[n], deg[n], abs_tol=1e-9) for n in deg)
