import json
import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from booth_bridge.state import BoothState, battery_verdict, interp_soc, rpy_matrix

CFG = yaml.safe_load((Path(__file__).resolve().parents[2] / 'config' / 'booth.yaml').read_text())
R = rpy_matrix(*[math.radians(a) for a in CFG['imu']['rpy_deg']])
R_IMU = np.array(CFG['imu']['xyz_m'])


@pytest.fixture
def st():
    s = BoothState(CFG)
    s.on_odom(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    return s


def feed_imu(s, t, w_base, a_base):
    w = np.asarray(w_base, float)
    a = np.asarray(a_base, float) + np.cross(w, np.cross(w, R_IMU))
    s.on_imu(t, t, (R.T @ w).tolist(), (R.T @ a).tolist())


def test_soc_table():
    table = CFG['battery']['soc_table']
    assert interp_soc(9.0, table) == 0
    assert interp_soc(14.0, table) == 100
    assert interp_soc(13.10, table) == pytest.approx(50)
    assert 60 < interp_soc(13.18, table) < 70


def test_battery_verdict_matches_artifact():
    """사용자 아티팩트 "배터리 잔량표"의 예시와 기준선."""
    b = CFG['battery']
    v = lambda topic: battery_verdict(topic, b['soc_table'], b['diode_drop_v'], b['charge_at_v'], b['stop_at_v'])  # noqa: E731
    # 10-31 실측: 토픽 12.82 V, 단자 13.09 V(다이오드 0.27 V). 단자 추정 13.12 V는 평평한 구간이다.
    r = v(12.82)
    assert r['zone'] == 'ok' and r['soc'] == pytest.approx(54) and (r['soc_lo'], r['soc_hi']) == (45, 65)
    assert v(12.59)['zone'] == 'charge' and v(12.59)['low']
    assert v(11.99)['zone'] == 'stop'
    assert v(13.40)['zone'] == 'charging' and v(13.40)['soc'] == 100
    assert v(13.30)['zone'] == 'ok'          # 단자 13.60, 경계 13.35 아래


def test_imu_mount_is_undone(st):
    # 센서 X가 뒤를 보도록 달려 있다. 로봇 기준으로 되돌리면 중력은 +z, 기울기 0
    for i in range(300):
        feed_imu(st, i * 0.01, [0, 0, 0], [0, 0, 9.81])
    imu = st.snapshot(3.0)['imu']
    assert abs(imu['roll']) < 0.05 and abs(imu['pitch']) < 0.05
    assert abs(imu['ax']) < 1e-3 and abs(imu['ay']) < 1e-3


def test_forward_acceleration_sign(st):
    for i in range(300):
        feed_imu(st, i * 0.01, [0, 0, 0], [0.3, 0, 9.81])
    assert st.snapshot(3.0)['imu']['ax'] == pytest.approx(0.3, abs=0.01)


def test_spin_in_place_does_not_read_as_acceleration(st):
    # 센서가 축 중점에서 16 cm 옆이라 2 rad/s면 구심 0.64 m/s². 빼고 나면 0이어야 한다
    for i in range(300):
        feed_imu(st, i * 0.01, [0, 0, 2.0], [0, 0, 9.81])
    imu = st.snapshot(3.0)['imu']
    assert imu['gyro_z'] == pytest.approx(2.0, abs=1e-6)
    assert abs(imu['ax']) < 0.01 and abs(imu['ay']) < 0.01


def test_fused_path_uses_gyro_heading(st):
    # 90도 돌고(자이로) 바퀴로 1 m 직진하면 자이로 경로는 +y로 간다
    for i in range(100):
        feed_imu(st, i * 0.01, [0, 0, math.pi / 2], [0, 0, 9.81])
    feed_imu(st, 1.0, [0, 0, 0], [0, 0, 9.81])
    r = CFG['robot']['wheel_radius_m']
    st.on_joint_states(1.0, [0.0, 0.0], [0.0, 0.0])
    st.on_joint_states(1.1, [1.0 / r, 1.0 / r], [1.0, 1.0])
    f = st.snapshot(1.2)['fused']
    assert f['x'] == pytest.approx(0.0, abs=0.02)
    assert f['y'] == pytest.approx(1.0, abs=0.02)


def test_wheel_mismatch_percent(st):
    st.on_odom(0.0, 0, 0, 0, 0.0, 0.875)          # 바퀴가 12.5 % 적게 잰다
    for i in range(400):
        feed_imu(st, i * 0.01, [0, 0, 1.0], [0, 0, 9.81])
    assert st.snapshot(4.0)['imu']['mismatch_pct'] == pytest.approx(-12.5, abs=0.3)


def test_wheel_targets_from_cmd(st):
    st.on_cmd(0.0, 'out', 0.2, 0.0)
    w = st.snapshot(0.1)['wheels']
    expect = 0.2 / CFG['robot']['mcu_wheel_radius_m'] * CFG['robot']['wheel_radius_m']
    assert w['l']['target'] == pytest.approx(expect, abs=1e-4)
    assert w['r']['target'] == pytest.approx(expect, abs=1e-4)
    assert st.snapshot(1.0)['wheels']['l']['target'] is None     # 오래된 지령은 목표가 아니다


def test_mux_priority(st):
    st.on_cmd(0.0, 'follow', 0.1, 0)
    st.on_cmd(0.0, 'teleop', 0.2, 0)
    assert st.snapshot(0.1)['mux']['active'] == 'teleop'
    st.on_cmd(0.4, 'follow', 0.1, 0)
    assert st.snapshot(0.45)['mux']['active'] == 'follow'


def test_battery_nan_and_filter(st):
    st.on_base_status(0.0, {'supply_voltage': float('nan'), 'agent_state': 2})
    assert st.snapshot(0.1)['battery'] is None
    st.on_base_status(0.2, {'supply_voltage': 12.9, 'agent_state': 2})
    b = st.snapshot(0.3)['battery']
    # 화면의 V는 단자 추정(토픽 + 다이오드 0.30), 토픽 값은 따로
    assert b['v_topic'] == 12.9 and b['v'] == 13.2 and b['soc'] == 70 and b['zone'] == 'ok'


def test_snapshot_is_strict_json(st):
    st.on_base_status(0.0, {'supply_voltage': float('nan'), 'duty_left': float('nan'), 'agent_state': 2})
    st.on_marker_track(0.0, {'status': 1, 'x': 1.0, 'y': 0.2, 'yaw': float('nan'), 'locked': True})
    st.on_detections(0.0, [(0, [1.5, 0.0, 0.9], [0, 0, 0, 1])])
    json.dumps(st.snapshot(0.1), allow_nan=False)


def test_ptt_requires_config_and_releases_on_stale_or_timeout():
    cfg = {**CFG, 'live': {'ptt_button': 4}}
    s = BoothState(cfg)
    s.on_joy(100, [0, 0, 0, 0, 1])
    assert s.snapshot(100.1)['ptt'] is True
    assert s.snapshot(100.6)['ptt'] is False
    s.on_joy(130.1, [0, 0, 0, 0, 1])
    assert s.snapshot(130.2)['ptt'] is False
    s.on_joy(131, [0, 0, 0, 0, 0])
    s.on_joy(132, [0, 0, 0, 0, 1])
    assert s.snapshot(132.1)['ptt'] is True
    s.on_joy(133, [])
    assert s.snapshot(133.1)['ptt'] is False
    disabled = BoothState({**CFG, 'live': {'ptt_button': None}})
    disabled.on_joy(100, [1] * 16)
    assert disabled.snapshot(100.1)['ptt'] is False


def test_pad_buttons_ptt_and_arm_stop():
    """패드 Y는 누르고 있는 동안 ptt, B는 눌리는 순간 한 번만 팔 멈춤을 부른다."""
    import inspect
    import booth_bridge.state as st
    cfg = make_cfg() if 'make_cfg' in globals() else None
    if cfg is None:
        import yaml
        from pathlib import Path
        cfg = yaml.safe_load((Path(inspect.getfile(st)).resolve().parents[2] / 'config' / 'booth.yaml').read_text(encoding='utf-8'))
    cfg['live']['ptt_button'] = 4
    cfg['arm']['stop_button'] = 1
    s = st.BoothState(cfg, sim=False)
    fired = []
    s.pad_stop_hook = lambda: fired.append(1)
    up = [0] * 12
    y = up.copy(); y[4] = 1
    b = up.copy(); b[1] = 1
    s.on_joy(1.0, up); assert not s.ptt_down
    s.on_joy(1.1, y); assert s.ptt_down
    s.on_joy(1.2, up); assert not s.ptt_down
    s.on_joy(1.3, b); s.on_joy(1.4, b); s.on_joy(1.5, up); s.on_joy(1.6, b)
    assert len(fired) == 2


def test_dpad_view_presses(st):
    presses = []
    st.dpad_hook = presses.append
    def joy(t, x, y):
        st.on_joy(t, [], [0] * 6 + [x, y])

    joy(1, 1, 0)  # 연결할 때 이미 누른 방향은 건너뛴다
    t = 1.0
    for x, y in [(1, 0), (0, 1), (-1, 0), (0, -1), (1, 1)]:
        joy(t + .1, 0, 0)
        joy(t + .2, x, y)
        joy(t + .3, x, y)
        t += .3
    joy(5, 0, -1)
    assert presses == ['left', 'up', 'right', 'down']


def test_lb_rt_gripper(st):
    lb = [0] * 6 + [1]

    def grip(t, rt, buttons=lb):
        st.on_joy(t, buttons, [0] * 4 + [rt])
        return st.pad_grip(t)

    assert grip(1, -1, []) is None
    assert grip(1.1, 1) is None
    assert grip(1.2, 0.95) == (1, 100)
    assert grip(1.3, 0) == (1, 53)
    assert grip(1.4, -1) == (1, 0)
    assert grip(1.5, 1) == (1, 100)
    assert grip(1.6, -1, []) is None
    assert grip(1.7, -1) == (2, 0)
    assert st.pad_grip(2.3) is None
