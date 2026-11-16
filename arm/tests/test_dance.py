"""장기자랑 팔춤. 안무(구간·박자·연속성)와 데몬 흐름(대기에서만, 중간에 끝내기)을 본다."""
import time
from pathlib import Path

import pytest
import yaml

from maknae_arm.dance import Choreography, peak_speed
from maknae_arm.kin import ARM_JOINTS

ROOT = Path(__file__).resolve().parents[2]
CFG = yaml.safe_load((ROOT / 'config' / 'arm.yaml').read_text(encoding='utf-8'))
D = CFG['dance']
REST = {**CFG['poses']['rest'], 'gripper': 5.0}


def test_sections_follow_the_l_hand_every_four_beats():
    c = Choreography(D)
    assert c.end == pytest.approx(12.83)
    for a, b, kind in c.sections[:-2]:
        assert (b - a) == pytest.approx(4 * c.beat)               # 4박마다 바뀐다
    assert c.at(0.8)['gripper'] == D['l_open']                    # 첫 L 한가운데
    assert c.at(2.4)['gripper'] == D['fist']
    assert c.at(12.5)['gripper'] == D['v_open']


def test_l_sways_left_then_right_on_the_beat():
    c = Choreography(D)
    pan0 = D['pose']['shoulder_pan']
    # 3.2 s(L 시작) = 8박. 9박에 한쪽 끝, 11박에 반대쪽 끝, 12박(구간 끝)에 가운데
    assert c.at(3.6)['shoulder_pan'] == pytest.approx(pan0 + D['sway_deg'], abs=0.5)
    assert c.at(4.4)['shoulder_pan'] == pytest.approx(pan0 - D['sway_deg'], abs=0.5)
    assert c.at(4.8)['shoulder_pan'] == pytest.approx(pan0, abs=0.5)


def test_trajectory_is_smooth_and_within_servo_speed():
    c = Choreography(D)
    steps = c.plan(REST, 0.05, 50, D['enter_s'], D['hold_s'])
    assert steps[-1][0] == pytest.approx(c.end + D['hold_s'], abs=0.03)
    top = peak_speed(steps)
    # STS3215가 12 V에서 내는 속도는 270 deg/s쯤이다. 여유를 둔다
    assert max(top[m] for m in ARM_JOINTS) < 200, top
    # 처음 한 걸음이 지금 자세에서 튀지 않는다
    first = steps[0][1]
    assert max(abs(first[m] - REST[m]) for m in ARM_JOINTS) < 1.0


def test_late_start_still_joins_gently():
    c = Choreography(D)
    steps = c.plan(REST, 6.0, 50, D['enter_s'])
    assert steps[0][0] > 6.0
    assert max(peak_speed(steps)[m] for m in ARM_JOINTS) < 250


# ---------------------------------------------------------------- 데몬 흐름 (가짜 팔)
needs_files = pytest.mark.skipif(
    not Path(CFG['arm']['urdf']).expanduser().exists() or not Path(CFG['cameras']['top']['intrinsics']).expanduser().exists(),
    reason='URDF나 탑뷰 내부 파라미터 파일 없음')


@pytest.fixture()
def app(tmp_path):
    from maknae_arm.__main__ import App
    cfg = yaml.safe_load((ROOT / 'config' / 'arm.yaml').read_text(encoding='utf-8'))
    cfg['motions_dir'] = str(tmp_path)
    cfg['arm']['speed_deg_s'] = cfg['arm']['slow_deg_s'] = 2000
    a = App(cfg, fake=True)
    a.errand.argon.delay = 0.05
    yield a
    a.arm.close()
    a.cams['top'].close()


def wait(app, state, timeout=20.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if app.errand.state == state and not app.errand.busy.locked():
            return
        time.sleep(0.05)
    raise AssertionError(f'{state}가 되지 않았다: {app.errand.state} {app.errand.detail}')


@needs_files
def test_dance_runs_to_the_music_and_folds(app):
    # 음악이 9초째인 것처럼 시작해 끝부분(L → 주먹 → V)만 춘다
    app.arm.record_start()
    r = app.command({'cmd': 'dance', 'start_epoch': time.time() - 9.0})
    assert r['started'] and r['seconds'] == pytest.approx(3.8, abs=0.1)
    time.sleep(0.2)
    assert app.errand.state == 'moving' and app.errand.detail == '장기자랑'
    wait(app, 'idle', 15)
    frames = app.arm.record_stop()
    grips = [g['gripper'] for _, g in frames]
    assert max(grips) > 95 and min(grips[len(grips) // 3:]) < 5      # L로 벌렸다가 주먹으로 닫았다
    assert [e['type'] for e in app.errand.events][-2:] == ['dance', 'dance_done']
    rest = CFG['poses']['rest']
    assert all(abs(app.arm.joints()[m] - rest[m]) < 1 for m in ARM_JOINTS)


@needs_files
def test_dance_stop_folds_without_error(app):
    app.command({'cmd': 'dance', 'start_epoch': time.time()})
    time.sleep(1.0)
    assert app.command({'cmd': 'dance_stop'}) == {'dancing': True}
    wait(app, 'idle', 10)
    assert 'stopped' not in [e['type'] for e in app.errand.events]
    assert app.command({'cmd': 'dance_stop'}) == {'dancing': False}


@needs_files
def test_dance_refused_while_holding_or_out_of_sync(app):
    with pytest.raises(Exception, match='시각'):
        app.command({'cmd': 'dance', 'start_epoch': time.time() - 30})
    app.command({'cmd': 'fetch', 'name': '귤'})
    wait(app, 'holding')
    with pytest.raises(Exception, match='들고 있어서'):
        app.command({'cmd': 'dance', 'start_epoch': time.time()})


@needs_files
def test_intro_gestures_return_to_start_without_opening_gripper(app):
    start = dict(app.arm.target)
    for name in ('hello', 'present', 'offer', 'look', 'nod'):
        app.arm.record_start()
        assert app.command({'cmd': 'intro_gesture', 'name': name}) == {'started': True}
        wait(app, 'idle', 5)
        frames = app.arm.record_stop()
        assert max(g['shoulder_lift'] - start['shoulder_lift'] for _, g in frames) >= 20
        assert all(g['gripper'] == pytest.approx(start['gripper']) for _, g in frames)
        if name == 'hello':
            tips = [app.kin.tip(app.arm.q(g)) for _, g in frames]
            assert max(p[0] for p in tips) > tips[0][0] + 0.12
            assert min(p[2] for p in tips) < tips[0][2] - 0.04
        assert app.arm.target == pytest.approx(start)
    app.errand.state = 'holding'
    with pytest.raises(Exception, match='자기소개'):
        app.command({'cmd': 'intro_gesture', 'name': 'offer'})
