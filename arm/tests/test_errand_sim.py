"""가짜 작업대(sim.py)로 심부름 상태머신을 끝까지 돌려 본다. 팔·카메라·인터넷이 필요 없다."""
import time
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
CFG = yaml.safe_load((ROOT / 'config' / 'arm.yaml').read_text(encoding='utf-8'))
pytestmark = pytest.mark.skipif(
    not Path(CFG['arm']['urdf']).expanduser().exists() or not Path(CFG['cameras']['top']['intrinsics']).expanduser().exists(),
    reason='URDF나 탑뷰 내부 파라미터 파일 없음')


@pytest.fixture()
def app(tmp_path):
    from maknae_arm.__main__ import App
    cfg = yaml.safe_load((ROOT / 'config' / 'arm.yaml').read_text(encoding='utf-8'))
    cfg['motions_dir'] = str(tmp_path)
    cfg['vision']['debug_dir'] = str(tmp_path / 'pick')
    cfg['arm']['speed_deg_s'] = cfg['arm']['slow_deg_s'] = 2000        # 시험에서는 빨리 움직인다
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


def kinds(app):
    return [e['type'] for e in app.errand.events]


def test_fetch_give_roundtrip(app):
    assert app.command({'cmd': 'fetch', 'name': '귤'}) == {'started': True}
    wait(app, 'holding')
    assert app.errand.object == '귤' and app.arm.joints()['gripper'] > 8
    with pytest.raises(Exception, match='들고 있어요'):
        app.command({'cmd': 'fetch', 'name': '약통'})                 # 들고 있는 동안에는 다른 것을 집지 않는다
    app.command({'cmd': 'give'})
    wait(app, 'idle')
    assert kinds(app)[-5:] == ['looking', 'picking', 'holding', 'released', 'done']


def test_give_refused_when_empty(app):
    with pytest.raises(Exception, match='들고 있는 물건이 없어요'):
        app.command({'cmd': 'give'})


def test_not_found_and_unreachable(app):
    app.command({'cmd': 'fetch', 'name': '지갑'})
    wait(app, 'idle')
    assert kinds(app)[-1] == 'not_found'
    app.command({'cmd': 'fetch', 'name': '물병'})
    wait(app, 'idle')
    assert kinds(app)[-1] == 'unreachable'


def test_grasp_point_obstructed(app, monkeypatch):
    monkeypatch.setattr(app.errand.argon, 'find', lambda frame, name, hint='': {
        'found': True, 'grasp_clear': False, 'reason': '집게 위치와 손가락이 겹침'})
    app.command({'cmd': 'fetch', 'name': '흰색 테이프'})
    wait(app, 'idle')
    assert kinds(app) == ['looking', 'shown']
    assert '집게가 내려갈 자리' in app.errand.events[-1]['reason']


def test_presented_object_picked_with_one_request(app, monkeypatch):
    find = app.errand.argon.find
    monkeypatch.setattr(app.errand.argon, 'find', lambda frame, name, hint='': {
        **find(frame, name), 'grasp_clear': True})
    app.command({'cmd': 'fetch', 'name': '귤'})
    wait(app, 'holding')
    assert kinds(app) == ['looking', 'picking', 'holding']


def test_only_requested_vision_calls(app):
    time.sleep(2)  # 기존 자동 스캔의 장면 안정 대기 시간을 넘긴다
    assert app.errand.argon.calls == 0
    app.command({'cmd': 'scan'})
    wait(app, 'idle')
    assert {i['label'] for i in app.errand.inventory} >= {'귤', '약통'}
    assert app.errand.argon.calls == 1
    app.command({'cmd': 'fetch', 'name': '약통'})
    wait(app, 'holding')
    assert app.errand.argon.calls == 2


def test_record_and_replay_motion(app):
    app.command({'cmd': 'record_start', 'name': '인사', 'source': 'hand'})
    assert app.errand.state == 'teleop' and not app.arm.torque
    app.arm.bus.pos.update({'shoulder_pan': 20.0}); time.sleep(0.3)
    app.arm.bus.pos.update({'shoulder_pan': -20.0}); time.sleep(0.3)
    saved = app.command({'cmd': 'record_stop'})
    assert saved['saved'] == '인사' and saved['frames'] > 5 and app.arm.torque
    assert '인사' in app.errand.motions()
    app.command({'cmd': 'motion', 'name': '인사'})
    wait(app, 'idle')
    assert 'motion_done' in kinds(app)


def test_throw_winds_back_then_opens_mid_swing(app):
    with pytest.raises(Exception, match='들고 있는 물건이 없어요'):
        app.command({'cmd': 'throw'})
    app.command({'cmd': 'fetch', 'name': '귤'})
    wait(app, 'holding')
    app.arm.record_start()
    assert app.command({'cmd': 'throw'}) == {'started': True}
    wait(app, 'idle')
    frames = app.arm.record_stop()
    assert kinds(app)[-3:] == ['throwing', 'thrown', 'done'] and app.errand.object == ''
    t = app.errand.cfg['throw']
    lift = [g['shoulder_lift'] for _, g in frames]
    back = min(range(len(lift)), key=lambda i: abs(lift[i] - t['back']['shoulder_lift']))
    opened = next(i for i, (_, g) in enumerate(frames) if g['gripper'] > 90)
    # 뒤로 뺀 다음에, 앞으로 뻗기 전에(휘두르는 도중에) 열린다
    assert opened > back
    assert t['back']['shoulder_lift'] + 15 < lift[opened] < t['front']['shoulder_lift'] - 5


def test_repeat_empty_pick_lowers_once(app, monkeypatch):
    errand = app.errand
    heights = []
    item = errand.argon.find(app.cams['top'].latest(), '귤')
    monkeypatch.setattr(errand.argon, 'find', lambda *args: {**item, 'box': None, 'height_m': 0.02})
    monkeypatch.setattr(errand, '_trim', lambda pos, roll: heights.append(pos[2] - errand.table_z()))
    monkeypatch.setattr(errand, '_gripper_settled', lambda *args: 0)
    for _ in range(3):
        app.command({'cmd': 'fetch', 'name': '귤'})
        wait(app, 'idle')
        assert kinds(app)[-1] == 'missed'
    assert heights == pytest.approx([0.008, 0.005, 0.008])


def test_reset_during_job_blocks_late_motion(app):
    import threading
    resume = threading.Event()

    def job():
        resume.wait(3)
        app.arm.gripper(100)

    app.errand._set('looking', '귤')
    app.errand.object = '귤'
    app.errand._run(job)
    before = dict(app.arm.target)
    assert app.command({'cmd': 'reset'})['pending']
    resume.set()
    wait(app, 'idle')
    assert app.arm.target == before
    assert app.errand.object == '' and app.errand.target is None
    assert app.command({'cmd': 'reset'}) == {'pending': False}
