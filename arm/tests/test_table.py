"""작업대 높이 마커(table.py): 가짜 작업대에 마커를 그려 놓고 높이를 잰다. 팔·카메라가 필요 없다."""
import time
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
CFG = yaml.safe_load((ROOT / 'config' / 'arm.yaml').read_text(encoding='utf-8'))
pytestmark = pytest.mark.skipif(
    not Path(CFG['arm']['urdf']).expanduser().exists() or not Path(CFG['cameras']['top']['intrinsics']).expanduser().exists(),
    reason='URDF나 탑뷰 내부 파라미터 파일 없음')


def make(tmp_path, true_z, marker=True, default=0.0):
    from maknae_arm.__main__ import App
    cfg = yaml.safe_load((ROOT / 'config' / 'arm.yaml').read_text(encoding='utf-8'))
    cfg['motions_dir'] = str(tmp_path)
    cfg['vision']['debug_dir'] = str(tmp_path / 'pick')
    cfg['arm']['speed_deg_s'] = cfg['arm']['slow_deg_s'] = 2000
    cfg['workspace']['table_z_m'] = default
    cfg['workspace']['table_marker']['enabled'] = marker
    cfg['sim'] = {'table_z_m': true_z}
    a = App(cfg, fake=True)
    a.errand.argon.delay = 0.05
    return a


def wait(app, state, timeout=20.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if app.errand.state == state and not app.errand.busy.locked():
            return
        time.sleep(0.05)
    raise AssertionError(f'{state}가 되지 않았다: {app.errand.state} {app.errand.detail}')


@pytest.mark.parametrize('true_z', [0.0, 0.06, 0.12])
def test_marker_gives_table_height(tmp_path, true_z):
    app = make(tmp_path, true_z, default=0.03)
    try:
        time.sleep(0.4)
        t = app.errand.measure_table()
        assert t['source'] == 'marker' and abs(t['z'] - true_z) < 0.008, t
        assert app.errand.table_z() == t['z'] and app.state()['errand']['table']['source'] == 'marker'
        assert '기준표' in app.errand._scene_hint()
    finally:
        app.arm.close(); app.cams['top'].close()


def test_no_marker_uses_current_scene(tmp_path):
    app = make(tmp_path, 0.05, marker=True, default=0.088)
    try:
        time.sleep(0.4)
        assert app.errand.measure_table()['source'] == 'marker'
        scene = app.cams['top'].scene
        scene.marker_xy = None
        scene.z0 = 0.07  # 마커가 없는 다른 작업면으로 바뀐다
        time.sleep(0.4)
        t = app.errand.measure_table()
        assert t['source'] == 'unknown' and app.state()['errand']['table']['z'] is None
        assert 'support_depth_cm' in app.errand._scene_hint()
        app.command({'cmd': 'fetch', 'name': '귤'})
        wait(app, 'holding')
        assert app.errand.table['source'] == 'vision'
        assert app.errand.table_z() == pytest.approx(0.07)
    finally:
        app.arm.close(); app.cams['top'].close()


def test_fetch_uses_measured_height_and_tells_the_model(tmp_path):
    # 설정값은 0인데 진짜 작업대는 6 cm 높다. 마커로 재지 않으면 물건 자리가 어긋나고 집게가 작업대 속으로 내려간다
    app = make(tmp_path, 0.06, default=0.0)
    try:
        time.sleep(0.4)
        app.command({'cmd': 'fetch', 'name': '귤'})
        wait(app, 'holding')
        assert app.errand.table['source'] == 'marker' and abs(app.errand.table_z() - 0.06) < 0.008
        assert '8 cm' in app.errand.argon.hint
    finally:
        app.arm.close(); app.cams['top'].close()


def test_marker_is_not_listed_as_an_object(tmp_path):
    app = make(tmp_path, 0.04, default=0.04)
    try:
        time.sleep(0.4)
        scene = app.cams['top'].scene
        real = scene.detections
        u, v = app.view.project([scene.marker_xy[0], scene.marker_xy[1], scene.z0])
        scene.detections = lambda: real() + [{'label': '흑백 마커', 'u': u, 'v': v, 'height_m': 0.005}]
        app.command({'cmd': 'scan'})
        time.sleep(0.2)
        wait(app, 'idle')
        labels = [i['label'] for i in app.errand.inventory]
        assert '귤' in labels and '흑백 마커' not in labels
    finally:
        app.arm.close(); app.cams['top'].close()


def test_tilted_or_absurd_marker_is_rejected(tmp_path):
    app = make(tmp_path, 0.05)
    try:
        m = app.errand.marker
        assert not m.valid({'z': 0.05, 'tilt_deg': 40.0}) and not m.valid({'z': 0.6, 'tilt_deg': 2.0}) and not m.valid(None)
        assert m.valid({'z': 0.05, 'tilt_deg': 5.0})
    finally:
        app.arm.close(); app.cams['top'].close()
