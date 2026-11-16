import math

import yaml

from booth_bridge.layout import Layout, compose, relative
from test_tasks import CFG, Rig


def test_compose_and_relative_are_inverse():
    a, p = (1.0, 2.0, math.pi / 2), (0.5, -0.3, 0.4)
    back = relative(a, compose(a, p))
    assert all(abs(u - v) < 1e-9 for u, v in zip(back, p))
    assert all(abs(u - v) < 1e-9 for u, v in zip(compose(a, (1.0, 0.0, 0.0)), (1.0, 3.0, math.pi / 2)))


def test_place_taught_by_parking_survives_restart_and_follows_the_start(tmp_path):
    f = tmp_path / 'layout.yaml'
    lay = Layout(f, ['desk', 'door'])
    assert not lay.request({'op': 'set_place', 'key': 'desk', 'here': True}, 0.0)['ok']       # 로봇 자리를 모른다
    lay.on_pose(1.0, 2.0, 1.0, math.pi / 2)
    assert lay.request({'op': 'set_place', 'key': 'desk', 'here': True}, 1.0)['ok']
    assert not lay.request({'op': 'set_place', 'key': 'sofa', 'here': True}, 1.0)['ok']
    again = Layout(f, ['desk', 'door'])
    assert all(abs(u - v) < 1e-3 for u, v in zip(again.place_odom('desk'), (2.0, 1.0, math.pi / 2)))
    # 시작 위치를 옮기면 장소도 같이 옮겨 간다(시작 기준으로 적어 뒀다)
    again.on_pose(5.0, 1.0, 0.0, 0.0)
    assert again.request({'op': 'set_start'}, 5.0)['ok']
    assert all(abs(u - v) < 1e-3 for u, v in zip(again.place_odom('desk'), (3.0, 1.0, math.pi / 2)))
    assert yaml.safe_load(f.read_text())['start_set'] is True


def test_numbers_and_marker_switch_and_clear(tmp_path):
    lay = Layout(tmp_path / 'l.yaml', ['desk'])
    assert lay.request({'op': 'set_place', 'key': 'desk', 'x': 2, 'y': -1, 'yaw_deg': 90}, 0.0)['ok']
    assert lay.snapshot()['places']['desk']['rel'] == [2.0, -1.0, round(math.pi / 2, 4)]
    assert not lay.request({'op': 'set_place', 'key': 'desk', 'x': 99, 'y': 0}, 0.0)['ok']
    assert lay.uses_marker('desk')
    lay.request({'op': 'place_marker', 'key': 'desk', 'on': False}, 0.0)
    assert not lay.uses_marker('desk')
    lay.request({'op': 'clear_place', 'key': 'desk'}, 0.0)
    assert lay.place_odom('desk') is None


def test_start_is_dropped_when_odometry_restarts(tmp_path):
    lay = Layout(tmp_path / 'l.yaml', ['desk'])
    lay.on_pose(0.0, 3.0, 0.5, 1.0)
    lay.request({'op': 'set_start'}, 0.0)
    lay.on_pose(0.1, 3.01, 0.5, 1.0)
    assert lay.start_set
    lay.on_pose(20.0, 0.0, 0.0, 0.0)                    # 한참 뒤 0에서 다시 온다: 로봇을 다시 켰다
    assert not lay.start_set and lay.anchor == (0.0, 0.0, 0.0)


class LayRig(Rig):
    def __init__(self, tmp_path, alive=True):
        super().__init__()
        self.layout = Layout(tmp_path / 'l.yaml', list(CFG['nav']['places']))
        self.tasks.layout = self.layout
        self.tasks.markers_alive = lambda: alive


def test_place_with_a_spot_drives_there_by_odometry_then_finishes_on_the_marker(tmp_path):
    r = LayRig(tmp_path)
    r.layout.request({'op': 'set_place', 'key': 'desk', 'x': 2.0, 'y': 1.0, 'yaw_deg': 90}, 0.0)
    assert r.tasks.request({'cmd': 'goto', 'place': 'desk'}, r.t)['ok']
    r.tick(5)
    assert r.sent[-1]['cmd'] == 'goto_pose' and (r.sent[-1]['x'], r.sent[-1]['y']) == (2.0, 1.0)
    r.nav['state'] = 'arrived'
    r.tick(3)
    assert r.sent[-1]['cmd'] == 'goto_marker' and r.events() == ['go_start']      # 아직 도착이라고 말하지 않는다
    r.nav.update(state='failed', detail='not_found')                              # 마커가 안 보여도 자리에는 왔다
    r.tick()
    assert r.snap['task']['state'] == 'done' and r.events() == ['go_start', 'arrived']


def test_spot_without_marker_or_camera_needs_no_marker(tmp_path):
    r = LayRig(tmp_path, alive=False)
    assert not r.tasks.request({'cmd': 'goto', 'place': 'desk'}, r.t)['ok']       # 자리도 카메라도 없다
    r.layout.request({'op': 'set_place', 'key': 'desk', 'x': 1.0, 'y': 0.0}, 0.0)
    assert r.tasks.request({'cmd': 'goto', 'place': 'desk'}, r.t)['ok']
    r.tick(5)
    r.nav['state'] = 'arrived'
    r.tick()
    assert [c['cmd'] for c in r.sent] == ['mark_home', 'goto_pose'] and r.events() == ['go_start', 'arrived']


def test_return_goes_to_the_chosen_start(tmp_path):
    r = LayRig(tmp_path)
    r.layout.on_pose(0.0, 0.4, -0.2, 0.3)
    r.layout.request({'op': 'set_start'}, 0.0)
    assert r.tasks.request({'cmd': 'home'}, r.t)['ok']                            # 떠난 적이 없어도 시작 위치로는 갈 수 있다
    r.tick(3)
    assert r.sent[-1]['cmd'] == 'goto_pose' and (r.sent[-1]['x'], r.sent[-1]['y'], r.sent[-1]['yaw']) == (0.4, -0.2, 0.3)
    assert r.snap['layout']['start_set']
