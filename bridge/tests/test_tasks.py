from pathlib import Path

import yaml

from booth_bridge.tasks import Tasks

CFG = yaml.safe_load((Path(__file__).resolve().parents[2] / 'config' / 'booth.yaml').read_text())


class Rig:
    """가짜 이동 노드와 가짜 팔. 순서기가 낸 명령을 적어 두고 상태를 손으로 바꾼다."""

    def __init__(self, arm=True):
        self.nav = {'state': 'idle', 'id': '', 'home': None, 'blocked': False, 'detail': ''}
        self.arm = {'errand': {'state': 'idle', 'object': '', 'events': []}} if arm else None
        self.mission = 'idle'
        self.sent, self.arm_sent, self.stops, self.snap = [], [], 0, None
        self.t = 0.0
        self.tasks = Tasks(CFG, nav_state=lambda: self.nav, arm_state=lambda: self.arm, mission_state=lambda: self.mission,
                           send_nav=self.send, arm_command=self.arm_command, stop_trip=self.stop, publish=self.publish)

    def send(self, cmd):
        self.sent.append(cmd)
        if cmd['cmd'] == 'mark_home':
            self.nav['home'] = [0.0, 0.0]
        elif cmd['cmd'].startswith('goto') or cmd['cmd'] in ('move', 'follow_marker'):
            self.nav.update(state='driving', id=cmd['id'], detail='')
        return True

    def arm_command(self, msg):
        self.arm_sent.append(msg)
        return {'ok': True}

    def stop(self):
        self.stops += 1
        self.mission = 'idle'

    def publish(self, snap):
        self.snap = snap

    def tick(self, n=1, dt=0.1):
        for _ in range(n):
            self.t += dt
            self.tasks.tick(self.t)

    def events(self):
        return [e['type'] for e in self.snap['events']]


def wait_fetch_thread(rig):
    import time
    for _ in range(50):
        if rig.arm_sent:
            break
        time.sleep(0.01)
    time.sleep(0.02)


def test_goto_marks_home_then_drives_to_the_place_marker():
    r = Rig()
    assert r.tasks.request({'cmd': 'goto', 'place': 'desk'}, r.t)['ok']
    r.tick(5)
    assert [c['cmd'] for c in r.sent] == ['mark_home', 'goto_marker']
    assert r.sent[1]['marker_id'] == CFG['nav']['places']['desk']['marker_id']
    assert r.snap['task']['state'] == 'running' and r.events() == ['go_start']
    r.nav['state'] = 'arrived'
    r.tick()
    assert r.snap['task']['state'] == 'done' and r.events() == ['go_start', 'arrived']


def test_unknown_place_and_dead_nav_are_refused_and_new_goal_replaces():
    r = Rig()
    assert not r.tasks.request({'cmd': 'goto', 'place': 'moon'}, r.t)['ok']
    assert not r.tasks.request({'cmd': 'home'}, r.t)['ok']            # 기억한 자리가 없다
    assert r.tasks.request({'cmd': 'goto', 'place': 'desk'}, r.t)['ok']
    assert r.tasks.request({'cmd': 'goto', 'place': 'door'}, r.t)['ok']         # 가는 중에 다른 곳을 시키면 그쪽으로 바꾼다
    assert r.snap['task']['place'] == 'door'
    dead = Rig()
    dead.nav = None
    assert '꺼져' in dead.tasks.request({'cmd': 'goto', 'place': 'desk'}, dead.t)['error']
    assert dead.sent == []


def test_fetch_from_goes_picks_and_comes_back_with_the_object():
    r = Rig()
    assert r.tasks.request({'cmd': 'fetch_from', 'place': 'desk', 'name': '약통'}, r.t)['ok']
    r.tick(5)
    r.nav['state'] = 'arrived'
    r.tick(2)
    wait_fetch_thread(r)
    assert r.arm_sent == [{'cmd': 'fetch', 'name': '약통'}]
    r.tick(3)
    assert r.snap['task']['state'] == 'running' and r.sent[-1]['cmd'] == 'goto_marker'   # 집을 때까지 기다린다
    r.arm['errand'].update(state='holding', object='약통')
    r.tick(2)
    assert r.sent[-1]['cmd'] == 'goto_home'
    r.nav['state'] = 'arrived'
    r.tick()
    assert r.snap['task']['state'] == 'done' and r.snap['task']['got'] == '약통'
    assert r.events() == ['go_start', 'arrived', 'fetching', 'return_start', 'returned']
    assert r.snap['events'][-1]['got'] == '약통'


def test_fetch_failure_still_returns_home_and_says_why():
    r = Rig()
    r.tasks.request({'cmd': 'fetch_from', 'place': 'desk', 'name': '귤'}, r.t)
    r.tick(5)
    r.nav['state'] = 'arrived'
    r.tick(2)
    wait_fetch_thread(r)
    r.tick()
    r.arm['errand']['events'].append({'seq': 1, 'type': 'not_found', 'text': '귤'})
    r.tick(2)
    assert r.sent[-1]['cmd'] == 'goto_home'
    r.nav['state'] = 'arrived'
    r.tick()
    t = r.snap['task']
    assert t['state'] == 'done' and t['got'] is None and t['problem'] == 'not_found'


def test_following_is_stopped_first_and_home_is_kept_across_places():
    r = Rig()
    r.mission = 'active'
    r.tasks.request({'cmd': 'goto', 'place': 'desk'}, r.t)
    r.tick(3)
    assert r.stops == 1 and r.sent == []                # 선 것을 확인하기 전에는 보내지 않는다
    r.tick(10)
    assert [c['cmd'] for c in r.sent] == ['mark_home', 'goto_marker']
    r.nav['state'] = 'arrived'
    r.tick()
    r.tasks.request({'cmd': 'goto', 'place': 'door'}, r.t)
    r.tick(5)
    assert [c['cmd'] for c in r.sent].count('mark_home') == 1      # 처음 떠난 자리를 그대로 둔다
    r.nav['state'] = 'arrived'
    r.tick()
    assert r.tasks.request({'cmd': 'home'}, r.t)['ok']
    r.tick(2)
    assert r.sent[-1]['cmd'] == 'goto_home'


def test_nav_failure_and_cancel_end_the_task():
    r = Rig()
    r.tasks.request({'cmd': 'goto', 'place': 'desk'}, r.t)
    r.tick(5)
    r.nav.update(state='failed', detail='not_found')
    r.tick()
    assert r.snap['task']['state'] == 'failed' and r.snap['events'][-1]['say'] == '마커를 찾지 못했어요'
    r.tasks.request({'cmd': 'goto', 'place': 'desk'}, r.t)
    r.tick(5)
    assert r.tasks.cancel('멈춤')
    assert r.sent[-1] == {'cmd': 'cancel'} and r.snap['task']['state'] == 'cancelled'
    assert not r.tasks.cancel()


def test_silent_nav_node_times_out():
    r = Rig()
    r.send = lambda cmd: True                           # 명령은 나가지만 노드가 받지 못한다
    r.tasks.send_nav = r.send
    r.nav['home'] = [0.0, 0.0]
    r.tasks.request({'cmd': 'goto', 'place': 'desk'}, r.t)
    r.tick(50)
    assert r.snap['task']['state'] == 'failed' and r.snap['task']['detail'] == 'no_reply'


def test_refuses_when_camera_is_dead_or_odometry_has_not_arrived():
    r = Rig()
    r.tasks.markers_alive = lambda: False
    assert '카메라' in r.tasks.request({'cmd': 'goto', 'place': 'desk'}, r.t)['error']
    r.nav['home'] = [1.0, 0.0]
    assert r.tasks.request({'cmd': 'home'}, r.t)['ok']             # 돌아오기는 카메라 없이도 된다
    r2 = Rig()
    r2.nav['odom'] = False
    assert '준비' in r2.tasks.request({'cmd': 'goto', 'place': 'desk'}, r2.t)['error']
    assert r2.sent == []


# ------------------------------------------------------------ 조금 움직이기·번호 마커
def test_move_back_sends_a_signed_distance_and_reports():
    r = Rig()
    reply = r.tasks.request({'cmd': 'move', 'dist_m': -0.3}, r.t)
    assert reply['ok'] and '뒤로 30 cm' in reply['title']
    r.tick(5)
    assert [c['cmd'] for c in r.sent] == ['mark_home', 'move'] and r.sent[1]['dist_m'] == -0.3
    r.nav['state'] = 'arrived'
    r.tick()
    assert r.snap['task']['state'] == 'done' and r.events() == ['move_start', 'moved']


def test_move_is_clamped_and_needs_an_amount():
    r = Rig()
    assert not r.tasks.request({'cmd': 'move'}, r.t)['ok']
    assert not r.tasks.request({'cmd': 'move', 'dist_m': 'much'}, r.t)['ok']
    assert r.tasks.request({'cmd': 'move', 'dist_m': 9, 'turn_deg': -900}, r.t)['ok']
    r.tick(5)
    assert r.sent[-1]['dist_m'] == 1.0 and r.sent[-1]['turn_deg'] == -360.0


def test_goto_marker_by_number_aligns_and_uses_place_name_when_known():
    r = Rig()
    assert r.tasks.request({'cmd': 'goto_marker', 'marker_id': 7}, r.t)['ok']
    r.tick(5)
    assert r.sent[-1] == {'cmd': 'goto_marker', 'marker_id': 7, 'standoff_m': CFG['nav']['standoff_m'], 'align': True, 'id': r.sent[-1]['id']}
    assert r.snap['task']['label'] == '7번 마커'
    r2 = Rig()
    desk = CFG['nav']['places']['desk']
    assert r2.tasks.request({'cmd': 'goto_marker', 'marker_id': desk['marker_id']}, r2.t)['ok']
    assert r2.snap['task']['label'] == desk['label']
    assert not Rig().tasks.request({'cmd': 'goto_marker', 'marker_id': 77}, 0.0)['ok']
    assert not Rig().tasks.request({'cmd': 'goto_marker', 'marker_id': None}, 0.0)['ok']


def test_follow_marker_runs_until_stopped_and_stops_badge_follow_first():
    r = Rig()
    r.mission = 'active'
    assert r.tasks.request({'cmd': 'follow_marker', 'marker_id': 3}, r.t)['ok']
    r.tick(12)
    assert r.stops == 1 and [c['cmd'] for c in r.sent] == ['mark_home', 'follow_marker']
    assert r.sent[-1]['range_m'] == CFG['nav']['follow_range_m']
    r.nav['state'] = 'following'
    r.tick(50)
    assert r.snap['task']['state'] == 'running' and r.snap['task']['detail'] == '따라가는 중'
    r.nav['guard'] = {'hold': True}
    r.tick()
    assert r.snap['task']['detail'] == '가까이 온 사람이 있어 멈춤'
    assert r.tasks.cancel('멈춤')
    assert r.sent[-1] == {'cmd': 'cancel'} and r.events() == ['follow_start', 'cancelled']


def test_follow_marker_lost_is_reported():
    r = Rig()
    r.tasks.request({'cmd': 'follow_marker', 'marker_id': 3}, r.t)
    r.tick(5)
    r.nav.update(state='failed', detail='lost')
    r.tick()
    assert r.snap['task']['state'] == 'failed' and r.snap['events'][-1]['say'] == '따라가던 마커를 놓쳤어요'


def test_new_request_replaces_a_running_move_quietly():
    r = Rig()
    r.tasks.request({'cmd': 'follow_marker', 'marker_id': 3}, r.t)
    r.tick(5)
    first = r.snap['task']['id']
    assert r.tasks.request({'cmd': 'goto', 'place': 'desk'}, r.t)['ok']
    assert r.snap['task']['id'] != first and r.snap['task']['state'] == 'running'
    assert 'cancelled' not in r.events()                    # "멈췄어요"를 말하지 않는다
    r.tick(5)
    assert [c['cmd'] for c in r.sent][-2:] == ['cancel', 'goto_marker']     # 떠나 있으니 자리는 다시 기억하지 않는다
    # 잘못된 요청은 하던 일을 건드리지 않는다
    assert not r.tasks.request({'cmd': 'goto', 'place': 'moon'}, r.t)['ok']
    assert r.snap['task']['state'] == 'running'


def test_request_during_pick_is_refused():
    r = Rig()
    r.tasks.request({'cmd': 'fetch_from', 'place': 'desk', 'name': '귤'}, r.t)
    r.tick(5)
    r.nav['state'] = 'arrived'
    r.tick(2)
    wait_fetch_thread(r)
    reply = r.tasks.request({'cmd': 'move', 'dist_m': -0.2}, r.t)
    assert not reply['ok'] and '집는 중' in reply['error']
