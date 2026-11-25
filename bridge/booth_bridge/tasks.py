"""말로 시킨 일을 여러 동작으로 이어 붙이는 순서기. ROS를 import하지 않는다.

    goto        장소(마커)로 간다
    goto_marker 번호로 말한 마커 앞으로 가서 정면에 선다
    move        지금 자리에서 조금 움직인다(앞·뒤로 몇 cm, 왼쪽·오른쪽으로 몇 도)
    follow_marker  번호로 말한 마커를 계속 따라간다(멈추라고 할 때까지)
    fetch_from  지금 자세를 기억 → 장소(또는 번호 마커)로 간다 → 팔로 물건을 집는다 → 기억한 자리로 돌아와 그때 보던 쪽을 본다
                (hand_over면 돌아와서 건넨다. 아니면 들고 기다리다 "줘"라고 하면 건넨다)
    home        기억한 자리로 돌아온다
    fetch·give  하던 일 뒤에 "지금 자리에서 집기", "건네기"를 잇는다(하는 일이 없으면 팔에 바로 시키라고 답한다)
    cancel      하던 일을 그만둔다

일 하나는 단계의 줄이다(mark → nav → settle → fetch → nav → give …). 한 번에 한 단계만 하고, 각 단계는 그쪽의
보고(/nav/status, 팔 errand 상태)를 보고 넘어간다. 바퀴는 Pi의 errand_nav.py(/nav/command)가, 팔은 팔 데몬이 실제로
움직이고, 여기서는 "다음에 무엇을"만 정한다.

하는 중에 새 요청이 오면: 같은 말(turn)에서 나온 것은 뒤에 잇고, 다른 말에서 나온 이동은 하던 것을 그만두고 바꾼다
(팔이 물건을 집거나 건네는 중만 빼고). 음성 모델이 "가서, 집고, 돌아와"를 도구 셋으로 나눠 불러도 차례대로 한다.

- 따라가는 중이면 먼저 멈춤을 보내고 선 뒤에 시작한다.
- 물건을 못 집어도(없음, 안 닿음, 놓침) 돌아오기는 한다. 결과에 이유를 남긴다.
- 멈춤(대시보드 Space, 패드 B, 음성 "멈춰")은 어느 단계에서든 일을 끝낸다. errand_nav가 스스로 취소돼도 마찬가지다.

화면과 음성은 snapshot()의 events(seq가 늘어난다)를 보고 알림과 대사를 낸다.
"""
from __future__ import annotations

import logging
import threading
import time

log = logging.getLogger('booth.tasks')

BUSY_MISSION = ('starting', 'active', 'cancelling')
ARM_FAIL = ('shown', 'not_found', 'unreachable', 'missed', 'failed', 'stopped')
NAV_REASON = {
    'not_found': '마커를 찾지 못했어요', 'blocked': '앞이 막혀 있어요', 'timeout': '시간이 너무 오래 걸렸어요',
    'no_odom': '바퀴 위치를 알 수 없어요', 'no_home': '돌아갈 자리를 기억하지 못했어요',
    'no_reply': '이동 노드가 응답하지 않아요', 'lost': '따라가던 마커를 놓쳤어요',
}
MARKER_IDS = range(0, 50)               # DICT_4X4_50


class Tasks:

    def __init__(self, cfg: dict, *, nav_state, arm_state, mission_state, send_nav, arm_command, stop_trip, publish,
                 markers_alive=lambda: True, layout=None):
        """콜백: nav_state() → /nav/status dict|None, arm_state() → 팔 상태 dict|None, mission_state() → str|None,
        send_nav(dict) → bool, arm_command(dict) → dict, stop_trip(), publish(snapshot)"""
        nav = cfg.get('nav', {})
        self.enabled = bool(nav.get('enabled'))
        self.places = {}
        for key, p in (nav.get('places') or {}).items():
            self.places[str(key)] = {
                'key': str(key), 'label': str(p.get('label', key)), 'marker_id': int(p['marker_id']),
                'standoff_m': float(p.get('standoff_m', nav.get('standoff_m', 0.6))),
                'aliases': [str(a) for a in p.get('aliases', [])],
                'align': bool(p.get('align', nav.get('align_places', False))),
            }
        self.standoff_m = float(nav.get('standoff_m', 0.6))
        self.follow_range_m = float(nav.get('follow_range_m', 1.0))
        self.nudge_m = float(nav.get('nudge_m', 0.3))
        self.turn_deg = float(nav.get('turn_deg', 90.0))
        self.follow_marker_id = int(cfg.get('stack', {}).get('marker_id', -1))
        self.reply_timeout_s = float(nav.get('reply_timeout_s', 3.0))
        self.fetch_timeout_s = float(nav.get('fetch_timeout_s', 90.0))
        self.fetch_standoff_m = float(nav.get('fetch_standoff_m', 0.42))
        self.fetch_align = bool(nav.get('fetch_align', True))
        self.settle_s = float(nav.get('settle_s', 0.8))
        self.nav_state, self.arm_state, self.mission_state = nav_state, arm_state, mission_state
        self.send_nav, self.arm_command, self.stop_trip, self.publish = send_nav, arm_command, stop_trip, publish
        self.markers_alive = markers_alive      # 마커 검출(/aruco_detections)이 들어오고 있는가
        self.layout = layout                    # 부스 배치(layout.py). 없으면 장소는 마커로만 간다
        if layout is not None:
            layout.changed = self._publish
        self.lock = threading.RLock()
        self.arm_dispatch = threading.Lock()
        self.arm_enabled = cfg.get('arm', {}).get('enabled', False)
        self.seq = 0
        self.n = 0
        self.events: list[dict] = []
        self.task: dict | None = None
        self.away = False                       # 기억한 자리를 떠나 있다
        self.steps: list[dict] = []
        self.i = 0
        self._step_at = 0.0
        self._started = False
        self._publish()

    # ------------------------------------------------------------ 요청
    def request(self, msg: dict, now: float | None = None) -> dict:
        """일을 시킨다. 하는 중에 온 요청은 셋 중 하나다.

        잇기    같은 말(turn이 같다)에서 나온 요청, 그리고 팔 일(fetch·give)은 하던 일 뒤에 붙인다.
                "책상으로 가서 약통 집고 돌아와"를 음성 모델이 도구 셋으로 나눠 불러도 차례대로 한다.
        바꾸기  다른 말에서 나온 이동 요청은 하던 이동을 그만두고 새로 한다("아니, 현관으로 가").
        거절    팔이 물건을 집거나 건네는 중에는 바꾸지 못한다.
        """
        now = time.monotonic() if now is None else now
        cmd = msg.get('cmd')
        turn = msg.get('turn')
        with self.lock:
            if cmd == 'reset':
                self.send_nav({'cmd': 'cancel'})
                self.task = None
                self.steps.clear()
                self.i = 0
                self._started = False
                self._step_at = 0.0
                self.events.clear()
                self._publish()
                with self.arm_dispatch:
                    if self.arm_enabled:
                        return self.arm_command({'cmd': 'reset'})
                return {'ok': True}
            if cmd == 'cancel':
                return {'ok': self.cancel('요청')}
            if not self.enabled:
                # 팔 일은 이동 기능이 꺼져 있어도 된다. 팔에 바로 시키라고 답한다
                return {'ok': False, 'direct': cmd in ('fetch', 'give'), 'error': '이동 기능이 꺼져 있어요.'}
            endless = self.running and self.task['kind'] == 'follow_marker'
            append = self.running and not endless and (cmd in ('fetch', 'give') or (turn is not None and turn == self.task.get('turn')))
            if cmd in ('fetch', 'give') and not append:
                if endless:
                    return {'ok': False, 'error': '따라가는 중이에요. 먼저 멈추라고 말해 주세요.'}
                return {'ok': False, 'direct': True, 'error': '하는 일이 없어요.'}      # 팔에 바로 시키면 된다
            if self.running and not append and self.steps[self.i]['kind'] in ('fetch', 'give', 'wait_arm'):
                return {'ok': False, 'error': '지금은 물건을 집는 중이에요. 끝난 뒤에 다시 말해 주세요.'}
            nav = self.nav_state()
            if nav is None:
                return {'ok': False, 'error': '이동 노드가 꺼져 있어요.'}
            if not nav.get('odom', True):
                return {'ok': False, 'error': '로봇이 아직 준비 중이에요. 잠시 뒤에 다시 말해 주세요.'}
            plan = self._plan(cmd, msg, nav, append)
            if 'error' in plan:
                return {'ok': False, 'error': plan['error']}
            if append:
                # same_turn: 같은 말에서 나온 요청이다(출발 안내가 이미 나갔으니 음성 모델은 조용히 있는다)
                return {**self._append(plan), 'same_turn': turn is not None and turn == self.task.get('turn')}
            return self._start(plan, now, turn)

    def _plan(self, cmd: str, msg: dict, nav: dict, append: bool) -> dict:
        """요청 하나를 단계 목록으로 푼다. {'kind', 'title', 'steps', 'place', 'object'} 또는 {'error'}."""
        arm = self.arm_state()
        arm_state = (arm or {}).get('errand', {}).get('state')
        after_arm = bool(msg.get('after_arm'))          # 같은 말에서 팔에 먼저 시킨 일(집기)이 끝난 뒤에 움직인다
        steps: list[dict] = []
        if cmd == 'give':
            return {'kind': 'give', 'title': '건네기', 'steps': [{'kind': 'give'}], 'place': None, 'object': ''}
        if cmd == 'fetch':
            name = str(msg.get('name', '')).strip()[:40]
            if not name:
                return {'error': '무엇을 가져올지 이름이 없어요.'}
            if arm is None:
                return {'error': '로봇팔이 연결돼 있지 않아요.'}
            return {'kind': 'fetch', 'title': f'{name} 집기', 'steps': [{'kind': 'settle'}, {'kind': 'fetch', 'name': name}],
                    'place': None, 'object': name}
        # 마커로 가는 일은 앞 카메라가 살아 있어야 한다. 꺼져 있으면 제자리에서 한 바퀴 헛돌고 끝난다
        by_pose = self.layout is not None and self.layout.place_odom(str(msg.get('place', ''))) is not None
        if cmd in ('goto', 'fetch_from', 'goto_marker', 'follow_marker') and not self.markers_alive() \
                and not (by_pose and cmd in ('goto', 'fetch_from')):
            return {'error': '앞 카메라가 꺼져 있어서 마커를 볼 수 없어요.'}
        if not append and not after_arm and arm is not None and arm_state not in ('idle', 'holding'):
            return {'error': '팔이 움직이는 중이라 지금은 출발할 수 없어요.'}
        if not append and self.mission_state() in BUSY_MISSION:
            steps.append({'kind': 'stop_follow'})
            self.away = False                   # 따라 걷다 온 곳이 새 출발점이다
        if after_arm and not append:
            steps.append({'kind': 'wait_arm'})
        # 돌아올 자리("돌아와"가 가는 곳)는 처음 떠날 때 한 번만 기억한다. 책상에 갔다가 현관으로 가도 "돌아와"는 처음 자리다.
        marked = self.running and any(s['kind'] == 'mark' and s['slot'] == 'home' for s in self.steps)
        leave = [] if (self.away and (nav.get('home') is not None or marked)) else [{'kind': 'mark', 'slot': 'home'}]
        # 배치 패널에서 시작 위치를 지정했으면 "돌아와"는 언제나 그 자리다
        start = self.layout.anchor if self.layout is not None and self.layout.start_set else None
        if cmd == 'home':
            if start is None and nav.get('home') is None and not marked:
                return {'error': '돌아갈 자리를 아직 기억하지 못했어요.'}
            steps.append({'kind': 'nav', 'cmd': {'cmd': 'goto_home'} if start is None else
                          {'cmd': 'goto_pose', 'x': start[0], 'y': start[1], 'yaw': start[2]},
                          'event': 'return_start', 'done': 'returned', 'dest': ('home',), 'of': 'home', 'clears_away': True})
            return {'kind': 'home', 'title': '시작 위치로 돌아가기' if start is not None else '제자리로 돌아가기', 'steps': steps,
                    'place': None, 'object': ''}
        if cmd == 'move':
            try:
                dist, turn = float(msg.get('dist_m') or 0.0), float(msg.get('turn_deg') or 0.0)
            except (TypeError, ValueError):
                return {'error': '거리나 각도를 알아듣지 못했어요.'}
            dist, turn = max(-1.0, min(1.0, dist)), max(-360.0, min(360.0, turn))
            if abs(dist) < 0.02 and abs(turn) < 3.0:
                return {'error': '얼마나 움직일지 알아듣지 못했어요.'}
            parts = []
            if abs(dist) >= 0.02:
                parts.append(f'{"앞으로" if dist > 0 else "뒤로"} {abs(dist) * 100:.0f} cm 가기')
            if abs(turn) >= 3.0:
                parts.append(f'{"왼쪽" if turn > 0 else "오른쪽"}으로 {abs(turn):.0f}도 돌기')
            steps += leave + [{'kind': 'nav', 'cmd': {'cmd': 'move', 'dist_m': dist, 'turn_deg': turn},
                               'event': 'move_start', 'done': 'moved', 'of': 'move'}]
            return {'kind': 'move', 'title': ', '.join(parts), 'steps': steps, 'place': None, 'object': ''}
        # 목적지: 이름으로 말한 장소, 또는 번호로 말한 마커
        place, mid = self.places.get(str(msg.get('place', ''))), None
        if place is None and (cmd in ('goto_marker', 'follow_marker') or msg.get('marker_id') not in (None, '')):
            try:
                mid = int(msg.get('marker_id'))
            except (TypeError, ValueError):
                mid = -1
            if mid not in MARKER_IDS:
                return {'error': '마커 번호는 0번부터 49번까지예요.'}
            if cmd == 'fetch_from':
                place = next((p for p in self.places.values() if p['marker_id'] == mid), None)   # 장소의 마커면 장소로 간다
        if cmd in ('goto_marker', 'follow_marker'):
            known = next((p for p in self.places.values() if p['marker_id'] == mid), None)
            spot = {'key': known['key'] if known else None, 'label': known['label'] if known else f'{mid}번 마커'}
            if cmd == 'goto_marker':
                standoff = float(known['standoff_m'] if known else self.standoff_m)
                steps += leave + [{'kind': 'nav', 'cmd': {'cmd': 'goto_marker', 'marker_id': mid, 'standoff_m': standoff,
                                                         'align': bool(msg.get('align', True))},
                                   'event': 'go_start', 'done': 'arrived', 'dest': ('marker', mid), 'of': 'goto', 'label': spot['label']}]
                return {'kind': 'goto', 'title': f'{spot["label"]} 앞으로 가기', 'steps': steps, 'place': spot, 'object': ''}
            if append:
                return {'error': '따라가기는 다른 일과 이어서 할 수 없어요. 따로 말해 주세요.'}
            steps += leave + [{'kind': 'nav', 'cmd': {'cmd': 'follow_marker', 'marker_id': mid, 'range_m': self.follow_range_m},
                               'event': 'follow_start', 'done': 'arrived', 'of': 'follow_marker', 'label': spot['label']}]
            return {'kind': 'follow_marker', 'title': f'{mid}번 마커 따라가기', 'steps': steps, 'place': spot, 'object': ''}
        if cmd not in ('goto', 'fetch_from'):
            return {'error': '모르는 명령이에요.'}
        if place is None and mid is None:
            return {'error': f'모르는 장소예요. 아는 곳: {", ".join(p["label"] for p in self.places.values()) or "없음"}'}
        if place is not None:
            label, dest = place['label'], ('marker', place['marker_id'])
            see = {'kind': 'nav', 'cmd': {'cmd': 'goto_marker', 'marker_id': place['marker_id'], 'standoff_m': place['standoff_m'],
                                          'align': place['align']}, 'event': 'go_start', 'done': 'arrived', 'dest': dest, 'label': label}
            spot = self.layout.place_odom(place['key']) if self.layout is not None else None
            if spot is None:
                go = [see]
            else:
                # 배치에 자리가 있다: 그 자리까지 오도메트리로 가고, 마커가 보이면 마커로 마무리한다(안 보여도 도착으로 친다)
                finish = self.layout.uses_marker(place['key']) and self.markers_alive()
                go = [{'kind': 'nav', 'cmd': {'cmd': 'goto_pose', 'x': spot[0], 'y': spot[1], 'yaw': spot[2]},
                       'event': 'go_start', 'done': None if finish else 'arrived', 'dest': dest, 'label': label}]
                if finish:
                    go.append({**see, 'event': None, 'optional': True})
        else:
            # 번호로 말한 마커 앞의 작업대. 팔이 닿게 가까이, 정면으로 선다
            label, dest, place = f'{mid}번 마커', ('marker', mid), {'key': None, 'label': f'{mid}번 마커'}
            go = [{'kind': 'nav', 'cmd': {'cmd': 'goto_marker', 'marker_id': mid, 'align': True,
                                          'standoff_m': self.fetch_standoff_m if cmd == 'fetch_from' else self.standoff_m},
                   'event': 'go_start', 'done': 'arrived', 'dest': dest, 'label': label}]
        if cmd == 'goto':
            for s in go:
                s['of'] = 'goto'
            return {'kind': 'goto', 'title': f'{label}(으)로 가기', 'steps': steps + leave + go, 'place': place, 'object': ''}
        name = str(msg.get('name', '')).strip()[:40]
        if not name:
            return {'error': '무엇을 가져올지 이름이 없어요.'}
        if arm is None:
            return {'error': '로봇팔이 연결돼 있지 않아요.'}
        if not append and not after_arm and arm_state == 'holding':
            return {'error': '이미 물건을 들고 있어요. 먼저 받아 주세요.'}
        for s in go:
            s['of'] = 'fetch_from'
            if s['cmd']['cmd'] == 'goto_marker' and self.fetch_align:
                # 팔이 작업대를 정면으로 봐야 물건이 집는 범위에 들어온다. 비스듬히 다가가면 가까이 서는 목표는
                # 책상 자체에 막혀 실패하기도 한다(nav_core는 정면 지점으로 먼저 간 뒤 곧게 다가간다)
                s['cmd'] = {**s['cmd'], 'align': True}
        # 다녀오는 심부름은 "시킨 그 자리, 그때 보던 쪽"으로 돌아온다. 그 전에 말로 조금 움직였거나 돌았어도
        # (그때 기억한 "돌아와" 자리는 그대로 두고) 지금 자세를 따로 기억했다가 그리로 온다.
        steps += [{'kind': 'mark', 'slot': 'trip'}] + leave + go + [{'kind': 'settle'}, {'kind': 'fetch', 'name': name}]
        back = msg.get('come_back', True) is not False
        if back:
            steps.append({'kind': 'nav', 'cmd': {'cmd': 'goto_home', 'slot': 'trip'}, 'event': 'return_start', 'done': 'returned',
                          'dest': ('trip',), 'of': 'fetch_from', 'clears_away': bool(leave)})
        if msg.get('hand_over'):
            steps.append({'kind': 'give'})
        return {'kind': 'fetch_from', 'title': f'{label}에서 {name} 가져오기' if back else f'{label}에 가서 {name} 집기',
                'steps': steps, 'place': place, 'object': name}

    def _start(self, plan: dict, now: float, turn=None) -> dict:
        if self.running:
            # 하던 이동은 조용히 그만둔다. 새 목표가 이동 노드의 것을 바로 덮는다
            log.info('작업 %s을(를) 그만두고 새 일을 한다', self.task['id'])
            self.send_nav({'cmd': 'cancel'})
            self._end('cancelled', 'replaced', None)
        place = plan['place']
        self.n += 1
        self.task = {'id': f't{self.n}', 'kind': plan['kind'], 'title': plan['title'], 'state': 'running', 'detail': '',
                     'place': place['key'] if place else None, 'label': place['label'] if place else None,
                     'object': plan['object'], 'got': None, 'problem': '', 'turn': turn}
        self.steps, self.i, self._started = plan['steps'], 0, False
        self.away = plan['kind'] != 'home' or self.away
        log.info('작업 시작 %s: %s (%s)', self.task['id'], plan['title'], ' → '.join(s['kind'] for s in self.steps))
        self._publish()
        return {'ok': True, 'task': self.task['id'], 'title': plan['title']}

    def _append(self, plan: dict) -> dict:
        """하던 일 뒤에 잇는다. 이미 들어 있는 것과 겹치는 단계(같은 곳으로 또 가기, 또 돌아오기, 또 건네기)는 뺀다."""
        t, rest, steps = self.task, self.steps[self.i:], list(plan['steps'])
        last_nav = next((s for s in reversed(rest) if s['kind'] == 'nav'), None)
        tail = rest[-1]['kind'] if rest else ''
        if steps and steps[0]['kind'] == 'mark' and steps[0]['slot'] == 'trip' and last_nav is not None and last_nav.get('dest') \
                and any(s['kind'] == 'nav' and s.get('dest') == last_nav['dest'] for s in steps):
            # "책상으로 가"에 이어 "책상에서 약통 가져와"가 왔다. 이미 가는 중이니 다시 가지 않고, 돌아올 곳은 처음 떠난 자리다
            steps = [s for s in steps if not (s['kind'] == 'mark' or (s['kind'] == 'nav' and s.get('dest') == last_nav['dest']))]
            for s in steps:
                if s['kind'] == 'nav' and s.get('dest') == ('trip',):
                    s['cmd'], s['dest'], s['clears_away'] = {'cmd': 'goto_home'}, ('home',), True
        elif last_nav is not None and last_nav.get('dest') and plan['kind'] in ('goto', 'home'):
            steps = [s for s in steps if not (s['kind'] == 'nav' and s.get('dest') == last_nav['dest'])]
        if plan['kind'] == 'home' and last_nav is not None and last_nav.get('done') == 'returned':
            steps = []                          # 이미 돌아오기로 돼 있다
        if plan['kind'] == 'give' and tail == 'give':
            steps = []
        if steps and rest and steps[-1]['kind'] == 'give' and tail == 'give':
            steps.pop()
        if not steps:
            return {'ok': True, 'task': t['id'], 'title': t['title'], 'chained': True}
        # 건네기는 언제나 맨 끝이다("가져와서 줘" 뒤에 "돌아와"가 붙어도 돌아온 다음에 건넨다)
        if tail == 'give' and plan['kind'] != 'give':
            self.steps[-1:-1] = steps
        else:
            self.steps += steps
        if plan['kind'] != 'give':
            t['title'] = f'{t["title"]} → {plan["title"]}'
        if plan['object']:
            t['object'] = plan['object']
        log.info('작업 %s에 잇는다: %s (%s)', t['id'], plan['title'], ' → '.join(s['kind'] for s in self.steps[self.i:]))
        self._publish()
        return {'ok': True, 'task': t['id'], 'title': t['title'], 'chained': True}

    def cancel(self, why: str = '') -> bool:
        with self.lock:
            if not self.running:
                return False
            self.send_nav({'cmd': 'cancel'})
            self._end('cancelled', why, 'cancelled')
            return True

    @property
    def running(self) -> bool:
        return self.task is not None and self.task['state'] == 'running'

    # ------------------------------------------------------------ 진행
    def tick(self, now: float | None = None) -> None:
        now = time.monotonic() if now is None else now
        with self.lock:
            if not self.running:
                return
            step = self.steps[self.i]
            first = not self._started
            if first:
                self._started, self._step_at = True, now
            before = self.task['detail']
            done = getattr(self, f'_step_{step["kind"]}')(step, first, now)
            if self.running and not done and self.task['detail'] != before:
                self._publish()                 # 화면의 진행 문구를 바꾼다
            if done and self.running:
                self.i += 1
                self._started = False
                self.task['detail'] = ''
                if self.i >= len(self.steps):
                    t = self.task
                    self._end('done', t['problem'], None)
                else:
                    self._publish()

    def _step_stop_follow(self, step: dict, first: bool, now: float) -> bool:
        if first:
            self.stop_trip()
        # 임무 노드가 멈춤을 확인하면 cancelled → idle로 간다. 못 받아도 3 s 뒤에는 넘어간다(nav가 우선순위에서 이긴다).
        stopped = self.mission_state() not in BUSY_MISSION
        return (stopped and now - self._step_at > 0.5) or now - self._step_at > 3.0

    def _step_mark(self, step: dict, first: bool, now: float) -> bool:
        if first:
            self.send_nav({'cmd': 'mark_home', 'slot': step['slot']})
        return now - self._step_at > 0.2

    def _step_settle(self, step: dict, first: bool, now: float) -> bool:
        """막 선 뒤 차체 흔들림이 가라앉고 팔 카메라가 새 작업대를 볼 때까지 잠깐 기다린다."""
        self.task['detail'] = '물건을 찾을 준비 중'
        return now - self._step_at > self.settle_s

    def _step_wait_arm(self, step: dict, first: bool, now: float) -> bool:
        """같은 말에서 팔에 먼저 시킨 일(지금 자리에서 집기)이 끝나기를 기다린다. 못 집어도 다음으로 간다."""
        arm = self.arm_state()
        errand = (arm or {}).get('errand') or {}
        if first:
            events = errand.get('events') or []
            step['seq'] = events[-1]['seq'] if events else 0
        self.task['detail'] = '물건을 집는 중'
        if now - self._step_at < 0.6:
            return False                        # 팔 상태가 "찾는 중"으로 바뀐 것이 여기까지 오는 데 걸리는 시간
        state = errand.get('state')
        if state == 'holding':
            self.task['got'] = errand.get('object') or self.task['object'] or None
            return True
        if arm is None:
            self.task['problem'] = 'arm_offline'
            return True
        if state in ('idle', 'error'):
            fails = [e['type'] for e in errand.get('events') or [] if e.get('seq', 0) > step['seq'] and e.get('type') in ARM_FAIL]
            self.task['problem'] = fails[-1] if fails else ''
            return True
        if now - self._step_at > self.fetch_timeout_s:
            self.task['problem'] = 'arm_timeout'
            return True
        return False

    def _dispatch_arm(self, step: dict, msg: dict) -> None:
        # 초기화와 HTTP 전송 순서를 맞춘다. 이미 지운 단계의 요청은 보내지 않는다.
        with self.arm_dispatch:
            if any(s is step for s in self.steps):
                step['reply'] = self.arm_command(msg)

    def _step_give(self, step: dict, first: bool, now: float) -> bool:
        """들고 있는 것을 건넨다. 들고 있지 않으면(못 집었다) 그냥 넘어간다."""
        arm = self.arm_state()
        errand = (arm or {}).get('errand') or {}
        if first:
            if errand.get('state') != 'holding':
                return True
            threading.Thread(target=self._dispatch_arm, args=(step, {'cmd': 'give'}),
                             name='task-give', daemon=True).start()
            return False
        self.task['detail'] = '건네는 중'
        reply = step.get('reply')
        if reply is not None and not reply.get('ok'):
            self.task['problem'] = str(reply.get('error') or '팔이 거절했어요')
            return True
        if reply is not None and errand.get('state') == 'idle':
            return True
        return arm is None or now - self._step_at > 20.0

    def _step_nav(self, step: dict, first: bool, now: float) -> bool:
        goal_id = f'{self.task["id"]}-{self.i}'
        if first:
            if not self.send_nav({**step['cmd'], 'id': goal_id}):
                self._fail('no_reply')
                return False
            if step['event']:
                self._event(step['event'])
        nav = self.nav_state()
        mine = nav is not None and nav.get('id') == goal_id
        if not mine:
            if now - self._step_at > self.reply_timeout_s:
                self._fail('no_reply')
            elif now - self._step_at > 1.0 and not step.get('resent'):
                step['resent'] = True           # 무선에서 명령 하나가 늦거나 빠져도 일 전체가 끝나지 않게 한 번 더 보낸다
                log.warning('이동 노드가 %s을(를) 아직 받지 않았다. 다시 보낸다', goal_id)
                self.send_nav({**step['cmd'], 'id': goal_id})
            return False
        state = nav.get('state')
        self.task['detail'] = '조이스틱으로 조종 중(놓으면 이어서 간다)' if nav.get('paused') else '앞이 막혀 기다리는 중' if nav.get('blocked') else \
            '가까이 온 사람이 있어 멈춤' if (nav.get('guard') or {}).get('hold') else \
            '마커를 놓쳐 기다리는 중' if state == 'following' and nav.get('detail') == 'lost' else \
            {'searching': '마커를 찾는 중', 'approaching': '다가가는 중', 'aligning': '마커 정면으로 옮기는 중', 'driving': '가는 중',
             'turning': '방향을 맞추는 중', 'following': '따라가는 중'}.get(state, '')
        missed = state == 'failed' and step.get('optional') and nav.get('detail') in ('not_found', 'timeout')
        if state == 'arrived' or missed:
            if step['done'] == 'returned' and step.get('clears_away', True):
                self.away = False
            if step['done']:
                self._event(step['done'])
            return True
        if state == 'failed':
            self._fail(str(nav.get('detail') or 'failed'))
        elif state == 'cancelled':
            self._end('cancelled', str(nav.get('detail') or ''), 'cancelled')
        return False

    def _step_fetch(self, step: dict, first: bool, now: float) -> bool:
        arm = self.arm_state()
        errand = (arm or {}).get('errand') or {}
        if first:
            events = errand.get('events') or []
            step['seq'] = events[-1]['seq'] if events else 0
            self._event('fetching')
            # 막는 호출(HTTP)이라 따로 돌린다. 그동안에도 멈춤을 받을 수 있어야 한다. 팔 데몬이 할 수 있는지 다시 검사한다.
            threading.Thread(target=self._dispatch_arm, args=(step, {'cmd': 'fetch', 'name': step['name']}),
                             name='task-fetch', daemon=True).start()
            return False
        reply = step.get('reply')
        if reply is None:
            if now - self._step_at > 8.0:
                self.task['problem'] = 'arm_offline'
                return True
            return False
        if not reply.get('ok'):
            self.task['problem'] = str(reply.get('error') or '팔이 거절했어요')
            return True                         # 못 집어도 돌아간다
        if errand.get('state') == 'holding':
            self.task['got'] = errand.get('object') or step['name']
            return True
        for e in errand.get('events') or []:
            if e.get('seq', 0) > step['seq'] and e.get('type') in ARM_FAIL:
                self.task['problem'] = e['type']
                return True
        if arm is None or now - self._step_at > self.fetch_timeout_s:
            self.task['problem'] = 'arm_timeout' if arm is not None else 'arm_offline'
            return True
        self.task['detail'] = '물건을 집는 중'
        return False

    # ------------------------------------------------------------ 끝·보고
    def _fail(self, reason: str) -> None:
        self._end('failed', reason, 'failed')

    def _end(self, state: str, detail: str, event: str | None) -> None:
        t = self.task
        t['state'], t['detail'] = state, detail
        if event:
            self._event(event, reason=detail)
        log.info('작업 끝 %s: %s %s', t['id'], state, detail)
        self._publish()

    def _event(self, kind: str, **extra) -> None:
        self.seq += 1
        t = self.task
        # 이어 붙인 일은 단계마다 목적지와 종류가 다르다. 지금 단계의 것을 먼저 쓴다
        step = self.steps[self.i] if self.i < len(self.steps) else {}
        e = {'seq': self.seq, 'type': kind, 'task': step.get('of') or t['kind'], 'label': step.get('label') or t['label'],
             'object': t['object'], 'got': t['got'], 'problem': t['problem'],
             'give': any(s['kind'] == 'give' for s in self.steps[self.i + 1:]), **extra}
        if kind == 'failed':
            e['say'] = NAV_REASON.get(extra.get('reason', ''), '이동하다 문제가 생겼어요')
        self.events = (self.events + [e])[-12:]
        self._publish()

    def snapshot(self) -> dict:
        with self.lock:
            t = self.task
            return {
                'enabled': self.enabled,
                # carry: 물건을 집는 중이거나 집었고, 아직 더 가야 한다(들고 돌아가는 길이다. "가져왔어요"는 도착해서 말한다)
                'task': None if t is None else {**t, 'step': min(self.i + 1, len(self.steps)), 'steps': len(self.steps), 'carry': self._carry()},
                'events': list(self.events),
                'layout': None if self.layout is None else self.layout.snapshot(),
            }

    def _carry(self) -> bool:
        if not self.running:
            return False
        picked = any(s['kind'] in ('fetch', 'wait_arm') for s in self.steps[:self.i + 1])
        return picked and any(s['kind'] == 'nav' for s in self.steps[self.i:])

    def _publish(self) -> None:
        self.publish(self.snapshot())

    def options(self) -> dict:
        """화면과 음성 도구가 알아야 할 값."""
        return {'enabled': self.enabled, 'follow_marker_id': self.follow_marker_id, 'nudge_m': self.nudge_m, 'turn_deg': self.turn_deg}

    def place_list(self) -> list[dict]:
        return [{'key': p['key'], 'label': p['label'], 'marker_id': p['marker_id'], 'aliases': p['aliases']}
                for p in self.places.values()]


class TaskRunner(threading.Thread):
    """순서기를 10 Hz로 돌린다."""

    def __init__(self, tasks: Tasks):
        super().__init__(name='tasks', daemon=True)
        self.tasks = tasks

    def run(self) -> None:
        while True:
            try:
                self.tasks.tick()
            except Exception:  # noqa: BLE001 — 한 단계가 터져도 순서기는 살아 있어야 멈춤을 받는다
                log.exception('작업 단계 오류')
                self.tasks.cancel('오류')
            time.sleep(0.1)
