"""말로 시킨 일을 여러 동작으로 이어 붙이는 순서기. ROS를 import하지 않는다.

    goto        장소(마커)로 간다
    goto_marker 번호로 말한 마커 앞으로 가서 정면에 선다
    move        지금 자리에서 조금 움직인다(앞·뒤로 몇 cm, 왼쪽·오른쪽으로 몇 도)
    follow_marker  번호로 말한 마커를 계속 따라간다(멈추라고 할 때까지)
    fetch_from  지금 자리를 기억 → 장소로 간다 → 팔로 물건을 집는다 → 기억한 자리로 돌아온다
    home        기억한 자리로 돌아온다
    cancel      하던 일을 그만둔다

한 번에 하나만 한다. 하는 중에 새 일을 시키면 하던 이동을 그만두고 새 일을 한다(팔이 물건을 집는 중만 빼고). 바퀴는 Pi의 errand_nav.py(/nav/command)가, 팔은 팔 데몬이 실제로 움직이고, 여기서는
"다음에 무엇을"만 정한다. 각 단계는 그쪽의 보고(/nav/status, 팔 errand 상태)를 보고 넘어간다.

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
        self.nav_state, self.arm_state, self.mission_state = nav_state, arm_state, mission_state
        self.send_nav, self.arm_command, self.stop_trip, self.publish = send_nav, arm_command, stop_trip, publish
        self.markers_alive = markers_alive      # 마커 검출(/aruco_detections)이 들어오고 있는가
        self.layout = layout                    # 부스 배치(layout.py). 없으면 장소는 마커로만 간다
        if layout is not None:
            layout.changed = self._publish
        self.lock = threading.RLock()
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
        now = time.monotonic() if now is None else now
        cmd = msg.get('cmd')
        with self.lock:
            if cmd == 'cancel':
                return {'ok': self.cancel('요청')}
            if not self.enabled:
                return {'ok': False, 'error': '이동 기능이 꺼져 있어요.'}
            if self.running and self.steps[self.i]['kind'] == 'fetch':
                return {'ok': False, 'error': '지금은 물건을 집는 중이에요. 끝난 뒤에 다시 말해 주세요.'}
            nav = self.nav_state()
            if nav is None:
                return {'ok': False, 'error': '이동 노드가 꺼져 있어요.'}
            if not nav.get('odom', True):
                return {'ok': False, 'error': '로봇이 아직 준비 중이에요. 잠시 뒤에 다시 말해 주세요.'}
            # 마커로 가는 일은 앞 카메라가 살아 있어야 한다. 꺼져 있으면 제자리에서 한 바퀴 헛돌고 끝난다
            by_pose = self.layout is not None and self.layout.place_odom(str(msg.get('place', ''))) is not None
            if cmd in ('goto', 'fetch_from', 'goto_marker', 'follow_marker') and not self.markers_alive() \
                    and not (by_pose and cmd in ('goto', 'fetch_from')):
                return {'ok': False, 'error': '앞 카메라가 꺼져 있어서 마커를 볼 수 없어요.'}
            arm = self.arm_state()
            if arm is not None and arm.get('errand', {}).get('state') not in ('idle', 'holding'):
                return {'ok': False, 'error': '팔이 움직이는 중이라 지금은 출발할 수 없어요.'}
            steps: list[dict] = []
            if self.mission_state() in BUSY_MISSION:
                steps.append({'kind': 'stop_follow'})
                self.away = False               # 따라 걷다 온 곳이 새 출발점이다
            # 돌아올 자리는 처음 떠날 때 한 번만 기억한다. 책상에 갔다가 현관으로 가도 "돌아와"는 처음 자리다.
            leave = [] if self.away and nav.get('home') is not None else [{'kind': 'mark_home'}]
            # 배치 패널에서 시작 위치를 지정했으면 "돌아와"는 언제나 그 자리다
            start = self.layout.anchor if self.layout is not None and self.layout.start_set else None
            back = {'kind': 'nav', 'cmd': {'cmd': 'goto_home'} if start is None else
                    {'cmd': 'goto_pose', 'x': start[0], 'y': start[1], 'yaw': start[2]}, 'event': 'return_start', 'done': 'returned'}
            if cmd == 'home':
                if start is None and nav.get('home') is None:
                    return {'ok': False, 'error': '돌아갈 자리를 아직 기억하지 못했어요.'}
                steps.append(back)
                return self._start('home', '시작 위치로 돌아가기' if start is not None else '제자리로 돌아가기', steps, now, place=None)
            if cmd == 'move':
                try:
                    dist, turn = float(msg.get('dist_m') or 0.0), float(msg.get('turn_deg') or 0.0)
                except (TypeError, ValueError):
                    return {'ok': False, 'error': '거리나 각도를 알아듣지 못했어요.'}
                dist, turn = max(-1.0, min(1.0, dist)), max(-360.0, min(360.0, turn))
                if abs(dist) < 0.02 and abs(turn) < 3.0:
                    return {'ok': False, 'error': '얼마나 움직일지 알아듣지 못했어요.'}
                parts = []
                if abs(dist) >= 0.02:
                    parts.append(f'{"앞으로" if dist > 0 else "뒤로"} {abs(dist) * 100:.0f} cm 가기')
                if abs(turn) >= 3.0:
                    parts.append(f'{"왼쪽" if turn > 0 else "오른쪽"}으로 {abs(turn):.0f}도 돌기')
                steps += leave + [{'kind': 'nav', 'cmd': {'cmd': 'move', 'dist_m': dist, 'turn_deg': turn},
                                   'event': 'move_start', 'done': 'moved'}]
                return self._start('move', ', '.join(parts), steps, now, place=None)
            if cmd in ('goto_marker', 'follow_marker'):
                try:
                    mid = int(msg.get('marker_id'))
                except (TypeError, ValueError):
                    mid = -1
                if mid not in MARKER_IDS:
                    return {'ok': False, 'error': '마커 번호는 0번부터 49번까지예요.'}
                known = next((p for p in self.places.values() if p['marker_id'] == mid), None)
                spot = {'key': known['key'] if known else None, 'label': known['label'] if known else f'{mid}번 마커'}
                if cmd == 'goto_marker':
                    align = bool(msg.get('align', True))
                    standoff = float(known['standoff_m'] if known else self.standoff_m)
                    steps += leave + [{'kind': 'nav', 'cmd': {'cmd': 'goto_marker', 'marker_id': mid, 'standoff_m': standoff,
                                                             'align': align}, 'event': 'go_start', 'done': 'arrived'}]
                    return self._start('goto', f'{spot["label"]} 앞으로 가기', steps, now, place=spot)
                steps += leave + [{'kind': 'nav', 'cmd': {'cmd': 'follow_marker', 'marker_id': mid, 'range_m': self.follow_range_m},
                                   'event': 'follow_start', 'done': 'arrived'}]
                return self._start('follow_marker', f'{mid}번 마커 따라가기', steps, now, place=spot)
            place = self.places.get(str(msg.get('place', '')))
            if place is None:
                return {'ok': False, 'error': f'모르는 장소예요. 아는 곳: {", ".join(p["label"] for p in self.places.values()) or "없음"}'}
            see = {'kind': 'nav', 'cmd': {'cmd': 'goto_marker', 'marker_id': place['marker_id'], 'standoff_m': place['standoff_m'],
                                          'align': place['align']}, 'event': 'go_start', 'done': 'arrived'}
            spot = self.layout.place_odom(place['key']) if self.layout is not None else None
            if spot is None:
                go = [see]
            else:
                # 배치에 자리가 있다: 그 자리까지 오도메트리로 가고, 마커가 보이면 마커로 마무리한다(안 보여도 도착으로 친다)
                finish = self.layout.uses_marker(place['key']) and self.markers_alive()
                go = [{'kind': 'nav', 'cmd': {'cmd': 'goto_pose', 'x': spot[0], 'y': spot[1], 'yaw': spot[2]},
                       'event': 'go_start', 'done': None if finish else 'arrived'}]
                if finish:
                    go.append({**see, 'event': None, 'optional': True})
            if cmd == 'goto':
                steps += leave + go
                return self._start('goto', f'{place["label"]}(으)로 가기', steps, now, place=place)
            if cmd == 'fetch_from':
                name = str(msg.get('name', '')).strip()[:40]
                if not name:
                    return {'ok': False, 'error': '무엇을 가져올지 이름이 없어요.'}
                if arm is None:
                    return {'ok': False, 'error': '로봇팔이 연결돼 있지 않아요.'}
                if arm.get('errand', {}).get('state') == 'holding':
                    return {'ok': False, 'error': '이미 물건을 들고 있어요. 먼저 받아 주세요.'}
                steps += leave + go + [{'kind': 'fetch', 'name': name}, back]
                return self._start('fetch_from', f'{place["label"]}에서 {name} 가져오기', steps, now, place=place, object=name)
            return {'ok': False, 'error': '모르는 명령이에요.'}

    def _start(self, kind: str, title: str, steps: list[dict], now: float, *, place, object: str = '') -> dict:
        if self.running:
            # 하던 이동은 조용히 그만둔다. 새 목표가 이동 노드의 것을 바로 덮는다
            log.info('작업 %s을(를) 그만두고 새 일을 한다', self.task['id'])
            self.send_nav({'cmd': 'cancel'})
            self._end('cancelled', 'replaced', None)
        self.n += 1
        self.task = {'id': f't{self.n}', 'kind': kind, 'title': title, 'state': 'running', 'detail': '',
                     'place': place['key'] if place else None, 'label': place['label'] if place else None,
                     'object': object, 'got': None, 'problem': ''}
        self.steps, self.i, self._started = steps, 0, False
        self.away = kind != 'home' or self.away
        log.info('작업 시작 %s: %s', self.task['id'], title)
        self._publish()
        return {'ok': True, 'task': self.task['id'], 'title': title}

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

    def _step_mark_home(self, step: dict, first: bool, now: float) -> bool:
        if first:
            self.send_nav({'cmd': 'mark_home'})
        return now - self._step_at > 0.2

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
            return False
        state = nav.get('state')
        self.task['detail'] = '조이스틱으로 조종 중(놓으면 이어서 간다)' if nav.get('paused') else '앞이 막혀 기다리는 중' if nav.get('blocked') else \
            '가까이 온 사람이 있어 멈춤' if (nav.get('guard') or {}).get('hold') else \
            '마커를 놓쳐 기다리는 중' if state == 'following' and nav.get('detail') == 'lost' else \
            {'searching': '마커를 찾는 중', 'approaching': '다가가는 중', 'aligning': '마커 정면으로 옮기는 중', 'driving': '가는 중',
             'turning': '방향을 맞추는 중', 'following': '따라가는 중'}.get(state, '')
        missed = state == 'failed' and step.get('optional') and nav.get('detail') in ('not_found', 'timeout')
        if state == 'arrived' or missed:
            if step['done'] == 'returned':
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
            threading.Thread(target=lambda: step.__setitem__('reply', self.arm_command({'cmd': 'fetch', 'name': step['name']})),
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
        e = {'seq': self.seq, 'type': kind, 'task': t['kind'], 'label': t['label'], 'object': t['object'],
             'got': t['got'], 'problem': t['problem'], **extra}
        if kind == 'failed':
            e['say'] = NAV_REASON.get(extra.get('reason', ''), '이동하다 문제가 생겼어요')
        self.events = (self.events + [e])[-12:]
        self._publish()

    def snapshot(self) -> dict:
        with self.lock:
            t = self.task
            return {
                'enabled': self.enabled,
                'task': None if t is None else {**t, 'step': min(self.i + 1, len(self.steps)), 'steps': len(self.steps)},
                'events': list(self.events),
                'layout': None if self.layout is None else self.layout.snapshot(),
            }

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
