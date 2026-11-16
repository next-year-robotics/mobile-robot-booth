"""부스 배치: 시작 위치와 장소(책상 …)의 자리. ROS를 import하지 않는다.

부스마다 책상이 놓이는 곳이 다르다. 지도는 없으므로 자리는 "시작 위치에서 앞으로 몇 m, 왼쪽으로 몇 m, 어느 쪽을 보고"로
적는다(시작 좌표). 대시보드의 배치 패널(키 P)에서 로봇을 그 자리에 세워 두고 "지금 자리로 지정"을 누르거나 숫자를 넣는다.

- 시작 위치(anchor)는 오도메트리(EKF) 좌표의 한 자세다. 지정하지 않으면 오도메트리 원점, 곧 로봇을 켠 자리다.
  로봇(베이스)을 다시 켜면 오도메트리가 0에서 다시 시작하므로 지정한 시작 위치는 버리고 원점으로 돌아간다.
  그래서 "로봇은 시작 위치에 두고 켠다"만 지키면 장소 자리는 다음 날에도 그대로 맞는다.
- 장소 자리는 시작 좌표로 저장한다(config/layout.local.yaml). 자리가 있는 장소는 먼저 그 자리까지 오도메트리로 가고,
  marker가 켜져 있으면 거기서 마커를 보고 마무리한다. 자리가 없는 장소는 예전처럼 마커만 찾아간다.
"""
from __future__ import annotations

import logging
import math
import threading
from pathlib import Path

import yaml

log = logging.getLogger('booth.layout')
Pose = tuple[float, float, float]


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def compose(a: Pose, b: Pose) -> Pose:
    """a 좌표계에서 본 b → a의 부모 좌표계."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2])


def relative(a: Pose, p: Pose) -> Pose:
    """부모 좌표계의 p → a 좌표계."""
    c, s = math.cos(a[2]), math.sin(a[2])
    dx, dy = p[0] - a[0], p[1] - a[1]
    return c * dx + s * dy, -s * dx + c * dy, wrap(p[2] - a[2])


class Layout:

    def __init__(self, path: str | Path | None, keys: list[str]):
        self.path = None if path is None else Path(path).expanduser()
        self.keys = list(keys)
        self.lock = threading.RLock()
        self.anchor: Pose = (0.0, 0.0, 0.0)
        self.start_set = False
        self.places: dict[str, dict] = {}
        self.pose: Pose | None = None
        self._pose_at = -1e9
        self.changed = lambda: None                 # 바뀌면 부른다(화면 갱신)
        self._load()

    # ------------------------------------------------------------ 파일
    def _load(self) -> None:
        if self.path is None or not self.path.exists():
            return
        try:
            data = yaml.safe_load(self.path.read_text(encoding='utf-8')) or {}
            for key, p in (data.get('places') or {}).items():
                if key in self.keys:
                    self.places[key] = {'x': float(p['x']), 'y': float(p['y']), 'yaw': float(p.get('yaw', 0.0)),
                                        'marker': bool(p.get('marker', True))}
            if data.get('start_set') and data.get('anchor'):
                self.anchor, self.start_set = tuple(float(v) for v in data['anchor'][:3]), True
        except (OSError, ValueError, TypeError, KeyError, yaml.YAMLError) as e:
            log.warning('배치 파일을 읽지 못했다(%s): %s', self.path, e)

    def _save(self) -> None:
        if self.path is None:
            return
        data = {'start_set': self.start_set, 'anchor': [round(v, 4) for v in self.anchor],
                'places': {k: {**{n: round(p[n], 4) for n in ('x', 'y', 'yaw')}, 'marker': p['marker']} for k, p in self.places.items()}}
        try:
            self.path.write_text('# 배치 패널(키 P)이 쓴다. 장소 자리는 시작 위치 기준 [m, rad]이다.\n'
                                 + yaml.safe_dump(data, allow_unicode=True, sort_keys=False), encoding='utf-8')
        except OSError as e:
            log.warning('배치 파일을 쓰지 못했다: %s', e)

    # ------------------------------------------------------------ 로봇 자세
    def on_pose(self, now: float, x: float, y: float, yaw: float) -> None:
        """EKF 자세가 올 때마다. 오도메트리가 0에서 다시 시작했으면(로봇을 다시 켰다) 지정한 시작 위치를 버린다."""
        with self.lock:
            old, gap = self.pose, now - self._pose_at
            self.pose, self._pose_at = (x, y, yaw), now
            if not self.start_set or old is None:
                return
            at_zero = math.hypot(x, y) < 0.05 and abs(yaw) < 0.03
            jumped = math.hypot(x - old[0], y - old[1]) > 0.5 and gap < 5.0
            if jumped or (gap > 3.0 and at_zero and math.hypot(*old[:2]) > 0.3):
                log.warning('오도메트리가 다시 시작됐다. 시작 위치를 로봇을 켠 자리로 되돌린다')
                self.anchor, self.start_set = (0.0, 0.0, 0.0), False
                self._save()
            else:
                return
        self.changed()

    def robot(self, now: float) -> Pose | None:
        with self.lock:
            return self.pose if now - self._pose_at < 1.0 else None

    # ------------------------------------------------------------ 요청
    def request(self, msg: dict, now: float) -> dict:
        op, key = msg.get('op'), str(msg.get('key', ''))
        with self.lock:
            here = self.robot(now)
            if op == 'set_start':
                if here is None:
                    return {'ok': False, 'error': '로봇 위치를 아직 모릅니다(오도메트리 없음).'}
                # 장소 자리는 시작 좌표라 그대로 둔다: 시작 위치를 옮기면 장소도 같이 옮겨 간다
                self.anchor, self.start_set = here, True
            elif op == 'clear_start':
                self.anchor, self.start_set = (0.0, 0.0, 0.0), False
            elif op in ('set_place', 'clear_place', 'place_marker'):
                if key not in self.keys:
                    return {'ok': False, 'error': '모르는 장소입니다.'}
                if op == 'clear_place':
                    self.places.pop(key, None)
                elif op == 'place_marker':
                    if key not in self.places:
                        return {'ok': False, 'error': '자리를 먼저 지정하세요.'}
                    self.places[key]['marker'] = bool(msg.get('on'))
                elif msg.get('here'):
                    if here is None:
                        return {'ok': False, 'error': '로봇 위치를 아직 모릅니다(오도메트리 없음).'}
                    x, y, yaw = relative(self.anchor, here)
                    self.places[key] = {'x': x, 'y': y, 'yaw': yaw, 'marker': self.places.get(key, {}).get('marker', True)}
                else:
                    try:
                        x, y, yaw = float(msg['x']), float(msg['y']), math.radians(float(msg.get('yaw_deg', 0.0)))
                    except (KeyError, TypeError, ValueError):
                        return {'ok': False, 'error': '자리 값을 읽지 못했습니다.'}
                    if not (abs(x) <= 20 and abs(y) <= 20):
                        return {'ok': False, 'error': '시작 위치에서 20 m 안이어야 합니다.'}
                    self.places[key] = {'x': x, 'y': y, 'yaw': wrap(yaw), 'marker': self.places.get(key, {}).get('marker', True)}
            else:
                return {'ok': False, 'error': '모르는 명령입니다.'}
            self._save()
        self.changed()
        return {'ok': True}

    # ------------------------------------------------------------ 읽기
    def place_odom(self, key: str) -> Pose | None:
        """그 장소에서 로봇이 설 자세(오도메트리 좌표). 지정하지 않았으면 None."""
        with self.lock:
            p = self.places.get(key)
            return None if p is None else compose(self.anchor, (p['x'], p['y'], p['yaw']))

    def uses_marker(self, key: str) -> bool:
        with self.lock:
            return bool(self.places.get(key, {}).get('marker', True))

    def snapshot(self) -> dict:
        r = lambda p: [round(p[0], 3), round(p[1], 3), round(p[2], 4)]  # noqa: E731
        with self.lock:
            return {'start': r(self.anchor), 'start_set': self.start_set,
                    'places': {k: {'pose': r(compose(self.anchor, (p['x'], p['y'], p['yaw']))),
                                   'rel': r((p['x'], p['y'], p['yaw'])), 'marker': p['marker']} for k, p in self.places.items()}}
