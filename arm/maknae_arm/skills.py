"""심부름 상태머신. 말(의도)이 들어오면 미리 정한 순서대로만 팔을 움직인다.

    대기 → 찾는 중 → 집는 중 → 들고 있음 → (건네줘) 놓는 중 → 대기
                                   └→ 동작 재생(녹화한 텔레옵) → 대기
    대기 → 장기자랑 팔춤(음악 시각에 맞춘 궤적) → 대기

- 바깥(음성 모델·대시보드)은 fetch(이름)·give()·throw()·motion(이름)·dance(음악 시각)·stop()·home()만 부를 수 있다.
- 지금 상태에서 안 되는 요청은 거절하고 이유를 돌려준다. 모델이 뭐라고 하든 순서를 건너뛰지 못한다.
- 그리퍼는 give()나 throw()가 왔을 때만 연다.
"""
from __future__ import annotations

import json
import logging
import math
import re
import threading
import time
from pathlib import Path

import cv2
import numpy as np

from .cameras import Camera, TopView
from .dance import Choreography
from .driver import Arm, ArmError
from .kin import ARM_JOINTS
from .table import TableMarker
from .vision import Argon, VisionError, grasp_axis

log = logging.getLogger('arm.skills')

IDLE, LOOKING, PICKING, HOLDING, RELEASING, MOVING, TELEOP, ERROR = (
    'idle', 'looking', 'picking', 'holding', 'releasing', 'moving', 'teleop', 'error')
# 집게가 다무는 방향은 끝점 좌표계의 y축이다(2026-11-15 손목 카메라 사진으로 확인: x로 비키면 다무는 방향과 직각으로 움직였다)
CLOSING_AXIS = 1
NAME = re.compile(r'^[\w가-힣-]{1,32}$')


class Refused(RuntimeError):
    """지금은 할 수 없는 요청. 메시지는 사람에게 그대로 읽어 줄 수 있는 한국어다."""


class Errand:

    def __init__(self, cfg: dict, arm: Arm, top: Camera | None, view: TopView | None, argon: Argon | None):
        self.cfg, self.arm, self.top, self.view, self.argon = cfg, arm, top, view, argon
        self.cams: dict | None = None               # 튜닝 사진용. __main__이 꽂는다
        self.state = IDLE
        self.since = time.monotonic()
        self.last_empty: tuple[str, float, float, float] | None = None
        self.object = ''
        self.detail = ''
        self.target: dict | None = None            # 지금 집으려는 물건 (화면 표시용)
        self.inventory: list[dict] = []
        self.inventory_at = 0.0
        self.scanning = False
        self.events: list[dict] = []
        self.seq = 0
        self.reset_lock = threading.Lock()
        self.busy = threading.Lock()               # 팔을 움직이는 작업은 한 번에 하나
        self.motions_dir = Path(cfg['motions_dir']).expanduser()
        self.motions_dir.mkdir(parents=True, exist_ok=True)
        self.recording = ''
        self._leader = None
        self._dance_cut = False                    # 장기자랑을 중간에 끝냈다(멈춤 버튼과 구별)
        # 작업대 높이: 작업대 위에 높이 마커가 보이면 그것으로 재고, 안 보이면 현재 영상에서 물건별 받침면을 추정한다
        w = cfg['workspace']
        self._hold_min_open = float(w.get('grasp_min_opening', 6.0))
        self._table_default = float(w['table_z_m'])
        self.table = {'z': None, 'source': 'unknown', 'at': time.monotonic(), 'spread_m': None}
        self._marker_px = None                     # 마지막으로 본 높이 마커의 네 귀퉁이(영상 좌표)
        m = w.get('table_marker') or {}
        self.marker = TableMarker(m, view) if m.get('enabled', True) and m and view is not None and top is not None else None
        if self.marker is not None:
            threading.Thread(target=self._table_watch, name='arm-table', daemon=True).start()
        threading.Thread(target=self._hold_watch, name='arm-hold', daemon=True).start()

    # ---------------------------------------------------------------- 상태·사건
    def _set(self, state: str, detail: str = '') -> None:
        if state != self.state:
            self.since = time.monotonic()
        self.state, self.detail = state, detail
        log.info('상태 %s %s', state, detail)

    def emit(self, kind: str, text: str = '', **extra) -> None:
        """밖으로 알릴 사건. 대시보드는 이걸로 미리 녹음한 대사를 틀고 음성 모델에게 결과를 알린다."""
        self.seq += 1
        self.events.append({'seq': self.seq, 'type': kind, 'text': text, 't': time.time(), **extra})
        del self.events[:-30]

    def snapshot(self) -> dict:
        return {'state': self.state, 'since_s': round(time.monotonic() - self.since, 1), 'object': self.object,
                'detail': self.detail, 'target': self.target,
                'table': {'z': round(self.table['z'], 3) if self.table['z'] is not None else None, 'source': self.table['source'], 'spread_m': self.table['spread_m'],
                          'marker_id': self.marker.id if self.marker is not None else None},
                'inventory': self.inventory, 'inventory_age_s': round(time.monotonic() - self.inventory_at, 1)
                if self.inventory_at else None, 'scanning': self.scanning,
                'motions': self.motions(), 'recording': self.recording, 'events': self.events[-12:]}

    # ---------------------------------------------------------------- 좌표
    def table_z(self) -> float:
        return float(self.cfg['workspace']['table_z_m'])

    def _arm_folded(self) -> bool:
        j, rest = self.arm.joints(), self.arm.poses['rest']
        return not self.arm.moving and all(abs(j[m] - rest[m]) < 6 for m in ARM_JOINTS)

    def measure_table(self, timeout: float = 1.2) -> dict:
        """매 작업마다 마커를 다시 확인한다. 미검출이면 이전 작업면을 지운다."""
        m = self.marker.measure(self.top, timeout) if self.marker is not None else None
        return self._apply_marker(m)

    def _apply_marker(self, m: dict | None) -> dict:
        if m is not None:
            new = {'xy': m['xy'], 'slope': m.get('slope'), 'z': m['z'], 'source': 'marker', 'at': time.monotonic(), 'spread_m': m['spread_m']}
            self._marker_px = m['corners']
        else:
            new = {'z': None, 'source': 'unknown', 'at': time.monotonic(), 'spread_m': None}
            self._marker_px = None
        self.table = new
        # 수동 동작의 바닥 제한용 기본값. 물건의 집기 좌표에는 현재 관측만 쓴다.
        self.cfg['workspace']['table_z_m'] = new['z'] if m is not None else self._table_default
        return new

    def _table_watch(self) -> None:
        """쉬는 동안 1초마다 다시 잰다. 로봇이 다른 작업대 앞으로 옮겨 가면 값이 따라간다. 일하는 중에는 건드리지 않는다."""
        while True:
            time.sleep(1.0)
            try:
                if self.state != IDLE or self.busy.locked() or self.scanning or not self._arm_folded():
                    continue
                before = self.since
                m = self.marker.measure(self.top, 0.8)
                if self.state != IDLE or self.busy.locked() or self.since != before:
                    continue                                    # 재는 사이에 일이 시작됐다. 그쪽이 직접 잰다
                self._apply_marker(m)
            except Exception:                                   # noqa: BLE001 — 재기가 터져도 심부름은 돌아야 한다
                log.exception('작업대 높이 재기 실패')

    def _scene_hint(self) -> str:
        """물건 찾기 모델에게 같이 넘길 작업대 정보(카메라에서 작업대까지 거리, 높이 마커)."""
        if self.view is None:
            return ''
        d = (float(self.view.C[2]) - self.table_z()) * 100
        if self.table['source'] == 'marker' and self.marker is not None:
            return (f'작업대 위의 흑백 사각 무늬(ArUco 마커, 한 변 {self.marker.size * 100:.0f} cm)는 작업대 높이를 재는 기준표다. 물건이 아니니 답에 넣지 않는다. '
                    f'이 마커로 잰 값으로 카메라에서 작업대 윗면까지는 약 {d:.0f} cm다. 마커 한 변의 길이와 이 거리를 물건 크기와 height_cm를 어림하는 기준으로 쓴다.')
        return (f'높이 기준표를 사용할 수 없다. 카메라 렌즈는 로봇팔 받침 원점보다 수직으로 {self.view.C[2] * 100:.1f} cm 높다. '
                '현재 사진에서 물건마다 렌즈에서 물건 바로 아래 받침면까지의 수직 거리(cm)를 눈대중으로 추정해 '
                'support_depth_cm에 담는다. height_cm는 물건 자체의 높이다.')

    def _on_marker(self, item: dict) -> bool:
        """모델이 높이 마커를 물건으로 답했는가(점이 마커 안에 있고 이름도 마커다)."""
        if self._marker_px is None or not re.search(r'마커|aruco|qr|바코드|무늬', item.get('label', ''), re.I):
            return False
        return cv2.pointPolygonTest(self._marker_px.astype(np.float32), (float(item['u']), float(item['v'])), False) >= 0

    def locate(self, item: dict) -> dict:
        """영상 속 물건(u, v, height_m)을 팔 좌표로 옮기고 집을 수 있는지 붙인다."""
        w = self.cfg['workspace']
        h = float(item.get('height_m', 0.03))
        z = self.table_z()
        if self.table['source'] != 'marker':
            depth = item.get('support_depth_m')
            if depth is None or not math.isfinite(depth) or depth <= 0:
                raise VisionError('현재 영상에서 물건 받침면까지 거리를 추정하지 못했어요.')
            z = float(self.view.C[2]) - depth
            lo, hi = (w.get('table_marker') or {}).get('z_range_m', (-0.08, 0.30))
            if not lo <= z <= hi:
                raise VisionError('영상으로 추정한 받침면 높이가 작업 범위 밖이에요.')
        p = self.view.to_plane(item['u'], item['v'], z + h)   # 점은 물건 윗면에 찍힌다
        out = dict(item, support_z_m=z, x=round(float(p[0]), 4), y=round(float(p[1]), 4))
        out['reachable'] = bool(w['x_m'][0] <= p[0] <= w['x_m'][1] and w['y_m'][0] <= p[1] <= w['y_m'][1])
        return out

    def _hold_watch(self) -> None:
        """들고 기다리는 동안 물건이 빠지면(집게가 끝까지 닫히면) 알리고 팔을 접는다."""
        low = 0
        while True:
            time.sleep(0.2)
            if self.state != HOLDING or self.busy.locked():
                low = 0
                continue
            low = low + 1 if self.arm.joints()['gripper'] <= self._hold_min_open else 0
            if low >= 4:
                low = 0
                name = self.object
                log.info('들고 있던 %s이(가) 빠졌다', name)
                try:
                    def job():
                        self.object, self.target = '', None
                        self._set(MOVING, '접는 중')
                        self.emit('missed', name)
                        self.arm.gripper(30.0)
                        self.arm.pose('rest')
                        self._set(IDLE)
                    self._run(job)
                except Refused:
                    pass

    # ---------------------------------------------------------------- 작업대 살피기
    def _scene_clear(self) -> bool:
        """팔이 접혀 있어 탑뷰를 가리지 않는가"""
        j, rest = self.arm.joints(), self.arm.poses['rest']
        return self.state == IDLE and not self.arm.moving and all(abs(j[m] - rest[m]) < 6 for m in ARM_JOINTS)

    def rescan(self) -> None:
        if self.top is None or self.argon is None:
            raise Refused('탑뷰 카메라나 물건 찾기 설정이 없어요.')
        if not self._scene_clear():
            raise Refused('팔을 접은 뒤 다시 살펴봐 주세요.')
        frame = self.top.latest()
        if frame is None:
            raise Refused('탑뷰 카메라 영상이 없어요.')
        self._run(self._scan, frame)

    def _scan(self, frame) -> None:
        self.scanning = True
        try:
            self.measure_table()
            items = [self.locate(i) for i in self.argon.list(frame, self._scene_hint()) if not self._on_marker(i)]
            self.inventory, self.inventory_at = items, time.monotonic()
            self.emit('scan_done')
            log.info('훑기 %.1f s: %s', self.argon.last_s, ', '.join(i['label'] for i in items) or '없음')
        except (VisionError, ValueError) as e:
            self.emit('scan_failed', str(e))
            log.warning('훑기 실패: %s', e)
        finally:
            self.scanning = False

    # ---------------------------------------------------------------- 공용
    def _run(self, fn, *args) -> None:
        """팔 작업을 뒤에서 돌린다. 이미 도는 것이 있으면 거절한다."""
        if self.arm.motion_blocked or not self.busy.acquire(blocking=False):
            raise Refused('지금 다른 동작을 하는 중이에요.')

        def work():
            try:
                fn(*args)
            except ArmError as e:
                if str(e) == '멈춤':
                    self._set(ERROR, '멈춤. 대시보드에서 "팔 접기"를 누르면 다시 시작한다')
                    self.emit('stopped')
                else:
                    self._set(ERROR, str(e))
                    self.emit('failed', str(e))
            except Exception as e:                      # 예상 못 한 오류도 상태로 남긴다
                log.exception('작업 실패')
                self._set(ERROR, f'{type(e).__name__}: {e}')
                self.emit('failed', '알 수 없는 오류')
            finally:
                self.busy.release()
        threading.Thread(target=work, name='arm-job', daemon=True).start()

    def stop(self) -> None:
        self.arm.stop()
        if self.state == TELEOP:
            self.teleop(False)

    def reset(self) -> dict:
        """현재 자세를 유지하고, 이전 작업이 끝나면 심부름 단계를 비운다."""
        with self.reset_lock:
            if self.arm.motion_blocked:
                return {'pending': True}
            self.stop()
            self.arm.stop(block_motion=True)

            def finish():
                try:
                    self.object, self.target, self.last_empty = '', None, None
                    self.events.clear()
                    self._set(IDLE)
                finally:
                    with self.arm.lock:
                        self.arm.motion_blocked = False
                    self.busy.release()

            if self.busy.acquire(blocking=False):
                finish()
                return {'pending': False}

            def wait_for_job():
                self.busy.acquire()
                finish()
            threading.Thread(target=wait_for_job, name='arm-reset', daemon=True).start()
            return {'pending': True}

    def home(self) -> None:
        """팔을 접는다. 오류에서 빠져나오는 길이기도 하다. 들고 있던 물건은 놓지 않는다."""
        if self.state == TELEOP:
            raise Refused('텔레옵을 먼저 끄세요.')
        self.arm.stop()

        def job():
            holding = self.arm.joints()['gripper'] > 8 and self.state in (HOLDING, ERROR) and bool(self.object)
            self._set(MOVING, '접는 중')
            self._retreat()
            self.arm.pose('rest')
            if holding:
                self._set(HOLDING, '접은 채 들고 있음')
            else:
                self.object, self.target = '', None
                self._set(IDLE)
        self._run(job)

    def _retreat(self) -> None:
        """끝점이 낮으면 먼저 위로 뺀다. 작업대 위 물건을 쓸지 않게."""
        tip = self.arm.tip()
        if tip[0] > 0.12 and tip[2] < self.table_z() + 0.07:
            up = self._solve([tip[0], tip[1], self.table_z() + 0.10], self._current_roll(), prefer_tilt=None)
            if up is not None:
                self.arm.move(up, self.arm.slow)

    def _current_roll(self) -> float:
        return float(self.arm.q()[4])

    # ---------------------------------------------------------------- 집기
    def _solve(self, pos, roll: float, prefer_tilt: float | None = 0.0) -> dict | None:
        """끝점 pos에 닿는 관절각. 수직으로 내려찍는 자세부터 시작해 멀면 앞으로 기울인다."""
        kin = self.arm.kin
        radial = np.array([pos[0] - 0.0388, pos[1], 0.0])
        n = np.linalg.norm(radial)
        radial = radial / n if n > 1e-6 else np.array([1.0, 0.0, 0.0])
        tilts = (0.0, 15.0, 30.0, 45.0, 60.0) if prefer_tilt is not None else (None,)
        for tilt in tilts:
            if tilt is None:
                q = kin.ik(pos, None, roll=roll, seed=self.arm.q())
            else:
                t = math.radians(tilt)
                q = kin.ik(pos, math.sin(t) * radial + np.array([0, 0, -math.cos(t)]), roll=roll, seed=self.arm.q())
                if q is not None and np.linalg.norm(kin.approach(q) - (math.sin(t) * radial + np.array([0, 0, -math.cos(t)]))) > 0.25:
                    q = None
            if q is None:
                continue
            deg = kin.to_lerobot(q)
            if not self.arm.guard({**deg, 'gripper': 0.0}):
                return deg
        return None

    def _roll_for(self, pos, axis_world: np.ndarray | None) -> float:
        """집게가 물건의 긴 축에 수직으로 다물리게 하는 wrist_roll [rad]. 축을 모르면 0."""
        if axis_world is None:
            return 0.0
        kin = self.arm.kin
        best, best_score = 0.0, -1.0
        for roll in np.radians(np.arange(-90, 91, 5)):
            deg = self._solve(pos, float(roll))
            if deg is None:
                continue
            closing = kin.fk(kin.to_urdf(deg))[:3, CLOSING_AXIS]
            closing = closing[:2] / (np.linalg.norm(closing[:2]) + 1e-9)
            score = 1.0 - abs(float(closing @ axis_world[:2]))      # 다무는 방향 ⟂ 긴 축
            if score > best_score + 1e-6:
                best, best_score = float(roll), score
        return best

    def fetch(self, name: str) -> dict:
        name = (name or '').strip()
        if not name:
            raise Refused('무엇을 가져올지 말씀해 주세요.')
        if self.state == HOLDING:
            raise Refused(f'지금 {self.object}을(를) 들고 있어요. 먼저 받아 주세요.')
        if self.state != IDLE:
            raise Refused('지금 다른 동작을 하는 중이에요.')
        if self.top is None or self.argon is None:
            raise Refused('탑뷰 카메라나 물건 찾기 설정이 없어요.')
        self._run(self._fetch, name)
        return {'started': True}

    def _fetch(self, name: str) -> None:
        previous, self.last_empty = self.last_empty, None
        self.object, self.target = name, None
        self._set(LOOKING, name)
        self.emit('looking', name)
        if not self._scene_clear():
            self.arm.pose('rest')
            time.sleep(0.4)
        # 이 작업대의 높이를 먼저 정한다(높이 마커가 있으면 잰 값). 집는 동안에는 팔이 마커를 가려도 이 값을 그대로 쓴다
        table = self.measure_table()
        frame = self.top.latest()
        if frame is None:
            raise ArmError('탑뷰 카메라 영상이 없다')
        try:
            found = self.argon.find(frame, name, self._scene_hint())
        except VisionError as e:
            raise ArmError(f'물건 찾기 실패: {e}') from None
        if not found.get('found'):
            self.object = ''
            self._set(IDLE)
            self.emit('not_found', name, reason=found.get('reason', ''))
            return
        if found.get('grasp_clear') is False:
            self.object = ''
            self._set(IDLE)
            self.emit('shown', name, reason='집게가 내려갈 자리를 비워 주세요.')
            return
        axis = None if found.get('tape_hole') else grasp_axis(frame, found.get('box'), found['u'], found['v'])
        if axis is not None:
            found['u'], found['v'] = axis[0], axis[1]
        try:
            item = self.locate(found)
        except VisionError as e:
            raise ArmError(str(e)) from None
        if table['source'] != 'marker':
            table = {'z': item['support_z_m'], 'source': 'vision', 'at': time.monotonic(), 'spread_m': None}
            self.table = table
            self.cfg['workspace']['table_z_m'] = table['z']
        if axis is not None:
            u, v, theta, width = axis
            du, dv = -math.sin(theta) * width / 2, math.cos(theta) * width / 2
            z = item['support_z_m'] + item['height_m']
            ends = [self.view.to_plane(u + k * du, v + k * dv, z) for k in (-1, 1)]
            item['width_m'] = float(np.linalg.norm((ends[1] - ends[0])[:2]))
        self.target = item
        log.info('찾음 %s → (%.3f, %.3f) h %.3f, 작업대 %+.3f(%s) %s', item['label'], item['x'], item['y'], item['height_m'],
                 table['z'], table['source'], '' if item['reachable'] else '범위 밖')
        if not item['reachable']:
            self.object = ''
            self._set(IDLE)
            self.emit('unreachable', item['label'])
            return
        self.object = item['label'] or name
        self._set(PICKING, self.object)
        self.emit('picking', self.object)
        key = ''.join(name.casefold().split())
        deeper = bool(previous and previous[0] == key and time.monotonic() - previous[3] < 120
                      and math.hypot(item['x'] - previous[1], item['y'] - previous[2]) <= 0.04)
        try:
            if item.get('same_board') and table.get('slope') is not None:
                mx, my = table['xy']
                x, y = item['x'], item['y']
                if math.hypot(x - mx, y - my) <= 0.18:
                    self.arm.pick_plane = (mx, my, *table['slope'], x, y)
                    z = self.arm.surface_height(x, y)
                    p = self.view.to_plane(item['u'], item['v'], z + item['height_m'])
                    if not (self.cfg['workspace']['x_m'][0] <= p[0] <= self.cfg['workspace']['x_m'][1]
                            and self.cfg['workspace']['y_m'][0] <= p[1] <= self.cfg['workspace']['y_m'][1]):
                        self.object, self.target = '', None
                        self._set(IDLE)
                        self.emit('unreachable', item['label'])
                        return
                    item.update(x=float(p[0]), y=float(p[1]), support_z_m=z)
                    log.info('같은 판의 상자 위치 받침면 %+.3f (마커 중심 %+.3f)', z, table['z'])
            if self._pick(item, axis[2] if axis is not None else None, deeper) and not deeper:
                self.last_empty = (key, item['x'], item['y'], time.monotonic())
        finally:
            self.arm.pick_plane = None

    def _pick(self, item: dict, image_angle: float | None, deeper: bool = False) -> bool:
        """실제로 집게를 닫았고 한 번도 물지 못했을 때만 True."""
        w = self.cfg['workspace']
        z0 = item['support_z_m']
        clear = float(w.get('grasp_clear_m', 0.012))
        h = float(np.clip(item['height_m'], 0.005, float(w.get('max_height_m', 0.16))))
        # 낮은 물건은 작업대 바로 위를, 키 큰 물건은 윗부분을 잡는다(손바닥이 물건 머리에 닿지 않게).
        # 높이는 모델의 눈대중이라 틀릴 수 있다. 높게 잡았다가 빈손이면 바닥 높이로 한 번 더 잡는다.
        levels = [z0 + max(clear, h - float(w.get('finger_depth_m', 0.045)))]
        if levels[0] > z0 + clear + 0.008:
            levels.append(z0 + clear)
        if deeper:
            floor = z0 + float(w['floor_clear_m'])
            levels = [max(floor, z - float(w.get('retry_deeper_m', 0.003))) for z in levels]
            log.info('같은 물건 빈손 재시도: 목표 높이 %s mm', [round((z - z0) * 1000) for z in levels])
        x0, y0 = item['x'], item['y']
        axis_world = None
        if image_angle is not None:
            # 영상 속 방향을 팔 좌표 방향으로: 카메라 x·y축의 수평 성분
            d = self.view.R[:, 0] * math.cos(image_angle) + self.view.R[:, 1] * math.sin(image_angle)
            axis_world = d / (np.linalg.norm(d[:2]) + 1e-9)
        arm, kin = self.arm, self.arm.kin
        # 테이프 롤은 고정 손가락을 구멍에 넣고 움직이는 손가락으로 바깥쪽을 문다.
        shift = 0.0 if item.get('tape_hole') else float(w.get('grasp_shift_m', 0.0))
        min_open = float(w.get('grasp_min_opening', 6.0))
        if item.get('tape_hole'):
            min_open = float(w.get('tape_min_opening', min_open))
        self._hold_min_open = min_open
        held, moved, contacted = False, False, False
        for n, grasp_z in enumerate(levels):
            roll = self._roll_for([x0, y0, grasp_z], axis_world)
            x, y = x0, y0
            down = self._solve([x, y, grasp_z], roll)
            if down is not None and shift:
                # 끝점 좌표계의 원점은 고정 집게 쪽에 있다. 물건이 두 집게 사이에 오도록 다무는 축을 따라 옮겨 잡는다
                ax = kin.fk(kin.to_urdf(down))[:3, CLOSING_AXIS]
                ax = ax[:2] / (np.linalg.norm(ax[:2]) + 1e-9)
                offset = shift
                width = item.get('width_m')
                if width is None and item.get('box'):
                    # 윤곽을 분리하지 못한 프레임은 모델 상자를 실제 닫힘 방향에 투영한다.
                    u0, v0, u1, v1 = item['box']
                    z = item['support_z_m'] + h
                    edges = [self.view.to_plane(u, v, z)[:2] @ ax for u in (u0, u1) for v in (v0, v1)]
                    width = max(edges) - min(edges)
                if width and width > 0:
                    offset = min(shift, width / 2)
                    log.info('물체 폭 %.0f mm, 고정 집게 목표 중심에서 %.0f mm', width * 1000, offset * 1000)
                x, y = x0 + offset * ax[0], y0 + offset * ax[1]
                grasp_z += self.arm.surface_height(x, y) - z0
                down = self._solve([x, y, grasp_z], roll)
            # 위에서 기다리는 높이. 작업대의 다른 물건(키 큰 병, 휴지)을 치지 않게 가장 큰 물건보다 높이 지나가고,
            # 닿지 않으면 조금씩 낮춘다
            tallest = float(w.get('max_height_m', 0.16))
            above = None
            for top in (tallest + 0.04, max(h, 0.02) + float(w['hover_m']), max(h, 0.02) + 0.06, max(h, 0.02) + 0.04):
                above = self._solve([x, y, z0 + top], roll)
                if above is not None:
                    break
            if above is None or down is None:
                if n + 1 < len(levels):
                    continue
                break
            if not moved:
                arm.move({**arm.poses['ready'], 'gripper': 100.0})
                moved = True
            else:
                arm.gripper(100.0)
            arm.move(above)
            arm.move(down, arm.slow)
            self._trim([x, y, grasp_z], roll)
            self._snap('down')
            arm.gripper(0.0)
            grip = self._gripper_settled()
            held = grip > min_open
            contacted |= held
            log.info('그리퍼 %.1f, %.0f mA (높이 %+.3f) → %s', grip, arm.gripper_current(), grasp_z - z0, '잡음' if held else '빈손')
            self._snap('closed')
            arm.move(above, arm.slow)
            if held and self._gripper_settled(0.6) <= min_open:      # 들어 올리다 빠졌다
                log.info('들어 올리는 중에 놓쳤다')
                held = False
            if held:
                break
        if not moved:
            self.object = ''
            self._set(IDLE)
            self.emit('unreachable', item['label'])
            return False
        if not held:
            arm.gripper(30.0)
            arm.pose('rest')
            self.object, self.target = '', None
            self._set(IDLE)
            self.emit('missed', item['label'])
            return not contacted
        arm.move({m: arm.poses['handover'][m] for m in ARM_JOINTS}, float(w.get('carry_deg_s', arm.slow)))   # 들고는 천천히
        if self._gripper_settled(0.5) <= min_open:              # 옮기다 떨어뜨렸다
            log.info('옮기는 중에 떨어뜨렸다')
            arm.gripper(30.0)
            arm.pose('rest')
            self.object, self.target = '', None
            self._set(IDLE)
            self.emit('missed', item['label'])
            return False
        self.target = None
        self._set(HOLDING, self.object)
        self.emit('holding', self.object)
        return False

    def _trim(self, pos, roll: float) -> None:
        """팔 무게로 처진 만큼(끝점이 5~9 mm 낮고 조금 짧게 온다)을 엔코더로 읽어 한 번 더 맞춘다."""
        time.sleep(0.25)
        err = np.asarray(pos, float) - self.arm.tip()
        if not 0.002 < float(np.linalg.norm(err)) < 0.03:
            return
        fix = self._solve(list(np.asarray(pos, float) + err), roll)
        if fix is not None:
            self.arm.move(fix, self.arm.slow)
            log.info('처짐 보정 %s mm → 남은 오차 %s mm', (err * 1000).round(0), ((np.asarray(pos) - self.arm.tip()) * 1000).round(0))

    def _gripper_settled(self, timeout: float = 1.8) -> float:
        """집게가 다 닫혀(또는 물건에 막혀) 멈출 때까지 기다렸다가 벌어진 정도를 돌려준다."""
        end = time.monotonic() + timeout
        last, still = self.arm.joints()['gripper'], 0
        while time.monotonic() < end and still < 4:
            time.sleep(0.08)
            now = self.arm.joints()['gripper']
            still = still + 1 if abs(now - last) < 0.25 else 0
            last = now
        return last

    def _snap(self, tag: str) -> None:
        """집는 순간의 탑뷰·손목 사진을 남긴다(튜닝용). vision.debug_dir이 있을 때만."""
        d = self.cfg['vision'].get('debug_dir')
        if not d or self.cams is None:
            return
        import cv2
        out = Path(d).expanduser()
        out.mkdir(parents=True, exist_ok=True)
        for name, cam in self.cams.items():
            f = cam.latest() if hasattr(cam, 'latest') else None
            if f is not None:
                cv2.imwrite(str(out / f'{tag}_{name}.jpg'), f)

    def give(self) -> dict:
        """들고 있는 물건을 놓는다. HOLDING에서만 된다."""
        if self.state != HOLDING:
            raise Refused('지금은 들고 있는 물건이 없어요.')

        def job():
            name = self.object
            self._set(RELEASING, name)
            self.arm.move({m: self.arm.poses['handover'][m] for m in ARM_JOINTS})
            self.emit('released', name)                 # 집게가 열리기 시작할 때 "여기요"가 나오게
            self.arm.gripper(100.0)
            time.sleep(1.2)
            self.arm.gripper(10.0, wait=False)
            self.arm.pose('rest')
            self.object = ''
            self._set(IDLE)
            self.emit('done', name)
        self._run(job)
        return {'started': True}

    def throw(self) -> dict:
        """들고 있는 물건을 앞으로 던진다. 팔을 뒤로 뺐다가 빠르게 앞으로 휘두르며 도중에 집게를 연다. HOLDING에서만 된다."""
        if self.state != HOLDING:
            raise Refused('지금은 들고 있는 물건이 없어요.')
        t = self.cfg.get('throw') or {}
        if not t.get('enabled', True):
            raise Refused('던지기는 꺼 두었어요.')
        back, front = t.get('back') or self.arm.poses.get('throw_back'), t.get('front') or self.arm.poses.get('throw_front')
        if not back or not front:
            raise Refused('던지는 자세가 설정돼 있지 않아요.')
        back = {m: float(back[m]) for m in ARM_JOINTS}
        front = {m: float(front[m]) for m in ARM_JOINTS}
        # 시작하기 전에 두 자세를 본다. 휘두르다 막히면 물건을 든 채 어중간하게 선다
        for name, pose in (('뒤로 뺀', back), ('앞으로 뻗은', front)):
            why = self.arm.guard({**pose, 'gripper': 0.0})
            if why:
                raise Refused(f'{name} 자세가 막혀 있어요: {why}')

        def job():
            name = self.object
            self._set(RELEASING, name)
            self.emit('throwing', name)                 # "던질게요, 받으세요!"
            self.arm.move(back, float(t.get('windup_deg_s', self.arm.speed)))
            time.sleep(float(t.get('pause_s', 0.5)))    # 대사가 나올 틈
            self.arm.swing(front, float(t.get('speed_deg_s', 220.0)), float(t.get('release_at', 0.45)))
            self.emit('thrown', name)
            time.sleep(0.5)
            self.arm.gripper(10.0, wait=False)
            self.arm.pose('rest')
            self.object = ''
            self._set(IDLE)
            self.emit('done', name)
        self._run(job)
        return {'started': True}

    def intro_gesture(self, name: str) -> dict:
        # 어깨에서 상체 전체를 숙인다. 설명에서는 팔꿈치를 펴거나 좌우를 돌아본다.
        offsets = {
            'hello': {'shoulder_lift': 50, 'elbow_flex': -4, 'wrist_flex': 18},
            'present': {'shoulder_pan': 14, 'shoulder_lift': 25, 'elbow_flex': -30, 'wrist_flex': 20},
            'offer': {'shoulder_lift': 45, 'elbow_flex': -16, 'wrist_flex': 26},
            'look': {'shoulder_pan': -24, 'shoulder_lift': 21, 'elbow_flex': -18},
            'nod': {'shoulder_lift': 40, 'elbow_flex': -4, 'wrist_flex': 15},
        }
        if name not in offsets or self.state != IDLE or not self.arm.torque:
            raise Refused('지금은 자기소개 몸짓을 할 수 없어요.')
        start = dict(self.arm.target)
        goal = {**start, **{j: start[j] + delta for j, delta in offsets[name].items()}}
        # 강조 자세에서 잠시 머문 뒤, 같은 경로로 조금 더 느리게 돌아온다.
        stroke = self.arm.plan_move(start, goal, 76)
        release = stroke[-1][0] + 0.32
        path = stroke + [(release + t, g) for t, g in self.arm.plan_move(goal, start, 60)]
        if any(self.arm.guard(g) for _, g in path):
            raise Refused('지금 자세에서는 자기소개 몸짓을 쉬어요.')

        def job():
            self._set(MOVING, '자기소개')
            now = time.monotonic()
            self.arm.follow([(now + t, g) for t, g in path])
            self._set(IDLE)
        self._run(job)
        return {'started': True}

    # ---------------------------------------------------------------- 장기자랑
    def dance(self, start_epoch: float) -> dict:
        """화면의 장기자랑 음악에 맞춰 팔춤을 춘다. start_epoch는 음악 0초가 스피커에서 나온 벽시계 시각(time.time)이다.

        대기 상태에서만 춘다. 물건을 들고 있으면(집게를 벌리면 떨어진다) 거절하고 화면만 춘다.
        """
        d = self.cfg.get('dance') or {}
        if not d.get('enabled', True):
            raise Refused('팔춤은 꺼 두었어요.')
        if self.state == HOLDING:
            raise Refused('물건을 들고 있어서 팔은 춤을 쉬어요.')
        if self.state != IDLE:
            raise Refused('지금 다른 동작을 하는 중이에요.')
        choreo = Choreography(d)
        wall, mono = time.time(), time.monotonic()
        m0 = wall - float(start_epoch)                   # 지금이 음악 몇 초인가
        if not -2.0 < m0 < choreo.end - 3.0:
            raise Refused('음악과 시각이 맞지 않아요.')
        lead = float(d.get('lead_s', 0.08))
        plan = choreo.plan({**self.arm.target}, max(m0, 0.0), self.arm.rate, float(d.get('enter_s', 2.0)),
                           float(d.get('hold_s', 0.6)))
        for _, g in plan[::5] + [plan[-1]]:
            why = self.arm.guard(g)
            if why:
                raise Refused(f'춤 자세가 막혀 있어요: {why}')
        steps = [(mono + (float(start_epoch) + m - wall) - lead, g) for m, g in plan]
        self._dance_cut = False

        def job():
            self._set(MOVING, '장기자랑')
            self.emit('dance')
            try:
                self.arm.follow(steps)
            except ArmError as e:
                if str(e) != '멈춤' or not self._dance_cut:
                    raise                                 # 멈춤 버튼은 다른 동작처럼 그 자리에 세운다
            self.arm.move({**self.arm.poses['rest'], 'gripper': 10.0})
            self._set(IDLE)
            self.emit('dance_done')
        self._run(job)
        return {'started': True, 'seconds': round(choreo.end - max(m0, 0.0), 1)}

    def dance_stop(self) -> dict:
        """장기자랑을 중간에 끝냈다. 춤추던 팔을 접는다(멈춤 버튼과 달리 오류로 남기지 않는다)."""
        if self.state != MOVING or self.detail != '장기자랑':
            return {'dancing': False}
        self._dance_cut = True
        self.arm.stop()
        return {'dancing': True}

    # ---------------------------------------------------------------- 녹화 동작
    def motions(self) -> list[str]:
        return sorted(p.stem for p in self.motions_dir.glob('*.json'))

    def _motion_path(self, name: str) -> Path:
        if not NAME.match(name or ''):
            raise Refused('동작 이름은 글자·숫자·밑줄 32자까지예요.')
        return self.motions_dir / f'{name}.json'

    def motion(self, name: str) -> dict:
        path = self._motion_path(name)
        if not path.exists():
            raise Refused(f'"{name}" 동작이 없어요.')
        if self.state not in (IDLE, HOLDING):
            raise Refused('지금 다른 동작을 하는 중이에요.')
        data = json.loads(path.read_text())
        frames = [(float(t), {m: float(g[m]) for m in ARM_JOINTS + ['gripper']}) for t, g in data['frames']]

        def job():
            was = self.state
            self._set(MOVING, name)
            self.emit('motion', name)
            self.arm.play(frames, float(data.get('rate', 1.0)))
            time.sleep(0.3)
            if data.get('end', 'rest') == 'rest':
                self.arm.pose('rest')
            released = self.arm.joints()['gripper'] > 40
            if was == HOLDING and not released:
                self._set(HOLDING, self.object)
            else:
                self.object = ''
                self._set(IDLE)
            self.emit('motion_done', name)
        self._run(job)
        return {'started': True}

    def teleop(self, on: bool) -> dict:
        """리더 팔로 팔로워를 직접 움직인다(동작 녹화용). 리더가 꽂혀 있어야 한다."""
        if on:
            if self.state not in (IDLE, HOLDING):
                raise Refused('지금 다른 동작을 하는 중이에요.')
            from .leader import Leader
            try:
                self._leader = Leader(self.cfg, self.arm)
            except Exception as e:
                raise Refused(f'리더 팔을 열지 못했어요: {e}') from None
            self.arm.set_teleop(self._leader.target)
            self._set(TELEOP, '리더 팔 따라가는 중')
        else:
            if self.recording:
                self.record_stop()
            self.arm.set_teleop(None)
            if self._leader is not None:
                self._leader.close()
                self._leader = None
            self._set(IDLE if self.arm.joints()['gripper'] < 12 or not self.object else HOLDING, self.object)
        return {'teleop': on}

    def record_start(self, name: str, source: str = 'leader') -> dict:
        """동작 녹화를 시작한다. source가 leader면 텔레옵을, hand면 토크를 풀어 손으로 움직이는 것을 담는다."""
        self._motion_path(name)
        if self.recording:
            raise Refused('이미 녹화 중이에요.')
        if source == 'leader':
            if self.state != TELEOP:
                self.teleop(True)
        elif source == 'hand':
            if self.state != IDLE:
                raise Refused('대기 상태에서만 손 녹화를 시작할 수 있어요.')
            self.arm.set_torque(False)
            self._set(TELEOP, '손으로 움직이는 중 (토크 꺼짐)')
        else:
            raise Refused('source는 leader 또는 hand')
        self.recording = name
        self.arm.record_start()
        return {'recording': name}

    def record_stop(self, end: str = 'rest') -> dict:
        if not self.recording:
            raise Refused('녹화 중이 아니에요.')
        name, self.recording = self.recording, ''
        frames = self.arm.record_stop()
        # 앞뒤로 가만히 있던 구간을 잘라 낸다
        def moved(a, b):
            return max(abs(a[m] - b[m]) for m in a) > 0.8
        i = 0
        while i + 1 < len(frames) and not moved(frames[i][1], frames[0][1]):
            i += 1
        j = len(frames) - 1
        while j > i and not moved(frames[j][1], frames[-1][1]):
            j -= 1
        cut = frames[max(0, i - 10):min(len(frames), j + 15)]
        t0 = cut[0][0] if cut else 0.0
        data = {'name': name, 'recorded': time.strftime('%Y-%m-%d %H:%M:%S'), 'rate': 1.0, 'end': end,
                'frames': [[round(t - t0, 3), {m: round(v, 2) for m, v in g.items()}] for t, g in cut]}
        self._motion_path(name).write_text(json.dumps(data))
        if not self.arm.torque:
            self.arm.set_torque(True)
            self._set(IDLE)
        log.info('동작 %s 저장: %d 프레임 %.1f s', name, len(cut), cut[-1][0] - t0 if cut else 0)
        return {'saved': name, 'frames': len(cut), 'seconds': round(cut[-1][0] - t0, 1) if cut else 0}

    def motion_delete(self, name: str) -> dict:
        self._motion_path(name).unlink(missing_ok=True)
        return {'deleted': name}
