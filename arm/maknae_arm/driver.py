"""SO-101 팔로워 구동. 제어 스레드 하나가 버스를 독점하고, 나머지는 목표만 넘긴다.

- 연결할 때 토크를 끄지 않는다. LeRobot의 connect()는 설정을 쓰느라 토크를 잠깐 풀어서, 팔이 들린 채
  데몬이 다시 뜨면 떨어진다. 여기서는 버스만 열고 지금 자세를 목표로 잡는다.
- 모든 이동은 관절 공간 최소 저크 보간이다. 속도는 가장 많이 도는 관절 기준 [deg/s].
- 이동 전에 경로 전체를 FK로 훑어 차체·작업대와 부딪히는지 본다(guard). 걸리면 움직이지 않는다.
"""
from __future__ import annotations

import logging
import math
import threading
import time
from pathlib import Path

import numpy as np

from .kin import ARM_JOINTS, Kinematics

log = logging.getLogger('arm.driver')
MOTORS = ARM_JOINTS + ['gripper']


class ArmError(RuntimeError):
    pass


class FakeBus:
    """팔 없이 화면과 흐름을 시험할 때 쓴다. 목표를 그대로 현재값으로 돌려준다."""

    def __init__(self, start: dict[str, float]):
        self.pos = dict(start)
        self.torque = True
        self.grip_floor = 0.0            # 가짜 장면(sim.py)이 물건을 물렸을 때 집게가 여기서 멈춘다

    def read(self) -> dict[str, float]:
        return dict(self.pos)

    def write(self, goal: dict[str, float]) -> None:
        if self.torque:
            self.pos.update(goal)
            self.pos['gripper'] = max(self.pos['gripper'], self.grip_floor)

    def aux(self) -> dict:
        return {'current_ma': {m: 300.0 if m == 'gripper' and self.grip_floor else 0.0 for m in MOTORS}, 'temp_c': {m: 30 for m in MOTORS}, 'volt': 12.0}

    def set_torque(self, on: bool) -> None:
        self.torque = on

    def close(self) -> None:
        pass

    def reopen(self) -> None:
        pass


class FeetechBus:

    def __init__(self, cfg: dict):
        from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
        self.robot = SO101Follower(SO101FollowerConfig(
            port=str(cfg['port']), id=cfg['id'],
            calibration_dir=Path(cfg['calibration_dir']).expanduser(),
            max_relative_target=None, use_degrees=True, disable_torque_on_disconnect=False))
        if not self.robot.calibration:
            raise ArmError(f"모터 캘리브레이션 파일이 없다: {cfg['calibration_dir']}/{cfg['id']}.json")
        self.bus = self.robot.bus
        self.bus.connect()
        self.ranges = {}
        for m in ARM_JOINTS:
            c = self.bus.calibration[m]
            half = (c.range_max - c.range_min) / 2 * 360 / 4095
            self.ranges[m] = (-half, half)

    def read(self) -> dict[str, float]:
        return {m: float(v) for m, v in self.bus.sync_read('Present_Position', num_retry=3).items()}

    def write(self, goal: dict[str, float]) -> None:
        self.bus.sync_write('Goal_Position', goal)

    def aux(self) -> dict:
        cur = self.bus.sync_read('Present_Current', normalize=False, num_retry=2)
        temp = self.bus.sync_read('Present_Temperature', normalize=False, num_retry=2)
        volt = self.bus.sync_read('Present_Voltage', normalize=False, num_retry=2)
        return {'current_ma': {m: float(v) * 6.5 for m, v in cur.items()},
                'temp_c': {m: int(v) for m, v in temp.items()},
                'volt': float(np.mean(list(volt.values()))) / 10.0}

    def set_torque(self, on: bool) -> None:
        if on:
            self.bus.enable_torque()
        else:
            self.bus.disable_torque()

    def close(self) -> None:
        self.bus.disconnect(disable_torque=False)

    def reopen(self) -> None:
        """USB가 끊겼다 다시 붙은 뒤 시리얼을 새로 연다. 못 열면 예외.

        허브가 잠깐 떨어지면 장치 번호가 바뀌고(ttyACM0 → ttyACM1) 쥐고 있던 포트는 죽는다. 그때 SDK의
        '사용 중' 표시가 켜진 채 남아 이후 모든 통신이 "Port is in use!"로 실패한다(2026-11-15).
        /dev/soarm_follower(udev 고정 이름)는 새 번호를 가리키므로 닫고 같은 이름으로 다시 열면 된다.
        모터는 12 V로 따로 먹어서 그동안 토크를 유지하고 있다. 토크는 건드리지 않는다.
        """
        port = self.bus.port_handler
        try:
            port.closePort()
        except Exception:  # noqa: BLE001 — 이미 사라진 장치
            pass
        port.is_using = False
        self.bus.connect()


def min_jerk(s: float) -> float:
    return 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5


class Arm:

    def __init__(self, cfg: dict, kin: Kinematics, fake: bool = False):
        self.cfg, self.kin = cfg, kin
        a = cfg['arm']
        self.rate = float(a['rate_hz'])
        self.pick_plane = None
        self.speed = float(a['speed_deg_s'])
        self.slow = float(a['slow_deg_s'])
        self.poses = {k: {m: float(v) for m, v in p.items()} for k, p in cfg['poses'].items()}
        self.bus = FakeBus({**self.poses['rest'], 'gripper': 5.0}) if fake else FeetechBus(a)
        self.fake = fake
        self.lock = threading.Lock()                 # 버스 접근
        self.present = self.bus.read()
        self.target = dict(self.present)             # 마지막으로 보낸 목표
        self.aux = {'current_ma': {m: 0.0 for m in MOTORS}, 'temp_c': {}, 'volt': 0.0}
        self.torque = True
        self.moving = False
        self.fault = ''
        self._queue: list[tuple[float, dict[str, float]]] = []   # (시각, 목표)
        self._cancel = threading.Event()
        self.motion_blocked = False
        self._stop = threading.Event()
        self._teleop = None                          # 호출하면 목표 dict를 돌려주는 함수(리더 팔)
        self._recorder: list | None = None
        self.thread = threading.Thread(target=self._loop, name='arm-control', daemon=True)
        self.thread.start()

    # ---------------------------------------------------------------- 제어 스레드
    def _loop(self) -> None:
        period = 1.0 / self.rate
        n = 0
        failed_since: float | None = None
        reopen_at = 0.0
        while not self._stop.is_set():
            t0 = time.monotonic()
            try:
                with self.lock:
                    goal = None
                    if self._teleop is not None and self.torque:
                        goal = self._teleop()
                    else:
                        while self._queue and self._queue[0][0] <= t0:
                            goal = self._queue.pop(0)[1]
                        self.moving = bool(self._queue)
                    if goal is not None and self.torque:
                        self.bus.write(goal)
                        self.target.update(goal)
                    self.present = self.bus.read()
                    if self._recorder is not None:
                        self._recorder.append((t0, dict(self.present if not self.torque else self.target)))
                    if n % int(self.rate / 5) == 0:
                        self.aux = self.bus.aux()
                self.fault = ''
                failed_since = None
            except Exception as e:                   # 버스 오류는 다음 주기에 다시 시도한다
                if self.fault != str(e):
                    log.warning('버스 오류: %s', e)
                self.fault = str(e)
                time.sleep(0.05)
                # 0.5 s 넘게 이어지면 케이블·허브가 끊긴 것으로 보고 1 s마다 포트를 새로 열어 본다
                now = time.monotonic()
                failed_since = failed_since or now
                if now - failed_since > 0.5 and now - reopen_at > 1.0:
                    reopen_at = now
                    try:
                        with self.lock:
                            self.bus.reopen()
                            self.present = self.bus.read()
                            # 끊긴 동안 밀린 궤적을 이어서 보내면 팔이 튄다. 버리고 지금 자세에서 다시 시작한다
                            self._queue.clear()
                            self.moving = False
                            self.target = dict(self.present)
                        self.fault, failed_since = '', None
                        log.warning('팔 버스를 다시 열었다')
                    except Exception as e2:          # noqa: BLE001 — 아직 안 붙었다
                        log.debug('다시 열기 실패: %s', e2)
            n += 1
            time.sleep(max(0.0, period - (time.monotonic() - t0)))

    def close(self) -> None:
        self._stop.set()
        self.thread.join(timeout=1.0)
        self.bus.close()

    # ---------------------------------------------------------------- 상태
    def joints(self) -> dict[str, float]:
        return dict(self.present)

    def q(self, deg: dict[str, float] | None = None) -> np.ndarray:
        return self.kin.to_urdf(deg or self.present)

    def tip(self) -> np.ndarray:
        return self.kin.tip(self.q())

    def gripper_current(self) -> float:
        return abs(float(self.aux['current_ma'].get('gripper', 0.0)))

    # ---------------------------------------------------------------- 안전 검사
    def surface_height(self, x, y) -> float:
        z = float(self.cfg['workspace']['table_z_m'])
        if self.pick_plane is None:
            return z
        mx, my, sx, sy, tx, ty = self.pick_plane
        # 현재 집기 주변에서만 판의 경사를 사용하고, 멀어지면 기존 바닥으로 이어 준다.
        weight = max(0.0, min(1.0, (0.09 - math.hypot(x - tx, y - ty)) / 0.04))
        return z + weight * max(-0.02, min(0.02, (x - mx) * sx + (y - my) * sy))

    def guard(self, deg: dict[str, float]) -> str:
        """이 자세가 차체·작업대와 부딪히면 이유를, 아니면 빈 문자열을 돌려준다."""
        ranges = getattr(self.bus, 'ranges', None)
        if ranges:
            for m in ARM_JOINTS:
                lo, hi = ranges[m]
                if not lo - 0.5 <= deg[m] <= hi + 0.5:
                    return f'{m} {deg[m]:.0f}°가 범위 [{lo:.0f}, {hi:.0f}] 밖'
        pts = self.kin.points(self.kin.to_urdf(deg))
        w = self.cfg['workspace']
        deck_x = float(w.get('deck_front_x_m', 0.10))
        for name, p in zip(('pan', 'lift', 'elbow', 'wrist', 'roll', 'tip'), pts):
            if name in ('pan', 'lift'):
                continue                              # 어깨는 움직이지 않는 자리다
            if p[0] < -0.12:
                return f'{name}이 뒤쪽 노트북 영역으로 들어간다 (x {p[0]:.2f})'
            table = self.surface_height(p[0], p[1]) + float(w['floor_clear_m'])
            floor = 0.035 if p[0] < deck_x else min(table, 0.035) if p[0] < deck_x + 0.03 else table
            if name != 'tip':
                floor += 0.02                         # 관절 몸통 두께
            if p[2] < floor - 1e-4:
                return f'{name}이 너무 낮다 (x {p[0]:.2f}, z {p[2]:.3f} < {floor:.3f})'
        return ''

    # ---------------------------------------------------------------- 이동
    def plan_move(self, start: dict, goal: dict, speed: float) -> list[tuple[float, dict]]:
        span = max(abs(goal[m] - start[m]) for m in ARM_JOINTS)
        gspan = abs(goal['gripper'] - start['gripper'])
        duration = max(1.875 * span / speed, gspan / 160.0, 0.25)
        n = max(2, int(duration * self.rate))
        steps = []
        for i in range(1, n + 1):
            s = min_jerk(i / n)
            steps.append((i / self.rate, {m: start[m] + s * (goal[m] - start[m]) for m in MOTORS}))
        return steps

    def move(self, goal: dict[str, float], speed: float | None = None, check: bool = True, wait: bool = True) -> None:
        """goal(일부 관절만 줘도 된다)로 간다. check가 참이면 경로 전체를 guard로 본다."""
        if not self.torque:
            raise ArmError('토크가 꺼져 있다')
        if self._teleop is not None:
            raise ArmError('텔레옵 중이다')
        with self.lock:
            start = dict(self.target)
        full = {**start, **{m: float(v) for m, v in goal.items() if m in MOTORS}}
        steps = self.plan_move(start, full, speed or self.speed)
        if check:
            for _, g in steps[::max(1, len(steps) // 25)] + [steps[-1]]:
                why = self.guard(g)
                if why:
                    raise ArmError(f'경로가 막힌다: {why}')
        t0 = time.monotonic()
        with self.lock:
            self._begin_motion()
            self._queue = [(t0 + t, g) for t, g in steps]
            self.moving = True
        if not wait:
            return
        end = t0 + steps[-1][0]
        while time.monotonic() < end + 0.05:
            if self._cancel.is_set():
                raise ArmError('멈춤')
            time.sleep(0.01)

    def swing(self, goal: dict[str, float], speed: float, release_at: float, open_to: float = 100.0) -> None:
        """goal까지 빠르게 휘두르며 도중(진행 비율 release_at)에 집게를 연다. 던지기에 쓴다.

        집게 서보는 목표를 한 번에 받는다(천천히 열면 물건이 손에 끌려간다). 경로는 guard로 본다.
        """
        if not self.torque:
            raise ArmError('토크가 꺼져 있다')
        if self._teleop is not None:
            raise ArmError('텔레옵 중이다')
        with self.lock:
            start = dict(self.target)
        full = {**start, **{m: float(v) for m, v in goal.items() if m in ARM_JOINTS}}
        steps = self.plan_move(start, full, speed)
        for i, (_, g) in enumerate(steps):
            g['gripper'] = float(open_to) if (i + 1) / len(steps) >= release_at else start['gripper']
        for _, g in steps[::max(1, len(steps) // 25)] + [steps[-1]]:
            why = self.guard(g)
            if why:
                raise ArmError(f'경로가 막힌다: {why}')
        t0 = time.monotonic()
        with self.lock:
            self._begin_motion()
            self._queue = [(t0 + t, g) for t, g in steps]
            self.moving = True
        end = t0 + steps[-1][0]
        while time.monotonic() < end + 0.05:
            if self._cancel.is_set():
                raise ArmError('멈춤')
            time.sleep(0.01)

    def pose(self, name: str, speed: float | None = None, **kw) -> None:
        if name not in self.poses:
            raise ArmError(f'모르는 자세: {name}')
        self.move(self.poses[name], speed, **kw)

    def set_gripper_target(self, value: float) -> None:
        with self.lock:
            if self.motion_blocked or not self.torque or self._teleop is not None or self._queue:
                raise ArmError('현재 집게를 직접 조작할 수 없다')
            self._queue.append((time.monotonic(), {'gripper': float(np.clip(value, 0, 100))}))

    def gripper(self, value: float, wait: bool = True) -> None:
        self.move({'gripper': float(np.clip(value, 0.0, 100.0))}, check=False, wait=wait)

    def _begin_motion(self) -> None:
        # 호출자가 lock을 잡고 큐를 교체한다. 이전 작업이 비전 응답 뒤 다시 움직이지 못하게 한다.
        if self.motion_blocked:
            raise ArmError('멈춤')
        self._cancel.clear()

    def stop(self, *, block_motion: bool = False) -> None:
        """지금 자리에서 멈춘다. 진행 중이던 move는 ArmError('멈춤')로 끝난다."""
        with self.lock:
            self._cancel.set()
            if block_motion:
                self.motion_blocked = True
            self._queue = []
            self.moving = False

    def set_torque(self, on: bool) -> None:
        self.stop()
        with self.lock:
            if on:
                self.present = self.bus.read()
                self.bus.write(dict(self.present))   # 켜는 순간 튀지 않게 지금 자세를 목표로
                self.target = dict(self.present)
            self.bus.set_torque(on)
            self.torque = on

    # ---------------------------------------------------------------- 텔레옵·녹화
    def set_teleop(self, source) -> None:
        self.stop()
        with self.lock:
            self._teleop = source

    def record_start(self) -> None:
        with self.lock:
            self._recorder = []

    def record_stop(self) -> list[tuple[float, dict]]:
        with self.lock:
            rec, self._recorder = self._recorder or [], None
        if not rec:
            return []
        t0 = rec[0][0]
        return [(t - t0, g) for t, g in rec]

    def follow(self, steps: list[tuple[float, dict]]) -> None:
        """정해진 시각(time.monotonic)마다 목표를 보낸다. 음악에 맞추는 춤에 쓴다. 경로 검사는 부르는 쪽이 한다.

        지나간 시각의 목표는 제어 스레드가 한꺼번에 건너뛴다(늦게 시작해도 음악 시각을 따라잡는다).
        """
        if not steps:
            return
        if not self.torque:
            raise ArmError('토크가 꺼져 있다')
        if self._teleop is not None:
            raise ArmError('텔레옵 중이다')
        with self.lock:
            self._begin_motion()
            self._queue = list(steps)
            self.moving = True
        end = steps[-1][0]
        while time.monotonic() < end + 0.05:
            if self._cancel.is_set():
                raise ArmError('멈춤')
            time.sleep(0.01)

    def play(self, frames: list[tuple[float, dict]], rate: float = 1.0) -> None:
        """녹화한 동작을 그대로 재생한다. 첫 자세까지는 천천히 간다."""
        if not frames:
            return
        self.move(frames[0][1], self.slow, check=False)
        t0 = time.monotonic() + 0.1
        with self.lock:
            self._begin_motion()
            self._queue = [(t0 + t / rate, g) for t, g in frames]
            self.moving = True
        end = t0 + frames[-1][0] / rate
        while time.monotonic() < end + 0.05:
            if self._cancel.is_set():
                raise ArmError('멈춤')
            time.sleep(0.01)
