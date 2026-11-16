"""로봇 토픽에서 화면에 보낼 값을 만든다.

ROS를 import하지 않는다. 콜백은 ros_io.py가 메시지를 풀어 숫자로 넘겨주고, 여기서는 그 숫자만
다룬다. 그래서 단위 시험을 ROS 없이 돌릴 수 있다.

모든 on_* 메서드는 ROS 스레드에서, snapshot()은 asyncio 스레드에서 부른다. 둘 다 self.lock을 잡는다.
"""
from __future__ import annotations

import math
import threading
import time
from collections import deque

import numpy as np

AGENT_STATES = {0: 'waiting', 1: 'available', 2: 'connected', 3: 'disconnected'}
TRACK_STATUS = {0: 'lost', 1: 'tracking', 2: 'holding'}
MISSION_STATES = {0: 'idle', 1: 'starting', 2: 'active', 3: 'arrived', 4: 'cancelling',
                  5: 'cancelled', 6: 'error', 255: 'unknown'}
TRACKING_STATES = {0: 'unavailable', 1: 'searching', 2: 'following', 3: 'lost', 4: 'error',
                   5: 'idle', 255: 'unknown'}
# twist_mux.yaml 우선순위 순서. 앞이 이긴다.
MUX_PRIORITY = ('teleop', 'align', 'follow')
MUX_TIMEOUT_S = 0.3

RATE_TOPICS = ('joint_states', 'base_status', 'imu', 'odom', 'marker_track', 'detections',
               'image', 'cmd_vel', 'phone_link', 'app_status', 'joy', 'scan', 'nav')


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF 고정축 rpy → 회전행렬 (Rz·Ry·Rx). 자식 좌표의 벡터를 부모 좌표로 옮긴다."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def interp_soc(volts: float, table: list[list[float]]) -> float:
    if volts <= table[0][0]:
        return float(table[0][1])
    for (v0, s0), (v1, s1) in zip(table, table[1:]):
        if volts <= v1:
            return s0 + (s1 - s0) * (volts - v0) / (v1 - v0)
    return float(table[-1][1])


def battery_verdict(topic_v: float, table: list[list[float]], diode: float,
                    charge_at: float, stop_at: float) -> dict:
    """토픽 전압 → 잔량과 판정. 사용자 아티팩트 "배터리 잔량표"와 같은 규칙이다.

    잔량은 단자 전압(토픽 + 다이오드 강하)으로 표에서 찾고, 휴지 전압의 흔들림 ±0.05 V가 만드는 범위를
    5 % 단위로 같이 낸다. 판정은 토픽 값으로 한다.
    """
    term = topic_v + diode
    if topic_v > table[-1][0] - diode + 0.05:
        return {'zone': 'charging', 'soc': 100.0, 'soc_lo': 100, 'soc_hi': 100, 'low': False}
    zone = 'stop' if topic_v < stop_at else 'charge' if topic_v < charge_at else 'ok'
    r5 = lambda x: int(5 * round(x / 5))  # noqa: E731
    return {'zone': zone, 'soc': _f(interp_soc(term, table), 0),
            'soc_lo': r5(interp_soc(term - 0.05, table)), 'soc_hi': r5(interp_soc(term + 0.05, table)),
            'low': zone != 'ok'}


def _f(x, nd: int = 4):
    """JSON에 NaN이 들어가면 브라우저 JSON.parse가 깨진다. 유한하지 않으면 null."""
    if x is None:
        return None
    x = float(x)
    return round(x, nd) if math.isfinite(x) else None


def _alpha(dt: float, tau: float) -> float:
    return 1.0 - math.exp(-max(dt, 0.0) / tau) if tau > 0 else 1.0


class RateMeter:
    """최근 window_s 동안의 수신 빈도."""

    def __init__(self, window_s: float = 2.0):
        self.window_s = window_s
        self.times: deque[float] = deque()
        self.last: float | None = None

    def tick(self, now: float) -> None:
        self.times.append(now)
        self.last = now

    def hz(self, now: float) -> float:
        while self.times and now - self.times[0] > self.window_s:
            self.times.popleft()
        return len(self.times) / self.window_s

    def age(self, now: float) -> float | None:
        return None if self.last is None else now - self.last


class BoothState:

    def __init__(self, cfg: dict, *, sim: bool = False):
        self.lock = threading.Lock()
        self.sim = sim
        robot, imu, bat = cfg['robot'], cfg['imu'], cfg['battery']
        self.r_wheel = float(robot['wheel_radius_m'])
        self.r_mcu = float(robot['mcu_wheel_radius_m'])
        self.sep_mcu = float(robot['mcu_wheel_separation_m'])
        self.cmd_fresh_s = float(robot['cmd_fresh_s'])
        self.R_imu = rpy_matrix(*[math.radians(a) for a in imu['rpy_deg']])
        self.r_imu = np.array(imu['xyz_m'], dtype=float)
        self.soc_table = [list(map(float, row)) for row in bat['soc_table']]
        self.bat_tau = float(bat['filter_tau_s'])
        self.diode = float(bat['diode_drop_v'])
        self.charge_at, self.stop_at = float(bat['charge_at_v']), float(bat['stop_at_v'])
        self.marker_scale = float(cfg['aruco']['pnp_translation_scale'])
        self.cam_origin = np.array(cfg['camera']['optical_origin_m'], dtype=float)

        self.ptt_button = cfg.get('live', {}).get('ptt_button')
        self.dpad_held = False
        self.dpad_hook = None
        self.ptt_down = False
        self.ptt_since = 0.0
        # 패드 B(멈춤)는 Pi에서 바퀴를 세운다. 같은 버튼으로 팔도 세우게 여기서 눌림을 잡아 넘긴다.
        self.pad_stop_button = cfg.get('arm', {}).get('stop_button')
        self.pad_stop_down = False
        self.pad_stop_hook = None
        self.grip_session = 0
        self.grip_value = None
        self.rt_axis = cfg.get('arm', {}).get('rt_axis', 4)
        self.lb_button = cfg.get('arm', {}).get('lb_button', 6)
        self.rates = {name: RateMeter() for name in RATE_TOPICS}

        # 바퀴
        self.wheel_pos = [0.0, 0.0]
        self.wheel_vel = [0.0, 0.0]
        self._prev_wheel_pos: list[float] | None = None
        self._still_since: float | None = None

        # MCU
        self.base: dict | None = None
        self._overruns: deque[tuple[float, int]] = deque()

        # 배터리
        self.v_raw: float | None = None
        self.v_filt: float | None = None
        self._v_time: float | None = None

        # 오도메트리 (/odom 그대로)
        self.odom: dict | None = None
        # 바퀴 거리 + 자이로 방위로 다시 적분한 경로. /odom과 나란히 그려 차이를 보여 준다.
        self.fused: dict | None = None

        # IMU
        self.gyro_heading = 0.0
        self.gyro_base = np.zeros(3)
        self.acc_fast = np.zeros(3)      # 약 4 Hz 저역. G 미터용
        self.acc_slow = np.array([0.0, 0.0, 9.81])   # 약 0.3 Hz 저역. 기울기용
        self._imu_stamp: float | None = None
        self._imu_seen = False
        self.v_imu = 0.0
        self.acc_bias_x = 0.0
        self.mismatch_pct: float | None = None

        # 마커
        self.track: dict | None = None
        self._rejects: deque[float] = deque()
        self.markers: list[dict] = []
        self._markers_time: float | None = None

        # 명령
        self.cmd: dict[str, tuple[float, float, float]] = {}   # source -> (time, v, w)

        # 시니어케어
        self.phone: dict | None = None
        self.app: dict | None = None
        self.cloud: dict | None = None
        self.pi: dict | None = None
        self.ops: dict | None = None      # 운영 버튼 마지막 결과 (ops.py)
        self.arm: dict | None = None      # 팔 데몬 상태 (arm_link.py). 데몬이 없으면 None
        self.lidar: dict | None = None    # 마지막 스캔 요약 (scan.py)
        self.filtered: tuple | None = None   # /odometry/filtered (시각, x, y, yaw)
        self.pose_hook = None
        self.nav: dict | None = None      # Pi errand_nav의 /nav/status
        self.tasks: dict | None = None    # 작업 순서기 (tasks.py)

    # ---------------------------------------------------------------- 바퀴
    def on_joint_states(self, now: float, pos: list[float], vel: list[float]) -> None:
        with self.lock:
            self.rates['joint_states'].tick(now)
            if len(pos) < 2:
                return
            self.wheel_pos = [float(pos[0]), float(pos[1])]
            if len(vel) >= 2:
                self.wheel_vel = [float(vel[0]), float(vel[1])]
            if self._prev_wheel_pos is not None and self.fused is not None:
                d_l = self.wheel_pos[0] - self._prev_wheel_pos[0]
                d_r = self.wheel_pos[1] - self._prev_wheel_pos[1]
                ds = self.r_wheel * (d_l + d_r) / 2.0
                # 방위는 바퀴 차분이 아니라 자이로 적분값을 쓴다. 이게 /odom과 갈리는 유일한 점이다.
                if self._imu_seen:
                    th = self.gyro_heading
                else:
                    th = self.odom['yaw'] if self.odom is not None else self.fused['yaw']
                self.fused['x'] += ds * math.cos(th)
                self.fused['y'] += ds * math.sin(th)
                self.fused['yaw'] = th
            self._prev_wheel_pos = list(self.wheel_pos)
            if max(abs(self.wheel_vel[0]), abs(self.wheel_vel[1])) < 0.05:
                if self._still_since is None:
                    self._still_since = now
            else:
                self._still_since = None

    def _is_still(self, now: float) -> bool:
        return self._still_since is not None and now - self._still_since > 0.3

    # ---------------------------------------------------------------- MCU
    def on_base_status(self, now: float, s: dict) -> None:
        with self.lock:
            self.rates['base_status'].tick(now)
            self.base = dict(s)
            self._overruns.append((now, int(s.get('loop_overruns', 0)), int(s.get('loop_count', 0))))
            while self._overruns and now - self._overruns[0][0] > 10.0:
                self._overruns.popleft()
            v = s.get('supply_voltage')
            if v is None or not math.isfinite(v):
                self.v_raw = None
                return
            self.v_raw = float(v)
            if self.v_filt is None or self._v_time is None:
                self.v_filt = self.v_raw
            else:
                self.v_filt += _alpha(now - self._v_time, self.bat_tau) * (self.v_raw - self.v_filt)
            self._v_time = now

    # ---------------------------------------------------------------- IMU
    def on_imu(self, now: float, stamp: float, gyro: list[float], acc: list[float]) -> None:
        with self.lock:
            self.rates['imu'].tick(now)
            w = self.R_imu @ np.asarray(gyro, dtype=float)
            a = self.R_imu @ np.asarray(acc, dtype=float)
            # 센서가 구동축 중점에서 16 cm 옆에 있다. 제자리 회전 2 rad/s면 구심 가속도가
            # 0.64 m/s²라 그대로 두면 G 미터가 회전을 가속으로 읽는다.
            a = a - np.cross(w, np.cross(w, self.r_imu))
            dt = 0.01 if self._imu_stamp is None else min(max(stamp - self._imu_stamp, 0.0), 0.05)
            self._imu_stamp = stamp
            if not self._imu_seen:
                self._imu_seen = True
                self.acc_fast = a.copy()
                self.acc_slow = a.copy()
                if self.fused is not None:
                    self.gyro_heading = self.fused['yaw']
            self.gyro_base = w
            self.gyro_heading = math.remainder(self.gyro_heading + w[2] * dt, math.tau)
            self.acc_fast += _alpha(dt, 1.0 / (2 * math.pi * 4.0)) * (a - self.acc_fast)
            self.acc_slow += _alpha(dt, 1.0 / (2 * math.pi * 0.3)) * (a - self.acc_slow)

            # 가속도만 적분한 전진 속도. 바퀴가 멈춰 있으면 0으로 되돌리고(ZUPT) 그동안의 평균을
            # bias로 잡는다. 멈추지 않으면 몇 초 안에 흘러가는 것이 정상이다.
            if self._is_still(now):
                self.acc_bias_x += _alpha(dt, 1.0) * (self.acc_fast[0] - self.acc_bias_x)
                self.v_imu = 0.0
            else:
                self.v_imu += (self.acc_fast[0] - self.acc_bias_x) * dt

            # 바퀴로 잰 회전율과 자이로의 차이. 회전 중일 때만 의미가 있다.
            if self.odom is not None and abs(w[2]) > 0.25:
                pct = (self.odom['w'] / w[2] - 1.0) * 100.0
                if math.isfinite(pct) and abs(pct) < 80:
                    self.mismatch_pct = pct if self.mismatch_pct is None else \
                        self.mismatch_pct + _alpha(dt, 1.5) * (pct - self.mismatch_pct)

    # ---------------------------------------------------------------- 오도메트리
    def on_odom(self, now: float, x: float, y: float, yaw: float, v: float, w: float) -> None:
        with self.lock:
            self.rates['odom'].tick(now)
            self.odom = {'x': x, 'y': y, 'yaw': yaw, 'v': v, 'w': w}
            if self.fused is None:
                self.fused = {'x': x, 'y': y, 'yaw': yaw}
                self.gyro_heading = yaw

    def reset_fused(self) -> None:
        """화면의 R 키. 자이로 경로를 지금 /odom 자세에 다시 맞춘다."""
        with self.lock:
            if self.odom is not None:
                self.fused = {k: self.odom[k] for k in ('x', 'y', 'yaw')}
                self.gyro_heading = self.odom['yaw']

    # ---------------------------------------------------------------- 마커
    def on_marker_track(self, now: float, t: dict) -> None:
        with self.lock:
            self.rates['marker_track'].tick(now)
            self.track = dict(t)
            self.track['time'] = now
            if t.get('rejected_jump'):
                self._rejects.append(now)
            while self._rejects and now - self._rejects[0] > 10.0:
                self._rejects.popleft()

    def on_detections(self, now: float, markers: list[tuple[int, list[float], list[float]]]) -> None:
        """/aruco_detections는 base_footprint 기준 원시 PnP다. 3D 표시용으로 marker_track과 같은 보정을 건다."""
        with self.lock:
            self.rates['detections'].tick(now)
            out = []
            for mid, p, q in markers:
                p = np.asarray(p, dtype=float)
                c = self.cam_origin + self.marker_scale * (p - self.cam_origin)
                out.append({'id': int(mid), 'p': [_f(v) for v in c], 'q': [_f(v, 5) for v in q]})
            self.markers = out
            self._markers_time = now

    def on_image(self, now: float) -> None:
        with self.lock:
            self.rates['image'].tick(now)

    # ---------------------------------------------------------------- 명령
    def on_cmd(self, now: float, source: str, v: float, w: float) -> None:
        with self.lock:
            if source == 'out':
                self.rates['cmd_vel'].tick(now)
            self.cmd[source] = (now, float(v), float(w))

    # ---------------------------------------------------------------- 시니어케어
    def on_phone_link(self, now: float, connected: bool, app_instance_id: str) -> None:
        with self.lock:
            self.rates['phone_link'].tick(now)
            self.phone = {'connected': bool(connected), 'app_instance_id': app_instance_id, 'time': now}

    def on_app_status(self, now: float, s: dict) -> None:
        with self.lock:
            self.rates['app_status'].tick(now)
            self.app = dict(s)
            self.app['time'] = now

    def on_joy(self, now: float, buttons=(), axes=()) -> None:
        with self.lock:
            age = self.rates['joy'].age(now)
            # Xbox joy_linux의 HAT0X/Y. 왼쪽·위쪽이 양수다.
            x, y = axes[6:8] if len(axes) >= 8 else (0, 0)
            horizontal, vertical = abs(x) > 0.5, abs(y) > 0.5
            direction = None
            if age is not None and age < 0.5 and not self.dpad_held and horizontal != vertical:
                if horizontal:
                    direction = 'left' if x > 0 else 'right'
                else:
                    direction = 'up' if y > 0 else 'down'
            self.dpad_held = horizontal or vertical
            dpad_hook = self.dpad_hook
            self.rates['joy'].tick(now)
            b = self.ptt_button
            down = type(b) is int and 0 <= b < len(buttons) and bool(buttons[b])
            if down and not self.ptt_down:
                self.ptt_since = now
            self.ptt_down = down
            sb = self.pad_stop_button
            stop = type(sb) is int and 0 <= sb < len(buttons) and bool(buttons[sb])
            fire = stop and not self.pad_stop_down and self.pad_stop_hook is not None
            self.pad_stop_down = stop
            if age is None or age >= 0.5:
                self.grip_value = None
            if (stop or not 0 <= self.lb_button < len(buttons) or not buttons[self.lb_button]
                    or not 0 <= self.rt_axis < len(axes) or not math.isfinite(axes[self.rt_axis])):
                self.grip_value = None
            else:
                # joy_linux RT는 놓으면 +1, 끝까지 누르면 -1. 처음 5%는 완전 열림.
                travel = min(1.0, max(0.0, (1 - axes[self.rt_axis]) / 2))
                if self.grip_value is None and travel > 0.01:
                    self.grip_session += 1
                if self.grip_value is not None or travel > 0.01:
                    self.grip_value = round(100 * min(1.0, (1 - travel) / 0.95))
        if direction and dpad_hook:
            dpad_hook(direction)
        if fire:
            self.pad_stop_hook()

    def pad_grip(self, now: float):
        with self.lock:
            age = self.rates['joy'].age(now)
            if self.grip_value is not None and age is not None and age < 0.5:
                return self.grip_session, self.grip_value
            return None

    # ---------------------------------------------------------------- 라이다·이동
    def on_scan(self, now: float, n: int, nearest: tuple[float, float, float] | None) -> None:
        with self.lock:
            self.rates['scan'].tick(now)
            self.lidar = {'n': int(n), 'nearest': nearest}

    def on_nav(self, now: float, status: dict) -> None:
        with self.lock:
            self.rates['nav'].tick(now)
            self.nav = dict(status)

    def nav_status(self, now: float | None = None) -> dict | None:
        """이동 노드가 살아 있으면 마지막 보고. 2 s 넘게 조용하면 None(노드가 내려갔다)."""
        now = time.monotonic() if now is None else now
        with self.lock:
            age = self.rates['nav'].age(now)
            return dict(self.nav) if self.nav is not None and age is not None and age < 2.0 else None

    def on_filtered(self, now: float, x: float, y: float, yaw: float) -> None:
        with self.lock:
            self.filtered = (now, x, y, yaw)
        if self.pose_hook is not None:
            self.pose_hook(now, x, y, yaw)      # 부스 배치(layout.py)가 로봇 자리를 안다

    def scan_poses(self, now: float) -> tuple[tuple | None, tuple | None]:
        """스캔 흔적을 쌓을 자세와, 같은 순간의 /odom 자세.

        화면의 로봇은 /odom(바퀴만)으로 그리는데 그쪽은 회전을 덜 재서, 그대로 쌓으면 돌 때마다 벽이 번진다.
        흔적은 EKF(/odometry/filtered)로 쌓고, 없으면 여기서 다시 적분한 경로(바퀴 거리 + 자이로 방위)로 쌓는다.
        화면은 두 자세의 차이만큼 흔적 전체를 옮겨 지금 로봇 자리에서 실시간 스캔과 겹치게 한다.
        """
        with self.lock:
            o = self.odom
            odom = None if o is None else (o['x'], o['y'], o['yaw'])
            if self.filtered is not None and now - self.filtered[0] < 0.5:
                return self.filtered[1:], odom
            if self.fused is not None:
                return (self.fused['x'], self.fused['y'], self.fused['yaw']), odom
            return odom, odom

    def markers_alive(self, now: float | None = None) -> bool:
        """마커 검출 메시지가 오고 있는가(마커가 안 보여도 검출기는 빈 메시지를 계속 낸다)."""
        now = time.monotonic() if now is None else now
        with self.lock:
            age = self.rates['detections'].age(now)
            return age is not None and age < 2.0

    def mission(self, now: float | None = None) -> str | None:
        now = time.monotonic() if now is None else now
        with self.lock:
            if self.app is None or now - self.app['time'] > 5.0:
                return None
            return MISSION_STATES.get(int(self.app.get('mission_state', 255)), 'unknown')

    def arm_state(self) -> dict | None:
        with self.lock:
            return self.arm

    def set_tasks(self, t: dict) -> None:
        with self.lock:
            self.tasks = t

    def current_trip_id(self) -> str | None:
        with self.lock:
            return (self.app or {}).get('trip_id') or None

    def set_arm(self, a: dict | None) -> None:
        with self.lock:
            self.arm = a

    def set_ops(self, o: dict) -> None:
        with self.lock:
            self.ops = o

    def set_cloud(self, c: dict) -> None:
        with self.lock:
            self.cloud = c

    def set_pi(self, p: dict) -> None:
        with self.lock:
            self.pi = p

    # ---------------------------------------------------------------- 출력
    def _wheel_targets(self, now: float) -> tuple[float, float] | None:
        out = self.cmd.get('out')
        if out is None or now - out[0] > self.cmd_fresh_s:
            return None
        _, v, w = out
        half = w * self.sep_mcu / 2.0
        # MCU가 하는 역기구학과 같다(공칭 반지름). 표시는 측정값과 같은 유효 반지름으로 m/s로 바꾼다.
        return ((v - half) / self.r_mcu * self.r_wheel, (v + half) / self.r_mcu * self.r_wheel)

    def snapshot(self, now: float) -> dict:
        with self.lock:
            rates = {k: round(m.hz(now), 1) for k, m in self.rates.items()}
            ages = {k: _f(m.age(now), 2) for k, m in self.rates.items()}
            targets = self._wheel_targets(now)
            base = self.base or {}
            base_fresh = self.rates['base_status'].age(now) is not None and \
                self.rates['base_status'].age(now) < 1.0

            def wheel(i: int, side: str) -> dict:
                return {
                    'v': _f(self.wheel_vel[i] * self.r_wheel),
                    'rpm': _f(self.wheel_vel[i] * 60.0 / math.tau, 1),
                    'pos': _f(self.wheel_pos[i], 3),
                    'target': None if targets is None else _f(targets[i]),
                    'duty': _f(base.get(f'duty_{side}')) if base_fresh else None,
                    'sat': bool(base.get(f'saturated_{side}')) if base_fresh else False,
                }

            overrun_10s = loop_hz = None
            if len(self._overruns) >= 2:
                (t0, o0, c0), (t1, o1, c1) = self._overruns[0], self._overruns[-1]
                overrun_10s = o1 - o0
                # TIM6 제어 루프 실측 주기. 수신 시각은 Wi-Fi로 몰려 오니 10 s 창으로 평균한다.
                if t1 - t0 > 1.0 and c1 >= c0:
                    loop_hz = (c1 - c0) / (t1 - t0)

            battery = None
            if self.v_raw is not None and base_fresh:
                battery = battery_verdict(self.v_filt, self.soc_table, self.diode, self.charge_at, self.stop_at)
                battery.update(v=_f(self.v_raw + self.diode, 2), v_topic=_f(self.v_raw, 2),
                               v_filt=_f(self.v_filt + self.diode, 2))

            mux_inputs = {}
            active = None
            for name in MUX_PRIORITY:
                c = self.cmd.get(name)
                age = None if c is None else now - c[0]
                mux_inputs[name] = _f(age, 2)
                if active is None and age is not None and age < MUX_TIMEOUT_S:
                    active = name
            out = self.cmd.get('out')
            out_fresh = out is not None and now - out[0] < self.cmd_fresh_s

            marker = None
            if self.track is not None:
                t = self.track
                x, y = float(t.get('x', 0.0)), float(t.get('y', 0.0))
                marker = {
                    'status': TRACK_STATUS.get(int(t.get('status', 0)), 'lost'),
                    'locked': bool(t.get('locked')),
                    'id': int(t.get('marker_id', 0)),
                    'x': _f(x, 3), 'y': _f(y, 3),
                    'range': _f(math.hypot(x, y), 3),
                    'bearing': _f(math.degrees(math.atan2(y, x)), 1),
                    'yaw': _f(t.get('yaw'), 3), 'yaw_valid': bool(t.get('yaw_valid')),
                    'hold_s': _f(t.get('hold_duration'), 2),
                    'rejects_10s': len(self._rejects),
                    'age_s': _f(now - t['time'], 2),
                }

            markers_age = None if self._markers_time is None else now - self._markers_time

            lidar = None
            scan_age = self.rates['scan'].age(now)
            if self.lidar is not None and scan_age is not None and scan_age < 1.5:
                near = self.lidar['nearest']
                lidar = {'n': self.lidar['n'], 'hz': rates['scan'],
                         'nearest_m': None if near is None else _f(near[0], 2),
                         'nearest_deg': None if near is None else _f(math.degrees(math.atan2(near[2], near[1])), 0)}
            nav_age = self.rates['nav'].age(now)
            nav = None if self.nav is None or nav_age is None or nav_age > 2.0 else self.nav

            imu = None
            if self._imu_seen:
                a_f, a_s = self.acc_fast, self.acc_slow
                imu = {
                    'gyro_z': _f(self.gyro_base[2]),
                    'odom_w': _f(self.odom['w']) if self.odom else None,
                    'mismatch_pct': _f(self.mismatch_pct, 1),
                    'ax': _f(a_f[0], 3), 'ay': _f(a_f[1], 3),
                    'roll': _f(math.degrees(math.atan2(a_s[1], a_s[2])), 2),
                    'pitch': _f(math.degrees(math.atan2(-a_s[0], math.hypot(a_s[1], a_s[2]))), 2),
                    'heading_gyro': _f(self.gyro_heading),
                    'v_imu': _f(self.v_imu, 3),
                    'v_wheel': _f((self.wheel_vel[0] + self.wheel_vel[1]) / 2.0 * self.r_wheel, 3),
                    'still': self._is_still(now),
                }

            def seconds_since(d: dict | None) -> float | None:
                return None if d is None else _f(now - d['time'], 1)

            return {
                'type': 'state',
                'sim': self.sim,
                'wall_ms': int(time.time() * 1000),
                'ptt': bool(self.ptt_down and ages['joy'] is not None and ages['joy'] < 0.5 and now - self.ptt_since < 30),
                'rates': rates,
                'ages': ages,
                'wheels': {'l': wheel(0, 'left'), 'r': wheel(1, 'right')},
                'base': None if self.base is None else {
                    'fresh': base_fresh,
                    'agent': AGENT_STATES.get(int(base.get('agent_state', 255)), 'unknown'),
                    'watchdog_ok': bool(base.get('cmd_watchdog_ok')),
                    'cmd_age_s': _f(base.get('cmd_age_s'), 2),
                    'loop_count': int(base.get('loop_count', 0)),
                    'loop_overruns': int(base.get('loop_overruns', 0)),
                    'overruns_10s': overrun_10s,
                    'loop_hz': _f(loop_hz, 1) if base_fresh else None,
                },
                'battery': battery,
                'odom': None if self.odom is None else {k: _f(v) for k, v in self.odom.items()},
                'fused': None if self.fused is None else {k: _f(v) for k, v in self.fused.items()},
                'imu': imu,
                'marker': marker,
                'markers3d': self.markers if markers_age is not None and markers_age < 1.0 else [],
                'markers3d_age': _f(markers_age, 2),
                'mux': {
                    'active': active,
                    'inputs': mux_inputs,
                    'v': _f(out[1], 3) if out_fresh else None,
                    'w': _f(out[2], 3) if out_fresh else None,
                },
                'phone': None if self.phone is None else {
                    'connected': self.phone['connected'],
                    'since_s': seconds_since(self.phone),
                },
                'app': None if self.app is None else {
                    'mission': MISSION_STATES.get(int(self.app.get('mission_state', 255)), 'unknown'),
                    'tracking': TRACKING_STATES.get(int(self.app.get('tracking_state', 255)), 'unknown'),
                    'trip_id': self.app.get('trip_id') or None,
                    'last_error': self.app.get('last_error') or None,
                    'age_s': seconds_since(self.app),
                },
                'cloud': self.cloud,
                'pi': self.pi,
                'ops': self.ops,
                'arm': self.arm,
                'lidar': lidar,
                'nav': nav,
                # 이동 노드가 쓰는 좌표(EKF)의 로봇 자세. 탑뷰 지도와 배치 패널이 이것으로 그린다
                'ekf': None if self.filtered is None or now - self.filtered[0] > 1.0 else [_f(self.filtered[1], 3), _f(self.filtered[2], 3), _f(self.filtered[3], 4)],
                'tasks': self.tasks,
            }
