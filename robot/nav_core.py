"""심부름 이동 제어법. rclpy를 import하지 않는다(배선은 errand_nav.py).

네 가지 목표를 받는다.

    goto_marker(id, standoff)   그 ArUco 마커 앞 standoff 거리에서 마커를 보고 선다. 안 보이면 제자리에서 돌며 찾는다.
                                align이면 마커 면의 정면(법선 위)에 선다.
    goto_pose(x, y, yaw)        오도메트리 좌표의 한 점으로 간다(출발한 자리로 돌아올 때).
    move(dist, turn)            지금 자리에서 곧게 dist(음수면 뒤로)만큼 가고 turn만큼 돈다("뒤로 조금만", "왼쪽으로 돌아").
    follow_marker(id, range)    그 마커를 range 거리로 계속 따라간다. 멈추라고 할 때까지 끝나지 않는다.

지도도 경로 계획도 없다. 목표를 향해 돌고, 곧게 가고, 앞이 막히면 선다.

- 마커는 볼 때마다 오도메트리 좌표로 옮겨 기억한다. 잠깐 가려져도 기억한 자리로 계속 가고,
  오래 못 보면(lost_s) 다시 돌며 찾는다. 도착 판정은 방금 본 값으로만 한다.
- 라이다: 차체 앞 통로(corridor) 안에 점이 있으면 전진을 멈추고 기다린다(대시보드가 "지나갈게요"라고 말한다).
  비켜 주면 가던 길을 이어서 간다. blocked_timeout_s를 넘기면 실패.
  차체가 평소 정지 거리보다 가까이 가는 마커 목표는 최신 스캔이 있어야 전진하며 마커 주변 점도 검사한다.
- 뒤로 갈 때는 차체 뒤 통로를 같은 방법으로 본다.
- 따라가는 중에는 Guard(아래)가 차체 둘레를 본다. 따라가는 사람이 아닌 누군가가 바짝 다가오면 선다.
- 조이스틱이 언제나 먼저다. 사람이 조종하는 동안(step의 paused)은 지령을 내지 않고 쉬다가, 놓으면 이어서 간다.
- 제자리 회전은 min_turn_rps 아래로 내지 않는다. 바닥 실측에서 0.20 rad/s는 20 %만 돌았다(mr_aruco align.yaml).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

TERMINAL = ('idle', 'arrived', 'failed', 'cancelled')


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class NavConfig:
    max_linear_mps: float = 0.22
    max_reverse_mps: float = 0.10
    max_angular_rps: float = 0.6
    max_linear_accel: float = 0.4
    max_angular_accel: float = 2.0
    k_linear: float = 0.7
    k_angular: float = 1.6
    min_turn_rps: float = 0.30
    turn_first_rad: float = 0.45        # 목표가 이보다 옆에 있으면 제자리에서 먼저 돈다
    pos_tol_m: float = 0.06
    yaw_tol_rad: float = 0.08
    range_tol_m: float = 0.06
    bearing_tol_rad: float = 0.08
    settle_s: float = 0.4               # 허용오차 안에서 이만큼 머물면 도착
    # 마커 찾기
    search_rps: float = 0.45
    search_max_rad: float = 2.2 * math.pi
    fresh_s: float = 0.5                # 이 안에 본 마커는 "지금 보인다"
    lost_s: float = 3.0                 # 접근 중 이만큼 못 보면 다시 찾는다
    memory_s: float = 30.0              # 기억한 마커 자리를 믿는 시간
    # 라이다
    body_front_m: float = 0.30          # base_footprint에서 차체 앞끝까지
    corridor_half_m: float = 0.26       # 차체 반폭 + 여유
    stop_dist_m: float = 0.28           # 앞끝에서 이 안에 점이 있으면 선다
    slow_dist_m: float = 0.80           # 이 안이면 속도를 줄인다
    scan_fresh_s: float = 0.6
    marker_clear_m: float = 0.30        # 목적지 마커 둘레 이 안의 점은 장애물로 안 본다
    dock_speed_mps: float = 0.06
    passing_gap_m: float = 0.15         # 차체 앞끝 기준
    passing_drop_m: float = 0.08        # 연속 스캔 사이에 갑자기 줄어든 거리
    goal_margin_m: float = 0.10         # 도착했을 때 차체 앞에 최소한 남아야 할 틈
    blocked_timeout_s: float = 45.0     # 앞이 막히면 이만큼 서서 기다린다. 비켜 주면 이어서 간다
    goal_timeout_s: float = 120.0
    body_rear_m: float = 0.25           # base_footprint에서 차체 뒤끝까지
    rear_stop_dist_m: float = 0.20
    # 마커 정면 맞추기(align)
    align_tol_m: float = 0.10           # 법선에서 옆으로 이만큼 안이면 정면으로 본다
    align_tries: float = 3              # 이만큼 다시 맞춰 보고 안 되면 마커를 보고 선 것으로 끝낸다
    normal_face_cos: float = 0.25       # 법선이 로봇 쪽을 이만큼도 안 보면 버린다(평면 마커의 뒤집힘)
    # 곧게 가기·돌기(move)
    move_tol_m: float = 0.02
    move_max_m: float = 1.5
    move_blocked_timeout_s: float = 4.0
    # 마커 따라가기(follow)
    follow_deadband_m: float = 0.12
    follow_hold_s: float = 1.0          # 이만큼은 기억한 자리로 계속 간다
    follow_lost_s: float = 20.0         # 이만큼 못 보면 그만둔다
    # 접근 감지(Guard). 차체 상자에서 이 거리 안에 점이 guard_min_points개 넘게 있으면 선다
    guard_front_m: float = 0.40
    guard_side_m: float = 0.25
    guard_rear_m: float = 0.30
    guard_release_m: float = 0.08       # 풀 때는 이만큼 더 물러나야 한다
    guard_release_s: float = 1.5
    guard_min_points: float = 3
    guard_exempt_m: float = 0.50        # 따라가는 마커(사람) 둘레는 빼고 본다
    guard_static_s: float = 2.0         # 이만큼 전부터 차 있던 자리는 고정물(벽, 책상)로 본다
    guard_forget_s: float = 8.0         # 이만큼 안 보인 자리는 잊는다
    guard_watch_m: float = 2.0          # 이 안의 점만 적어 둔다
    body_half_m: float = 0.25


@dataclass
class Goal:
    kind: str                           # 'marker' | 'pose' | 'move' | 'follow'
    id: str = ''
    marker_id: int = -1
    standoff_m: float = 0.6
    x: float = 0.0
    y: float = 0.0
    yaw: float | None = None
    started: float = 0.0
    align: bool = False
    dist: float = 0.0                   # move: 곧게 갈 거리(음수면 뒤로)
    turn: float = 0.0                   # move: 돌 각도 [rad]. 한 바퀴를 넘겨도 된다


class Guard:
    """따라가는 중에 누가 차체로 바짝 다가왔는지 본다.

    차체 상자를 앞·옆·뒤로 조금 키운 영역 안에 스캔 점이 여러 개 들어오면 걸린다. 다만 원래 거기 있던 것(벽, 책상,
    기둥)은 세지 않는다: 스캔 점을 오도메트리 좌표의 격자 칸으로 적어 두고, guard_static_s 넘게 전부터 차 있던 칸의
    점은 고정물로 본다. 그래서 책상 앞에 서 있거나 벽을 끼고 가도 걸리지 않고, 새로 다가온 것만 걸린다.
    따라가는 사람의 다리는 exempt(그 마커의 자리)로 뺀다. 한 번 걸리면 그 사람이 가만히 서 있어도 풀리지 않고
    (걸린 순간의 고정물만 뺀다), 영역을 조금 더 키운 안이 guard_release_s 동안 비어야 풀린다.
    """

    CELL = 0.10

    def __init__(self, cfg: NavConfig):
        self.cfg = cfg
        self.cells: dict[tuple[int, int], list[float]] = {}     # 칸 → [처음 본 시각, 마지막으로 본 시각]
        self.started: float | None = None
        self.hold = False
        self.near: tuple[float, float] | None = None
        self._clear_since: float | None = None
        self._ref = 0.0                                         # 고정물 판정의 기준 시각(걸린 순간에 얼린다)

    def reset(self) -> None:
        """멈춤을 푼다. 적어 둔 칸은 그대로 둔다."""
        self.hold, self.near, self._clear_since = False, None, None

    def _cell(self, px: float, py: float, pose) -> tuple[int, int]:
        x, y, th = pose
        c, s = math.cos(th), math.sin(th)
        return round((x + c * px - s * py) / self.CELL), round((y + s * px + c * py) / self.CELL)

    def observe(self, points, now: float, pose) -> None:
        """스캔 한 바퀴를 적어 둔다. 따라가지 않을 때도 계속 부른다."""
        if pose is None:
            return
        c = self.cfg
        self.started = now if self.started is None else self.started
        for px, py in points:
            if abs(px) > c.guard_watch_m or abs(py) > c.guard_watch_m:
                continue
            k = self._cell(px, py, pose)
            seen = self.cells.get(k)
            if seen is None or now - seen[1] > c.guard_forget_s:
                self.cells[k] = [now, now]
            else:
                seen[1] = now
        if len(self.cells) > 4000:
            self.cells = {k: v for k, v in self.cells.items() if now - v[1] <= c.guard_forget_s}

    def _static(self, k: tuple[int, int], ref: float) -> bool:
        limit = ref - self.cfg.guard_static_s
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                seen = self.cells.get((k[0] + dx, k[1] + dy))
                if seen is not None and seen[0] <= limit:
                    return True
        return False

    def update(self, points, now: float, pose, exempt=()) -> bool:
        c = self.cfg
        # 적어 둔 것이 아직 짧으면(막 켰다) 무엇이 고정물인지 모른다. 그동안은 걸지 않는다
        if pose is None or self.started is None or now - self.started < c.guard_static_s + 0.5:
            return self.hold
        if not self.hold:
            self._ref = now
        grow = c.guard_release_m if self.hold else 0.0
        x0, x1 = -c.body_rear_m - c.guard_rear_m - grow, c.body_front_m + c.guard_front_m + grow
        half = c.body_half_m + c.guard_side_m + grow
        n, best, near = 0, math.inf, None
        for px, py in points:
            if not (x0 < px < x1 and abs(py) < half):
                continue
            if any(math.hypot(px - ex, py - ey) < c.guard_exempt_m for ex, ey in exempt):
                continue
            if self._static(self._cell(px, py, pose), self._ref):
                continue
            n += 1
            d = math.hypot(px, py)
            if d < best:
                best, near = d, (px, py)
        if n >= c.guard_min_points:
            self.hold, self.near, self._clear_since = True, near, None
        elif self.hold:
            self._clear_since = self._clear_since or now
            if now - self._clear_since >= c.guard_release_s:
                self.reset()
        return self.hold


@dataclass
class Navigator:
    cfg: NavConfig = field(default_factory=NavConfig)

    def __post_init__(self) -> None:
        self.pose: tuple[float, float, float] | None = None
        self.markers: dict[int, tuple[float, float, float]] = {}    # id → (odom x, odom y, 본 시각)
        self.scan: list[tuple[float, float]] = []                   # base_footprint 기준 점
        self.scan_at = -1e9
        self.goal: Goal | None = None
        self.state = 'idle'
        self.detail = ''
        self.home: tuple[float, float, float] | None = None
        self.v = self.w = 0.0
        self.blocked = False
        self._blocked_since: float | None = None
        self._settled_since: float | None = None
        self._searched = 0.0
        self._search_dir = 1.0
        self._last_step: float | None = None
        self._final = False
        self._ahead: float | None = None
        self._passing = False
        self.paused = False
        self.normals: dict[int, tuple[float, float]] = {}           # id → (바깥 법선의 odom yaw, 본 시각)
        self.guard = Guard(self.cfg)
        self._aligning = False
        self._align_count = 0
        self._origin: tuple[float, float, float] | None = None      # move를 시작한 자세
        self._turned = 0.0
        self._last_th: float | None = None
        self._straight_done = False

    # ------------------------------------------------------------ 입력
    def set_odom(self, x: float, y: float, yaw: float) -> None:
        self.pose = (x, y, yaw)

    def see_marker(self, marker_id: int, bx: float, by: float, now: float, normal: float | None = None) -> None:
        """base_footprint 기준으로 본 마커를 오도메트리 좌표로 옮겨 기억한다.

        normal은 마커 면 바깥 법선의 방향(base_footprint yaw)이다. 정면 맞추기에 쓴다.
        """
        if self.pose is None:
            return
        x, y, th = self.pose
        c, s = math.cos(th), math.sin(th)
        mid = int(marker_id)
        self.markers[mid] = (x + c * bx - s * by, y + s * bx + c * by, now)
        if normal is None:
            return
        # 법선은 로봇 쪽을 봐야 한다. 평면 마커는 멀리서 기울기가 뒤집혀 나오기도 한다(PnP 모호성). 그런 값은 버린다.
        to_robot = math.atan2(-by, -bx)
        if math.cos(wrap(normal - to_robot)) < self.cfg.normal_face_cos:
            return
        yaw = wrap(normal + th)
        old = self.normals.get(mid)
        if old is not None and now - old[1] < self.cfg.memory_s:
            yaw = wrap(old[0] + 0.3 * wrap(yaw - old[0]))           # 느리게 따라간다
        self.normals[mid] = (yaw, now)

    def set_scan(self, points: list[tuple[float, float]], now: float) -> None:
        previous = self._clearance(now) if self.active and self.v > 0 else None
        self.scan, self.scan_at = points, now
        if previous is not None or self._passing:
            gap = self._clearance(now)
            if gap > self.cfg.passing_gap_m + 0.02:
                self._passing = False
            elif previous is not None and gap <= self.cfg.passing_gap_m and previous - gap >= self.cfg.passing_drop_m:
                self._passing = True
        self.guard.observe(points, now, self.pose)

    # ------------------------------------------------------------ 명령
    def goto_marker(self, marker_id: int, standoff_m: float, now: float, goal_id: str = '', align: bool = False) -> None:
        self._begin(Goal('marker', goal_id, int(marker_id), float(standoff_m) or 0.6, started=now, align=bool(align)))
        seen = self.markers.get(int(marker_id))
        if seen and now - seen[2] < self.cfg.memory_s:
            self.state = 'approaching'
            # 기억한 쪽으로 먼저 돈다
            _, bearing = self._to_base(seen[0], seen[1])
            self._search_dir = 1.0 if bearing >= 0 else -1.0
        else:
            self.state = 'searching'

    def goto_pose(self, x: float, y: float, yaw: float | None, now: float, goal_id: str = '') -> None:
        self._begin(Goal('pose', goal_id, x=float(x), y=float(y), yaw=yaw, started=now))
        self.state = 'driving'

    def move(self, dist_m: float, turn_rad: float, now: float, goal_id: str = '') -> None:
        c = self.cfg
        dist = max(-c.move_max_m, min(c.move_max_m, float(dist_m)))
        turn = max(-4 * math.pi, min(4 * math.pi, float(turn_rad)))
        self._begin(Goal('move', goal_id, started=now, dist=dist, turn=turn))
        self._origin = self.pose
        self._last_th = None if self.pose is None else self.pose[2]
        self.state = 'driving' if abs(dist) > c.move_tol_m else 'turning'

    def follow_marker(self, marker_id: int, range_m: float, now: float, goal_id: str = '') -> None:
        self._begin(Goal('follow', goal_id, int(marker_id), float(range_m) or 1.0, started=now))
        seen = self.markers.get(int(marker_id))
        self.state = 'following' if seen and now - seen[2] < self.cfg.fresh_s else 'searching'

    def mark_home(self) -> bool:
        if self.pose is None:
            return False
        self.home = self.pose
        return True

    def goto_home(self, now: float, goal_id: str = '') -> bool:
        if self.home is None:
            return False
        self.goto_pose(*self.home, now, goal_id)
        return True

    def cancel(self, why: str = '') -> bool:
        """움직이는 중이었으면 True. 노드가 0을 한 번 쏜다."""
        if self.state in TERMINAL:
            return False
        self._finish('cancelled', why)
        return True

    def _begin(self, goal: Goal) -> None:
        self.goal = goal
        self._passing = False
        self.detail = ''
        self.blocked = False
        self._blocked_since = self._settled_since = None
        self._searched = 0.0
        self._final = False
        self._aligning = False
        self._align_count = 0
        self._turned = 0.0
        self._straight_done = False
        self.guard.reset()

    def _finish(self, state: str, detail: str = '') -> None:
        self.state, self.detail = state, detail
        self.v = self.w = 0.0
        self.blocked = False
        self.guard.reset()

    @property
    def active(self) -> bool:
        return self.state not in TERMINAL

    # ------------------------------------------------------------ 한 주기
    def step(self, now: float, paused: bool = False) -> tuple[float, float]:
        """paused: 사람이 조이스틱으로 몰고 있다. 목표는 그대로 두고 쉰다. 놓으면 그 자리에서 이어서 간다."""
        dt = 0.05 if self._last_step is None else max(1e-3, min(0.2, now - self._last_step))
        self._last_step = now
        self.paused = paused and self.active
        if not self.active or self.goal is None:
            self.v = self.w = 0.0
            return 0.0, 0.0
        if paused:
            # 쉬는 동안은 제한 시간도 멈춘다. 다시 갈 때는 속도 0에서 출발한다
            self.goal.started += dt
            self.v = self.w = 0.0
            self.blocked = False
            self._blocked_since = self._settled_since = None
            self.guard.reset()
            if self.pose is not None:
                self._last_th = self.pose[2]         # 조이스틱으로 돈 각도는 "돌아" 명령에 세지 않는다
            return 0.0, 0.0
        c, g = self.cfg, self.goal
        if self.pose is None:
            self._finish('failed', 'no_odom')
            return 0.0, 0.0
        if g.kind != 'follow' and now - g.started > c.goal_timeout_s:
            self._finish('failed', 'timeout')
            return 0.0, 0.0

        v = w = 0.0
        if g.kind == 'marker':
            v, w = self._marker(now, dt)
        elif g.kind == 'move':
            v, w = self._move()
        elif g.kind == 'follow':
            v, w = self._follow(now, dt)
        else:
            v, w = self._pose()
        if not self.active:
            return 0.0, 0.0

        # 제자리 회전의 하한. 주행 중 조향에는 걸지 않는다(곡선이 거칠어진다).
        if v == 0.0 and w != 0.0 and abs(w) < c.min_turn_rps:
            w = math.copysign(c.min_turn_rps, w)

        # 라이다 정지·감속
        ahead = self._clearance(now)
        self._ahead = ahead
        self.blocked = False
        if v > 0.0 and ahead is None and self._close_marker_goal():
            v = w = 0.0
            self.blocked = True
        elif v > 0.0 and ahead is not None:
            # 목표가 코앞이면 남은 거리 + 여유만 비어 있으면 된다. 책상 앞 0.6 m에 서려면 차체 앞끝과 책상 사이가
            # 0.3 m뿐이라, 평소 정지 거리를 그대로 쓰면 목적지 자체에 막힌다.
            stop = min(c.stop_dist_m, self._remaining() + c.goal_margin_m)
            seen = self.markers.get(g.marker_id)
            if (self._close_marker_goal() and seen is not None and now - seen[2] < c.fresh_s
                    and self._to_base(seen[0], seen[1])[0] <= g.standoff_m + 0.45):
                v = min(v, c.dock_speed_mps)
                stop = c.goal_margin_m + c.dock_speed_mps * c.scan_fresh_s
            if self._passing:
                stop = max(stop, c.passing_gap_m + 0.02)
            if ahead <= stop:
                v, self.blocked = 0.0, True
                w = 0.0                          # 막힌 채로 돌면 옆을 긁는다
            elif ahead < c.slow_dist_m:
                v = min(v, max(0.05, c.max_linear_mps * (ahead - stop) / (c.slow_dist_m - stop)))
        elif v < 0.0:
            rear = self._clearance_rear(now)
            if rear is not None and rear < c.rear_stop_dist_m:
                v, w, self.blocked = 0.0, 0.0, True
        # 따라가는 중: 따라가는 사람 말고 누가 바짝 붙었으면 선다
        if g.kind == 'follow' and now - self.scan_at <= c.scan_fresh_s:
            seen = self.markers.get(g.marker_id)
            exempt = []
            if seen is not None and now - seen[2] < c.lost_s:
                rng, bearing = self._to_base(seen[0], seen[1])
                exempt.append((rng * math.cos(bearing), rng * math.sin(bearing)))
            if self.guard.update(self.scan, now, self.pose, exempt):
                v = w = 0.0
                self.v = self.w = 0.0
        if self.blocked:
            self._blocked_since = self._blocked_since or now
            limit = c.move_blocked_timeout_s if g.kind == 'move' else c.blocked_timeout_s
            if g.kind != 'follow' and now - self._blocked_since > limit:
                self._finish('failed', 'blocked')
                return 0.0, 0.0
        else:
            self._blocked_since = None

        # 가속 한계
        dv, dw = c.max_linear_accel * dt, c.max_angular_accel * dt
        self.v += max(-dv, min(dv, v - self.v))
        self.w += max(-dw, min(dw, w - self.w))
        if self.blocked:
            self.v = 0.0                         # 장애물 앞에서는 감속 구간 없이 바로 선다
        return self.v, self.w

    # ------------------------------------------------------------ 마커로
    def _marker(self, now: float, dt: float) -> tuple[float, float]:
        c, g = self.cfg, self.goal
        seen = self.markers.get(g.marker_id)
        age = None if seen is None else now - seen[2]
        if self.state == 'searching':
            if age is not None and age < c.fresh_s:
                self.state = 'approaching'
            else:
                self._searched += abs(self.w) * dt
                if self._searched > c.search_max_rad:
                    self._finish('failed', 'not_found')
                    return 0.0, 0.0
                return 0.0, self._search_dir * c.search_rps
        # 정면 맞추기: 마커 법선 위의 standoff 지점으로 먼저 간다. 가는 동안 마커가 화면 밖으로 나가도 기억한 자리로 간다.
        lat = self._lateral(now) if g.align else None
        if lat is not None and seen is not None and age < c.memory_s:
            n = self.normals[g.marker_id][0]
            px, py = seen[0] + g.standoff_m * math.cos(n), seen[1] + g.standoff_m * math.sin(n)
            dist, to_p = self._to_base(px, py)
            if not self._aligning and abs(lat) > c.align_tol_m and self._align_count < c.align_tries:
                self._aligning = True
                self._align_count += 1
                self._settled_since = None
            if self._aligning:
                if dist < c.pos_tol_m:
                    self._aligning = False       # 지점에 왔다. 아래에서 마커를 보고 돈다
                else:
                    self.state = 'aligning'
                    if abs(to_p) > c.turn_first_rad:
                        return 0.0, self._turn(to_p)
                    return min(c.max_linear_mps, max(0.04, c.k_linear * dist)), self._turn(to_p)
            self.state = 'approaching'
        if age is None or age > c.lost_s:
            # 가다가 놓쳤다. 마지막으로 본 쪽으로 돌며 다시 찾는다.
            if seen is not None:
                _, bearing = self._to_base(seen[0], seen[1])
                self._search_dir = 1.0 if bearing >= 0 else -1.0
            self.state, self._searched, self._settled_since = 'searching', 0.0, None
            return 0.0, self._search_dir * c.search_rps
        rng, bearing = self._to_base(seen[0], seen[1])
        err = rng - g.standoff_m
        ok = abs(err) < c.range_tol_m and abs(bearing) < c.bearing_tol_rad
        if ok and lat is not None and abs(lat) > c.align_tol_m and self._align_count < c.align_tries:
            ok = False                           # 거리와 방향은 맞지만 비스듬히 서 있다. 다음 주기에 다시 맞춘다
        if ok and age < c.fresh_s:
            self._settled_since = self._settled_since or now
            if now - self._settled_since >= c.settle_s:
                self._finish('arrived')
            return 0.0, 0.0
        if not ok:
            self._settled_since = None
        if abs(bearing) > c.turn_first_rad:
            return 0.0, self._turn(bearing)
        v = 0.0 if abs(err) < c.range_tol_m * 0.6 else max(-c.max_reverse_mps, min(c.max_linear_mps, c.k_linear * err))
        w = 0.0 if abs(bearing) < c.bearing_tol_rad * 0.5 else self._turn(bearing)
        return v, w

    def _lateral(self, now: float) -> float | None:
        """마커 법선에서 옆으로 벗어난 거리 [m]. 법선을 모르면 None."""
        g = self.goal
        seen, nrm = self.markers.get(g.marker_id), self.normals.get(g.marker_id)
        if seen is None or nrm is None or now - nrm[1] > self.cfg.memory_s:
            return None
        rx, ry = self.pose[0] - seen[0], self.pose[1] - seen[1]
        return -math.sin(nrm[0]) * rx + math.cos(nrm[0]) * ry

    # ------------------------------------------------------------ 곧게 가기·돌기
    def _move(self) -> tuple[float, float]:
        c, g = self.cfg, self.goal
        x, y, th = self.pose
        if self._origin is None:
            self._origin, self._last_th = self.pose, th
        x0, y0, th0 = self._origin
        self._turned += wrap(th - self._last_th)
        self._last_th = th
        if not self._straight_done:
            gone = (x - x0) * math.cos(th0) + (y - y0) * math.sin(th0)
            left = g.dist - gone
            if abs(left) < c.move_tol_m or left * g.dist <= 0:
                self._straight_done = True
                self._turned = 0.0
            else:
                self.state = 'driving'
                top = c.max_linear_mps if left > 0 else c.max_reverse_mps
                v = math.copysign(min(top, max(0.04, c.k_linear * abs(left))), left)
                return v, max(-0.3, min(0.3, c.k_angular * wrap(th0 - th)))   # 곧게 가도록 처음 방향을 잡는다
        left = g.turn - self._turned
        if abs(left) < c.yaw_tol_rad:
            self._finish('arrived')
            return 0.0, 0.0
        self.state = 'turning'
        return 0.0, self._turn(left)

    # ------------------------------------------------------------ 마커 따라가기
    def _follow(self, now: float, dt: float) -> tuple[float, float]:
        c, g = self.cfg, self.goal
        seen = self.markers.get(g.marker_id)
        age = None if seen is None else now - seen[2]
        if self.state == 'searching':
            # 처음에 안 보이면 한 바퀴 돌며 찾는다
            if age is not None and age < c.fresh_s:
                self.state = 'following'
            else:
                self._searched += abs(self.w) * dt
                if self._searched > c.search_max_rad:
                    self._finish('failed', 'not_found')
                    return 0.0, 0.0
                return 0.0, self._search_dir * c.search_rps
        if age is None or age > c.follow_hold_s:
            # 놓쳤다. 서서 기다린다(돌며 찾으면 사람이 다가올 때 위험하다)
            self.detail = 'lost'
            if age is None or age > c.follow_lost_s:
                self._finish('failed', 'lost')
            return 0.0, 0.0
        self.detail = ''
        rng, bearing = self._to_base(seen[0], seen[1])
        err = rng - g.standoff_m
        if abs(bearing) > c.turn_first_rad:
            return 0.0, self._turn(bearing)
        v = 0.0 if abs(err) < c.follow_deadband_m else max(-c.max_reverse_mps, min(c.max_linear_mps, c.k_linear * err))
        w = 0.0 if abs(bearing) < c.bearing_tol_rad else self._turn(bearing)
        return v, w

    # ------------------------------------------------------------ 좌표로
    def _pose(self) -> tuple[float, float]:
        c, g = self.cfg, self.goal
        x, y, th = self.pose
        dist = math.hypot(g.x - x, g.y - y)
        if not self._final and dist < c.pos_tol_m:
            self._final = True
        elif self._final and dist > 3 * c.pos_tol_m:
            self._final = False                  # 밀려났다. 다시 간다
        if self._final:
            self.state = 'turning'
            if g.yaw is None:
                self._finish('arrived')
                return 0.0, 0.0
            err = wrap(g.yaw - th)
            if abs(err) < c.yaw_tol_rad:
                self._finish('arrived')
                return 0.0, 0.0
            return 0.0, self._turn(err)
        self.state = 'driving'
        err = wrap(math.atan2(g.y - y, g.x - x) - th)
        if abs(err) > c.turn_first_rad:
            return 0.0, self._turn(err)
        return min(c.max_linear_mps, max(0.04, c.k_linear * dist)), self._turn(err)

    def _remaining(self) -> float:
        """앞으로 더 가야 할 거리 [m]."""
        g = self.goal
        if g.kind == 'pose':
            return math.hypot(g.x - self.pose[0], g.y - self.pose[1])
        if self._aligning and self._close_marker_goal():
            seen = self.markers[g.marker_id]
            n = self.normals[g.marker_id][0]
            return self._to_base(seen[0] + g.standoff_m * math.cos(n),
                                 seen[1] + g.standoff_m * math.sin(n))[0]
        if g.kind in ('move', 'follow') or self._aligning:
            return math.inf                      # 목적지에 물체가 있는 것이 아니다. 평소 정지 거리를 그대로 쓴다
        seen = self.markers.get(g.marker_id)
        return math.inf if seen is None else max(0.0, self._to_base(seen[0], seen[1])[0] - g.standoff_m)

    def _turn(self, err: float) -> float:
        c = self.cfg
        return max(-c.max_angular_rps, min(c.max_angular_rps, c.k_angular * err))

    def _to_base(self, ox: float, oy: float) -> tuple[float, float]:
        """오도메트리 좌표의 점 → 지금 로봇에서 본 거리와 방위."""
        x, y, th = self.pose
        dx, dy = ox - x, oy - y
        return math.hypot(dx, dy), wrap(math.atan2(dy, dx) - th)

    def _close_marker_goal(self) -> bool:
        g = self.goal
        return (g is not None and g.kind == 'marker'
                and g.standoff_m < self.cfg.body_front_m + self.cfg.stop_dist_m)

    def _clearance(self, now: float) -> float | None:
        """차체 앞끝에서 통로 안 가장 가까운 점까지 [m]. 스캔이 없거나 낡았으면 None."""
        c = self.cfg
        if now - self.scan_at > c.scan_fresh_s:
            return None
        skip = None
        if self.goal is not None and self.goal.kind in ('marker', 'follow') and not self._close_marker_goal():
            seen = self.markers.get(self.goal.marker_id)
            if seen is not None and self.pose is not None:
                rng, bearing = self._to_base(seen[0], seen[1])
                skip = (rng * math.cos(bearing), rng * math.sin(bearing))
        best = math.inf
        for px, py in self.scan:
            if px <= c.body_front_m or abs(py) > c.corridor_half_m:
                continue
            if skip is not None and math.hypot(px - skip[0], py - skip[1]) < c.marker_clear_m:
                continue
            best = min(best, px - c.body_front_m)
        return best

    def _clearance_rear(self, now: float) -> float | None:
        """차체 뒤끝에서 뒤 통로 안 가장 가까운 점까지 [m]."""
        c = self.cfg
        if now - self.scan_at > c.scan_fresh_s:
            return None
        best = math.inf
        for px, py in self.scan:
            if px < -c.body_rear_m and abs(py) <= c.corridor_half_m:
                best = min(best, -px - c.body_rear_m)
        return best

    # ------------------------------------------------------------ 보고
    def status(self, now: float) -> dict:
        g = self.goal
        out = {
            'state': self.state, 'detail': self.detail, 'id': g.id if g else '',
            'kind': g.kind if g else '', 'marker_id': g.marker_id if g and g.kind in ('marker', 'follow') else None,
            'guard': {'hold': self.guard.hold, 'near': None if self.guard.near is None else [round(v, 2) for v in self.guard.near]},
            'paused': self.paused, 'blocked': self.blocked, 'lidar': now - self.scan_at <= self.cfg.scan_fresh_s,
            'passing': self.active and self.blocked and self._passing and now - self.scan_at <= self.cfg.scan_fresh_s,
            'odom': self.pose is not None,
            'ahead_m': None if self._ahead is None or math.isinf(self._ahead) else round(self._ahead, 2),
            'goal': None, 'home': None, 'dist_m': None,
            'visible': sorted(i for i, (_, _, t) in self.markers.items() if now - t < self.cfg.fresh_s),
        }
        if self.pose is None:
            return out

        def rel(ox: float, oy: float) -> list[float]:
            rng, bearing = self._to_base(ox, oy)
            return [round(rng * math.cos(bearing), 3), round(rng * math.sin(bearing), 3)]

        if self.home is not None:
            out['home'] = rel(self.home[0], self.home[1])
        if g is not None and self.state != 'idle':
            if g.kind == 'pose':
                out['goal'] = rel(g.x, g.y)
            elif g.kind != 'move' and g.marker_id in self.markers:
                out['goal'] = rel(*self.markers[g.marker_id][:2])
            if out['goal'] is not None:
                d = math.hypot(*out['goal'])
                out['dist_m'] = round(max(0.0, d - g.standoff_m) if g.kind in ('marker', 'follow') else d, 2)
        return out
