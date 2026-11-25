"""robot/nav_core.py: 이동 제어법을 단순한 차동 구동 모형으로 돌려 본다."""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'robot'))
from nav_core import Navigator  # noqa: E402

DT = 0.05


class World:
    """참 자세로 굴리고, 보이는 마커와 스캔을 넣어 준다."""

    def __init__(self, marker=None, obstacles=()):
        self.x = self.y = self.th = 0.0
        self.t = 0.0
        self.nav = Navigator()
        self.marker = marker                  # (id, x, y[, 바깥 법선 yaw]) odom
        self.obstacles = list(obstacles)      # odom 점
        self.min_see = 0.0                    # 이보다 가까우면 마커가 화면 밖으로 나가 안 보인다
        self.nav.set_odom(0.0, 0.0, 0.0)

    def base(self, ox, oy):
        c, s = math.cos(self.th), math.sin(self.th)
        dx, dy = ox - self.x, oy - self.y
        return c * dx + s * dy, -s * dx + c * dy

    def run(self, seconds, scan=True):
        for _ in range(int(seconds / DT)):
            self.t += DT
            self.nav.set_odom(self.x, self.y, self.th)
            if self.marker:
                bx, by = self.base(*self.marker[1:3])
                if bx > max(0.2, self.min_see) and abs(math.atan2(by, bx - 0.25)) < 0.55:      # 카메라 화각 안
                    normal = None if len(self.marker) < 4 else self.marker[3] - self.th
                    self.nav.see_marker(self.marker[0], bx, by, self.t, normal)
            if scan:
                self.nav.set_scan([self.base(*p) for p in self.obstacles], self.t)
            v, w = self.nav.step(self.t)
            self.th += w * DT
            self.x += v * math.cos(self.th) * DT
            self.y += v * math.sin(self.th) * DT
            if not self.nav.active:
                break
        return self.nav.state


def test_marker_ahead_stops_at_standoff():
    w = World(marker=(5, 2.0, 0.3))
    w.nav.goto_marker(5, 0.6, w.t, 'a')
    assert w.run(40) == 'arrived'
    assert abs(math.hypot(2.0 - w.x, 0.3 - w.y) - 0.6) < 0.08
    assert w.nav.status(w.t)['id'] == 'a'


def test_marker_behind_is_found_by_turning():
    w = World(marker=(5, -1.5, 0.5))
    w.nav.goto_marker(5, 0.6, w.t)
    assert w.nav.state == 'searching'
    assert w.run(60) == 'arrived'
    assert abs(math.hypot(-1.5 - w.x, 0.5 - w.y) - 0.6) < 0.08


def test_missing_marker_fails_after_one_turn_without_driving():
    w = World()
    w.nav.goto_marker(9, 0.6, w.t)
    assert w.run(60) == 'failed' and w.nav.detail == 'not_found'
    assert math.hypot(w.x, w.y) < 1e-6


def test_return_home_and_face_the_same_way():
    w = World(marker=(5, 2.0, -0.4))
    assert w.nav.mark_home()
    w.nav.goto_marker(5, 0.6, w.t)
    assert w.run(40) == 'arrived'
    assert w.nav.goto_home(w.t, 'back')
    assert w.run(60) == 'arrived'
    assert math.hypot(w.x, w.y) < 0.08 and abs(math.atan2(math.sin(w.th), math.cos(w.th))) < 0.1
    home = w.nav.status(w.t)['home']
    assert math.hypot(*home) < 0.08


def test_obstacle_blocks_then_times_out_and_clearing_resumes():
    w = World(obstacles=[(1.0, 0.0), (1.0, 0.1)])
    w.nav.goto_pose(2.5, 0.0, None, w.t)
    w.run(8)
    assert w.nav.active and w.nav.blocked
    assert w.x < 1.0 - 0.30 - 0.2                       # 차체 앞끝이 닿기 전에 섰다
    stopped_at = w.x
    w.obstacles.clear()
    assert w.run(40) == 'arrived' and w.x > stopped_at
    # 안 비키면 실패로 끝난다
    w2 = World(obstacles=[(1.0, 0.0)])
    w2.nav.goto_pose(2.5, 0.0, None, w2.t)
    assert w2.run(70) == 'failed' and w2.nav.detail == 'blocked'


def test_desk_holding_the_marker_is_not_an_obstacle():
    desk = [(1.5, y / 20) for y in range(-6, 7)]        # 마커가 붙은 책상 앞면
    w = World(marker=(5, 1.5, 0.0), obstacles=desk)
    w.nav.goto_marker(5, 0.42, w.t, align=True)
    assert w.run(40) == 'arrived'
    assert 0.10 <= 1.5 - w.x - w.nav.cfg.body_front_m <= 0.18


def test_close_marker_approach_requires_fresh_scan_and_sees_protruding_obstacle():
    w = World(marker=(5, 1.5, 0.0), obstacles=[(1.30, 0.02)])
    w.nav.goto_marker(5, 0.42, w.t, align=True)
    w.run(2, scan=False)
    assert w.nav.blocked and w.x == 0.0
    w.run(15)
    assert w.nav.blocked and 0 < w.x < 1.30 - w.nav.cfg.body_front_m - 0.10
    w.obstacles.clear()
    assert w.run(40) == 'arrived'


def test_desk_with_recessed_marker_creeps_past_old_stop_without_passing_alert():
    w = World(marker=(5, 0.714, 0.056), obstacles=[(0.57, 0.0)])
    w.nav.goto_marker(5, 0.42, w.t, align=True)
    w.run(8)
    assert w.x > 0.10 and w.nav.blocked
    assert not w.nav.status(w.t)['passing']


def test_close_sudden_obstruction_triggers_passing_until_clear():
    w = World(marker=(5, 1.5, 0.0))
    w.nav.goto_marker(5, 0.42, w.t)
    w.run(1)
    w.obstacles.append((w.x + 0.43, 0.0))
    w.run(2)
    assert w.nav.blocked and w.nav.status(w.t)['passing']
    w.obstacles.clear()
    assert w.run(40) == 'arrived'
    assert not w.nav.status(w.t)['passing']


def test_oblique_approach_to_a_wide_desk_is_not_blocked_by_the_desk():
    desk = [(2.0, 0.3 + i * 0.02) for i in range(61)]   # x=2.0에 선 폭 1.2 m 앞면, 마커는 가운데
    w = World(marker=(5, 2.0, 0.9), obstacles=desk)
    w.nav.goto_marker(5, 0.6, w.t)
    assert w.run(60) == 'arrived'
    assert min(math.hypot(x - w.x, y - w.y) for x, y in desk) > 0.30 + 0.08     # 차체 앞끝이 닿지 않았다


def test_obstacle_right_at_the_goal_still_blocks():
    w = World(obstacles=[(1.25, 0.0)])                  # 목표 지점 바로 앞에 놓인 물건
    w.nav.goto_pose(1.0, 0.0, None, w.t)
    assert w.run(70) == 'failed' and w.nav.detail == 'blocked'
    assert w.x < 1.25 - 0.30 - 0.05


def test_without_lidar_it_still_goes_and_says_so():
    w = World()
    w.nav.goto_pose(1.0, 0.5, None, w.t)
    assert w.run(40, scan=False) == 'arrived'
    assert w.nav.status(w.t)['lidar'] is False


def test_cancel_stops_and_reports():
    w = World()
    w.nav.goto_pose(3.0, 0.0, None, w.t)
    w.run(3)
    assert w.nav.cancel('pad') and w.nav.state == 'cancelled'
    assert w.nav.step(w.t + DT) == (0.0, 0.0)
    assert not w.nav.cancel()


# ------------------------------------------------------------ 곧게 가기·돌기
def test_move_backward_a_little_and_stop():
    w = World()
    w.nav.move(-0.3, 0.0, w.t, 'm')
    assert w.run(20) == 'arrived'
    assert abs(w.x + 0.3) < 0.04 and abs(w.y) < 0.02 and abs(w.th) < 0.05


def test_move_forward_then_turn_left():
    w = World()
    w.nav.move(0.5, math.radians(90), w.t)
    assert w.run(30) == 'arrived'
    assert abs(w.x - 0.5) < 0.05 and abs(w.th - math.pi / 2) < 0.12


def test_turn_all_the_way_around():
    w = World()
    w.nav.move(0.0, math.radians(-360), w.t)
    assert w.nav.state == 'turning'
    turned, last = 0.0, 0.0
    for _ in range(600):
        w.run(DT)
        turned += math.atan2(math.sin(w.th - last), math.cos(w.th - last))
        last = w.th
        if not w.nav.active:
            break
    assert w.nav.state == 'arrived' and abs(turned + 2 * math.pi) < 0.15 and math.hypot(w.x, w.y) < 1e-6


def test_reverse_stops_for_something_behind():
    w = World(obstacles=[(-0.40, 0.0), (-0.40, 0.1)])            # 차체 뒤끝(0.25)에서 15 cm
    w.nav.move(-0.5, 0.0, w.t)
    assert w.run(10) == 'failed' and w.nav.detail == 'blocked'
    assert abs(w.x) < 0.02
    w2 = World(obstacles=[(0.6, 0.0)])                           # 앞에 있는 것은 뒤로 가는 데 상관없다
    w2.nav.move(-0.3, 0.0, w2.t)
    assert w2.run(20) == 'arrived'


# ------------------------------------------------------------ 마커 정면 맞추기
def test_align_ends_on_the_marker_normal_facing_it():
    # 마커는 (2, 0)에 있고 면이 +y쪽(법선 yaw 90°)을 본다. 로봇은 비스듬히 앞에서 출발한다.
    w = World(marker=(5, 2.0, 0.0, math.pi / 2))
    w.x, w.y, w.th = 0.8, 1.6, -0.9
    w.nav.goto_marker(5, 0.6, w.t, align=True)
    assert w.run(75) == 'arrived'
    assert abs(w.x - 2.0) < 0.12 and abs(w.y - 0.6) < 0.08                    # 법선 위 0.6 m
    assert abs(math.atan2(math.sin(w.th + math.pi / 2), math.cos(w.th + math.pi / 2))) < 0.15   # 마커를 정면으로 본다


def test_without_align_it_just_faces_the_marker_from_where_it_came():
    w = World(marker=(5, 2.0, 0.0, math.pi / 2))
    w.x, w.y, w.th = 0.8, 1.6, -0.9
    w.nav.goto_marker(5, 0.6, w.t)
    assert w.run(60) == 'arrived'
    assert abs(w.x - 2.0) > 0.2                                               # 온 방향 그대로 비스듬히 서 있다


def test_flipped_normal_is_ignored():
    w = World()
    w.nav.see_marker(5, 2.0, 0.0, 0.0, normal=0.0)                           # 로봇 반대쪽을 보는 법선(뒤집힌 PnP 해)
    assert 5 not in w.nav.normals
    w.nav.see_marker(5, 2.0, 0.0, 0.1, normal=math.pi)
    assert 5 in w.nav.normals


# ------------------------------------------------------------ 마커 따라가기
class Walker(World):
    """마커를 든 사람이 +x로 걷는다."""

    def __init__(self, speed=0.15, **kw):
        super().__init__(marker=(3, 1.5, 0.0), **kw)
        self.speed = speed
        self.walk_until = 1e9

    def run(self, seconds, scan=True):
        for _ in range(int(seconds / DT)):
            if self.marker and self.t < self.walk_until:
                self.marker = (3, self.marker[1] + self.speed * DT, self.marker[2])
            super().run(DT, scan)
        return self.nav.state


def test_follow_marker_keeps_the_range_and_never_arrives():
    w = Walker()
    w.nav.follow_marker(3, 1.0, w.t, 'f')
    assert w.run(20) == 'following'
    assert w.x > 2.0 and abs((w.marker[1] - w.x) - 1.0) < 0.25
    w.walk_until = w.t                                                        # 사람이 선다
    assert w.run(8) == 'following' and abs(w.nav.v) < 1e-6
    assert abs((w.marker[1] - w.x) - 1.0) < 0.15


def test_follow_waits_when_lost_then_gives_up():
    w = Walker()
    w.nav.follow_marker(3, 1.0, w.t)
    w.run(5)
    x = w.x
    w.marker = None                                                           # 마커가 가려졌다
    assert w.run(5) == 'following' and w.nav.detail == 'lost'
    assert abs(w.x - x) < 0.25                                                # 기억한 자리까지만 가고 선다
    assert w.run(30) == 'failed' and w.nav.detail == 'lost'


def test_follow_stops_when_someone_else_comes_close_and_resumes():
    w = Walker()
    w.nav.follow_marker(3, 1.0, w.t)
    w.run(6)
    assert w.nav.v > 0.05 and not w.nav.status(w.t)['guard']['hold']
    # 옆에서 누가 차체 10 cm 옆까지 왔다(로봇 기준 왼쪽). 다리 두 개가 점 여럿으로 찍힌다
    def beside():
        c, s = math.cos(w.th), math.sin(w.th)
        return [(w.x + c * dx - s * 0.35, w.y + s * dx + c * 0.35) for dx in (-0.05, 0.0, 0.05, 0.1)]
    x = w.x
    for _ in range(60):
        w.obstacles = beside()
        w.run(DT)
    assert w.nav.status(w.t)['guard']['hold'] and abs(w.x - x) < 0.03 and w.nav.state == 'following'
    w.obstacles = []
    w.run(1.0)
    assert w.nav.status(w.t)['guard']['hold']                                 # 1.5 s는 비어 있어야 풀린다
    w.run(4.0)
    assert not w.nav.status(w.t)['guard']['hold'] and w.x > x + 0.2


def test_the_followed_persons_legs_do_not_trip_the_guard():
    w = Walker(speed=0.0)
    w.marker = (3, 0.75, 0.0)                                                 # 사람이 로봇 코앞(차체 앞끝에서 45 cm)에 서 있다
    w.obstacles = [(0.72, -0.08), (0.72, -0.04), (0.72, 0.04), (0.72, 0.08)]  # 그 사람의 다리
    w.nav.follow_marker(3, 1.0, w.t)
    w.run(3)
    assert not w.nav.status(w.t)['guard']['hold']
    assert w.x < -0.05                                                        # 가까우니 물러난다


def guard_after(seconds, scene):
    """scene을 seconds 동안 본 접근 감지기."""
    from nav_core import Guard, NavConfig
    g, t = Guard(NavConfig()), 0.0
    while t < seconds:
        g.observe(scene, t, (0.0, 0.0, 0.0))
        t += 0.1
    return g, t


def test_guard_ignores_a_single_stray_point_and_needs_history():
    someone = [(0.4, 0.0), (0.42, 0.03), (0.44, 0.06)]
    g, t = guard_after(5.0, [])
    assert not g.update([(0.4, 0.0), (3.0, 1.0)], t, (0.0, 0.0, 0.0))
    g.observe(someone, t, (0.0, 0.0, 0.0))
    assert g.update(someone, t, (0.0, 0.0, 0.0)) and g.near == (0.4, 0.0)
    fresh, t = guard_after(1.0, [])                         # 막 켠 감지기는 무엇이 원래 있던 것인지 모른다
    assert not fresh.update(someone, t, (0.0, 0.0, 0.0))


def test_guard_ignores_what_was_already_there_but_not_someone_who_walks_up_and_stays():
    desk = [(0.55, y / 20) for y in range(-5, 6)]           # 차체 앞끝에서 24 cm에 있는 책상
    g, t = guard_after(5.0, desk)
    assert not g.update(desk, t, (0.0, 0.0, 0.0))
    # 로봇이 돌아도(오도메트리로 같은 자리) 책상은 책상이다
    th = 0.5
    turned = [(math.cos(-th) * x - math.sin(-th) * y, math.sin(-th) * x + math.cos(-th) * y) for x, y in desk]
    g.observe(turned, t, (0.0, 0.0, th))
    assert not g.update(turned, t, (0.0, 0.0, th))
    # 누가 옆으로 와서 선다. 오래 서 있어도 풀리지 않는다
    someone = [(0.0, 0.40), (0.05, 0.40), (0.10, 0.42)]
    for _ in range(100):
        t += 0.1
        g.observe(desk + someone, t, (0.0, 0.0, 0.0))
        held = g.update(desk + someone, t, (0.0, 0.0, 0.0))
    assert held
    for _ in range(20):
        t += 0.1
        g.observe(desk, t, (0.0, 0.0, 0.0))
        held = g.update(desk, t, (0.0, 0.0, 0.0))
    assert not held


# ------------------------------------------------------------ 조이스틱이 먼저
def test_joystick_pauses_without_timing_out_and_resumes_from_where_it_was_left():
    w = World()
    w.nav.goto_pose(2.0, 0.0, None, w.t, 'j')
    w.run(3)
    assert w.nav.v > 0.1
    # 사람이 조이스틱으로 옆으로 끌고 가 한참 세워 둔다. 그동안 이동 노드는 아무것도 내지 않는다
    w.x, w.y, w.th = 0.8, -0.7, 1.0
    for _ in range(int(300 / DT)):                       # 제한 시간(120 s)보다 길게
        w.t += DT
        w.nav.set_odom(w.x, w.y, w.th)
        assert w.nav.step(w.t, paused=True) == (0.0, 0.0)
    assert w.nav.active and w.nav.status(w.t)['paused'] and w.nav.status(w.t)['id'] == 'j'
    assert w.run(40) == 'arrived' and math.hypot(w.x - 2.0, w.y) < 0.08
    assert not w.nav.status(w.t)['paused']


def test_turn_command_does_not_count_rotation_done_by_joystick():
    w = World()
    w.nav.move(0.0, math.radians(90), w.t)
    for _ in range(20):                                  # 조이스틱으로 반대로 60도 돌려 놓는다
        w.t += DT
        w.th -= math.radians(3)
        w.nav.set_odom(w.x, w.y, w.th)
        w.nav.step(w.t, paused=True)
    start = w.th
    assert w.run(20) == 'arrived' and abs((w.th - start) - math.pi / 2) < 0.15


def test_trip_slot_returns_to_where_it_was_told_and_keeps_home():
    # "돌아와"의 자리(home)는 원점. 그 뒤 돌아선 자세에서 다녀오는 심부름을 시켰다: 돌아선 그 자세로 돌아온다
    w = World(marker=(5, 0.3, 2.2))
    assert w.nav.mark_home()
    w.nav.move(0.0, math.radians(90), w.t)
    assert w.run(20) == 'arrived'
    told = (w.x, w.y, w.th)
    assert w.nav.mark_home('trip') and w.nav.home == (0.0, 0.0, 0.0)
    w.nav.goto_marker(5, 0.6, w.t)
    assert w.run(40) == 'arrived'
    assert w.nav.goto_home(w.t, 'back', 'trip')
    assert w.run(60) == 'arrived'
    assert math.hypot(w.x - told[0], w.y - told[1]) < 0.08
    assert abs(math.atan2(math.sin(w.th - told[2]), math.cos(w.th - told[2]))) < 0.1       # 떠날 때 보던 쪽
    assert w.nav.goto_home(w.t, 'h') and w.run(40) == 'arrived' and abs(math.atan2(math.sin(w.th), math.cos(w.th))) < 0.1


def test_unknown_slot_falls_back_to_home():
    w = World()
    w.nav.mark_home()
    assert w.nav.goto_home(w.t, 'x', 'trip')


def test_final_heading_is_corrected_after_coasting_past_it():
    # 실물은 0을 보낸 뒤에도 조금 더 돈다. 허용오차를 벗어나 섰으면 다시 맞추고 끝낸다
    w = World()
    w.nav.mark_home()
    w.th = 2.6
    w.nav.goto_home(w.t, 'b')
    coasted = False
    for _ in range(600):
        w.t += DT
        w.nav.set_odom(w.x, w.y, w.th)
        v, om = w.nav.step(w.t)
        w.th += om * DT
        if not coasted and om == 0.0 and w.nav.active:
            w.th -= 0.15                         # 선 순간 관성으로 8.6° 더 돌았다
            coasted = True
        if not w.nav.active:
            break
    assert coasted and w.nav.state == 'arrived' and abs(w.th) < 0.1


def test_backs_off_from_the_desk_before_turning_around():
    # 책상(앞 0.12 m)에 붙어 선 채 뒤로 돌아가야 한다. 그대로 돌면 모서리가 책상을 긁는다
    desk = [(0.42, y / 100) for y in range(-40, 41, 4)]
    w = World(obstacles=desk)
    w.nav.goto_pose(-1.5, 0.0, 0.0, w.t)
    w.run(0.7)
    assert w.x < -0.03 and abs(w.th) < 0.02                    # 돌기 전에 곧게 물러난다
    assert w.run(60) == 'arrived' and abs(w.x + 1.5) < 0.08 and abs(math.atan2(math.sin(w.th), math.cos(w.th))) < 0.1


def test_no_back_off_when_something_is_behind_or_nothing_is_ahead():
    w = World(obstacles=[(0.42, 0.0), (-0.40, 0.0), (-0.40, 0.1)])
    w.nav.goto_pose(0.0, 1.5, None, w.t)
    w.run(1.0)
    assert abs(w.x) < 0.02 and w.th > 0.2                      # 뒤가 막혀 있다: 물러나지 않고 돈다
    w = World()
    w.nav.goto_pose(-1.5, 0.0, None, w.t)
    w.run(1.0)
    assert abs(w.x) < 0.02 and abs(w.th) > 0.2


def test_close_desk_is_reached_squarely_from_oblique_starts():
    # 가까이(0.42 m) 서는 책상. 비스듬히 출발해도 마커 정면 0.8 m 지점을 거쳐 곧게 들어가 정면으로 선다.
    # 정렬 없이, 또는 0.42 m 지점으로 바로 가면 차체 앞 모서리가 책상에 걸려 blocked로 끝난다
    desk = [(3.6, 1.0 + i * 0.03) for i in range(41)] + [(3.6 + i * 0.03, y) for i in range(24) for y in (1.0, 2.2)]
    for start in ((2.69, 0.27, -1.02), (1.68, 0.50, 0.41), (2.6, 2.9, 0.0), (1.0, -0.5, 3.0)):
        w = World(marker=(5, 3.6, 1.6, math.pi), obstacles=desk)
        w.x, w.y, w.th = start
        w.nav.set_odom(*start)
        w.nav.goto_marker(5, 0.42, w.t, 'a', True)
        assert w.run(120) == 'arrived', (start, w.nav.detail)
        assert abs(w.y - 1.6) < 0.06 and abs(math.atan2(math.sin(w.th), math.cos(w.th))) < 0.15, start
        assert abs((3.6 - w.x) - 0.42) < 0.08


def test_docking_finishes_on_the_remembered_marker_when_it_leaves_the_frame():
    # 카메라 코앞(0.6 m 안)에서는 마커가 화면 밖으로 나간다. 책상 옆에서 돌며 찾지 않고 기억한 자리로 마무리한다
    desk = [(3.6, 1.0 + i * 0.03) for i in range(41)]
    w = World(marker=(5, 3.6, 1.6, math.pi), obstacles=desk)
    w.min_see = 0.6
    w.x, w.y, w.th = 1.68, 0.50, 0.41
    w.nav.set_odom(w.x, w.y, w.th)
    w.nav.goto_marker(5, 0.42, w.t, 'a', True)
    turned, last = 0.0, w.th
    for _ in range(120):
        st = w.run(1.0)
        turned += abs(w.th - last)
        last = w.th
        if st == 'arrived':
            break
    assert st == 'arrived' and abs((3.6 - w.x) - 0.42) < 0.08 and abs(w.y - 1.6) < 0.06
    assert turned < math.pi                     # 한 바퀴 돌며 찾지 않았다
    # 그 자리에서 다시 시켜도(마커는 여전히 안 보인다) 기억으로 곧 도착한다
    w.nav.goto_marker(5, 0.42, w.t, 'b', True)
    assert w.run(10) == 'arrived'
