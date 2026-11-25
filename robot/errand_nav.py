#!/usr/bin/env python3
"""심부름 이동 노드. Pi에서 mr-booth@nav로 돈다(시뮬레이션에서는 노트북에서 가짜 로봇과 같이).

대시보드 브리지가 /nav/command(JSON)로 시키고, 이 노드가 /cmd_vel_align으로 바퀴를 몬다.
제어법은 nav_core.py에 있고 여기는 배선만 한다.

    {"cmd": "goto_marker", "id": "t3", "marker_id": 5, "standoff_m": 0.6, "align": true}   align: 마커 정면에 선다
    {"cmd": "move", "id": "t6", "dist_m": -0.3, "turn_deg": 0}           곧게 가고(음수면 뒤로) 돈다
    {"cmd": "follow_marker", "id": "t7", "marker_id": 3, "range_m": 1.0}  그 마커를 계속 따라간다
    {"cmd": "mark_home"}            지금 자세(자리와 보는 쪽)를 돌아올 곳으로 기억한다. "slot": "trip"이면 그 심부름만의 자리
    {"cmd": "goto_home", "id": "t4"}  기억한 자리로 가서 그때 보던 쪽으로 돌아선다. "slot"은 위와 같다
    {"cmd": "goto_pose", "id": "t5", "x": 1.0, "y": 0.0, "yaw": 0.0}     오도메트리 좌표
    {"cmd": "cancel"}

/nav/status(JSON, 5 Hz와 바뀔 때마다)에 state(idle·searching·approaching·aligning·driving·turning·following·arrived·
failed·cancelled), detail(not_found·blocked·timeout·no_odom·lost), 목표와 돌아올 곳의 로봇 기준 위치를 낸다.

접근 감지(guard): 따라가는 중(추종 노드의 /cmd_vel_follow가 나오는 동안, 또는 follow_marker)에 따라가는 사람이 아닌
누군가가 차체에 바짝 붙으면 align 자리에 0을 계속 내서 세운다. 물러나면 놓는다. status.guard.hold로 알린다.
대시보드가 "가까이 오셨네요" 대사를 낸다. guard_enabled:=false로 끈다.

- 조이스틱(패드 LB·RB + 스틱)이 언제나 먼저다. /cmd_vel_teleop이 나오는 동안은 지령을 내지 않고 목표를 든 채 쉬고,
  놓으면 1초 뒤 그 자리에서 이어서 간다. 접근 감지와 라이다 정지도 조이스틱은 막지 않는다. 그만두게 하려면 패드 B다.
- twist_mux 입력은 align 자리(우선순위 60)를 쓴다. 부스 스택에는 align 노드가 없어 비어 있다. 패드 LB(teleop)가
  항상 이기고, 따라가기(follow)보다는 위다.
- 멈춤은 지금 있는 경로를 전부 듣는다: /phone/trip_command의 취소(앱·대시보드 Space·패드 B가 여기로 온다)와
  패드 B. 출발(따라가기)이 들어와도 이동을 그만둔다.
- 발행 규칙은 follow 노드와 같다. 움직이는 동안만 내고, 끝나면 0을 한 번 쏘고 놓는다.
- 자세는 /odometry/filtered(EKF)를 쓰고, 안 오면 /odom(바퀴만)으로 간다.
- 마커는 /aruco_detections 전부를 본다(marker_track은 따라갈 배지 하나만 고른다). 거리 보정은 marker_track.yaml과 같다.
"""
from __future__ import annotations

import json
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Joy, LaserScan
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from nav_core import NavConfig, Navigator  # noqa: E402

RELIABLE = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE)


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ErrandNav(Node):

    def __init__(self):
        super().__init__('errand_nav')
        p = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731
        cfg = NavConfig()
        for name in vars(cfg):
            setattr(cfg, name, float(p(name, float(getattr(cfg, name)))))
        self.nav = Navigator(cfg)
        self.guard = self.nav.guard             # 접근 감지. 추종 노드가 몰 때는 여기서, follow_marker 때는 Navigator가 본다
        self.rate_hz = float(p('control_rate_hz', 20.0))
        # mr_description robot_params.yaml lidar_xyz_m / lidar_rpy_deg (base_link 기준, 평면만 쓴다)
        lx, ly = p('lidar_xy_m', [0.018, -0.004])
        self.lidar = (float(lx), float(ly), math.radians(float(p('lidar_yaw_deg', 177.5))))
        # 로봇 자기 몸(팔, 기둥)에 맞은 점은 버린다
        self.self_radius = float(p('lidar_self_radius_m', 0.20))
        # 차체 틀의 네 기둥이 늘 찍힌다(2026-11-15 실측). base_footprint 기준 [x0, x1, y0, y1] 안은 버린다.
        # 부스 config/booth.yaml lidar.self_box_m과 같은 값
        self.self_box = tuple(float(v) for v in p('lidar_self_box_m', [-0.25, 0.31, -0.25, 0.25]))
        # mr_aruco marker_track.yaml
        self.pnp_scale = float(p('pnp_translation_scale', 0.960729786))
        self.cam_xy = tuple(float(v) for v in p('camera_origin_xy_m', [0.250, 0.0]))
        self.b_stop = int(p('button_stop', 1))
        self.guard_enabled = bool(p('guard_enabled', True))
        if not self.guard_enabled:
            cfg.guard_min_points = 1e9          # Navigator 안의 것도 같이 끈다
        # Nav2 전역 경로 주행 연동(옵션). 켜 두면 goto_pose·goto_home을 Nav2 NavigateToPose(A*+RPP)로
        # 위임하고, 근접 마커 정렬(goto_marker)·추종(follow_marker)·소이동(move)은 nav_core가 맡는다.
        self.nav2_enabled = bool(p('nav2_enabled', os.environ.get('NAV2_ENABLED', '0') in ('1', 'true', 'True')))
        self.nav2_frame = str(p('nav2_frame', 'map'))
        self._nav2_client = None
        self._nav2_handle = None
        self._nav2_state = 'idle'
        self._nav2_detail = ''
        self._nav2_goal_id = ''
        self._nav2_dist: float | None = None
        self._nav2_goal_xy: tuple[float, float] | None = None
        if self.nav2_enabled:
            try:
                from nav2_msgs.action import NavigateToPose
                from rclpy.action import ActionClient
                self._nav2_action_cls = NavigateToPose
                self._nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
                self.get_logger().info(f'Nav2 전역 주행 모드 활성(frame={self.nav2_frame})')
            except ImportError:
                self.get_logger().warning('nav2_msgs가 없어 nav_core 직선 주행으로 동작한다')
                self.nav2_enabled = False
        self.follow_cmd_at = -1e9
        self.track = None                       # 추종 노드가 따라가는 마커 (x, y, 시각)
        self.scan_pts: list = []
        self.scan_at = -1e9
        self.holding = False

        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel_align', 10)
        self.status_pub = self.create_publisher(String, '/nav/status', RELIABLE)
        self.create_subscription(String, '/nav/command', self.on_command, RELIABLE)
        self.create_subscription(Odometry, '/odometry/filtered', lambda m: self.on_odom(m, True), 10)
        self.create_subscription(Odometry, '/odom', lambda m: self.on_odom(m, False), 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Joy, '/joy', self.on_joy, 10)
        try:
            from aruco_opencv_msgs.msg import ArucoDetection
            self.create_subscription(ArucoDetection, '/aruco_detections', self.on_detections, 10)
        except ImportError:
            self.get_logger().error('aruco_opencv_msgs가 없다: 마커로 가기는 못 한다')
        self.create_subscription(TwistStamped, '/cmd_vel_follow', self.on_follow_cmd, 10)
        # 조이스틱(패드 LB·RB + 스틱)이 내는 지령. 이것이 나오는 동안은 이동을 쉰다
        self.create_subscription(TwistStamped, '/cmd_vel_teleop', self.on_teleop_cmd, 10)
        self.teleop_at = -1e9
        self.teleop_hold_s = float(p('teleop_hold_s', 1.0))      # 조이스틱을 놓고 이만큼 지나야 이어서 간다
        self.paused = False
        try:
            from mr_aruco_msgs.msg import MarkerTrack
            self.LOST = MarkerTrack.STATUS_LOST
            self.create_subscription(MarkerTrack, '/marker_track', self.on_track, 10)
        except ImportError:
            self.get_logger().warning('mr_aruco_msgs가 없다: 접근 감지가 따라가는 사람을 가려내지 못한다')
        try:
            from mr_care_msgs.msg import TripCommand
            self.create_subscription(TripCommand, '/phone/trip_command', self.on_trip, RELIABLE)
        except ImportError:
            self.get_logger().warning('mr_care_msgs가 없다: 앱·대시보드 멈춤을 못 듣는다(/nav/command cancel만)')

        self.filtered_at = -1e9
        self.was_active = False
        self.stop_down = False
        self.last_state = ''
        self.create_timer(1.0 / self.rate_hz, self.on_control)
        self.create_timer(0.2, self.publish_status)
        self.get_logger().info(f'errand_nav: {self.rate_hz:g} Hz, v_max {cfg.max_linear_mps:g} m/s, 라이다 정지 {cfg.stop_dist_m:g} m')

    # ------------------------------------------------------------ 입력
    def on_odom(self, m: Odometry, filtered: bool) -> None:
        now = time.monotonic()
        if filtered:
            self.filtered_at = now
        elif now - self.filtered_at < 1.0:
            return                              # EKF가 살아 있으면 바퀴만 쓴 값은 버린다
        pose = m.pose.pose
        self.nav.set_odom(pose.position.x, pose.position.y, yaw_of(pose.orientation))

    def on_detections(self, m) -> None:
        if m.header.frame_id != 'base_footprint':
            return                              # 빈 검출은 카메라 프레임으로 온다(marker_track_node 주석)
        now = time.monotonic()
        cx, cy = self.cam_xy
        for mk in m.markers:
            p, q = mk.pose.position, mk.pose.orientation
            # 마커 z축(면에서 나오는 쪽)의 수평 성분. 거의 누운 마커(바닥·천장)는 방향을 못 쓴다
            nx, ny = 2.0 * (q.x * q.z + q.w * q.y), 2.0 * (q.y * q.z - q.w * q.x)
            normal = math.atan2(ny, nx) if math.hypot(nx, ny) > 0.5 else None
            self.nav.see_marker(mk.marker_id, cx + self.pnp_scale * (p.x - cx), cy + self.pnp_scale * (p.y - cy), now, normal)

    def on_teleop_cmd(self, m: TwistStamped) -> None:
        self.teleop_at = time.monotonic()

    def on_follow_cmd(self, m: TwistStamped) -> None:
        self.follow_cmd_at = time.monotonic()

    def on_track(self, m) -> None:
        if m.status != self.LOST:
            self.track = (float(m.x), float(m.y), time.monotonic())

    def on_scan(self, m: LaserScan) -> None:
        lx, ly, lyaw = self.lidar
        pts = []
        bx0, bx1, by0, by1 = self.self_box
        a = m.angle_min + lyaw
        for r in m.ranges:
            if m.range_min < r < m.range_max and r > self.self_radius:
                x, y = lx + r * math.cos(a), ly + r * math.sin(a)
                if not (bx0 < x < bx1 and by0 < y < by1):
                    pts.append((x, y))
            a += m.angle_increment
        self.scan_pts, self.scan_at = pts, time.monotonic()
        self.nav.set_scan(pts, self.scan_at)

    def on_joy(self, m: Joy) -> None:
        down = 0 <= self.b_stop < len(m.buttons) and bool(m.buttons[self.b_stop])
        if down and not self.stop_down:
            self.cancel('pad')
        self.stop_down = down

    def on_trip(self, m) -> None:
        self.cancel('stop' if m.command == m.COMMAND_CANCEL else 'follow')

    def on_command(self, m: String) -> None:
        try:
            c = json.loads(m.data)
            cmd, goal_id = c['cmd'], str(c.get('id', ''))
        except (ValueError, KeyError, TypeError):
            return
        now = time.monotonic()
        if cmd == 'cancel':
            self.cancel('cancel')
        elif cmd == 'mark_home':
            slot = str(c.get('slot', 'home'))
            ok = self.nav.mark_home(slot)
            self.get_logger().info(f'돌아올 자리 기억({slot}) {self.nav.pose}' if ok else '돌아올 자리를 못 잡았다: 오도메트리 없음')
        elif cmd == 'goto_home':
            slot = str(c.get('slot', 'home'))
            if self.nav2_enabled and self._nav2_client is not None:
                pose = self.nav.home if slot == 'home' else self.nav.marks.get(slot, self.nav.home)
                if pose is None:
                    self.refuse(goal_id, 'no_home')
                else:
                    self._send_nav2_pose(goal_id, pose[0], pose[1], pose[2], c.get('frame_id'))
            elif not self.nav.goto_home(now, goal_id, slot):
                self.refuse(goal_id, 'no_home')
        elif cmd == 'goto_marker':
            self._cancel_nav2()
            self.nav.goto_marker(int(c['marker_id']), float(c.get('standoff_m', 0.6)), now, goal_id, bool(c.get('align')))
        elif cmd == 'move':
            self._cancel_nav2()
            self.nav.move(float(c.get('dist_m', 0.0)), math.radians(float(c.get('turn_deg', 0.0))), now, goal_id)
        elif cmd == 'follow_marker':
            self._cancel_nav2()
            self.nav.follow_marker(int(c['marker_id']), float(c.get('range_m', 1.0)), now, goal_id)
        elif cmd == 'goto_pose':
            yaw = c.get('yaw')
            if self.nav2_enabled and self._nav2_client is not None:
                self._send_nav2_pose(goal_id, float(c['x']), float(c['y']), None if yaw is None else float(yaw), c.get('frame_id'))
            else:
                self.nav.goto_pose(float(c['x']), float(c['y']), None if yaw is None else float(yaw), now, goal_id)
        else:
            return
        self.get_logger().info(f'명령 {cmd} {goal_id} → {self._nav2_state if self._nav2_active() else self.nav.state}')
        self.publish_status()

    def _nav2_active(self) -> bool:
        return self.nav2_enabled and bool(self._nav2_goal_id) and self._nav2_state != 'idle'

    def _cancel_nav2(self) -> None:
        if self._nav2_handle is not None:
            try:
                self._nav2_handle.cancel_goal_async()
            except Exception:
                pass
            self._nav2_handle = None
        self._nav2_goal_xy = None
        if self._nav2_state == 'driving':
            self._nav2_state, self._nav2_detail = 'cancelled', 'cancel'
        else:
            self._nav2_state, self._nav2_detail, self._nav2_goal_id = 'idle', '', ''

    def _send_nav2_pose(self, goal_id: str, x: float, y: float, yaw: float | None, frame_id: str | None = None) -> None:
        from action_msgs.msg import GoalStatus
        from geometry_msgs.msg import PoseStamped

        self.nav.cancel('nav2')
        self._cancel_nav2()
        if not self._nav2_client.wait_for_server(timeout_sec=1.0):
            # Nav2 서버가 꺼져 있으면 기존 nav_core 직선 주행으로 안전 폴백한다.
            self.get_logger().warning('Nav2 서버 미응답 → nav_core 직선 주행으로 폴백')
            self.nav.goto_pose(x, y, yaw, time.monotonic(), goal_id)
            return

        gyaw = float(yaw) if yaw is not None else (self.nav.pose[2] if self.nav.pose is not None else 0.0)
        goal = self._nav2_action_cls.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = str(frame_id or self.nav2_frame)
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(0.5 * gyaw)
        goal.pose.pose.orientation.w = math.cos(0.5 * gyaw)

        self._nav2_goal_id = goal_id
        self._nav2_state = 'driving'
        self._nav2_detail = 'nav2'
        self._nav2_dist = None
        self._nav2_goal_xy = (float(x), float(y))

        def on_fb(fb):
            self._nav2_dist = float(fb.feedback.distance_remaining)

        def on_res(fut):
            if self._nav2_goal_id != goal_id:
                return
            res = fut.result()
            self._nav2_handle = None
            self._nav2_goal_xy = None
            if res is None:
                self._nav2_state, self._nav2_detail = 'failed', 'no_result'
            elif res.status == GoalStatus.STATUS_SUCCEEDED:
                self._nav2_state, self._nav2_detail = 'arrived', 'nav2_succeeded'
            elif res.status == GoalStatus.STATUS_CANCELED:
                self._nav2_state, self._nav2_detail = 'cancelled', 'nav2_canceled'
            else:
                self._nav2_state, self._nav2_detail = 'failed', f'nav2_status_{res.status}'
            self.publish_status()

        def on_resp(fut):
            h = fut.result()
            if h is None or not h.accepted:
                if self._nav2_goal_id == goal_id:
                    self._nav2_state, self._nav2_detail = 'failed', 'rejected'
                    self.publish_status()
                return
            self._nav2_handle = h
            h.get_result_async().add_done_callback(on_res)

        self._nav2_client.send_goal_async(goal, feedback_callback=on_fb).add_done_callback(on_resp)

    def refuse(self, goal_id: str, why: str) -> None:
        from nav_core import Goal
        self.nav.goal = Goal('pose', goal_id)
        self.nav.state, self.nav.detail = 'failed', why

    def cancel(self, why: str) -> None:
        self._cancel_nav2()
        if self.nav.cancel(why):
            self.get_logger().info(f'이동 취소 ({why})')
            self.on_control()

    # ------------------------------------------------------------ 출력
    def on_control(self) -> None:
        now = time.monotonic()
        teleop = now - self.teleop_at < self.teleop_hold_s
        v, w = self.nav.step(now, paused=teleop)
        if self.nav.active and teleop:
            pass                                # 사람이 몬다. 아무것도 내지 않는다(mux에서도 teleop이 이기지만 섞지 않는다)
        elif self.nav.active:
            self.publish_cmd(v, w)
        elif self.was_active:
            self.publish_cmd(0.0, 0.0)         # 놓기 전에 0을 한 번. 그 뒤로는 발행하지 않는다
            self.get_logger().info(f'이동 끝: {self.nav.state} {self.nav.detail}')
        self.was_active = self.nav.active
        # 추종 노드가 모는 중의 접근 감지. align 자리(추종보다 위)에 0을 내는 동안 로봇은 서 있다
        hold = False
        # 한 번 세운 뒤에는 추종 지령이 끊겨도(임무가 끝났거나 mux 뒤에서 멈췄거나) 물러날 때까지 잡고 있는다
        following = now - self.follow_cmd_at < 1.0 or self.guard.hold
        if teleop != self.paused:
            self.paused = teleop
            if self.nav.active or self.was_active:
                self.get_logger().info('조이스틱 조종: 이동을 쉰다' if teleop else '조이스틱을 놓았다: 이어서 간다')
            self.publish_status()
        if self.guard_enabled and not teleop and not self.nav.active and following and now - self.scan_at < 0.6:
            exempt = [self.track[:2]] if self.track is not None and now - self.track[2] < 3.0 else []
            hold = self.guard.update(self.scan_pts, now, self.nav.pose, exempt)
        elif self.guard.hold and not self.nav.active:
            self.guard.reset()
        if hold:
            self.publish_cmd(0.0, 0.0)
        if hold != self.holding:
            self.holding = hold
            self.get_logger().info('가까이 다가온 사람이 있어 멈춘다' if hold else '물러났다. 다시 따라간다')
            self.publish_status()
        if self.nav.state != self.last_state:
            self.last_state = self.nav.state
            self.publish_status()

    def publish_cmd(self, v: float, w: float) -> None:
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()     # MCU는 낡은 stamp를 거절한다
        m.header.frame_id = 'base_footprint'
        m.twist.linear.x, m.twist.angular.z = float(v), float(w)
        self.cmd_pub.publish(m)

    def publish_status(self) -> None:
        status = self.nav.status(time.monotonic())
        if self.nav2_enabled:
            status['backend'] = 'nav2'
        if self._nav2_active():
            status['state'] = self._nav2_state
            status['detail'] = self._nav2_detail
            status['id'] = self._nav2_goal_id
            status['kind'] = 'pose'
            status['backend'] = 'nav2'
            if self._nav2_goal_xy is not None and self.nav.pose is not None:
                px, py, th = self.nav.pose
                dx, dy = self._nav2_goal_xy[0] - px, self._nav2_goal_xy[1] - py
                c, s = math.cos(th), math.sin(th)
                status['goal'] = [round(c * dx + s * dy, 3), round(-s * dx + c * dy, 3)]
                if self._nav2_dist is None:
                    status['dist_m'] = round(math.hypot(dx, dy), 2)
            if self._nav2_dist is not None:
                status['dist_m'] = round(self._nav2_dist, 2)
        if self.holding:
            status['guard'] = {'hold': True, 'near': None if self.guard.near is None else [round(v, 2) for v in self.guard.near]}
        status['guard']['enabled'] = self.guard_enabled
        self.status_pub.publish(String(data=json.dumps(status, separators=(',', ':'))))


def main():
    rclpy.init()
    node = ErrandNav()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node.nav.active:
                node.publish_cmd(0.0, 0.0)
        except Exception:  # noqa: BLE001 — 컨텍스트가 먼저 닫혔다
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
