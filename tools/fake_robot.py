#!/usr/bin/env python3
"""가짜 로봇. 실제 로봇과 같은 토픽·타입으로 발행해 브리지와 화면을 리허설한다.

    ROS_DOMAIN_ID=77 python3 tools/fake_robot.py

실제 로봇과 섞이지 않게 반드시 다른 도메인에서 띄운다(scripts/run.sh --sim이 77을 쓴다).

장면: 방 안에서 작업자(마커 0)가 8자 경로를 걷고 로봇이 1.1 m 거리로 따라간다. 벽에 고정 마커 10이 있고,
config/booth.yaml nav.places의 마커가 책상과 벽에 붙어 있다. 라이다(/scan)는 벽·책상·작업자 다리를 2D로 쏜 값이다.

명령에도 반응한다. /phone/trip_command 출발·취소로 따라가기를 켜고 끄고(--idle이면 꺼진 채 시작),
/cmd_vel_align(robot/errand_nav.py)이 오면 그쪽이 바퀴를 잡는다. twist_mux의 우선순위와 같다.
바퀴 오도메트리는 실제 바닥처럼 회전을 12.5 % 적게 잰다. 자이로는 참값이다.
--intruder를 주면 30 s마다 6 s 동안 누군가 로봇 왼쪽 옆 20 cm까지 다가온다(따라가는 중의 접근 감지를 시험한다).
영상은 바닥 격자와 마커를 원근 투영해 그린다. 검출은 영상보다 한 프레임 늦게 발행해 동기화 경로를 시험한다.
"""
from __future__ import annotations

import math
import random
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from aruco_opencv_msgs.msg import ArucoDetection, MarkerPose
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TwistStamped
from mr_aruco_msgs.msg import MarkerTrack
from mr_care_msgs.msg import PhoneLinkStatus, RobotAppStatus, TripCommand
from mr_msgs.msg import BaseStatus
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Imu, JointState, LaserScan

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'bridge'))
from booth_bridge.overlay import CameraModel  # noqa: E402
from booth_bridge.state import rpy_matrix  # noqa: E402

BE = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
REL = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
LATCHED = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=1,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)

ODOM_TURN_SCALE = 0.875     # 바퀴가 자이로보다 회전을 12.5 % 적게 잰다 (기록 2f절)
R_EFF, SEP_EFF = 0.04979, 0.30912
MARKER_SIZE = 0.15
TARGET_RANGE = 1.1
MUX_TIMEOUT = 0.3

# 방(odom 좌표). 벽은 선분, 가구는 상자다. 작업자의 8자(x -0.2~3.4, y ±1.1)와 겹치지 않게 둔다.
ROOM = (-2.0, 5.6, -3.2, 3.2)                          # x0, x1, y0, y1
BOXES = [(3.6, 4.3, 1.0, 2.2),                          # 책상
         (3.25, 3.45, -0.9, -0.3),                      # 고정 마커 10이 붙은 기둥
         (-1.6, -0.9, -2.9, -2.2)]                      # 상자 더미
# --nav2 시뮬레이션에서 우회 경로(A*)가 뚜렷하게 드러나도록 거실–현관 사이 파티션을 추가한다.
NAV2_EXTRA_BOXES = [(0.55, 2.35, -1.45, -1.05)]         # 거실–현관 사이 파티션
# 장소 마커를 붙일 자리: (위치, 바깥 법선). nav.places 순서대로 쓴다.
PLACE_SPOTS = [((3.6, 1.6, 0.41), (-1.0, 0.0)),         # 책상 앞면
               ((0.6, -3.2, 0.41), (0.0, 1.0)),         # 남쪽 벽
               ((-2.0, 1.2, 0.41), (1.0, 0.0))]         # 서쪽 벽
LIDAR_RAYS = 720
LIDAR_MAX = 12.0


def scene_segments(extra_boxes: list[tuple[float, float, float, float]] = ()) -> np.ndarray:
    """(N, 4) 선분 x1 y1 x2 y2."""
    x0, x1, y0, y1 = ROOM
    segs = [(x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)]
    for bx0, bx1, by0, by1 in (*BOXES, *extra_boxes):
        segs += [(bx0, by0, bx1, by0), (bx1, by0, bx1, by1), (bx1, by1, bx0, by1), (bx0, by1, bx0, by0)]
    return np.array(segs)


def raycast(origin: np.ndarray, angles: np.ndarray, segs: np.ndarray, circles: list[tuple[float, float, float]]) -> np.ndarray:
    """origin에서 angles 방향으로 쏜 광선이 처음 닿는 거리. 안 닿으면 inf."""
    d = np.stack([np.cos(angles), np.sin(angles)], axis=1)              # (R, 2)
    a, b = segs[:, :2], segs[:, 2:]
    e = b - a                                                            # (S, 2)
    ao = a - origin                                                      # (S, 2)
    den = d[:, None, 0] * e[None, :, 1] - d[:, None, 1] * e[None, :, 0]  # (R, S)
    den = np.where(np.abs(den) < 1e-12, np.nan, den)
    t = (ao[None, :, 0] * e[None, :, 1] - ao[None, :, 1] * e[None, :, 0]) / den
    u = (ao[None, :, 0] * d[:, None, 1] - ao[None, :, 1] * d[:, None, 0]) / den
    hit = np.where((t > 0) & (u >= 0) & (u <= 1), t, np.inf)
    best = np.nanmin(np.where(np.isnan(hit), np.inf, hit), axis=1)
    for cx, cy, r in circles:
        oc = origin - np.array([cx, cy])
        bq = d @ oc
        disc = bq * bq - (oc @ oc - r * r)
        tc = np.where(disc >= 0, -bq - np.sqrt(np.maximum(disc, 0)), np.inf)
        best = np.minimum(best, np.where(tc > 0, tc, np.inf))
    return best


def stamp_of(t: float) -> Time:
    sec = int(t)
    return Time(sec=sec, nanosec=int((t - sec) * 1e9))


def quat_from_matrix(R: np.ndarray) -> list[float]:
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return [x, y, z, w]


def marker_frame(normal_xy: np.ndarray) -> np.ndarray:
    """바깥 법선이 수평 normal_xy인 세운 마커의 회전행렬(열 = 마커 x, y, z)."""
    z = np.array([normal_xy[0], normal_xy[1], 0.0])
    z /= np.linalg.norm(z)
    y = np.array([0.0, 0.0, 1.0])
    x = np.cross(y, z)
    return np.stack([x, y, z], axis=1)


class FakeRobot(Node):

    def __init__(self):
        super().__init__('fake_robot')
        cfg = yaml.safe_load((ROOT / 'config' / 'booth.yaml').read_text(encoding='utf-8'))
        self.cam = CameraModel(cfg['camera'])
        self.cam_cfg = cfg['camera']
        self.R_imu = rpy_matrix(*[math.radians(a) for a in cfg['imu']['rpy_deg']])
        self.r_imu = np.array(cfg['imu']['xyz_m'])
        self.scale = cfg['aruco']['pnp_translation_scale']
        d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        nav = cfg.get('nav', {})
        self.places = [(int(pl['marker_id']), np.array(pos), np.array(normal))
                       for pl, (pos, normal) in zip((nav.get('places') or {}).values(), PLACE_SPOTS)]
        self.marker_img = {mid: self._marker_image(d, mid) for mid in (0, 10, *(p[0] for p in self.places))}
        lidar = cfg.get('lidar', {})
        self.lidar_xy = np.array(lidar.get('xyz_m', [0.018, -0.004, 0.25])[:2], dtype=float)
        self.lidar_yaw = math.radians(lidar.get('rpy_deg', [0, 0, 177.5])[2])
        self.nav2_mode = '--nav2' in sys.argv
        self.publish_tf = self.nav2_mode or ('--tf' in sys.argv)
        self.segs = scene_segments(NAV2_EXTRA_BOXES if self.nav2_mode else ())

        self.pub_js = self.create_publisher(JointState, '/joint_states', BE)
        self.pub_status = self.create_publisher(BaseStatus, '/base/status', BE)
        self.pub_imu = self.create_publisher(Imu, '/imu/data_raw', BE)
        self.pub_odom = self.create_publisher(Odometry, '/odom', REL)
        self.pub_track = self.create_publisher(MarkerTrack, '/marker_track', REL)
        self.pub_det = self.create_publisher(ArucoDetection, '/aruco_detections', REL)
        self.pub_img = self.create_publisher(CompressedImage, '/camera/image_raw/compressed', BE)
        self.pub_info = self.create_publisher(CameraInfo, '/camera/camera_info', BE)
        self.pub_cmd = self.create_publisher(TwistStamped, '/cmd_vel', REL)
        self.pub_follow = self.create_publisher(TwistStamped, '/cmd_vel_follow', REL)
        self.pub_link = self.create_publisher(PhoneLinkStatus, '/phone/link_status', LATCHED)
        self.pub_app = self.create_publisher(RobotAppStatus, '/robot/app_status', LATCHED)
        self.pub_scan = self.create_publisher(LaserScan, '/scan', BE)
        self.pub_filtered = self.create_publisher(Odometry, '/odometry/filtered', REL)
        self.pub_tf = None
        if self.publish_tf:
            from geometry_msgs.msg import TransformStamped
            from tf2_msgs.msg import TFMessage
            self._TransformStamped, self._TFMessage = TransformStamped, TFMessage
            self.pub_tf = self.create_publisher(TFMessage, '/tf', REL)
        self.create_subscription(TripCommand, '/phone/trip_command', self.on_trip, REL)
        self.create_subscription(TwistStamped, '/cmd_vel_align', self.on_nav_cmd, REL)
        self.create_subscription(TwistStamped, '/cmd_vel_nav', self.on_nav2_cmd, REL)
        self.following = '--idle' not in sys.argv
        self.trip_id = 'demo-trip' if self.following else ''
        self.nav_cmd = (-1e9, 0.0, 0.0)
        self.nav2_cmd = (-1e9, 0.0, 0.0)

        # 참값 상태
        self.t0 = time.time()
        self.x = self.y = self.th = 0.0          # 참 자세
        self.ox = self.oy = self.oth = 0.0       # /odom 자세 (회전 과소)
        self.v = self.w = 0.0
        self.a_fwd = 0.0
        self.wl = self.wr = 0.0                  # 바퀴 누적각
        self.cmd = (0.0, 0.0)
        self.loop_count = 0
        self.battery = 13.20                    # 배터리 단자. 토픽은 역극성 다이오드만큼 낮다
        self.lock_since = None
        self.last_seen = None
        self.pending_det: ArucoDetection | None = None
        self.track_hold = None
        self.dock = (np.array([3.2, -0.6, 0.35]), np.array([-1.0, 0.0]))   # 벽 마커 위치·법선(odom)

        self.dt = 0.01
        self.k = 0
        self.create_timer(self.dt, self.step)
        # 영상 그리기와 JPEG 압축이 100 Hz 동역학 타이머를 밀지 않게 따로 돈다(cv2는 GIL을 놓는다).
        threading.Thread(target=self._camera_loop, daemon=True).start()
        self.create_timer(1.0, self.slow_tick)
        self.create_timer(0.1, self.publish_scan)
        self.publish_link(True)

    @staticmethod
    def _marker_image(d, mid: int) -> np.ndarray:
        inner = cv2.aruco.drawMarker(d, mid, 180)
        img = np.full((240, 240), 255, np.uint8)
        img[30:210, 30:210] = inner
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    # ------------------------------------------------------------ 장면
    def person(self, t: float) -> tuple[np.ndarray, np.ndarray]:
        """작업자 위치(odom)와 진행 방향. 8자를 걷다가 가끔 멈춘다."""
        s = t * 0.06
        s -= 0.6 * max(0.0, math.sin(t * 0.21)) ** 8      # 멈칫하는 구간
        p = np.array([1.6 + 1.8 * math.sin(s), 1.1 * math.sin(2 * s), 0.50])   # 카메라가 41 cm 높이라 허리 아래
        dp = np.array([1.8 * math.cos(s), 2.2 * math.cos(2 * s)])
        return p, dp / (np.linalg.norm(dp) + 1e-9)

    def to_base(self, p_odom: np.ndarray) -> np.ndarray:
        """참 자세 기준 odom → base_footprint (마커는 참값으로 보인다)."""
        c, s = math.cos(self.th), math.sin(self.th)
        dx, dy = p_odom[0] - self.x, p_odom[1] - self.y
        return np.array([c * dx + s * dy, -s * dx + c * dy, p_odom[2]])

    def rot_to_base(self, v_odom_xy: np.ndarray) -> np.ndarray:
        c, s = math.cos(self.th), math.sin(self.th)
        return np.array([c * v_odom_xy[0] + s * v_odom_xy[1], -s * v_odom_xy[0] + c * v_odom_xy[1]])

    def visible_markers(self, t: float) -> list[tuple[int, np.ndarray, np.ndarray]]:
        out = []
        pp, heading = self.person(t)
        # 조끼 마커: 작업자가 로봇 쪽으로 몸을 돌려 주는 장면. 진행 방향 따라 ±25°쯤 흔들린다.
        p_base = self.to_base(pp)
        to_robot = -p_base[:2] / (np.linalg.norm(p_base[:2]) + 1e-9)
        lateral = float(np.cross(np.append(to_robot, 0), np.append(self.rot_to_base(heading), 0))[2])
        a = 0.45 * lateral
        normal = np.array([to_robot[0] * math.cos(a) - to_robot[1] * math.sin(a),
                           to_robot[0] * math.sin(a) + to_robot[1] * math.cos(a)])
        out.append((0, p_base, marker_frame(normal)))
        dp, dn = self.dock
        out.append((10, self.to_base(dp), marker_frame(self.rot_to_base(dn))))
        for mid, pos, normal in self.places:
            out.append((mid, self.to_base(pos), marker_frame(self.rot_to_base(normal))))
        seen = []
        for mid, p, R in out:
            h = MARKER_SIZE / 2
            corners = p + np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]]) @ R.T
            uv = self.cam.project(corners)
            to_cam = self.cam.origin - p
            facing = float(R[:, 2] @ to_cam) > 0.15 * np.linalg.norm(to_cam)
            if uv is None or not facing or np.linalg.norm(to_cam) > 4.0:
                continue
            if (uv[:, 0] < 4).any() or (uv[:, 0] > 1276).any() or (uv[:, 1] < 4).any() or (uv[:, 1] > 716).any():
                continue
            seen.append((mid, p, R))
        return seen

    # ------------------------------------------------------------ 100 Hz 동역학
    def step(self) -> None:
        now = time.time()
        t = now - self.t0
        self.k += 1
        seen = {mid: p for mid, p, _ in self.visible_markers(t)}

        # 추종 제어(follow.yaml 게인·한계)를 20 Hz로
        now_mono = time.monotonic()
        align_live = now_mono - self.nav_cmd[0] < MUX_TIMEOUT
        nav2_live = now_mono - self.nav2_cmd[0] < MUX_TIMEOUT
        if align_live or nav2_live:
            # twist_mux 우선순위와 동일: align(60) > nav(55) > follow(50)
            self.cmd = self.nav_cmd[1:] if align_live else self.nav2_cmd[1:]
            if self.k % 5 == 0:
                self.publish_cmd_out(now)
                if self.following:
                    # 실제 추종 노드는 mux에서 밀려도 지령을 계속 낸다(접근 감지가 이것으로 따라가는 중인지 안다)
                    self.pub_follow.publish(TwistStamped())
        elif not self.following:
            self.cmd = (0.0, 0.0)
        elif self.k % 5 == 0:
            if 0 in seen:
                p = seen[0]
                rng, brg = math.hypot(p[0], p[1]), math.atan2(p[1], p[0])
                v = 0.0 if abs(rng - TARGET_RANGE) < 0.12 else 0.6 * (rng - TARGET_RANGE)
                v = max(-0.1, min(0.25, v)) if abs(brg) < 0.44 else 0.0
                w = 0.0 if abs(brg) < 0.07 else max(-0.7, min(0.7, 1.2 * brg))
                self.cmd = (v, w)
                self.last_seen = t
                self.last_fix = (rng, brg)
            elif self.last_seen is not None and t - self.last_seen < 0.4:
                pass
            else:
                # 놓치면 마지막 방위 쪽으로 돌고, 너무 가까웠으면 조금 물러난다
                rng, brg = getattr(self, 'last_fix', (2.0, 1.0))
                self.cmd = (-0.08 if rng < 1.0 else 0.0, 0.5 if brg >= 0 else -0.5)
            self.pub_follow.publish(self.publish_cmd_out(now))

        # 가속 한계(0.4 m/s², 2 rad/s²)와 1차 지연
        v_prev = self.v
        dv = max(-0.4 * self.dt, min(0.4 * self.dt, self.cmd[0] - self.v))
        dw = max(-2.0 * self.dt, min(2.0 * self.dt, self.cmd[1] - self.w))
        self.v += dv
        self.w += dw
        self.a_fwd = (self.v - v_prev) / self.dt
        self.th += self.w * self.dt
        self.x += self.v * math.cos(self.th) * self.dt
        self.y += self.v * math.sin(self.th) * self.dt
        w_odom = self.w * ODOM_TURN_SCALE
        self.oth += w_odom * self.dt
        self.ox += self.v * math.cos(self.oth) * self.dt
        self.oy += self.v * math.sin(self.oth) * self.dt
        vl = (self.v - w_odom * SEP_EFF / 2) / R_EFF
        vr = (self.v + w_odom * SEP_EFF / 2) / R_EFF
        self.wl += vl * self.dt
        self.wr += vr * self.dt
        self.loop_count += 1

        self.publish_imu(now)
        if self.k % 2 == 0:
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg() if self.publish_tf else stamp_of(now)
            js.name = ['left_wheel_joint', 'right_wheel_joint']
            js.position = [self.wl, self.wr]
            js.velocity = [vl + random.gauss(0, 0.03), vr + random.gauss(0, 0.03)]
            self.pub_js.publish(js)
            self.publish_odom(now, w_odom)
        if self.k % 10 == 0:
            self.publish_status(now, vl, vr)

    def publish_cmd_out(self, now: float) -> TwistStamped:
        msg = TwistStamped()
        msg.header.stamp = stamp_of(now)
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x, msg.twist.angular.z = self.cmd
        self.pub_cmd.publish(msg)
        return msg

    def on_nav_cmd(self, m: TwistStamped) -> None:
        self.nav_cmd = (time.monotonic(), m.twist.linear.x, m.twist.angular.z)

    def on_nav2_cmd(self, m: TwistStamped) -> None:
        self.nav2_cmd = (time.monotonic(), m.twist.linear.x, m.twist.angular.z)

    def on_trip(self, m: TripCommand) -> None:
        self.following = m.command == TripCommand.COMMAND_START
        self.trip_id = m.trip_id if self.following else ''
        self.last_seen = None
        self.publish_app(time.time())

    def publish_scan(self) -> None:
        now = time.time()
        pp, heading = self.person(now - self.t0)
        side = np.array([-heading[1], heading[0]]) * 0.09
        legs = [(pp[0] + side[0], pp[1] + side[1], 0.06), (pp[0] - side[0], pp[1] - side[1], 0.06)]
        c, s = math.cos(self.th), math.sin(self.th)
        if '--intruder' in sys.argv and (now - self.t0) % 30.0 > 24.0:
            # 로봇 왼쪽 옆(차체에서 20 cm)에 선 사람의 두 다리
            for bx in (-0.02, 0.16):
                legs.append((self.x + c * bx - s * 0.45, self.y + s * bx + c * 0.45, 0.06))
        origin = np.array([self.x + c * self.lidar_xy[0] - s * self.lidar_xy[1],
                           self.y + s * self.lidar_xy[0] + c * self.lidar_xy[1]])
        inc = math.tau / LIDAR_RAYS
        angles = -math.pi + inc * np.arange(LIDAR_RAYS)
        r = raycast(origin, angles + self.th + self.lidar_yaw, self.segs, legs)
        r = r + np.random.normal(0, 0.008, LIDAR_RAYS)
        r[(r > LIDAR_MAX) | (np.random.random(LIDAR_RAYS) < 0.01)] = math.inf
        m = LaserScan()
        m.header.stamp = self.get_clock().now().to_msg() if self.publish_tf else stamp_of(now)
        m.header.frame_id = 'laser_link'
        m.angle_min, m.angle_max, m.angle_increment = -math.pi, math.pi - inc, inc
        m.scan_time, m.time_increment = 0.1, 0.1 / LIDAR_RAYS
        m.range_min, m.range_max = 0.05, LIDAR_MAX
        m.ranges = r.astype(np.float32).tolist()
        self.pub_scan.publish(m)

    def publish_imu(self, now: float) -> None:
        w = np.array([0.0, 0.0, self.w])
        a = np.array([self.a_fwd, self.v * self.w, 9.81])
        a = a + np.cross(w, np.cross(w, self.r_imu))
        gyro = self.R_imu.T @ w + np.random.normal(0, 0.003, 3)
        acc = self.R_imu.T @ a + np.random.normal(0, 0.04, 3)
        m = Imu()
        m.header.stamp = stamp_of(now)
        m.header.frame_id = 'imu_link'
        m.orientation_covariance[0] = -1.0
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = gyro
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = acc
        self.pub_imu.publish(m)

    def publish_odom(self, now: float, w_odom: float) -> None:
        stamp = self.get_clock().now().to_msg() if self.publish_tf else stamp_of(now)
        m = Odometry()
        m.header.stamp = stamp
        m.header.frame_id = 'odom'
        m.child_frame_id = 'base_footprint'
        m.pose.pose.position.x, m.pose.pose.position.y = self.ox, self.oy
        m.pose.pose.orientation.z = math.sin(self.oth / 2)
        m.pose.pose.orientation.w = math.cos(self.oth / 2)
        m.twist.twist.linear.x = self.v
        m.twist.twist.angular.z = w_odom
        self.pub_odom.publish(m)
        # EKF(바퀴 + 자이로) 자리. 자이로가 참값이라 참 자세를 그대로 낸다
        f = Odometry()
        f.header, f.child_frame_id = m.header, m.child_frame_id
        f.pose.pose.position.x, f.pose.pose.position.y = self.x, self.y
        f.pose.pose.orientation.z = math.sin(self.th / 2)
        f.pose.pose.orientation.w = math.cos(self.th / 2)
        f.twist.twist.linear.x, f.twist.twist.angular.z = self.v, self.w
        self.pub_filtered.publish(f)
        if self.pub_tf is not None:
            tf = self._TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = 'odom'
            tf.child_frame_id = 'base_footprint'
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation.z = math.sin(self.th / 2)
            tf.transform.rotation.w = math.cos(self.th / 2)
            self.pub_tf.publish(self._TFMessage(transforms=[tf]))

    def publish_status(self, now: float, vl: float, vr: float) -> None:
        m = BaseStatus()
        m.header.stamp = stamp_of(now)
        m.agent_state = BaseStatus.AGENT_CONNECTED
        m.cmd_watchdog_ok = True
        m.cmd_age_s = 0.02 + random.random() * 0.03
        m.duty_left = max(-1.0, min(1.0, vl / 10.5 + random.gauss(0, 0.01)))
        m.duty_right = max(-1.0, min(1.0, vr / 10.5 + random.gauss(0, 0.01)))
        m.saturated_left = abs(m.duty_left) > 0.98
        m.saturated_right = abs(m.duty_right) > 0.98
        m.loop_count = self.loop_count
        m.loop_overruns = 0
        self.battery -= 0.00004
        sag = 0.25 * (abs(m.duty_left) + abs(m.duty_right)) / 2
        m.supply_voltage = self.battery - 0.30 - sag + random.gauss(0, 0.005)
        self.pub_status.publish(m)

    # ------------------------------------------------------------ 30 Hz 카메라
    def _camera_loop(self) -> None:
        nxt = time.time()
        while rclpy.ok():
            self.camera_tick()
            nxt += 1 / 30
            time.sleep(max(0.0, nxt - time.time()))

    def camera_tick(self) -> None:
        now = time.time()
        t = now - self.t0
        if self.pending_det is not None:
            self.pub_det.publish(self.pending_det)      # 직전 영상의 검출을 한 프레임 늦게
        seen = self.visible_markers(t)
        stamp = stamp_of(now)

        img = self.render(seen)
        ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 75])
        cm = CompressedImage()
        cm.header.stamp = stamp
        cm.header.frame_id = 'camera_optical_frame'
        cm.format = 'rgb8; jpeg compressed bgr8'
        cm.data = jpg.tobytes()
        self.pub_img.publish(cm)

        det = ArucoDetection()
        det.header.stamp = stamp
        det.header.frame_id = 'base_footprint'
        for mid, p, R in seen:
            raw = self.cam.origin + (p - self.cam.origin) / self.scale
            mp = MarkerPose()
            mp.marker_id = mid
            mp.pose.position.x, mp.pose.position.y, mp.pose.position.z = raw
            q = quat_from_matrix(R)
            mp.pose.orientation.x, mp.pose.orientation.y, mp.pose.orientation.z, mp.pose.orientation.w = q
            det.markers.append(mp)
        self.pending_det = det
        self.publish_track(now, {mid: (p, R) for mid, p, R in seen})

    def publish_track(self, now: float, seen: dict) -> None:
        m = MarkerTrack()
        m.header.stamp = stamp_of(now)
        m.header.frame_id = 'base_footprint'
        m.marker_id = 0
        if 0 in seen:
            p, R = seen[0]
            self.track_hold = (now, p, R)
            self.lock_since = self.lock_since or now
            m.status = MarkerTrack.STATUS_TRACKING
            m.locked = now - self.lock_since > 0.3
            m.hold_duration = 0.0
        elif self.track_hold is not None and now - self.track_hold[0] < 1.0:
            _, p, R = self.track_hold
            m.status = MarkerTrack.STATUS_HOLDING
            m.locked = True
            m.hold_duration = now - self.track_hold[0]
        else:
            self.lock_since = None
            m.status = MarkerTrack.STATUS_LOST
            self.pub_track.publish(m)
            return
        m.x, m.y = float(p[0]), float(p[1])
        m.yaw = math.atan2(R[1, 2], R[0, 2])
        m.yaw_valid = True
        m.rejected_jump = random.random() < 0.004
        self.pub_track.publish(m)

    def render(self, seen) -> np.ndarray:
        """바닥 격자와 마커를 실제 카메라 모델로 그린다."""
        img = np.zeros((720, 1280, 3), np.uint8)
        img[:] = (36, 30, 26)
        grad = np.linspace(0, 1, 720)[:, None]
        img = (img * (0.55 + 0.45 * grad[..., None])).astype(np.uint8)
        c, s = math.cos(self.th), math.sin(self.th)
        gx0, gy0 = math.floor(self.x) - 6, math.floor(self.y) - 6
        for i in range(13):
            for horizontal in (True, False):
                pts = []
                for j in np.linspace(0, 12, 49):
                    ox, oy = (gx0 + j, gy0 + i) if horizontal else (gx0 + i, gy0 + j)
                    dx, dy = ox - self.x, oy - self.y
                    pts.append([c * dx + s * dy, -s * dx + c * dy, 0.0])
                pts = np.array(pts)
                # 시야 밖 점은 왜곡 다항식이 접혀 엉뚱한 곳에 찍힌다
                pts = pts[(pts[:, 0] > 0.5) & (np.abs(pts[:, 1]) < 1.0 * pts[:, 0])]
                if len(pts) < 2:
                    continue
                uv = self.cam.project(pts)
                if uv is not None:
                    cv2.polylines(img, [uv.astype(np.int32)], False, (70, 62, 54), 1, cv2.LINE_AA)
        for mid, p, R in seen:
            h = MARKER_SIZE / 2 * (240 / 180)            # 흰 여백까지
            corners = p + np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]]) @ R.T
            uv = self.cam.project(corners)
            if mid == 0:
                vest = p + np.array([[-0.2, 0.25, -0.01], [0.2, 0.25, -0.01], [0.2, -0.35, -0.01],
                                     [-0.2, -0.35, -0.01]]) @ R.T
                vuv = self.cam.project(vest)
                if vuv is not None:
                    cv2.fillConvexPoly(img, vuv.astype(np.int32), (40, 70, 160), cv2.LINE_AA)
            src = np.float32([[0, 0], [240, 0], [240, 240], [0, 240]])
            H = cv2.getPerspectiveTransform(src, uv.astype(np.float32))
            warped = cv2.warpPerspective(self.marker_img[mid], H, (1280, 720))
            mask = cv2.warpPerspective(np.full((240, 240), 255, np.uint8), H, (1280, 720))
            img[mask > 0] = warped[mask > 0]
        img = cv2.GaussianBlur(img, (3, 3), 0)
        cv2.putText(img, 'SIMULATION', (24, 690), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (90, 90, 90), 2, cv2.LINE_AA)
        return img

    # ------------------------------------------------------------ 1 Hz
    def slow_tick(self) -> None:
        now = time.time()
        info = CameraInfo()
        info.header.stamp = stamp_of(now)
        info.header.frame_id = 'camera_optical_frame'
        info.width, info.height = self.cam_cfg['width'], self.cam_cfg['height']
        info.distortion_model = 'plumb_bob'
        info.k = [float(v) for v in self.cam_cfg['K']]
        info.d = [float(v) for v in self.cam_cfg['D']]
        self.pub_info.publish(info)
        if int(now) % 2 == 0:
            self.publish_app(now)

    def publish_app(self, now: float) -> None:
        m = RobotAppStatus()
        m.header.stamp = stamp_of(now)
        m.robot_id = 'mr-001'
        if self.following:
            m.mission_state = RobotAppStatus.MISSION_ACTIVE
            tracking = self.last_seen is not None and (now - self.t0) - self.last_seen < 1.0
            m.tracking_state = RobotAppStatus.TRACKING_FOLLOWING if tracking else RobotAppStatus.TRACKING_SEARCHING
        else:
            m.mission_state = RobotAppStatus.MISSION_IDLE
            m.tracking_state = RobotAppStatus.TRACKING_IDLE
        m.trip_id = self.trip_id
        m.battery_percent = -1
        self.pub_app.publish(m)

    def publish_link(self, connected: bool) -> None:
        m = PhoneLinkStatus()
        m.header.stamp = stamp_of(time.time())
        m.connected = connected
        m.app_instance_id = 'galaxy-s26' if connected else ''
        m.session_id = 'sim'
        self.pub_link.publish(m)


def main() -> None:
    rclpy.init()
    node = FakeRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
