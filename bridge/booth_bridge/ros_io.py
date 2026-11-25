"""ROS 2 구독. 메시지를 숫자로 풀어 BoothState·FrameSync에 넘긴다.

노트북에서 Pi 토픽을 Wi-Fi로 받는다. 구독은 전부 BEST_EFFORT다. Pi 쪽 발행자가 RELIABLE이어도
매칭되고, 그러면 Pi가 노트북용 재전송 버퍼를 잡지 않는다. 래치 토픽 둘만 TRANSIENT_LOCAL로 받는다.
발행은 운영 버튼의 /phone/trip_command와 작업 순서기의 /nav/command 둘이다(RELIABLE).

사용자 메시지(mr_msgs, mr_aruco_msgs, mr_care_msgs)는 scripts/build_msgs.sh가 만든 오버레이에서 온다.
없으면 그 구독만 빼고 나머지는 띄운다.
"""
from __future__ import annotations

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy)

from .overlay import CameraModel, Frame, FrameSync, marker_overlay
from .scan import ScanShaper
from .state import BoothState

SENSOR = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
IMAGE = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=2)
LATCHED = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
RELIABLE = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)


def _stamp_s(header) -> float:
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def _stamp_ns(header) -> int:
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class BoothNode(Node):

    def __init__(self, cfg: dict, state: BoothState, cam: CameraModel, sync: FrameSync):
        super().__init__('booth_dashboard')
        self.state, self.cam, self.sync = state, cam, sync
        self.marker_size = float(cfg['aruco']['marker_size_m'])
        self.pnp_scale = float(cfg['aruco']['pnp_translation_scale'])
        topics = cfg['ros']['topics']
        self.missing: list[str] = []
        self.shaper = ScanShaper(cfg.get('lidar', {}))
        self.on_scan_packet = None        # (머리 dict, 본문 bytes)를 받을 함수. 서버가 꽂는다

        from geometry_msgs.msg import TwistStamped
        from nav_msgs.msg import Odometry, Path as NavPath
        from sensor_msgs.msg import CameraInfo, CompressedImage, Imu, JointState, Joy, LaserScan
        from std_msgs.msg import String
        import json
        self._json, self._String = json, String
        self._map_to_odom = (0.0, 0.0, 0.0)
        if cfg.get('lidar', {}).get('enabled', True):
            self.create_subscription(LaserScan, topics.get('scan', '/scan'), self.on_scan, SENSOR)
        # 이동 노드(robot/errand_nav.py). 명령은 놓치면 안 되니 RELIABLE, 상태는 5 Hz로 다시 오니 BEST_EFFORT로 받는다
        self.nav_pub = self.create_publisher(String, topics.get('nav_command', '/nav/command'), RELIABLE)
        self.create_subscription(String, topics.get('nav_status', '/nav/status'), self.on_nav_status, SENSOR)
        self.create_subscription(NavPath, topics.get('nav_plan', '/plan'), self.on_plan, SENSOR)
        try:
            from tf2_msgs.msg import TFMessage
            self.create_subscription(TFMessage, '/tf', self.on_tf, SENSOR)
        except ImportError:
            pass

        self.create_subscription(JointState, topics['joint_states'], self.on_joint_states, SENSOR)
        self.create_subscription(Imu, topics['imu'], self.on_imu, SENSOR)
        self.create_subscription(Odometry, topics['odom'], self.on_odom, SENSOR)
        self.create_subscription(Odometry, topics.get('odom_filtered', '/odometry/filtered'), self.on_filtered, SENSOR)
        self.create_subscription(CompressedImage, topics['image'], self.on_image, IMAGE)
        self.create_subscription(CameraInfo, topics['camera_info'], self.on_camera_info, SENSOR)
        self.create_subscription(TwistStamped, topics['cmd_vel'],
                                 lambda m: self.on_cmd('out', m), SENSOR)
        for name, topic in topics['cmd_inputs'].items():
            self.create_subscription(TwistStamped, topic,
                                     lambda m, n=name: self.on_cmd(n, m), SENSOR)
        # joy_linux가 버튼과 십자키 축을 30 Hz로 다시 낸다.
        self.create_subscription(Joy, topics['joy'], lambda m: self.state.on_joy(time.monotonic(), m.buttons, m.axes), SENSOR)

        self._optional('mr_msgs.msg', 'BaseStatus', topics['base_status'], self.on_base_status, SENSOR)
        self._optional('mr_aruco_msgs.msg', 'MarkerTrack', topics['marker_track'],
                       self.on_marker_track, SENSOR)
        self._optional('aruco_opencv_msgs.msg', 'ArucoDetection', topics['detections'],
                       self.on_detections, SENSOR)
        self._optional('mr_care_msgs.msg', 'PhoneLinkStatus', topics['phone_link'],
                       self.on_phone_link, LATCHED)
        self._optional('mr_care_msgs.msg', 'RobotAppStatus', topics['app_status'],
                       self.on_app_status, LATCHED)
        # 운영 버튼 출발·멈춤. 앱 브리지와 같은 토픽·QoS(mr_care_msgs TripCommand.msg)
        self.trip_pub = None
        self._sos_pubs = None
        try:
            from mr_care_msgs.msg import TripCommand
            self._TripCommand = TripCommand
            self.trip_pub = self.create_publisher(TripCommand, topics['trip_command'], RELIABLE)
        except ImportError:
            pass
        if self.missing:
            self.get_logger().warn(
                f'메시지 타입이 없어 빠진 구독: {", ".join(self.missing)} — scripts/build_msgs.sh 확인')

    def send_sos(self, *, raise_: bool, sos_id: str) -> bool:
        """음성 "도와줘"·대시보드 SOS. 앱의 SOS 버튼과 같은 토픽(/phone/sos_raise, /phone/sos_cancel)으로 낸다."""
        try:
            from mr_care_msgs.msg import SosClear, SosRequest
        except ImportError:
            return False
        if self._sos_pubs is None:
            self._sos_pubs = (self.create_publisher(SosRequest, '/phone/sos_raise', RELIABLE),
                              self.create_publisher(SosClear, '/phone/sos_cancel', RELIABLE))
        if raise_:
            m = SosRequest()
            m.reason = SosRequest.REASON_MANUAL
        else:
            m = SosClear()
            m.resolution = SosClear.RESOLUTION_CANCELLED
        m.header.stamp = self.get_clock().now().to_msg()
        m.sos_id = sos_id
        self._sos_pubs[0 if raise_ else 1].publish(m)
        return True

    def send_nav(self, cmd: dict) -> bool:
        self.nav_pub.publish(self._String(data=self._json.dumps(cmd)))
        return True

    def on_nav_status(self, m) -> None:
        try:
            status = self._json.loads(m.data)
        except ValueError:
            return
        if isinstance(status, dict):
            self.state.on_nav(time.monotonic(), status)

    def on_tf(self, m) -> None:
        for tf in m.transforms:
            p = tf.header.frame_id.lstrip('/')
            c = tf.child_frame_id.lstrip('/')
            if p == 'map' and c == 'odom':
                t = tf.transform.translation
                self._map_to_odom = (float(t.x), float(t.y), _yaw(tf.transform.rotation))

    def on_plan(self, m) -> None:
        frame = (m.header.frame_id or '').lstrip('/')
        pts = []
        if frame == 'map':
            tx, ty, tyaw = self._map_to_odom
            c, s = math.cos(tyaw), math.sin(tyaw)
            for ps in m.poses:
                dx = float(ps.pose.position.x) - tx
                dy = float(ps.pose.position.y) - ty
                pts.append((c * dx + s * dy, -s * dx + c * dy))
            out_frame = 'odom'
        else:
            for ps in m.poses:
                pts.append((float(ps.pose.position.x), float(ps.pose.position.y)))
            out_frame = frame or 'odom'
        self.state.on_plan(time.monotonic(), pts, out_frame)

    def on_scan(self, m) -> None:
        shaped = self.shaper.shape(m.ranges, m.angle_min, m.angle_increment, m.range_min, m.range_max)
        self.state.on_scan(time.monotonic(), len(shaped['points']), shaped['nearest'])
        if self.on_scan_packet is not None:
            pose, odom = self.state.scan_poses(time.monotonic())
            self.on_scan_packet(*self.shaper.pack(shaped, pose, odom))

    def can_trip(self) -> bool:
        return self.trip_pub is not None

    def send_trip(self, *, start: bool, trip_id: str, from_place: str = '', to_place: str = '') -> None:
        T = self._TripCommand
        m = T()
        m.header.stamp = self.get_clock().now().to_msg()
        m.request_id = f'booth-{trip_id or "cancel"}-{time.monotonic_ns()}'
        m.trip_id = trip_id
        if start:
            m.command = T.COMMAND_START
            m.from_place_id, m.to_place_id = from_place, to_place
        else:
            m.command = T.COMMAND_CANCEL
            m.reason = 'user_request'
        self.trip_pub.publish(m)

    def _optional(self, module: str, name: str, topic: str, cb, qos) -> None:
        try:
            mod = __import__(module, fromlist=[name])
            self.create_subscription(getattr(mod, name), topic, cb, qos)
        except (ImportError, AttributeError):
            self.missing.append(f'{topic} ({module.split(".")[0]}/{name})')

    # ------------------------------------------------------------------
    def on_joint_states(self, m) -> None:
        self.state.on_joint_states(time.monotonic(), list(m.position), list(m.velocity))

    def on_base_status(self, m) -> None:
        self.state.on_base_status(time.monotonic(), {
            'agent_state': m.agent_state,
            'cmd_watchdog_ok': m.cmd_watchdog_ok,
            'cmd_age_s': m.cmd_age_s,
            'duty_left': m.duty_left, 'duty_right': m.duty_right,
            'saturated_left': m.saturated_left, 'saturated_right': m.saturated_right,
            'loop_count': m.loop_count, 'loop_overruns': m.loop_overruns,
            'supply_voltage': getattr(m, 'supply_voltage', float('nan')),
        })

    def on_imu(self, m) -> None:
        g, a = m.angular_velocity, m.linear_acceleration
        self.state.on_imu(time.monotonic(), _stamp_s(m.header), [g.x, g.y, g.z], [a.x, a.y, a.z])

    def on_odom(self, m) -> None:
        p, t = m.pose.pose, m.twist.twist
        self.state.on_odom(time.monotonic(), p.position.x, p.position.y, _yaw(p.orientation),
                           t.linear.x, t.angular.z)

    def on_filtered(self, m) -> None:
        p = m.pose.pose
        self.state.on_filtered(time.monotonic(), p.position.x, p.position.y, _yaw(p.orientation))

    def on_marker_track(self, m) -> None:
        self.state.on_marker_track(time.monotonic(), {
            'status': m.status, 'locked': m.locked, 'marker_id': m.marker_id,
            'x': m.x, 'y': m.y, 'yaw': m.yaw, 'yaw_valid': m.yaw_valid,
            'hold_duration': m.hold_duration, 'rejected_jump': m.rejected_jump,
        })

    def on_detections(self, m) -> None:
        now = time.monotonic()
        raw = []
        for mk in m.markers:
            p, q = mk.pose.position, mk.pose.orientation
            raw.append((mk.marker_id, [p.x, p.y, p.z], [q.x, q.y, q.z, q.w]))
        self.state.on_detections(now, raw)
        self.sync.add_overlay(_stamp_ns(m.header), now,
                              marker_overlay(self.cam, raw, self.marker_size, self.pnp_scale))

    def on_image(self, m) -> None:
        now = time.monotonic()
        self.state.on_image(now)
        mime = 'image/png' if 'png' in m.format else 'image/jpeg'
        self.sync.add_frame(Frame(_stamp_ns(m.header), now, bytes(m.data), mime))

    def on_camera_info(self, m) -> None:
        if not self.cam.from_camera_info and len(m.k) == 9 and m.k[0] > 0:
            self.cam.set_intrinsics(list(m.k), list(m.d), m.width, m.height)
            self.cam.from_camera_info = True
            self.get_logger().info(f'camera_info 수신: {m.width}x{m.height} fx={m.k[0]:.1f}')

    def on_cmd(self, source: str, m) -> None:
        self.state.on_cmd(time.monotonic(), source, m.twist.linear.x, m.twist.angular.z)

    def on_phone_link(self, m) -> None:
        self.state.on_phone_link(time.monotonic(), m.connected, m.app_instance_id)

    def on_app_status(self, m) -> None:
        self.state.on_app_status(time.monotonic(), {
            'mission_state': m.mission_state, 'tracking_state': m.tracking_state,
            'trip_id': m.trip_id, 'last_error': m.last_error,
            'battery_percent': m.battery_percent,
        })


class JpegQualityTuner(threading.Thread):
    """Pi 카메라 노드의 압축 품질을 낮춘다. 카메라가 다시 뜨면 기본값(95)으로 돌아가므로 주기적으로 본다.

    파라미터 이름은 image_transport 버전과 네임스페이스에 따라 달라서(`image_raw.compressed.jpeg_quality`,
    `.out.jpeg_quality` 등) 목록에서 이름 끝이 jpeg_quality인 것을 모두 맞춘다.
    """

    def __init__(self, node: Node, target: str, quality: int):
        super().__init__(name='jpeg-tuner', daemon=True)
        from rclpy.parameter_client import AsyncParameterClient
        self.node, self.quality = node, int(quality)
        self.client = AsyncParameterClient(node, target)
        self.target = target

    @staticmethod
    def _wait(fut, timeout: float = 3.0):
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < timeout:
            time.sleep(0.02)
        return fut.result() if fut.done() else None

    def run(self) -> None:
        from rclpy.parameter import Parameter
        while rclpy.ok():
            checked = False
            try:
                if self.client.services_are_ready():
                    names = self._wait(self.client.list_parameters())
                    names = [n for n in (names.result.names if names else []) if n.endswith('jpeg_quality')]
                    # usb_cam은 `.image_raw.jpeg_quality`도 목록에 내놓지만 설정하면 Invalid parameter 경고만 낸다
                    names = [n for n in names if 'compressed' in n] or names
                    if names:
                        checked = True
                        got = self._wait(self.client.get_parameters(names))
                        values = [v.integer_value for v in got.values] if got else [None] * len(names)
                        todo = [n for n, v in zip(names, values) if v != self.quality]
                        if todo:
                            self._wait(self.client.set_parameters(
                                [Parameter(n, Parameter.Type.INTEGER, self.quality) for n in todo]))
                            self.node.get_logger().info(
                                f'{self.target} JPEG 품질 {values} → {self.quality} ({", ".join(todo)})')
            except Exception as e:  # noqa: BLE001 — 카메라 노드가 없으면 다음 주기에 다시 본다
                self.node.get_logger().debug(f'jpeg tuner: {e}')
            # 카메라 노드를 찾기 전에는 자주, 찾은 뒤에는 재시작 감시만 한다
            time.sleep(10.0 if checked else 1.0)


def start_ros(cfg: dict, state: BoothState, cam: CameraModel, sync: FrameSync,
              tune_camera: bool = True) -> tuple[Node, threading.Thread]:
    rclpy.init()
    node = BoothNode(cfg, state, cam, sync)
    thread = threading.Thread(target=_spin, args=(node,), name='ros', daemon=True)
    thread.start()
    q = cfg['camera'].get('jpeg_quality')
    if tune_camera and q is not None:
        JpegQualityTuner(node, cfg['camera']['node'], q).start()
    return node, thread


def _spin(node: Node) -> None:
    try:
        rclpy.spin(node)
    except Exception:  # noqa: BLE001 — 종료 중 컨텍스트가 먼저 닫히는 경우
        pass


def stop_ros(node: Node) -> None:
    try:
        node.destroy_node()
        rclpy.try_shutdown()
    except Exception:  # noqa: BLE001
        pass
