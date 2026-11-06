#!/usr/bin/env python3
"""부스 패드 버튼 → 임무 명령. Pi에서 mr-booth@pad로 돈다.

    A                출발. 앱의 trip.start와 같은 /phone/trip_command라 앱에도 주행 중으로 뜬다
    B                멈춤. 오류로 멈춘 뒤에도 B를 누르면 대기로 돌아간다
    View+Menu 3 s    Pi 끄기. 멈춤을 먼저 보내고 systemctl poweroff (스택은 systemd가 정상 종료한다)
    LB/RB + 스틱     수동 조종. teleop_joy(follow 스택)가 따로 맡는다. 놓으면 0.3 s 뒤 추종으로 돌아간다

버튼 번호는 커널 joydev 순서다(블루투스 hid-microsoft, 2026-10-31 JSIOCGBTNMAP: 0 BTN_A, 1 BTN_B,
10 BTN_SELECT, 11 BTN_START). mr_bringup teleop_joy.yaml의 LB 6·RB 7과 같은 표다.

패드가 한동안 조용했다가 다시 오면 첫 메시지는 기준으로만 쓴다. 패드를 깨우거나 다시 붙일 때 누른 버튼이
출발로 읽히지 않게 하려는 것이다.

부스용 보정 둘도 여기서 한다. 둘 다 mobile-robot 설정을 고치지 않고 부스에서만 적용하려는 것이다.
- 마커 크기: 부스 마커는 8 cm로 인쇄됐다(aruco_tracker.yaml은 15 cm). /aruco_tracker의 marker_size를 계속 맞춘다.
  aruco_opencv는 실행 중 변경을 바로 반영한다(2026-10-31 실측, 거리가 정확히 8/15로 줄었다).
- 시계 점프: 인터넷 없이 부팅해 나중에 chrony가 시계를 옮기면 usb_cam stamp가 그만큼 과거로 남아 follow가
  움직이지 않는다(stack.sh follow 주석). 벽시계-단조시계 차이가 뛰면 주행 중이 아닐 때 mr-booth@follow를 다시 띄운다.
- MCU 지령 거부: MCU가 세션을 유지한 채 어느 순간부터 /cmd_vel을 전부 버리는 일이 있다(2026-08-30 두 번, 10-31
  22:46:59). 시계는 맞았고(MCU stamp와 Pi 차이 6 ms) 리셋만 풀었다. /cmd_vel이 흐르는데 /base/status cmd_age_s가
  계속 크면 그 상태로 보고, SWD로 MCU 계측을 읽어 로그에 남긴 뒤 st-flash reset 한다. 원인 경로(USART2 레거시
  링크 고장 → 모터 fault latch)는 10-31 23:06 펌웨어에서 micro-ROS 빌드에 한해 막았다(mobile-robot
  fix/mcu-legacy-link-latch). 이 감시는 혹시 남은 다른 경로를 위한 안전망이다.
"""
from __future__ import annotations

import struct
import subprocess
import tempfile
import threading
import time
import uuid
from collections import deque

import rclpy
from geometry_msgs.msg import TwistStamped
from mr_care_msgs.msg import RobotAppStatus, TripCommand
from mr_msgs.msg import BaseStatus
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Joy

# mr_mission node.py와 같은 QoS
RELIABLE = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE)
LATCHED = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
BUSY = (RobotAppStatus.MISSION_STARTING, RobotAppStatus.MISSION_ACTIVE, RobotAppStatus.MISSION_CANCELLING)
# joy_linux autorepeat 30 Hz. 이보다 오래 조용하면 패드가 끊긴 것으로 본다.
JOY_GAP_S = 1.0
# 벽시계가 단조시계보다 이만큼 뛰면 시계를 옮긴 것으로 본다. 평소 chrony 슬루는 초당 수 ms다.
CLOCK_JUMP_S = 2.0

# MCU 지령 거부 판정. 받는 중이면 cmd_age_s는 0.25 s 안이다(20 Hz 지령, 워치독 200 ms).
GUARD_FLOW_HZ = 10          # 최근 1 s 안에 /cmd_vel이 이만큼 있으면 지령이 흐르는 중
GUARD_AGE_S = 1.0
GUARD_STUCK_S = 2.0         # 위 상태가 이만큼 이어지면 리셋
GUARD_COOLDOWN_S = 20.0     # 리셋 뒤 재접속·시계 동기 동안 다시 판정하지 않는다
# SWD로 읽는 MCU 계측 주소. firmware/base_controller build/Debug(2026-10-31 23:06, text 143484 B)에서
# arm-none-eabi-gdb로 뽑았다(12:33 빌드와 같다). 펌웨어를 다시 구우면 주소가 바뀔 수 있으니 다시 뽑는다
# (틀리면 로그 숫자만 엉뚱하다).
SWD_READS = ((0x200049A8, 0x128), (0x200088A8, 0x94))
SWD_FIELDS = (
    ('rx_overflow', 0x200049AC), ('uart_err', 0x200049B0), ('rx_rearm_fail', 0x200049B8),
    ('tx_fail', 0x200049BC), ('tx_build_fail', 0x200049C0), ('ascii_rejected', 0x200049D0),
    ('time_sync_fail', 0x20004A6C), ('epoch_synced', 0x20004A70), ('cmd_vel', 0x20004A74),
    ('applied', 0x20004A78), ('rejected', 0x20004A7C), ('last_verdict', 0x20004A80),
    ('uart1_tx_fail', 0x20004ACC), ('fault_flags', 0x200088B0), ('stale_cmd', 0x20008928),
    ('urgent_stop', 0x20008924),
)


class PadTrip(Node):

    def __init__(self):
        super().__init__('booth_pad')
        p = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731
        self.b_start = p('button_start', 0)
        self.b_stop = p('button_stop', 1)
        self.b_off = (p('button_view', 10), p('button_menu', 11))
        self.off_hold_s = p('poweroff_hold_s', 3.0)
        self.from_place = p('from_place', 'place_home')
        self.to_place = p('to_place', 'place_senior_center')
        self.marker_size = p('marker_size', 0.0)      # 0이면 건드리지 않는다

        self.pub = self.create_publisher(TripCommand, '/phone/trip_command', RELIABLE)
        self.create_subscription(RobotAppStatus, '/robot/app_status', self.on_status, LATCHED)
        self.create_subscription(Joy, '/joy', self.on_joy, 10)
        self.create_timer(0.1, self.on_tick)

        self.mission: int | None = None
        self.trip_id = ''
        self.prev: list[int] | None = None
        self.joy_at = 0.0
        self.off_since: float | None = None
        self.start_at = 0.0

        self.clock_base = time.time() - time.monotonic()
        self.restart_follow = False
        self.create_timer(1.0, self.on_clock)
        self.cmd_times: deque[float] = deque()
        self.cmd_age: float | None = None
        self.status_at = 0.0
        self.stuck_since: float | None = None
        self.reset_at = -GUARD_COOLDOWN_S
        self.guard_busy = False
        self.resets = 0
        self.create_subscription(TwistStamped, '/cmd_vel', self.on_cmd_vel, qos_profile_sensor_data)
        self.create_subscription(BaseStatus, '/base/status', self.on_base_status, qos_profile_sensor_data)
        self.create_timer(0.2, self.on_guard)
        if self.marker_size > 0:
            self.aruco = AsyncParameterClient(self, '/aruco_tracker')
            self.aruco_busy = False
            self.create_timer(3.0, self.on_marker_size)
        self.get_logger().info(
            f'pad ready: A({self.b_start}) 출발 {self.from_place} -> {self.to_place}, B({self.b_stop}) 멈춤, '
            f'View+Menu{self.b_off} {self.off_hold_s:g} s 끄기, marker_size {self.marker_size:g} m')

    def on_status(self, m: RobotAppStatus):
        self.mission, self.trip_id = m.mission_state, m.trip_id

    def on_joy(self, m: Joy):
        now = time.monotonic()
        b = list(m.buttons)
        fresh = self.prev is not None and now - self.joy_at < JOY_GAP_S
        self.joy_at = now
        prev, self.prev = self.prev, b
        if not fresh:
            self.off_since = None
            return

        def down(i: int) -> bool:
            return 0 <= i < len(b) and bool(b[i])

        def pressed(i: int) -> bool:
            return down(i) and not (i < len(prev) and prev[i])

        if pressed(self.b_stop):
            self.cancel('pad B')
        elif pressed(self.b_start):
            self.start(now)
        if all(down(i) for i in self.b_off):
            if self.off_since is None:
                self.off_since = now
                self.get_logger().warning(f'View+Menu: {self.off_hold_s:g} s 쥐고 있으면 Pi를 끈다')
        elif self.off_since is not None:
            self.off_since = None
            self.get_logger().info('끄기 취소')

    def on_tick(self):
        now = time.monotonic()
        if self.off_since is None:
            return
        if now - self.joy_at > JOY_GAP_S:      # 쥔 채로 패드가 끊겼다
            self.off_since = None
        elif now - self.off_since >= self.off_hold_s:
            self.off_since = None
            self.poweroff()

    def on_marker_size(self):
        """검출기가 (다시) 뜨면 15 cm로 돌아가 있다. 3 s마다 보고 다르면 맞춘다."""
        if self.aruco_busy or not self.aruco.services_are_ready():
            return
        self.aruco_busy = True
        self.aruco.get_parameters(['marker_size']).add_done_callback(self._got_marker_size)

    def _got_marker_size(self, fut):
        try:
            now = fut.result().values[0].double_value
        except Exception:  # noqa: BLE001 — 검출기가 내려가는 중이면 다음 주기에 다시 본다
            self.aruco_busy = False
            return
        if abs(now - self.marker_size) < 1e-6:
            self.aruco_busy = False
            return

        def done(f):
            self.aruco_busy = False
            ok = f.exception() is None and all(r.successful for r in f.result().results)
            self.get_logger().info(f'aruco marker_size {now:g} -> {self.marker_size:g} m {"" if ok else "실패"}')
        self.aruco.set_parameters(
            [Parameter('marker_size', Parameter.Type.DOUBLE, self.marker_size)]).add_done_callback(done)

    def on_clock(self):
        base = time.time() - time.monotonic()
        if abs(base - self.clock_base) > CLOCK_JUMP_S:
            self.get_logger().warning(f'시계가 {base - self.clock_base:+.1f} s 옮겨졌다: 카메라 stamp를 다시 맞추려고 follow를 다시 띄운다')
            self.restart_follow = True
        self.clock_base = base
        # 주행 중에 베이스를 내리지 않는다. 멈춘 뒤에 다시 띄운다.
        if self.restart_follow and self.mission not in BUSY:
            self.restart_follow = False
            subprocess.Popen(['sudo', '-n', 'systemctl', 'restart', 'mr-booth@follow.service'])

    # ------------------------------------------------------------ MCU 지령 거부

    def on_cmd_vel(self, _m: TwistStamped):
        self.cmd_times.append(time.monotonic())

    def on_base_status(self, m: BaseStatus):
        self.cmd_age, self.status_at = float(m.cmd_age_s), time.monotonic()

    def on_guard(self):
        now = time.monotonic()
        while self.cmd_times and now - self.cmd_times[0] > 1.0:
            self.cmd_times.popleft()
        stuck = (len(self.cmd_times) >= GUARD_FLOW_HZ and now - self.status_at < 0.5
                 and self.cmd_age is not None and self.cmd_age > GUARD_AGE_S)
        if not stuck:
            self.stuck_since = None
            return
        if self.stuck_since is None:
            self.stuck_since = now
            return
        if self.guard_busy or now - self.stuck_since < GUARD_STUCK_S or now - self.reset_at < GUARD_COOLDOWN_S:
            return
        self.guard_busy = True
        msg = f'MCU가 지령을 버린다: /cmd_vel {len(self.cmd_times)}/s 인데 cmd_age {self.cmd_age:.1f} s'
        threading.Thread(target=self._reset_mcu, args=(msg,), daemon=True).start()

    def _reset_mcu(self, msg: str):
        log = self.get_logger()
        try:
            log.error(f'{msg}. SWD 계측 {swd_snapshot()}')
            r = subprocess.run(['st-flash', 'reset'], capture_output=True, text=True, timeout=20)
            self.resets += 1
            tail = (r.stdout + r.stderr).strip().splitlines()[-1:] or ['']
            log.error(f'st-flash reset rc={r.returncode} (이번 부팅 {self.resets}번째) {tail[0][:80]}')
        except Exception as e:  # noqa: BLE001
            log.error(f'MCU 리셋 실패: {e}')
        finally:
            self.reset_at = time.monotonic()
            self.stuck_since = None
            self.guard_busy = False

    def start(self, now: float):
        if self.mission in BUSY:
            self.get_logger().info(f'출발 무시: 이미 주행 중 (trip {self.trip_id}, state {self.mission})')
            return
        if now - self.start_at < 1.0:
            return
        self.start_at = now
        m = TripCommand()
        m.header.stamp = self.get_clock().now().to_msg()
        m.request_id = f'pad-{uuid.uuid4()}'
        # 폰 브리지는 UUID trip_id만 앱에 넘긴다(bridge.py is_uuid)
        m.trip_id = str(uuid.uuid4())
        m.command = TripCommand.COMMAND_START
        m.from_place_id, m.to_place_id = self.from_place, self.to_place
        self.pub.publish(m)
        self.get_logger().info(f'출발 trip {m.trip_id}')

    def cancel(self, why: str):
        m = TripCommand()
        m.header.stamp = self.get_clock().now().to_msg()
        m.request_id = f'pad-{uuid.uuid4()}'
        m.trip_id = self.trip_id
        m.command = TripCommand.COMMAND_CANCEL
        m.reason = 'user_request'
        self.pub.publish(m)
        self.get_logger().info(f'멈춤 ({why}) trip {self.trip_id or "-"}')

    def poweroff(self):
        self.cancel('poweroff')
        self.get_logger().warning('Pi를 끈다: systemctl poweroff')
        time.sleep(0.5)     # 멈춤이 나가게
        subprocess.Popen(['sudo', '-n', 'systemctl', 'poweroff'])


def swd_snapshot() -> str:
    """MCU를 멈추지 않고 RAM만 읽는다(st-flash read). 실패하면 사유 문자열."""
    vals = {}
    try:
        with tempfile.TemporaryDirectory() as d:
            for addr, size in SWD_READS:
                path = f'{d}/{addr:x}.bin'
                subprocess.run(['st-flash', 'read', path, hex(addr), hex(size)],
                               capture_output=True, timeout=10, check=True)
                buf = open(path, 'rb').read()
                for name, a in SWD_FIELDS:
                    if addr <= a < addr + len(buf) - 3:
                        vals[name] = struct.unpack_from('<I', buf, a - addr)[0]
    except Exception as e:  # noqa: BLE001
        return f'읽기 실패: {e}'[:120]
    vals['fault_flags'] = hex(vals.get('fault_flags', 0))
    return ' '.join(f'{k}={v}' for k, v in vals.items())


def main():
    rclpy.init()
    node = PadTrip()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
