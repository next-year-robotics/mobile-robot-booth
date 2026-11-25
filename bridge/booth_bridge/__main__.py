"""부스 대시보드 브리지.

    python3 -m booth_bridge --config config/booth.yaml          # 실제 로봇
    python3 -m booth_bridge --sim                                # tools/fake_robot.py와 같이, 화면에 SIM 표시
    python3 -m booth_bridge --no-ros                             # 화면 배치만 볼 때

ROS 환경(source /opt/ros/jazzy + ros_ws/install)은 scripts/run.sh가 잡는다.
"""
from __future__ import annotations

import argparse
import asyncio
import logging
import signal
import threading
from pathlib import Path

import yaml

from .ops import Ops
from .overlay import CameraModel, FrameSync
from .server import BoothServer
from .state import BoothState
from .watchers import CloudWatcher, PiWatcher

ROOT = Path(__file__).resolve().parents[2]


def main() -> None:
    ap = argparse.ArgumentParser(prog='booth_bridge')
    ap.add_argument('--config', default=str(ROOT / 'config' / 'booth.yaml'))
    ap.add_argument('--static', default=str(ROOT / 'web' / 'dist'))
    ap.add_argument('--port', type=int)
    ap.add_argument('--host')
    ap.add_argument('--sim', action='store_true', help='가짜 로봇 데이터임을 화면에 표시한다')
    ap.add_argument('--no-ros', action='store_true')
    ap.add_argument('--no-cloud', action='store_true')
    ap.add_argument('--no-pi', action='store_true')
    args = ap.parse_args()

    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(name)s %(message)s', datefmt='%H:%M:%S')
    # 정적 파일 요청마다 'connection failed (200 OK)'를 찍는다. 정상 동작이라 감춘다.
    logging.getLogger('websockets').setLevel(logging.WARNING)
    cfg = yaml.safe_load(Path(args.config).read_text(encoding='utf-8'))
    if args.port:
        cfg['server']['port'] = args.port
    if args.host:
        cfg['server']['host'] = args.host

    state = BoothState(cfg, sim=args.sim)
    cam = CameraModel(cfg['camera'])
    sync = FrameSync(float(cfg['aruco']['sync_hold_s']))

    # rclpy.init이 잡는 INT·TERM은 ROS를 닫은 뒤 그 전에 있던 핸들러로 넘어간다. 바탕화면 바로가기처럼
    # 뒤로 띄우면 INT가 무시된 채 시작해서 웹 서버만 살아남으니, 둘 다 KeyboardInterrupt가 되게 먼저 깔아 둔다.
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    node = None
    if not args.no_ros:
        from .ros_io import start_ros
        node, _ = start_ros(cfg, state, cam, sync, tune_camera=not args.sim)
    # 시뮬레이션 중에 진짜 클라우드·Pi 상태를 섞으면 화면이 서로 모순된다.
    if cfg['cloud']['enabled'] and not args.no_cloud and not args.sim:
        CloudWatcher(cfg['cloud'], state).start()
    if cfg['pi']['enabled'] and not args.no_pi and not args.sim:
        PiWatcher(cfg['pi'], state, cfg['stack']['units']).start()

    arm = None
    if cfg.get('arm', {}).get('enabled'):
        from .arm_link import ArmLink
        arm = ArmLink(cfg['arm'], state, sim=args.sim)
        arm.start()
        state.pad_stop_hook = lambda: threading.Thread(target=arm.command, args=({'cmd': 'stop'},), daemon=True).start()

    ops = Ops(cfg, state, node)
    # 말로 시킨 이동·심부름의 순서기. ROS가 없으면(--no-ros) 요청을 받아도 "이동 노드가 꺼져 있다"로 거절한다.
    from .layout import Layout
    from .tasks import TaskRunner, Tasks
    nav_cfg = cfg.get('nav', {})
    # 시뮬레이션의 가짜 방 배치가 실제 부스 배치 파일을 덮지 않게 따로 둔다
    layout_file = Path(args.config).with_name('layout.sim.yaml' if args.sim else 'layout.local.yaml')
    layout = Layout(layout_file, list((nav_cfg.get('places') or {}).keys()))
    state.pose_hook = layout.on_pose
    tasks = Tasks(
        cfg, nav_state=state.nav_status, arm_state=state.arm_state, mission_state=state.mission,
        send_nav=(node.send_nav if node is not None else lambda cmd: False),
        arm_command=(arm.command if arm is not None else lambda msg: {'ok': False, 'error': '팔이 연결돼 있지 않아요.'}),
        stop_trip=lambda: ops.handle('stop'), publish=state.set_tasks, markers_alive=state.markers_alive, layout=layout)
    TaskRunner(tasks).start()
    stop_arm = state.pad_stop_hook

    def pad_stop() -> None:
        # 패드 B: 바퀴는 Pi가 세운다(errand_nav도 /joy를 듣는다). 여기서는 팔과 순서기를 같이 끝낸다.
        tasks.cancel('패드')
        if stop_arm is not None:
            stop_arm()
    state.pad_stop_hook = pad_stop

    def reset_steps() -> None:
        result = tasks.request({'cmd': 'reset'})
        if not result.get('ok'):
            log = logging.getLogger('booth.reset')
            log.warning('단계 초기화: %s', result.get('error'))
    state.pad_reset_hook = lambda: threading.Thread(target=reset_steps, name='pad-reset', daemon=True).start()

    server = BoothServer(cfg, state, cam, sync, Path(args.static), ops, arm, tasks)
    if node is not None:
        node.on_scan_packet = server.put_scan
    try:
        asyncio.run(server.run())
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            from .ros_io import stop_ros
            stop_ros(node)


if __name__ == '__main__':
    main()
