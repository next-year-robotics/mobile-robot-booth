"""팔 데몬.

    python -m maknae_arm                       # 실제 팔
    python -m maknae_arm --fake                # 팔·카메라 없이 가짜 작업대로 흐름을 끝까지 (화면 개발·리허설용)

LeRobot이 깔린 파이썬으로 띄운다(scripts/arm.sh가 conda 환경 lerobot을 쓴다).
"""
from __future__ import annotations

import argparse
import logging
import os
import signal
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

from .cameras import Camera, TopView
from .driver import Arm, ArmError
from .kin import ARM_JOINTS, Kinematics
from .server import serve
from .skills import Errand, Refused
from .vision import Argon

ROOT = Path(__file__).resolve().parents[2]
log = logging.getLogger('arm')


class App:

    def __init__(self, cfg: dict, fake: bool):
        self.cfg, self.fake = cfg, fake
        self.kin = Kinematics(cfg['arm']['urdf'], cfg['arm']['base_link'], cfg['arm']['tip_link'],
                              cfg.get('joint_offset_deg'))
        self.arm = Arm(cfg, self.kin, fake=fake)
        self.cams: dict[str, Camera] = {}
        self.view = None
        if not fake or os.environ.get('ARM_FAKE_CAMERAS') == '1':
            for name, c in cfg['cameras'].items():
                self.cams[name] = Camera(name, c)
        if 'top' in cfg['cameras']:
            self.view = TopView(cfg['cameras']['top'])
        argon = Argon(cfg['vision'])
        if fake and not self.cams and self.view is not None:
            from .sim import Scene, SimArgon, SimCamera          # 팔·카메라 없이 흐름을 끝까지 돌려 보는 가짜 장면
            scene = Scene(cfg, self.arm, self.view)
            self.cams['top'] = SimCamera(scene)
            argon = SimArgon(scene)
        self.errand = Errand(cfg, self.arm, self.cams.get('top'), self.view, argon)
        self.errand.cams = self.cams
        self.started = time.time()

    # ---------------------------------------------------------------- 상태
    def state(self) -> dict:
        arm = self.arm
        j = arm.joints()
        tip = arm.tip()
        return {
            'ok': not arm.fault, 'fake': self.fake, 'fault': arm.fault, 'uptime_s': round(time.time() - self.started),
            'joints': {m: round(v, 1) for m, v in j.items()},
            'target': {m: round(v, 1) for m, v in arm.target.items()},
            'tip': [round(float(v), 3) for v in tip],
            'torque': arm.torque, 'moving': arm.moving,
            'volt': round(arm.aux.get('volt', 0.0), 1),
            'temp_c': max(arm.aux.get('temp_c', {}).values(), default=None),
            'gripper_ma': round(arm.gripper_current()),
            'table_z': self.errand.table_z(),
            'cams': {n: {'age_s': None if c.age == float('inf') else round(c.age, 2)} for n, c in self.cams.items()},
            'poses': list(arm.poses),
            'workspace': self.workspace_px(),
            'errand': self.errand.snapshot(),
        }

    def workspace_px(self) -> list | None:
        """집을 수 있는 범위를 탑뷰 픽셀 사각형으로 (화면 표시용)"""
        if self.view is None:
            return None
        w, z = self.cfg['workspace'], self.errand.table_z()
        try:
            return [[round(c) for c in self.view.project([x, y, z])] for x, y in (
                (w['x_m'][0], w['y_m'][0]), (w['x_m'][1], w['y_m'][0]), (w['x_m'][1], w['y_m'][1]), (w['x_m'][0], w['y_m'][1]))]
        except cv2.error:
            return None

    def frame(self, name: str) -> bytes | None:
        cam = self.cams.get(name)
        return None if cam is None else cam.jpeg()

    # ---------------------------------------------------------------- 명령
    def command(self, msg: dict) -> dict | None:
        cmd = msg.get('cmd')
        e, arm = self.errand, self.arm
        if cmd == 'stop':
            e.stop()
            return None
        if cmd == 'fetch':
            return e.fetch(str(msg.get('name', '')))
        if cmd == 'give':
            return e.give()
        if cmd == 'throw':
            return e.throw()
        if cmd == 'home':
            e.home()
            return None
        if cmd == 'motion':
            return e.motion(str(msg.get('name', '')))
        if cmd == 'intro_gesture':
            return e.intro_gesture(str(msg.get('name', '')))
        if cmd == 'dance':
            try:
                start = float(msg['start_epoch'])
            except (KeyError, TypeError, ValueError):
                raise Refused('음악 시작 시각이 없어요.') from None
            return e.dance(start)
        if cmd == 'dance_stop':
            return e.dance_stop()
        if cmd == 'scan':
            e.rescan()
            return None
        if cmd == 'teleop':
            return e.teleop(bool(msg.get('on')))
        if cmd == 'record_start':
            return e.record_start(str(msg.get('name', '')), str(msg.get('source', 'leader')))
        if cmd == 'record_stop':
            return e.record_stop(str(msg.get('end', 'rest')))
        if cmd == 'motion_delete':
            return e.motion_delete(str(msg.get('name', '')))
        # 아래는 운영자용 직접 조작. 심부름 중에는 받지 않는다.
        if e.state not in ('idle', 'holding', 'error'):
            raise Refused('심부름 중에는 직접 조작할 수 없어요.')
        if cmd == 'pose':
            name = str(msg.get('name', ''))
            e._run(lambda: arm.pose(name, arm.slow if msg.get('slow') else None))
            return None
        if cmd == 'pad_gripper':
            if not e.busy.acquire(blocking=False):
                raise Refused('지금 다른 동작을 하는 중이에요.')
            try:
                arm.set_gripper_target(float(msg['value']))
            finally:
                e.busy.release()
            return None
        if cmd == 'gripper':
            value = float(msg.get('value', 0))
            e._run(lambda: arm.gripper(value))
            return None
        if cmd == 'joints':
            goal = {m: float(v) for m, v in dict(msg.get('goal', {})).items() if m in ARM_JOINTS + ['gripper']}
            e._run(lambda: arm.move(goal, float(msg.get('speed', arm.slow))))
            return None
        if cmd == 'reach':
            pos = [float(v) for v in msg['pos']]
            deg = e._solve(pos, float(np.radians(msg.get('roll_deg', 0.0))))
            if deg is None:
                raise Refused('그 자리에는 닿지 않아요.')
            e._run(lambda: arm.move(deg, arm.slow))
            return {'joints': deg}
        if cmd == 'torque':
            arm.set_torque(bool(msg.get('on')))
            return None
        if cmd == 'pixel':
            if self.view is None:
                raise Refused('탑뷰 설정이 없어요.')
            p = self.view.to_plane(float(msg['u']), float(msg['v']), e.table_z() + float(msg.get('h', 0.0)))
            return {'x': round(float(p[0]), 4), 'y': round(float(p[1]), 4)}
        raise Refused(f'모르는 명령: {cmd}')


def merge(base: dict, over: dict) -> dict:
    """over의 값을 base에 덮어쓴다. dict는 안쪽까지 합친다."""
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            merge(base[k], v)
        else:
            base[k] = v
    return base


def main() -> None:
    ap = argparse.ArgumentParser(prog='maknae_arm')
    ap.add_argument('--config', default=str(ROOT / 'config' / 'arm.yaml'))
    ap.add_argument('--fake', action='store_true', help='팔과 카메라 없이 흐름만 돌린다')
    ap.add_argument('--port', type=int)
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(name)s %(message)s', datefmt='%H:%M:%S')
    cfg = yaml.safe_load(Path(args.config).read_text(encoding='utf-8'))
    local = Path(args.config).with_name('arm.local.yaml')     # 실측 보정값(카메라 자세, 관절 영점, 작업대 높이)
    if local.exists():
        merge(cfg, yaml.safe_load(local.read_text(encoding='utf-8')) or {})
    app = App(cfg, args.fake)
    httpd = serve(app, cfg['server']['host'], args.port or int(cfg['server']['port']))
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    stop.wait()
    log.info('종료: 팔은 지금 자세에서 토크를 유지한다')
    app.errand.stop()
    app.arm.close()
    os._exit(0)                                        # 카메라·HTTP 스레드를 기다리지 않고 바로 끝낸다


if __name__ == '__main__':
    main()
