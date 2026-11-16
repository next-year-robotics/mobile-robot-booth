"""버스가 죽었다가 살아나면 제어 스레드가 포트를 다시 열고 이어 간다."""
import time
from pathlib import Path

import yaml

from maknae_arm.driver import Arm
from maknae_arm.kin import Kinematics

ROOT = Path(__file__).resolve().parents[2]


def test_bus_recovers_after_usb_drop_without_jumping():
    cfg = yaml.safe_load((ROOT / 'config' / 'arm.yaml').read_text())
    arm = Arm(cfg, Kinematics(cfg['arm']['urdf'], cfg['arm']['base_link'], cfg['arm']['tip_link']), fake=True)
    try:
        bus = arm.bus
        state = {'dead': False, 'reopened': 0}
        read = bus.read

        def flaky_read():
            if state['dead']:
                raise OSError(5, 'Input/output error')
            return read()

        def reopen():
            state['reopened'] += 1
            if state['reopened'] < 2:
                raise OSError('아직 안 붙었다')
            state['dead'] = False
        write = bus.write

        def flaky_write(goal):
            if state['dead']:
                raise OSError(5, 'Input/output error')
            write(goal)
        bus.read, bus.write, bus.reopen = flaky_read, flaky_write, reopen
        before = dict(arm.present)
        state['dead'] = True
        arm._queue.extend((time.monotonic() + 0.3 + i * 0.02, {**before, 'shoulder_pan': 40.0}) for i in range(200))   # 끊긴 동안 밀린 궤적
        time.sleep(0.4)
        assert arm.fault
        deadline = time.monotonic() + 5
        while arm.fault and time.monotonic() < deadline:
            time.sleep(0.05)
        assert not arm.fault and state['reopened'] == 2
        time.sleep(0.2)
        assert abs(arm.present['shoulder_pan'] - before['shoulder_pan']) < 1e-6     # 밀린 목표는 버렸다
        assert not arm._queue
    finally:
        arm.close()
