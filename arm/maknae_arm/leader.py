"""리더 팔(SO-101 리더). 텔레옵으로 동작을 녹화할 때만 연다."""
from __future__ import annotations

import time
from pathlib import Path

from .driver import MOTORS


class Leader:

    def __init__(self, cfg: dict, arm, sync_speed: float = 45.0):
        from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig
        lc = cfg.get('leader', {})
        self.dev = SO101Leader(SO101LeaderConfig(
            port=str(lc.get('port', '/dev/soarm_leader')), id=lc.get('id', 'tp_leader'),
            calibration_dir=Path(lc.get('calibration_dir', '~/dev/so101-pickplace/calib/lerobot/leader')).expanduser(),
            use_degrees=True))
        self.dev.bus.connect()
        self.arm = arm
        self.sync_speed = sync_speed
        self.synced = False
        self.last = time.monotonic()

    def target(self) -> dict[str, float]:
        """제어 스레드가 주기마다 부른다. 처음에는 리더 자세까지 천천히 다가간다(팔이 튀지 않게)."""
        want = {m: float(v) for m, v in self.dev.bus.sync_read('Present_Position', num_retry=2).items()}
        now = time.monotonic()
        dt, self.last = min(0.1, now - self.last), now
        if self.synced:
            return {m: want[m] for m in MOTORS}
        cur = self.arm.target
        step = self.sync_speed * dt
        out, far = {}, False
        for m in MOTORS:
            d = want[m] - cur[m]
            if abs(d) > step:
                far = True
                d = step if d > 0 else -step
            out[m] = cur[m] + d
        self.synced = not far
        return out

    def close(self) -> None:
        try:
            self.dev.bus.disconnect()
        except Exception:
            pass
