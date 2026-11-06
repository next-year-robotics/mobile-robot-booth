"""대시보드 운영 버튼. 화면의 버튼·단축키가 /ws로 보낸 명령을 실행한다.

- 출발·멈춤: 노트북 rclpy가 /phone/trip_command를 직접 낸다. 앱·Pi 패드(robot/pad_trip.py)와 같은 경로라
  임무 노드는 누가 보냈는지 구분하지 않고, 앱에도 주행 중으로 뜬다.
- 스택 재시작·MCU 리셋: ssh rpi5. 오래 걸리니 스레드에서 돌리고 결과를 화면 알림으로 돌려준다.

결과는 BoothState.set_ops로 넘긴다. 화면은 seq가 바뀔 때마다 알림을 띄운다.
"""
from __future__ import annotations

import logging
import subprocess
import threading
import time
import uuid

from .state import BoothState
from .watchers import ssh_cmd

log = logging.getLogger('booth.ops')

# 명령 → (화면 이름, Pi에서 돌릴 것, 제한 시간 s)
SSH_OPS = {
    'restart': ('스택 재시작', 'sudo -n systemctl restart mr-booth.target', 40.0),
    'mcu_reset': ('MCU 리셋', 'st-flash reset 2>&1 | tail -1', 20.0),
}


class Ops:

    def __init__(self, cfg: dict, state: BoothState, node=None):
        self.state, self.node = state, node
        self.host = cfg['pi']['ssh_host']
        self.trip_from = cfg['stack']['trip_from']
        self.trip_to = cfg['stack']['trip_to']
        self.busy: str | None = None
        self.seq = 0
        self.lock = threading.Lock()

    def handle(self, cmd: str) -> None:
        if cmd in ('start', 'stop'):
            self._trip(cmd)
        elif cmd in SSH_OPS:
            with self.lock:
                if self.busy:
                    self._report(f'{SSH_OPS[self.busy][0]} 진행 중', 'warn')
                    return
                self.busy = cmd
            self._report(f'{SSH_OPS[cmd][0]} 중…', 'info')
            threading.Thread(target=self._ssh, args=(cmd,), name=f'ops-{cmd}', daemon=True).start()

    def _trip(self, cmd: str) -> None:
        if self.node is None or not self.node.can_trip():
            self._report('ROS 메시지(mr_care_msgs)가 없어 보낼 수 없음', 'warn')
            return
        if cmd == 'start':
            trip_id = new_trip_id()
            self.node.send_trip(start=True, trip_id=trip_id, from_place=self.trip_from, to_place=self.trip_to)
            self._report('출발 보냄', 'info')
        else:
            trip_id = self.state.current_trip_id() or ''
            self.node.send_trip(start=False, trip_id=trip_id)
            self._report('멈춤 보냄', 'info')
        log.info('%s trip %s', '출발' if cmd == 'start' else '멈춤', trip_id or '-')

    def _ssh(self, cmd: str) -> None:
        label, remote, timeout = SSH_OPS[cmd]
        t0 = time.monotonic()
        try:
            r = subprocess.run(ssh_cmd(self.host, remote), capture_output=True, text=True, timeout=timeout)
            out = (r.stdout + r.stderr).strip().splitlines()
            tail = out[-1][:80] if out else ''
            if r.returncode == 0:
                self._report(f'{label} 끝 ({time.monotonic() - t0:.0f} s){" · " + tail if tail else ""}', 'ok')
            else:
                self._report(f'{label} 실패: {tail or r.returncode}', 'warn')
            log.info('%s rc=%s %s', label, r.returncode, tail)
        except subprocess.TimeoutExpired:
            self._report(f'{label} 시간 초과', 'warn')
        except Exception as e:  # noqa: BLE001
            self._report(f'{label} 실패: {e}'[:100], 'warn')
        finally:
            with self.lock:
                self.busy = None

    def _report(self, text: str, level: str) -> None:
        with self.lock:
            self.seq += 1
            seq = self.seq
        self.state.set_ops({'seq': seq, 'text': text, 'level': level, 'busy': self.busy})


def new_trip_id() -> str:
    # 폰 브리지는 UUID trip_id만 앱에 넘긴다(mr_phone_bridge bridge.py is_uuid)
    return str(uuid.uuid4())
