"""느린 주기로 바깥 상태를 확인하는 스레드 둘.

- CloudWatcher: 보호자 대시보드(패드)가 읽는 Firebase RTDB 경로가 지금 갱신되고 있는지.
  대시보드는 /robots/{id}/status의 updated_at_ms가 15 s 안이면 로봇을 온라인으로 그린다. 같은 기준을 쓴다.
  RTDB 클라이언트는 mobile-robot의 mr_care_telemetry/rtdb.py를 그대로 import한다.
- PiWatcher: ssh로 Pi 온도·스로틀·부하와 스택 유닛(mr-booth@*) 상태. 이 Pi는 벅 전원이 약해 부하가 오르면
  저전압이 뜬다.
"""
from __future__ import annotations

import os
import re
import subprocess
import sys
import threading
import time

from .state import BoothState


class CloudWatcher(threading.Thread):

    def __init__(self, cfg: dict, state: BoothState):
        super().__init__(name='cloud', daemon=True)
        self.cfg, self.state = cfg, state
        self.stop = threading.Event()

    def run(self) -> None:
        c = self.cfg
        src = os.path.expanduser(c['telemetry_src'])
        if src not in sys.path:
            sys.path.insert(0, src)
        try:
            from mr_care_telemetry.rtdb import RtdbClient
            client = RtdbClient.from_settings(c['database_url'],
                                              os.path.expanduser(c['service_account_path']),
                                              timeout_s=4.0)
        except Exception as e:  # noqa: BLE001
            self.state.set_cloud({'ok': False, 'error': f'설정 오류: {e}'[:120]})
            return
        rid = c['robot_id']
        events_every = 2
        n = 0
        events: list[dict] = []
        while not self.stop.is_set():
            try:
                status = client.get(f'robots/{rid}/status') or {}
                if n % events_every == 0:
                    raw = client.get(f'robots/{rid}/events', orderBy='"$key"', limitToLast=6) or {}
                    events = [
                        {'ts_ms': e.get('ts_ms'), 'kind': e.get('kind'), 'severity': e.get('severity'),
                         'message': e.get('message')}
                        for _, e in sorted(raw.items())
                    ][::-1]
                updated = status.get('updated_at_ms')
                age = None if not updated else max(0.0, time.time() - updated / 1000.0)
                self.state.set_cloud({
                    'ok': True,
                    'checked_ms': int(time.time() * 1000),
                    'updated_age_s': None if age is None else round(age, 1),
                    'online': age is not None and age < float(c['online_age_s']),
                    'phone_link': (status.get('phone_link') or {}).get('connected'),
                    'mission': status.get('mission_state'),
                    'tracking': status.get('tracking_state'),
                    'events': events,
                })
            except Exception as e:  # noqa: BLE001 — 오프라인 부스에서도 화면은 계속 돈다
                self.state.set_cloud({'ok': False, 'checked_ms': int(time.time() * 1000),
                                      'error': str(e)[:120]})
            n += 1
            self.stop.wait(float(c['status_period_s']))


def ssh_cmd(host: str, remote: str) -> list[str]:
    """ssh 명령줄. 5 s마다 새 핸드셰이크를 하지 않도록 연결을 재사용한다(운영 버튼도 같은 연결을 탄다)."""
    control = os.path.expanduser('~/.ssh/booth-cm-%r@%h-%p')
    return ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=3',
            '-o', 'ControlMaster=auto', '-o', f'ControlPath={control}', '-o', 'ControlPersist=60',
            host, remote]


class PiWatcher(threading.Thread):

    CMD = 'vcgencmd measure_temp; vcgencmd get_throttled; cat /proc/loadavg; nproc; echo units:'

    def __init__(self, cfg: dict, state: BoothState, units: list[str]):
        super().__init__(name='pi', daemon=True)
        self.cfg, self.state, self.units = cfg, state, list(units)
        self.stop = threading.Event()

    def run(self) -> None:
        remote = self.CMD + ''.join(f'; systemctl is-active mr-booth@{u}' for u in self.units)
        cmd = ssh_cmd(self.cfg['ssh_host'], remote)
        while not self.stop.is_set():
            try:
                out = subprocess.run(cmd, capture_output=True, text=True, timeout=8).stdout
                self.state.set_pi(parse_pi(out, self.units))
            except Exception as e:  # noqa: BLE001
                self.state.set_pi({'ok': False, 'error': str(e)[:80]})
            self.stop.wait(float(self.cfg['period_s']))


def parse_pi(out: str, units: list[str] | None = None) -> dict:
    temp = re.search(r"temp=([\d.]+)", out)
    thr = re.search(r"throttled=(0x[0-9a-fA-F]+)", out)
    lines = [ln for ln in out.strip().splitlines() if ln.strip()]
    load = ncpu = None
    for i, ln in enumerate(lines):
        parts = ln.split()
        if len(parts) >= 5 and '/' in parts[3]:
            load = float(parts[0])
            if i + 1 < len(lines) and lines[i + 1].strip().isdigit():
                ncpu = int(lines[i + 1])
    if temp is None and thr is None:
        return {'ok': False, 'error': 'ssh 응답 없음'}
    flags = int(thr.group(1), 16) if thr else 0
    # units: 다음 줄부터 유닛마다 systemctl is-active 한 줄(active, activating, failed, inactive)
    stacks = None
    if units and 'units:' in lines:
        states = lines[lines.index('units:') + 1:]
        stacks = {u: (states[i].strip() if i < len(states) else 'unknown') for i, u in enumerate(units)}
    return {
        'ok': True,
        'temp_c': float(temp.group(1)) if temp else None,
        'throttled': hex(flags),
        'undervoltage_now': bool(flags & 0x1),
        'throttled_now': bool(flags & 0x6),
        'undervoltage_seen': bool(flags & 0x10000),
        'load1': load,
        'ncpu': ncpu,
        'stacks': stacks,
    }
