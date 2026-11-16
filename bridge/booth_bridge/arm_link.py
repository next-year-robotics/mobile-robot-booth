"""팔 데몬(arm/maknae_arm, 다른 파이썬 환경의 별도 프로세스)과의 연결.

상태는 스레드가 주기적으로 읽어 BoothState에 넣고, 명령은 HTTP POST로 넘긴다. 데몬이 없으면 arm 칸이 None일 뿐
대시보드의 나머지는 그대로 돈다.
"""
from __future__ import annotations

import json
import logging
import threading
import time
from urllib.error import URLError
from urllib.request import Request, urlopen

log = logging.getLogger('booth.arm')


class ArmLink(threading.Thread):

    def __init__(self, cfg: dict, state, sim: bool = False):
        super().__init__(name='arm-link', daemon=True)
        self.url = str(cfg.get('url', 'http://127.0.0.1:8091')).rstrip('/')
        self.period = float(cfg.get('period_s', 0.1))
        self.state = state
        self.up = False
        self.sim = sim              # 시뮬레이션 브리지다. 진짜 팔(fake가 아닌 데몬)에는 명령을 보내지 않는다
        self.real = False
        self.sent_grip = None

    def run(self) -> None:
        while True:
            t0 = time.monotonic()
            try:
                with urlopen(self.url + '/state', timeout=1.0) as r:
                    data = json.loads(r.read())
                if not self.up:
                    log.info('팔 데몬 연결 %s', self.url)
                self.up = True
                self.real = not data.get('fake', False)
                self.state.set_arm(data)
                self.update_grip()
            except (URLError, OSError, ValueError):
                if self.up:
                    log.warning('팔 데몬 끊김')
                self.sent_grip = None
                self.up = False
                self.state.set_arm(None)
                time.sleep(1.0)
            time.sleep(max(0.0, self.period - (time.monotonic() - t0)))

    def update_grip(self) -> None:
        goal = self.state.pad_grip(time.monotonic())
        if goal is None:
            self.sent_grip = None
        elif goal != self.sent_grip:
            session, value = goal
            opening = self.sent_grip is None or session != self.sent_grip[0]
            # 첫 열림은 기존 궤적으로 끝까지 실행한다. 이후에는 가장 최근 RT 값만 보낸다.
            if opening:
                value = 100
            reply = self.command({'cmd': 'gripper' if opening else 'pad_gripper', 'value': value})
            if reply.get('ok'):
                self.sent_grip = (session, value)

    def command(self, msg: dict) -> dict:
        """막는 호출이다. asyncio에서는 to_thread로 부른다."""
        if self.sim and (self.real or not self.up) and msg.get('cmd') != 'stop':
            # 가짜 로봇을 돌려 보는 중에 진짜 팔이 움직이면 안 된다(2026-11-16 시험 중에 실제 팔로 "찾기"가 나갔다)
            return {'ok': False, 'error': '시뮬레이션에서는 진짜 팔을 움직이지 않아요.'}
        try:
            req = Request(self.url + '/cmd', data=json.dumps(msg).encode(),
                          headers={'Content-Type': 'application/json'})
            with urlopen(req, timeout=5.0) as r:
                return json.loads(r.read())
        except (URLError, OSError, ValueError):
            return {'ok': False, 'error': '팔 데몬에 연결할 수 없어요.'}
