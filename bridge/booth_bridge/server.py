"""브라우저 쪽. 포트 하나로 정적 파일(web/dist)과 WebSocket(/ws)을 같이 낸다.

/ws로 두 종류를 보낸다.
- 텍스트: 상태 JSON, state_rate_hz 주기.
- 바이너리: 카메라 프레임. [u32 LE 헤더 길이][헤더 JSON][JPEG]. 헤더에 같은 stamp의 마커 오버레이가 붙는다.

영상은 노트북에서 디코드하지 않고 받은 JPEG를 그대로 넘긴다. 브라우저가 느리면 그 클라이언트에는
프레임을 건너뛴다(쌓아 두면 지연이 늘기만 한다).
"""
from __future__ import annotations

import asyncio
import http
import json
import logging
import mimetypes
import struct
import time
from pathlib import Path

import websockets

from .live_token import issue_token
from .ops import Ops
from .overlay import CameraModel, FrameSync
from .state import BoothState

log = logging.getLogger('booth.server')
mimetypes.add_type('model/stl', '.stl')
mimetypes.add_type('font/woff2', '.woff2')
mimetypes.add_type('text/javascript', '.js')

MAX_CLIENT_BUFFER = 1_500_000


class BoothServer:

    def __init__(self, cfg: dict, state: BoothState, cam: CameraModel, sync: FrameSync, static_dir: Path,
                 ops: Ops):
        self.cfg, self.state, self.cam, self.sync, self.ops = cfg, state, cam, sync, ops
        self.static_dir = static_dir.resolve()
        self.clients: set = set()
        self.state_period = 1.0 / float(cfg['server']['state_rate_hz'])

    # ------------------------------------------------------------ HTTP
    async def process_request(self, path: str, _headers):
        path = path.split('?', 1)[0]
        if path == '/ws':
            return None
        rel = 'index.html' if path in ('', '/') else path.lstrip('/')
        target = (self.static_dir / rel).resolve()
        if not target.is_relative_to(self.static_dir) or not target.is_file():
            if not (self.static_dir / 'index.html').is_file():
                body = '웹 빌드가 없습니다. scripts/run.sh가 web/dist를 만듭니다.'.encode()
                return http.HTTPStatus.SERVICE_UNAVAILABLE, [('Content-Type', 'text/plain; charset=utf-8')], body
            return http.HTTPStatus.NOT_FOUND, [('Content-Type', 'text/plain')], b'not found'
        ctype = mimetypes.guess_type(target.name)[0] or 'application/octet-stream'
        if ctype.startswith('text/') or ctype.endswith('javascript'):
            ctype += '; charset=utf-8'
        # index.html은 매번 새로 받아 새 빌드가 바로 보이게 한다. 해시 붙은 자산은 오래 둔다.
        cache = 'no-cache' if target.name == 'index.html' or target.suffix in ('.vrm', '.mp3') else 'max-age=3600'
        return http.HTTPStatus.OK, [('Content-Type', ctype), ('Cache-Control', cache)], await asyncio.to_thread(target.read_bytes)

    # ------------------------------------------------------------ WS
    async def handler(self, ws) -> None:
        self.clients.add(ws)
        log.info('브라우저 연결 %s (총 %d)', ws.remote_address, len(self.clients))
        last_token = -float("inf")
        token_task = None

        async def send_token():
            result = await asyncio.to_thread(issue_token, self.cfg.get('live', {}))
            try:
                await ws.send(json.dumps({'type': 'live_token', **result}))
            except websockets.ConnectionClosed:
                pass
        try:
            await ws.send(json.dumps(self.hello()))
            async for raw in ws:
                if isinstance(raw, str):
                    try:
                        msg = json.loads(raw)
                    except ValueError:
                        continue
                    if not isinstance(msg, dict):
                        continue
                    if msg.get('cmd') == 'live_token':
                        if (token_task is not None and not token_task.done()) or time.monotonic() - last_token < 5:
                            await ws.send(json.dumps({'type': 'live_token', 'ok': False, 'error': 'rate_limited'}))
                        else:
                            last_token = time.monotonic()
                            token_task = asyncio.create_task(send_token())
                    else:
                        self.on_command(raw)
        except websockets.ConnectionClosed:
            pass
        finally:
            if token_task is not None:
                token_task.cancel()
                await asyncio.gather(token_task, return_exceptions=True)
            self.clients.discard(ws)
            log.info('브라우저 끊김 (총 %d)', len(self.clients))

    def hello(self) -> dict:
        c = self.cfg
        return {
            'type': 'hello',
            'sim': self.state.sim,
            'camera': {'origin': c['camera']['optical_origin_m'], 'width': self.cam.width,
                       'height': self.cam.height, 'K': self.cam.K.flatten().tolist()},
            'marker_size': c['aruco']['marker_size_m'],
            'battery_charge_at': c['battery']['charge_at_v'],
        }

    def on_command(self, raw: str) -> None:
        try:
            msg = json.loads(raw)
        except ValueError:
            return
        if not isinstance(msg, dict):
            return
        cmd = msg.get('cmd')
        if cmd == 'reset_fused':
            self.state.reset_fused()
        elif cmd == 'live_event':
            phase = msg.get('phase')
            duration = msg.get('duration_s')
            if phase in ('start', 'end') and type(duration) is int and 0 <= duration <= 86400:
                log.info('아바타 세션 %s duration_s=%d', phase, duration)
        elif isinstance(cmd, str) and cmd.startswith('op:'):
            self.ops.handle(cmd[3:])

    async def _send_all(self, data) -> None:
        for ws in list(self.clients):
            transport = getattr(ws, 'transport', None)
            if transport is not None and transport.get_write_buffer_size() > MAX_CLIENT_BUFFER:
                continue
            try:
                await ws.send(data)
            except websockets.ConnectionClosed:
                self.clients.discard(ws)

    async def state_loop(self) -> None:
        while True:
            t0 = time.monotonic()
            if self.clients:
                snap = self.state.snapshot(t0)
                await self._send_all(json.dumps(snap, ensure_ascii=False, separators=(',', ':')))
            await asyncio.sleep(max(0.0, self.state_period - (time.monotonic() - t0)))

    async def frame_loop(self) -> None:
        while True:
            ready = self.sync.pop_ready(time.monotonic())
            if ready is not None and self.clients:
                frame, overlay = ready
                header = json.dumps({
                    'type': 'frame', 'stamp_ns': frame.stamp_ns, 'mime': frame.mime,
                    'w': self.cam.width, 'h': self.cam.height, 'overlay': overlay,
                }, separators=(',', ':')).encode()
                await self._send_all(struct.pack('<I', len(header)) + header + frame.data)
            await asyncio.sleep(0.008)

    async def run(self) -> None:
        s = self.cfg['server']
        async with websockets.serve(self.handler, s['host'], int(s['port']),
                                    process_request=self.process_request,
                                    compression=None, max_size=2 ** 16, ping_interval=10):
            log.info('http://%s:%s 에서 대기', s['host'], s['port'])
            await asyncio.gather(self.state_loop(), self.frame_loop())
