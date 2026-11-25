"""브라우저 쪽. 포트 하나로 정적 파일(web/dist)과 WebSocket(/ws)을 같이 낸다.

/ws로 두 종류를 보낸다.
- 텍스트: 상태 JSON, state_rate_hz 주기.
- 바이너리: 카메라 프레임. [u32 LE 헤더 길이][헤더 JSON][JPEG]. 헤더에 같은 stamp의 마커 오버레이가 붙는다.
  라이다 한 바퀴도 같은 틀로 간다(type scan, 본문은 int16 mm 점열). 가장 최근 것만 보낸다.

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
from .look import LookError, ask as look_ask
from .tts import TtsError, stream as tts_stream
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
                 ops: Ops, arm=None, tasks=None):
        self.cfg, self.state, self.cam, self.sync, self.ops, self.arm = cfg, state, cam, sync, ops, arm
        self.tasks = tasks
        self.scan: bytes | None = None        # ROS 스레드가 넣고 scan_loop가 꺼내 보낸다
        self._looking = False                 # 앞 보기는 한 번에 하나만
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
        cache = 'no-cache' if target.name == 'index.html' or target.suffix in ('.vrm', '.mp3', '.json') else 'max-age=3600'
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
                    elif msg.get('cmd') == 'tts':
                        asyncio.create_task(self.tts(ws, msg))
                    elif msg.get('cmd') == 'task':
                        # 장소로 가기·가져오기·돌아오기. 수락·거절을 id와 함께 돌려준다
                        body = msg.get('msg')
                        if self.tasks is None or not isinstance(body, dict):
                            # direct: 팔 일(집기·건네기)은 순서기 없이 팔에 바로 시키면 된다
                            result = {'ok': False, 'error': '이동 기능이 꺼져 있어요.',
                                      'direct': isinstance(body, dict) and body.get('cmd') in ('fetch', 'give')}
                        else:
                            result = await asyncio.to_thread(self.tasks.request, body)
                            log.info('작업 %s → %s', body.get('cmd'), 'ok' if result.get('ok') else result.get('error'))
                        await ws.send(json.dumps({'type': 'reply', 'id': msg.get('id'), **result}, ensure_ascii=False))
                    elif msg.get('cmd') == 'layout':
                        # 배치 패널: 시작 위치와 장소 자리 지정
                        body = msg.get('msg')
                        layout = getattr(self.tasks, 'layout', None)
                        if layout is None or not isinstance(body, dict):
                            result = {'ok': False, 'error': '배치 기능이 꺼져 있어요.'}
                        else:
                            result = layout.request(body, time.monotonic())
                            log.info('배치 %s %s → %s', body.get('op'), body.get('key', ''), 'ok' if result.get('ok') else result.get('error'))
                        await ws.send(json.dumps({'type': 'reply', 'id': msg.get('id'), **result}, ensure_ascii=False))
                    elif msg.get('cmd') == 'arm':
                        # 팔 명령은 답을 기다리는 쪽(음성 도구·대시보드 버튼)이 있어 id를 붙여 돌려준다
                        asyncio.create_task(self.arm_command(ws, msg))
                    elif msg.get('cmd') == 'look':
                        asyncio.create_task(self.look(ws, msg))
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

    async def tts(self, ws, msg: dict) -> None:
        """문장 하나를 Hina 목소리 PCM으로, 오는 대로 조각내어 돌려준다. 이진 프레임: 길이(4) + JSON 머리 + PCM."""
        text, loop = msg.get('text'), asyncio.get_running_loop()

        def frame(pcm: bytes = b'', **extra) -> bytes:
            header = json.dumps({'type': 'tts', 'id': msg.get('id'), 'rate': 24000, **extra}).encode()
            return struct.pack('<I', len(header)) + header + pcm

        def work() -> str:
            try:
                for chunk in tts_stream(self.cfg.get('live', {}).get('tts', {}), text):
                    asyncio.run_coroutine_threadsafe(ws.send(frame(chunk, ok=True, last=False)), loop).result(5)
                return ''
            except TtsError as e:
                return str(e)
            except Exception:                        # 브라우저가 끊겼다
                return 'closed'
        error = 'bad_request' if not isinstance(text, str) else await asyncio.to_thread(work)
        if error:
            log.warning('TTS 실패: %s', error)
        try:
            await ws.send(frame(ok=not error, error=error, last=True))
        except websockets.ConnectionClosed:
            pass

    async def arm_command(self, ws, msg: dict) -> None:
        body = msg.get('msg')
        if self.arm is None or not isinstance(body, dict):
            result = {'ok': False, 'error': '팔이 연결돼 있지 않아요.'}
        else:
            result = await asyncio.to_thread(self.arm.command, body)
            log.info('팔 명령 %s → %s', body.get('cmd'), 'ok' if result.get('ok') else result.get('error'))
        try:
            await ws.send(json.dumps({'type': 'reply', 'id': msg.get('id'), **result}, ensure_ascii=False))
        except websockets.ConnectionClosed:
            pass

    async def look(self, ws, msg: dict) -> None:
        """음성 도구 look_front. 가장 최근 전방 카메라 한 장을 Gemini에게 묻고 글 답을 돌려준다."""
        body = msg.get('msg') if isinstance(msg.get('msg'), dict) else {}
        c = self.cfg.get('look', {})
        frame = self.sync.latest
        if not c.get('enabled', False):
            result = {'ok': False, 'error': '앞 보기 기능이 꺼져 있어요.'}
        elif self._looking:
            result = {'ok': False, 'error': '지금 보고 있어요. 잠깐만요.'}
        elif frame is None or time.monotonic() - frame.recv > float(c.get('max_age_s', 2.0)):
            result = {'ok': False, 'error': '앞 카메라 영상이 안 들어와요.'}
        else:
            self._looking = True
            try:
                key = c.get('key_env_file') or self.cfg.get('live', {}).get('key_env_file', '')
                result = {'ok': True, **await asyncio.to_thread(look_ask, c, key, frame.data, frame.mime, str(body.get('question', '')))}
            except LookError as e:
                result = {'ok': False, 'error': str(e)}
            finally:
                self._looking = False
        try:
            await ws.send(json.dumps({'type': 'reply', 'id': msg.get('id'), **result}, ensure_ascii=False))
        except websockets.ConnectionClosed:
            pass

    def hello(self) -> dict:
        c = self.cfg
        return {
            'type': 'hello',
            'sim': self.state.sim,
            'camera': {'origin': c['camera']['optical_origin_m'], 'width': self.cam.width,
                       'height': self.cam.height, 'K': self.cam.K.flatten().tolist()},
            'marker_size': c['aruco']['marker_size_m'],
            'battery_charge_at': c['battery']['charge_at_v'],
            'arm_url': c.get('arm', {}).get('url') if c.get('arm', {}).get('enabled') else None,
            'places': self.tasks.place_list() if self.tasks is not None and self.tasks.enabled else [],
            'nav': self.tasks.options() if self.tasks is not None else {'enabled': False},
            'lidar': {'z': float(c.get('lidar', {}).get('xyz_m', [0, 0, 0.25])[2]),
                      'xy': [float(v) for v in c.get('lidar', {}).get('xyz_m', [0, 0, 0.25])[:2]]},
            'look': bool(c.get('look', {}).get('enabled')),
            'live': {'enabled': bool(c.get('live', {}).get('enabled')), 'attract_s': c.get('live', {}).get('attract_s', 0),
                     'voice': c.get('live', {}).get('voice', 'Leda'), 'chat_voice': c.get('live', {}).get('chat_voice', 'gemini'),
                     'tts': bool(c.get('live', {}).get('tts', {}).get('voice_id'))},
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
            # 멈춤과 출발(따라가기)은 하던 심부름 이동도 끝낸다
            if cmd in ('op:stop', 'op:start') and self.tasks is not None:
                self.tasks.cancel('멈춤' if cmd == 'op:stop' else '따라가기')
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

    def put_scan(self, header: dict, body: bytes) -> None:
        """ROS 스레드에서 부른다. 못 보낸 지난 바퀴는 덮어쓴다."""
        h = json.dumps(header, separators=(',', ':')).encode()
        self.scan = struct.pack('<I', len(h)) + h + body

    async def scan_loop(self) -> None:
        while True:
            packet, self.scan = self.scan, None
            if packet is not None and self.clients:
                await self._send_all(packet)
            await asyncio.sleep(0.03)

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
            loop = asyncio.get_running_loop()

            def send_dpad(direction):
                websockets.broadcast(self.clients, json.dumps({'type': 'dpad', 'direction': direction}))

            self.state.dpad_hook = lambda direction: loop.call_soon_threadsafe(send_dpad, direction)
            try:
                await asyncio.gather(self.state_loop(), self.frame_loop(), self.scan_loop())
            finally:
                self.state.dpad_hook = None
