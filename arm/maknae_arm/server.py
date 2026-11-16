"""팔 데몬의 HTTP 창구. 127.0.0.1에서만 연다.

    GET  /state            팔·심부름 상태 JSON
    GET  /cam/top.jpg      최신 한 장 (wrist도 같다)
    GET  /cam/top.mjpg     MJPEG 스트림 (대시보드 <img>가 바로 쓴다)
    POST /cmd              {"cmd": "fetch", "name": "귤"} → {"ok": true, ...} 또는 {"ok": false, "error": "…"}
"""
from __future__ import annotations

import json
import logging
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from .driver import ArmError
from .skills import Refused

log = logging.getLogger('arm.server')


def make_handler(app):

    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def log_message(self, *args):               # 요청마다 찍지 않는다
            pass

        def _json(self, obj, code=200):
            body = json.dumps(obj, ensure_ascii=False, separators=(',', ':')).encode()
            self.send_response(code)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split('?', 1)[0]
            if path == '/state':
                return self._json(app.state())
            if path.startswith('/cam/'):
                name, _, ext = path[5:].partition('.')
                cam = app.cams.get(name)
                if cam is None:
                    return self._json({'ok': False, 'error': 'no camera'}, 404)
                if ext == 'jpg':
                    data = app.frame(name)
                    if data is None:
                        return self._json({'ok': False, 'error': 'no frame'}, 503)
                    self.send_response(200)
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', str(len(data)))
                    self.send_header('Cache-Control', 'no-store')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    self.wfile.write(data)
                    return
                if ext == 'mjpg':
                    self.send_response(200)
                    self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                    self.send_header('Cache-Control', 'no-store')
                    self.send_header('Connection', 'close')
                    self.end_headers()
                    seq = -1
                    try:
                        while True:
                            seq = cam.wait_next(seq, 1.0)
                            data = app.frame(name)
                            if data is None:
                                continue
                            self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                             + str(len(data)).encode() + b'\r\n\r\n' + data + b'\r\n')
                            time.sleep(0.05)            # 20 fps면 충분하다
                    except (BrokenPipeError, ConnectionResetError, OSError):
                        return
            self._json({'ok': False, 'error': 'not found'}, 404)

        def do_POST(self):
            if self.path != '/cmd':
                return self._json({'ok': False, 'error': 'not found'}, 404)
            try:
                n = int(self.headers.get('Content-Length', '0'))
                msg = json.loads(self.rfile.read(min(n, 65536)) or b'{}')
                if not isinstance(msg, dict):
                    raise ValueError
            except ValueError:
                return self._json({'ok': False, 'error': 'bad json'}, 400)
            try:
                result = app.command(msg)
                self._json({'ok': True, **(result or {})})
            except (Refused, ArmError) as e:
                self._json({'ok': False, 'error': str(e)})
            except Exception as e:
                log.exception('명령 실패 %s', msg.get('cmd'))
                self._json({'ok': False, 'error': f'{type(e).__name__}: {e}'}, 500)

    return Handler


def serve(app, host: str, port: int) -> ThreadingHTTPServer:
    httpd = ThreadingHTTPServer((host, port), make_handler(app))
    httpd.daemon_threads = True
    log.info('http://%s:%d 에서 대기', host, port)
    return httpd
