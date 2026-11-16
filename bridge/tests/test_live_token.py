import asyncio
import json
from pathlib import Path
from unittest.mock import patch
from urllib.error import HTTPError, URLError

import pytest

from booth_bridge.live_token import issue_token
from booth_bridge.server import BoothServer


@pytest.fixture
def cfg(tmp_path):
    key = tmp_path / 'gemini.env'
    key.write_text('GEMINI_API_KEY=test-secret-not-for-browser\n')
    return {'enabled': True, 'key_env_file': str(key), 'model': 'gemini-3.8-live', 'voice': 'Leda'}


def test_missing_config(cfg):
    assert issue_token({})['error'] == 'disabled'
    assert issue_token({'enabled': True})['error'] == 'no_key'
    cfg['key_env_file'] = '/missing-key'
    assert issue_token(cfg)['error'] == 'no_key'


def test_request_and_response(cfg, caplog):
    class Response:
        def __enter__(self): return self
        def __exit__(self, *args): pass
        def read(self, _n): return b'{"name":"auth_tokens/one-time"}'
    with patch('booth_bridge.live_token.urlopen', return_value=Response()) as request:
        result = issue_token(cfg)
    req = request.call_args.args[0]
    assert req.get_method() == 'POST'
    assert req.full_url.endswith('/v1beta/auth_tokens')
    assert req.get_header('X-goog-api-key') == 'test-secret-not-for-browser'
    assert json.loads(req.data)['uses'] == 1
    assert result == {'ok': True, 'token': 'auth_tokens/one-time', 'model': 'gemini-3.8-live', 'voice': 'Leda'}
    assert 'test-secret' not in json.dumps(result)
    assert not caplog.text


def test_unexpected_body_is_rejected(cfg):
    class Response:
        def __enter__(self): return self
        def __exit__(self, *args): pass
        def read(self, _n): return b'{"error":"nope"}'
    with patch('booth_bridge.live_token.urlopen', return_value=Response()):
        assert issue_token(cfg) == {'ok': False, 'error': 'invalid_response'}


@pytest.mark.parametrize('error, expected', [
    (HTTPError('private', 401, 'secret', {}, None), 'auth_error'),
    (HTTPError('private', 500, 'secret', {}, None), 'upstream_error'),
    (URLError('secret'), 'network_error'),
])
def test_failure_is_sanitized(cfg, error, expected, caplog):
    with patch('booth_bridge.live_token.urlopen', side_effect=error):
        assert issue_token(cfg) == {'ok': False, 'error': expected}
    assert 'secret' not in caplog.text


def test_static_is_async_and_confined(tmp_path):
    server = object.__new__(BoothServer)
    server.static_dir = tmp_path / 'dist'; server.static_dir.mkdir()
    (server.static_dir / 'index.html').write_text('home')
    (server.static_dir / 'model.vrm').write_bytes(b'vrm')
    sibling = tmp_path / 'dist-private'; sibling.mkdir()
    (sibling / 'key').write_text('secret')
    response = asyncio.run(server.process_request('/../dist-private/key', {}))
    assert response[0] == 404
    response = asyncio.run(server.process_request('/model.vrm', {}))
    assert ('Cache-Control', 'no-cache') in response[1]
    assert response[2] == b'vrm'


def test_token_request_does_not_block_stop_command():
    async def run():
        server = object.__new__(BoothServer)
        server.clients = set(); server.cfg = {'live': {}}
        server.hello = lambda: {'type': 'hello'}
        commands = []
        server.on_command = commands.append
        started = asyncio.Event(); finish = asyncio.Event()
        async def slow_issue(*args):
            started.set()
            await finish.wait()
            return {'ok': False, 'error': 'no_key'}
        class WS:
            remote_address = ('127.0.0.1', 1234)
            async def send(self, _data): pass
            def __aiter__(self): return self.messages()
            async def messages(self):
                yield '{"cmd":"live_token"}'
                await started.wait()
                yield '{"cmd":"op:stop"}'
                assert commands == ['{"cmd":"op:stop"}']
                finish.set()
        with patch('booth_bridge.server.asyncio.to_thread', side_effect=slow_issue):
            await asyncio.wait_for(server.handler(WS()), 1)
    asyncio.run(run())
