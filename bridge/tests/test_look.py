"""앞 보기(look_front): 전방 카메라 한 장을 물을 때만 Gemini에 보내고, 키·사진이 답이나 로그로 새지 않는다."""
import asyncio
import base64
import json
import time
from unittest.mock import patch
from urllib.error import HTTPError

import pytest

from booth_bridge.look import LookError, ask
from booth_bridge.overlay import Frame, FrameSync
from booth_bridge.server import BoothServer

JPEG = b'\xff\xd8fake-jpeg\xff\xd9'


@pytest.fixture
def key(tmp_path):
    p = tmp_path / 'gemini.env'
    p.write_text('GEMINI_API_KEY=test-secret-not-for-browser\n')
    return str(p)


def response(body: dict):
    class R:
        def __enter__(self): return self
        def __exit__(self, *a): pass
        def read(self): return json.dumps(body).encode()
    return R()


ANSWER = {'candidates': [{'content': {'parts': [{'text': '생각', 'thought': True}, {'text': '파란 옷을 입은 분이 보여요.'}]}}],
          'usageMetadata': {'promptTokenCount': 446, 'candidatesTokenCount': 12}}


def test_sends_one_photo_and_returns_short_answer(key, caplog):
    cfg = {'model': 'gemini-3.5-flash-lite', 'media_resolution': 'MEDIA_RESOLUTION_LOW', 'thinking_level': 'minimal'}
    with patch('booth_bridge.look.urlopen', return_value=response(ANSWER)) as req:
        r = ask(cfg, key, JPEG, 'image/jpeg', '앞에 "뭐가" 보여?')
    sent = req.call_args.args[0]
    body = json.loads(sent.data)
    assert sent.full_url.endswith('/models/gemini-3.5-flash-lite:generateContent')
    parts = body['contents'][0]['parts']
    assert base64.b64decode(parts[0]['inlineData']['data']) == JPEG
    assert '앞에  뭐가  보여?' in parts[1]['text']              # 따옴표는 프롬프트를 깨지 않게 지운다
    assert body['generationConfig']['mediaResolution'] == 'MEDIA_RESOLUTION_LOW'
    assert body['generationConfig']['thinkingConfig'] == {'thinkingLevel': 'minimal'}
    assert r['answer'] == '파란 옷을 입은 분이 보여요.' and r['tokens'] == 446   # 생각 부분은 빼고 답만
    assert 'test-secret' not in caplog.text and '파란 옷' not in caplog.text   # 키도 답도 로그에 남기지 않는다


def test_failures_become_sentences(key):
    with patch('booth_bridge.look.urlopen', side_effect=HTTPError('u', 503, 'busy', {}, None)):
        with pytest.raises(LookError, match='다시 물어봐'):
            ask({}, key, JPEG, 'image/jpeg', '앞')
    with patch('booth_bridge.look.urlopen', return_value=response({'candidates': [{'content': {'parts': []}}]})):
        with pytest.raises(LookError):
            ask({}, key, JPEG, 'image/jpeg', '앞')
    with pytest.raises(LookError, match='키'):
        ask({}, '/missing', JPEG, 'image/jpeg', '앞')


class FakeWs:
    def __init__(self): self.sent = []
    async def send(self, data): self.sent.append(json.loads(data))


def server(cfg, sync):
    s = BoothServer.__new__(BoothServer)
    s.cfg, s.sync, s._looking = cfg, sync, False
    return s


def run_look(s, question='앞에 뭐가 보여?'):
    ws = FakeWs()
    asyncio.run(s.look(ws, {'cmd': 'look', 'id': 3, 'msg': {'question': question}}))
    return ws.sent[-1]


def test_server_refuses_without_fresh_frame(key):
    sync = FrameSync(0.1)
    s = server({'look': {'enabled': True, 'key_env_file': key, 'max_age_s': 2.0}}, sync)
    assert run_look(s) == {'type': 'reply', 'id': 3, 'ok': False, 'error': '앞 카메라 영상이 안 들어와요.'}
    sync.add_frame(Frame(1, time.monotonic() - 5, JPEG, 'image/jpeg'))          # 5초 묵은 영상
    assert run_look(s)['error'] == '앞 카메라 영상이 안 들어와요.'
    s.cfg['look']['enabled'] = False
    assert '꺼져' in run_look(s)['error']


def test_server_answers_with_latest_frame(key):
    sync = FrameSync(0.1)
    sync.add_frame(Frame(1, time.monotonic(), b'old', 'image/jpeg'))
    sync.add_frame(Frame(2, time.monotonic(), JPEG, 'image/jpeg'))
    sync.pop_ready(time.monotonic())                                            # 화면으로 내보낸 뒤에도 남아 있다
    s = server({'look': {'enabled': True, 'key_env_file': key}}, sync)
    with patch('booth_bridge.server.look_ask', return_value={'answer': '귤이 보여요.', 'seconds': 1.2, 'tokens': 446}) as a:
        r = run_look(s)
    assert a.call_args.args[2] == JPEG
    assert r == {'type': 'reply', 'id': 3, 'ok': True, 'answer': '귤이 보여요.', 'seconds': 1.2, 'tokens': 446}
