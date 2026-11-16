"""앞 보기. "앞에 뭐가 보여?" 같은 질문이 왔을 때만 전방 카메라(C920) 한 장을 Gemini에게 보여 주고 짧은 답을 받는다.

대화 모델(Gemini Live) 세션에는 사진을 넣지 않는다. 세션에 들어간 사진은 대화가 끝날 때까지 문맥에 남아 턴마다
다시 입력 토큰이 된다. 여기서는 사진 한 장을 따로 묻고 글 답만 대화 모델에 돌려준다.
사진과 답은 어디에도 저장하지 않는다. 로그에는 걸린 시간과 토큰 수만 남긴다.
"""
from __future__ import annotations

import base64
import json
import logging
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from .live_token import read_key

log = logging.getLogger('booth.look')
URL = 'https://generativelanguage.googleapis.com/v1beta/models/{model}:generateContent'

PROMPT = """너는 돌봄 로봇 "막내"의 눈이다. 이 사진은 로봇 앞쪽 카메라(바닥에서 약 40 cm 높이)로 방금 찍은 것이다.
관람객의 질문: "{question}"
사진에 보이는 것만으로 질문에 답한다. 한국어로 한두 문장, 말하듯이 짧게 쓴다. 목록·기호·이모지는 쓰지 않는다.
- 사람이 보이면 옷 색, 손동작, 들고 있는 물건처럼 겉으로 보이는 것만 말한다. 누구인지, 나이, 인종, 건강 상태는 짐작하지 않는다.
- 잘 안 보이거나 확실하지 않으면 그렇다고 말한다. 사진에 없는 것을 지어내지 않는다.
- 질문이 비어 있으면 앞에 무엇이 보이는지 한 문장으로 말한다."""


class LookError(RuntimeError):
    pass


def ask(cfg: dict, key_file: str, jpeg: bytes, mime: str, question: str) -> dict:
    """사진 한 장에 대한 답. {'answer': str, 'seconds': float, 'tokens': int}. 실패하면 LookError(사람에게 읽어 줄 문장)."""
    key = read_key(key_file)
    if not key:
        raise LookError('Gemini API 키가 없어요.')
    q = ' '.join(str(question or '').split())[:120].replace('"', ' ')
    gen = {'temperature': 0.2, 'maxOutputTokens': int(cfg.get('max_output_tokens', 200)),
           'thinkingConfig': {'thinkingLevel': str(cfg.get('thinking_level', 'low'))}}
    if cfg.get('media_resolution'):
        gen['mediaResolution'] = str(cfg['media_resolution'])
    body = {'contents': [{'parts': [
        {'inlineData': {'mimeType': mime, 'data': base64.b64encode(jpeg).decode()}},
        {'text': PROMPT.format(question=q)}]}], 'generationConfig': gen}
    model = str(cfg.get('model', 'gemini-4-hydrogen'))
    req = Request(URL.format(model=model), data=json.dumps(body).encode(),
                  headers={'x-goog-api-key': key, 'Content-Type': 'application/json'})
    t0 = time.monotonic()
    try:
        with urlopen(req, timeout=float(cfg.get('timeout_s', 12))) as r:
            data = json.loads(r.read())
    except HTTPError as e:
        log.warning('앞 보기 HTTP %s (%s)', e.code, model)
        raise LookError('잠깐 눈이 침침해요. 다시 물어봐 주세요.') from None
    except (URLError, TimeoutError, OSError):
        raise LookError('인터넷이 느려서 앞을 못 봤어요.') from None
    seconds = time.monotonic() - t0
    try:
        parts = data['candidates'][0]['content']['parts']
        answer = ' '.join(p.get('text', '') for p in parts if not p.get('thought')).strip()
    except (KeyError, IndexError, TypeError):
        answer = ''
    if not answer:
        raise LookError('앞을 봤는데 뭐라고 말할지 모르겠어요.')
    usage = data.get('usageMetadata') or {}
    tokens = int(usage.get('promptTokenCount') or 0)
    log.info('앞 보기 %s %.1f s, 입력 %d 토큰, 출력 %s 토큰', model, seconds, tokens, usage.get('candidatesTokenCount'))
    return {'answer': answer[:300], 'seconds': round(seconds, 2), 'tokens': tokens}
