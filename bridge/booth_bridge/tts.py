"""대화 대답을 Hina 목소리(ElevenLabs)로 읽힌다. 컨트롤 패널에서 '대화 목소리: Hina'를 고르면 쓴다.

브라우저는 Gemini Live가 말한 문장(받아쓰기)만 보내고, 키는 브리지에만 있다. 문장과 소리는 저장하지 않는다.
돌려주는 것은 24 kHz 16비트 모노 PCM이다.
"""
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen
import json

MAX_CHARS = 400


class TtsError(RuntimeError):
    pass


def read_key(path: str) -> str:
    try:
        for line in Path(path).expanduser().read_text().splitlines():
            if line.startswith('ELEVENLABS_API_KEY='):
                return line.split('=', 1)[1].strip().strip('"\'')
    except OSError:
        pass
    return ''


def stream(cfg: dict, text: str):
    """PCM 조각을 오는 대로 낸다(첫 소리가 빨리 나오게). 실패하면 TtsError."""
    text = ' '.join(str(text).split())[:MAX_CHARS]
    if not text:
        raise TtsError('empty')
    key = read_key(cfg.get('key_env_file', '~/.config/mobile-robot/elevenlabs.env'))
    if not key:
        raise TtsError('no_key')
    body = {'text': text, 'model_id': cfg.get('model', 'eleven_v4_turbo'), 'language_code': 'ko',
            'voice_settings': {k: cfg[k] for k in ('stability', 'similarity_boost', 'speed') if k in cfg}}
    req = Request(f"https://api.elevenlabs.io/v1/text-to-speech/{cfg.get('voice_id', '')}/stream?output_format=pcm_24000",
                  data=json.dumps(body).encode(), headers={'xi-api-key': key, 'Content-Type': 'application/json'})
    try:
        with urlopen(req, timeout=float(cfg.get('timeout_s', 12))) as r:
            carry = b''
            while True:
                chunk = r.read(9600)                 # 0.2 s
                if not chunk:
                    break
                chunk = carry + chunk
                carry = chunk[len(chunk) & ~1:]      # 16비트 표본이 반으로 잘리지 않게
                if len(chunk) > 1:
                    yield chunk[:len(chunk) & ~1]
    except HTTPError as e:
        raise TtsError('auth_error' if e.code in (401, 403) else 'quota' if e.code in (402, 429) else 'upstream_error') from None
    except (URLError, TimeoutError, OSError):
        raise TtsError('network_error') from None


def synth(cfg: dict, text: str) -> tuple[bytes | None, str]:
    """(PCM 전체, 오류 코드). 성공이면 오류 코드는 빈 문자열."""
    try:
        return b''.join(stream(cfg, text)), ''
    except TtsError as e:
        return None, str(e)
