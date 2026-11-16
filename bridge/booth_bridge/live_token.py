"""Gemini Live 임시 토큰 발급. API 키는 브리지에만 있고 브라우저는 한 번 쓰는 토큰만 받는다.

키 파일은 KEY=VALUE 형식(~/.config/mobile-robot/gemini.env의 GEMINI_API_KEY)이다. 응답 본문과 키는 로그에 남기지 않는다.
"""
import datetime
import json
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

URL = 'https://generativelanguage.googleapis.com/v1beta/auth_tokens'


def read_key(path: str) -> str:
    try:
        text = Path(path).expanduser().read_text()
    except OSError:
        return ''
    for line in text.splitlines():
        if line.startswith('GEMINI_API_KEY='):
            return line.split('=', 1)[1].strip().strip('"\'')
    return text.strip() if '=' not in text else ''


def issue_token(cfg):
    if not cfg.get('enabled', False):
        return {'ok': False, 'error': 'disabled'}
    key = read_key(cfg.get('key_env_file', ''))
    if not key:
        return {'ok': False, 'error': 'no_key'}
    now = datetime.datetime.now(datetime.timezone.utc)
    stamp = '%Y-%m-%dT%H:%M:%SZ'
    body = {
        'uses': 1,
        # 세션은 길어야 대화 한 번이다. 넉넉히 30분, 새 세션은 1분 안에만 열 수 있다.
        'expireTime': (now + datetime.timedelta(minutes=30)).strftime(stamp),
        'newSessionExpireTime': (now + datetime.timedelta(minutes=1)).strftime(stamp),
    }
    req = Request(URL, data=json.dumps(body).encode(),
                  headers={'x-goog-api-key': key, 'Content-Type': 'application/json'})
    try:
        with urlopen(req, timeout=12) as response:
            data = json.loads(response.read(65536))
        name = data.get('name', '')
        if not isinstance(name, str) or not name.startswith('auth_tokens/'):
            return {'ok': False, 'error': 'invalid_response'}
        return {'ok': True, 'token': name, 'model': cfg.get('model', 'gemini-3.8-live'),
                'voice': cfg.get('voice', 'Leda')}
    except HTTPError as error:
        return {'ok': False, 'error': 'auth_error' if error.code in (401, 403) else 'upstream_error'}
    except (URLError, TimeoutError, OSError):
        return {'ok': False, 'error': 'network_error'}
    except (ValueError, TypeError, AttributeError):
        return {'ok': False, 'error': 'invalid_response'}
