"""ElevenLabs credentials stay on the bridge; never log upstream bodies/URLs."""
import json
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.parse import urlencode, urlparse
from urllib.request import Request, urlopen


def issue_token(cfg):
    if not cfg.get('enabled', False):
        return {'ok': False, 'error': 'disabled'}
    agent = cfg.get('agent_id', '')
    if not isinstance(agent, str) or not agent.strip():
        return {'ok': False, 'error': 'no_agent'}
    try:
        key = Path(cfg.get('api_key_path', '')).expanduser().read_text().strip()
    except OSError:
        return {'ok': False, 'error': 'no_key'}
    if not key:
        return {'ok': False, 'error': 'no_key'}
    req = Request('https://api.elevenlabs.io/v1/convai/conversation/get-signed-url?' +
                  urlencode({'agent_id': agent}), headers={'xi-api-key': key})
    try:
        with urlopen(req, timeout=12) as response:
            data = json.loads(response.read(65536))
        url = data.get('signed_url', '')
        parsed = urlparse(url)
        if parsed.scheme != 'wss' or parsed.hostname != 'api.elevenlabs.io':
            return {'ok': False, 'error': 'invalid_response'}
        return {'ok': True, 'signed_url': url}
    except HTTPError as error:
        return {'ok': False, 'error': 'auth_error' if error.code in (401, 403) else 'upstream_error'}
    except (URLError, TimeoutError, OSError):
        return {'ok': False, 'error': 'network_error'}
    except (ValueError, TypeError, AttributeError):
        return {'ok': False, 'error': 'invalid_response'}
