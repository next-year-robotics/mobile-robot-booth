#!/usr/bin/env bash
# 대시보드 창을 연다(run.sh, desktop.sh가 부른다). 넘긴 인자는 그대로 크롬에 붙는다.
#
#   scripts/chrome.sh http://127.0.0.1:8080/ --start-fullscreen
#
# 마이크 허용: 전에는 --use-fake-ui-for-media-stream으로 권한 창을 건너뛰었는데, 크롬이 창 위에
# "unsupported command-line flag" 경고 띠를 띄운다. 대신 이 대시보드 전용 프로필에 마이크 허용을 미리 적어 둔다.
# 프로필을 쓰는 크롬이 떠 있는 동안에는 고치지 않는다(크롬이 끝날 때 덮어쓴다). 그때는 이미 적혀 있다.
set -uo pipefail
URL="${1:?주소}"; shift
PROFILE="$HOME/.cache/booth-dashboard-chrome"
if ! pgrep -f -- "--user-data-dir=$PROFILE" >/dev/null; then
  python3 - "$PROFILE/Default/Preferences" "${URL%/}" <<'PY' || true
import json, os, sys, time
path, origin = sys.argv[1], sys.argv[2]
os.makedirs(os.path.dirname(path), exist_ok=True)
try:
    prefs = json.load(open(path, encoding='utf-8'))
except (OSError, ValueError):
    prefs = {}
mic = prefs.setdefault('profile', {}).setdefault('content_settings', {}).setdefault('exceptions', {}) \
    .setdefault('media_stream_mic', {})
key = f'{origin},*'
if mic.get(key, {}).get('setting') != 1:
    # last_modified는 1601년부터의 마이크로초(크롬 시각)
    mic[key] = {'last_modified': str(int((time.time() + 11644473600) * 1e6)), 'setting': 1}
    tmp = path + '.tmp'
    json.dump(prefs, open(tmp, 'w', encoding='utf-8'), separators=(',', ':'))
    os.replace(tmp, path)
PY
fi
# 따로 만든 프로필이라 세션 복원 창이나 번역 막대가 뜨지 않는다.
exec google-chrome --user-data-dir="$PROFILE" --no-first-run --autoplay-policy=no-user-gesture-required \
  --disable-features=Translate --disable-session-crashed-bubble --app="$URL" "$@"
