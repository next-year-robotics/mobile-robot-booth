#!/usr/bin/env bash
# 바탕화면 바로가기(부스 대시보드)가 부른다. 터미널 없이 돈다.
#
# 브리지가 안 떠 있으면 run.sh --no-browser를 뒤에서 띄우고, 대시보드를 최대화된 앱 창으로 연다.
# 브리지가 이미 떠 있으면 창만 다시 연다(열려 있던 창은 닫는다). 창을 닫아도 브리지는 남으니 다시 누르면 바로 뜬다.
# 전부 내리고 새로 띄우려면 바탕화면의 로봇 브링업(scripts/bringup.sh)을 누른다.
# 로그는 ~/.cache/booth-dashboard/run.log
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PORT=8080
URL="http://127.0.0.1:$PORT/"
CACHE="$HOME/.cache/booth-dashboard"
LOG="$CACHE/run.log"
mkdir -p "$CACHE"

# 바탕화면에서 뜨면 nvm이 PATH에 없어서 시스템 node(v18)로 웹 빌드를 하게 된다
NVM_BIN=$(ls -d "$HOME"/.nvm/versions/node/*/bin 2>/dev/null | sort -V | tail -1)
[[ -n "$NVM_BIN" ]] && export PATH="$NVM_BIN:$PATH"

up() { curl -sf -o /dev/null --max-time 1 "$URL"; }

# 두 번 눌러도 브리지가 둘 뜨지 않게 기동하는 동안만 잠근다
exec 9>"$CACHE/start.lock"
flock -n 9 || exit 0
if ! up; then
  # 9>&-: 브리지가 잠금을 물려받으면 창을 닫은 뒤 다시 눌러도 아무것도 안 뜬다
  setsid "$ROOT/scripts/run.sh" --no-browser >"$LOG" 2>&1 </dev/null 9>&- &
  # 처음이면 메시지 패키지와 웹을 빌드하느라 오래 걸린다
  for _ in $(seq 240); do up && break; kill -0 $! 2>/dev/null || break; sleep 0.25; done
  if ! up; then
    notify-send -i dialog-error "부스 대시보드" "브리지가 뜨지 않았습니다. 로그: $LOG"
    exit 1
  fi
fi
exec 9>&-

# 창은 하나만 둔다. 열려 있던 대시보드 창이 있으면 닫고 새로 연다(여러 번 눌러도 창이 쌓이지 않는다).
pkill -f -- "--user-data-dir=$HOME/.cache/booth-dashboard-chrome" 2>/dev/null && sleep 1
exec "$ROOT/scripts/chrome.sh" "$URL" --start-maximized >/dev/null 2>&1
