#!/usr/bin/env bash
# 부스 대시보드 실행.
#
#   scripts/run.sh                 실제 로봇 (ROS 도메인 0, Pi와 같은 네트워크)
#   scripts/run.sh --sim           가짜 로봇까지 노트북에서 띄운다 (도메인 77, 이 노트북 안에서만)
#   scripts/run.sh --peer 10.0.0.5 멀티캐스트가 막힌 네트워크에서 Pi 주소를 직접 알려 준다
#   scripts/run.sh --no-browser    브라우저는 직접 연다 (http://127.0.0.1:8080)
#
# 창은 전체 화면 앱 모드로 뜬다. F11로 빠져나오고 Ctrl+C로 전부 내린다.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM=0; PEER=""; BROWSER=1; PORT=8080
while [[ $# -gt 0 ]]; do
  case "$1" in
    --sim) SIM=1 ;;
    --peer) PEER="$2"; shift ;;
    --no-browser) BROWSER=0 ;;
    --port) PORT="$2"; shift ;;
    *) echo "모르는 옵션: $1" >&2; exit 2 ;;
  esac
  shift
done

set +u
source /opt/ros/jazzy/setup.bash
[[ -f "$ROOT/ros_ws/install/setup.bash" ]] || "$ROOT/scripts/build_msgs.sh"
source "$ROOT/ros_ws/install/setup.bash"
set -u

# 웹 빌드가 없거나 소스보다 오래됐으면 다시 만든다(오프라인에서도 된다. node_modules만 있으면).
if [[ ! -f "$ROOT/web/dist/index.html" ]] || [[ -n "$(find "$ROOT/web/src" "$ROOT/web/index.html" "$ROOT/web/public" -newer "$ROOT/web/dist/index.html" -print -quit)" ]]; then
  (cd "$ROOT/web" && npm run build --silent)
fi

PIDS=()
cleanup() {
  for p in "${PIDS[@]}"; do kill -INT "$p" 2>/dev/null || true; done
  sleep 0.5
  for p in "${PIDS[@]}"; do kill -TERM "$p" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM

BRIDGE_ARGS=(--port "$PORT")
if [[ $SIM -eq 1 ]]; then
  # 실제 로봇(도메인 0)과 절대 섞이지 않게 도메인도 바꾸고 이 노트북 밖으로 나가지도 않게 한다.
  export ROS_DOMAIN_ID=77
  export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  python3 "$ROOT/tools/fake_robot.py" &
  PIDS+=($!)
  BRIDGE_ARGS+=(--sim)
else
  export ROS_DOMAIN_ID=0          # Pi 펌웨어가 정한 도메인 (base_contract.md)
  if [[ -n "$PEER" ]]; then export ROS_STATIC_PEERS="$PEER"; fi
fi
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID ${ROS_STATIC_PEERS:+ROS_STATIC_PEERS=$ROS_STATIC_PEERS}"

(cd "$ROOT/bridge" && exec python3 -m booth_bridge "${BRIDGE_ARGS[@]}") &
PIDS+=($!)

if [[ $BROWSER -eq 1 ]]; then
  for _ in $(seq 40); do curl -sf -o /dev/null "http://127.0.0.1:$PORT/" && break; sleep 0.25; done
  # 따로 만든 프로필이라 세션 복원 창이나 번역 막대가 뜨지 않는다.
  google-chrome --user-data-dir="$HOME/.cache/booth-dashboard-chrome" --no-first-run --autoplay-policy=no-user-gesture-required \
    --disable-features=Translate --disable-session-crashed-bubble \
    --app="http://127.0.0.1:$PORT/" --start-fullscreen >/dev/null 2>&1 &
  PIDS+=($!)
fi
wait "${PIDS[@]:0:2}"
