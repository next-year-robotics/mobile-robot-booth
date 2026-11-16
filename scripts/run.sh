#!/usr/bin/env bash
# 부스 대시보드 실행.
#
#   scripts/run.sh                 실제 로봇 (ROS 도메인 0, Pi와 같은 네트워크)
#   scripts/run.sh --sim           가짜 로봇·가짜 팔(가짜 작업대)까지 노트북에서 띄운다 (도메인 77, 이 노트북 안에서만).
#                                  로봇 없이 음성 → 집기 → 건네기를 리허설할 때 쓴다
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

# 이미 떠 있는 대시보드가 있으면 아무것도 건드리지 않고 끝낸다(끝내면서 남의 카메라를 내리지 않게).
if curl -sf -o /dev/null --max-time 1 "http://127.0.0.1:$PORT/"; then
  echo "대시보드가 이미 떠 있다: http://127.0.0.1:$PORT/ (내리려면 scripts/stop.sh)" >&2; exit 1
fi

set +u
source /opt/ros/jazzy/setup.bash
[[ -f "$ROOT/ros_ws/install/setup.bash" ]] || "$ROOT/scripts/build_msgs.sh"
source "$ROOT/ros_ws/install/setup.bash"
set -u

# 웹 빌드가 없거나 소스보다 오래됐으면 다시 만든다(오프라인에서도 된다. node_modules만 있으면).
if [[ ! -f "$ROOT/web/dist/index.html" ]] || [[ -n "$(find "$ROOT/web/src" "$ROOT/web/index.html" "$ROOT/web/public" -newer "$ROOT/web/dist/index.html" -print -quit)" ]]; then
  (cd "$ROOT/web" && npm run build --silent)
fi

arm_enabled() { python3 -c "import sys,yaml;sys.exit(0 if yaml.safe_load(open(sys.argv[1])).get('arm',{}).get('enabled') else 1)" "$ROOT/config/booth.yaml"; }
PIDS=()
cleanup() {
  for p in "${PIDS[@]}"; do kill -INT "$p" 2>/dev/null || true; done
  sleep 0.5
  for p in "${PIDS[@]}"; do kill -TERM "$p" 2>/dev/null || true; done
  [[ ${FRONT_CAMERA:-0} -eq 1 ]] && "$ROOT/scripts/front_camera.sh" stop >/dev/null 2>&1 || true
  [[ ${FAKE_ARM:-0} -eq 1 ]] && "$ROOT/scripts/arm.sh" stop >/dev/null 2>&1 || true
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
  # Pi에서 mr-booth@nav로 도는 이동 노드를 여기서 같이 띄운다(가짜 로봇을 실제와 같은 토픽으로 몬다)
  python3 "$ROOT/robot/errand_nav.py" &
  PIDS+=($!)
  # 팔 데몬이 꺼져 있을 때만 가짜 팔을 띄우고, 끝나면 내린다. 진짜 팔이 떠 있으면 건드리지 않는다.
  if arm_enabled && ! "$ROOT/scripts/arm.sh" status >/dev/null 2>&1; then
    "$ROOT/scripts/arm.sh" start --fake && FAKE_ARM=1 || true
  fi
else
  export ROS_DOMAIN_ID=0          # Pi 펌웨어가 정한 도메인 (base_contract.md)
  if [[ -n "$PEER" ]]; then export ROS_STATIC_PEERS="$PEER"; fi
  # 전방 카메라가 노트북에 꽂혀 있으면 카메라와 ArUco 검출을 여기서 띄운다(stack.front_camera)
  "$ROOT/scripts/front_camera.sh" start || true
  FRONT_CAMERA=1
  # 로봇팔 데몬. 대시보드를 내려도 그대로 둔다(팔이 그 자세로 버틴다). 내리려면 scripts/arm.sh stop
  if arm_enabled; then
    # 리허설하다 남은 가짜 팔이 떠 있으면 진짜로 바꿔 띄운다
    if "$ROOT/scripts/arm.sh" status 2>/dev/null | grep -q '(가짜)'; then "$ROOT/scripts/arm.sh" stop; fi
    "$ROOT/scripts/arm.sh" start || true
  fi
fi
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID ${ROS_STATIC_PEERS:+ROS_STATIC_PEERS=$ROS_STATIC_PEERS}"

(cd "$ROOT/bridge" && exec python3 -m booth_bridge "${BRIDGE_ARGS[@]}") &
PIDS+=($!)

if [[ $BROWSER -eq 1 ]]; then
  for _ in $(seq 40); do curl -sf -o /dev/null "http://127.0.0.1:$PORT/" && break; sleep 0.25; done
  "$ROOT/scripts/chrome.sh" "http://127.0.0.1:$PORT/" --start-fullscreen >/dev/null 2>&1 &
  PIDS+=($!)
fi
if [[ $SIM -eq 1 ]]; then wait "${PIDS[@]:0:3}"; else wait "${PIDS[0]}"; fi
