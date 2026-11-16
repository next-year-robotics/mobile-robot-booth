#!/usr/bin/env bash
# 로봇팔 데몬(arm/maknae_arm)을 띄우고 내린다. LeRobot이 깔린 conda 환경(lerobot)의 파이썬을 쓴다.
#
#   scripts/arm.sh start [--fake]   이미 떠 있으면 그대로 둔다
#   scripts/arm.sh stop             팔은 그 자세에서 토크를 유지한다
#   scripts/arm.sh restart | status | log
#   scripts/arm.sh cmd '{"cmd":"home"}'
#
# 로그는 ~/.cache/booth-dashboard/arm.log. run.sh가 start를 부른다.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CACHE="$HOME/.cache/booth-dashboard"; mkdir -p "$CACHE"
PID_FILE="$CACHE/arm.pid"
PY="${ARM_PYTHON:-$HOME/miniconda3/envs/lerobot/bin/python}"
PORT=$(python3 -c "import sys,yaml;print(yaml.safe_load(open(sys.argv[1]))['server']['port'])" "$ROOT/config/arm.yaml")
running() { [[ -s $PID_FILE ]] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; }

case "${1:-status}" in
start)
  running && { echo "팔 데몬 이미 실행 중"; exit 0; }
  [[ -x $PY ]] || { echo "파이썬이 없다: $PY (ARM_PYTHON으로 바꿀 수 있다)"; exit 1; }
  shift
  # 뒤에서 도는 셸까지 통째로 로그로 돌리고 exec로 바꿔 끼운다. 셸이 남아 있으면 PID 파일이 데몬이 아니라 그 셸을 가리키고,
  # 이 스크립트의 출력을 파이프로 받는 쪽(| tail 등)이 데몬이 끝날 때까지 안 끝난다.
  (cd "$ROOT/arm" && PYTHONPATH="$ROOT/arm" exec setsid nohup "$PY" -u -m maknae_arm "$@") >"$CACHE/arm.log" 2>&1 </dev/null &
  echo $! >"$PID_FILE"
  for _ in $(seq 40); do curl -sf -o /dev/null --max-time 1 "http://127.0.0.1:$PORT/state" && break; running || break; sleep 0.25; done
  if curl -sf -o /dev/null --max-time 1 "http://127.0.0.1:$PORT/state"; then echo "팔 데몬 기동 (로그 $CACHE/arm.log)"
  else echo "팔 데몬이 뜨지 않았다:"; tail -5 "$CACHE/arm.log"; exit 1; fi ;;
stop)
  if running; then kill -TERM "$(cat "$PID_FILE")"; for _ in $(seq 20); do running || break; sleep 0.1; done; running && kill -KILL "$(cat "$PID_FILE")"; fi
  # 손으로 띄웠거나 PID 파일을 잃은 것도 내린다. 포트가 풀릴 때까지 기다려야 바로 다시 띄울 수 있다.
  pkill -TERM -f 'python.* -m maknae_arm' 2>/dev/null
  for _ in $(seq 30); do curl -sf -o /dev/null --max-time 0.3 "http://127.0.0.1:$PORT/state" || break; sleep 0.1; done
  rm -f "$PID_FILE"; echo "팔 데몬 내림" ;;
restart) "$0" stop; shift; "$0" start "$@" ;;
status)
  if running; then curl -s --max-time 2 "http://127.0.0.1:$PORT/state" | python3 -c "
import sys,json
d=json.load(sys.stdin); e=d['errand']
print('실행 중', '(가짜)' if d['fake'] else '', '| 심부름', e['state'], e['object'], '| 관절', d['joints'], '| 끝점', d['tip'], '|', d['volt'], 'V', d['temp_c'], '°C', '| 고장:', d['fault'] or '없음')"
  else echo "꺼짐"; exit 1; fi ;;
log) exec tail -n 60 -f "$CACHE/arm.log" ;;
cmd) curl -s --max-time 5 -X POST "http://127.0.0.1:$PORT/cmd" -d "${2:?JSON}"; echo ;;
*) echo "start [--fake] | stop | restart | status | log | cmd JSON"; exit 2 ;;
esac
