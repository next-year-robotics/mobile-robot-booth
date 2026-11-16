#!/usr/bin/env bash
# 떠 있는 대시보드(run.sh, 브리지, 전방 카메라)를 내린다. 팔 데몬은 그대로 둔다(팔이 자세를 유지한다).
#   scripts/stop.sh          내리기만
#   scripts/stop.sh --arm    팔 데몬도 내린다
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PORT=8080
pid=$(ss -ltnpH "sport = :$PORT" 2>/dev/null | grep -o 'pid=[0-9]*' | head -1 | cut -d= -f2)
if [[ -n "$pid" ]]; then
  parent=$(ps -o ppid= -p "$pid" | tr -d ' ')
  # 뒤에서 뜬 셸은 INT를 무시한다. run.sh가 부모면 그쪽에 TERM을 줘서 정리(카메라 내리기)를 타게 한다
  if [[ -n "$parent" ]] && ps -o args= -p "$parent" | grep -q 'run\.sh'; then kill -TERM "$parent"; else kill -TERM "$pid"; fi
  for _ in $(seq 40); do ss -ltnH "sport = :$PORT" 2>/dev/null | grep -q . || break; sleep 0.25; done
  # run.sh의 뒷정리(카메라 내리기)가 끝날 때까지 기다린다. 안 그러면 바로 다시 띄운 카메라를 그 뒷정리가 내려 버린다
  for _ in $(seq 40); do kill -0 "$parent" 2>/dev/null && ps -o args= -p "$parent" | grep -q 'run\.sh' || break; sleep 0.25; done
fi
"$ROOT/scripts/front_camera.sh" stop >/dev/null 2>&1
[[ "${1:-}" == "--arm" ]] && "$ROOT/scripts/arm.sh" stop
ss -ltnH "sport = :$PORT" 2>/dev/null | grep -q . && { echo "아직 떠 있다"; exit 1; } || echo "대시보드 내림"
