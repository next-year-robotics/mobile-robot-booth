#!/usr/bin/env bash
# 바탕화면 바로가기(로봇 브링업)가 부른다. 터미널 창에서 돌며 진행을 보여 준다.
#
# 누를 때마다 전부 내리고 처음부터 다시 띄운다. 이미 떠 있어도 마찬가지다.
#   1. Pi가 네트워크에 붙을 때까지 기다린다(ssh rpi5, 최대 90 s)
#   2. mobile-robot의 follow.launch.py가 Pi 것과 다르면 올리고 mr_bringup을 다시 빌드한다
#   3. robot/ 파일(스택 실행기, 패드·이동 노드, systemd 유닛)이 다르면 새로 설치하고, Pi 스택을 다시 띄운다
#   4. 유닛이 다 뜰 때까지 기다리고 상태를 보여 준다
#   5. 대시보드(브리지·전방 카메라·팔 데몬)와 창을 내리고 새로 띄운다
# 한 번에 하나만 돈다. 도는 중에 또 누르면 먼저 돌던 것을 끝내고 새로 시작한다.
# 주행 중에 누르면 로봇이 선다(스택이 내려가면서 바퀴 지령이 끊긴다).
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HOST=rpi5
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=5 "$HOST")
LAUNCH="${MOBILE_ROBOT:-$HOME/dev/mobile-robot}/software/ros_pkgs/mr_bringup/launch/follow.launch.py"
PI_LAUNCH='ros2_ws/src/mr_bringup/launch/follow.launch.py'
UNITS=$(python3 -c "import sys,yaml;print(' '.join(yaml.safe_load(open(sys.argv[1]))['stack']['units']))" "$ROOT/config/booth.yaml")

# 먼저 돌던 브링업이 있으면 끝낸다. 그쪽 터미널 창도 같이 닫힌다.
CACHE="$HOME/.cache/booth-dashboard"; mkdir -p "$CACHE"
PIDFILE="$CACHE/bringup.pid"
OLD=$(cat "$PIDFILE" 2>/dev/null)
if [[ -n $OLD && $OLD != $$ ]] && ps -o args= -p "$OLD" 2>/dev/null | grep -q 'bringup\.sh'; then
  echo "먼저 돌던 브링업($OLD)을 끝낸다"
  pkill -TERM -P "$OLD" 2>/dev/null; kill -TERM "$OLD" 2>/dev/null
  for _ in $(seq 20); do kill -0 "$OLD" 2>/dev/null || break; sleep 0.1; done
  kill -KILL "$OLD" 2>/dev/null
fi
echo $$ >"$PIDFILE"
trap '[[ $(cat "$PIDFILE" 2>/dev/null) == $$ ]] && rm -f "$PIDFILE"' EXIT
# 둘이 거의 동시에 떠도 하나만 지나가게 잠근다(위에서 못 끝낸 경우의 안전망)
exec 8>"$CACHE/bringup.lock"
flock -w 15 8 || { echo "다른 브링업이 아직 돌고 있다"; sleep 3; exit 1; }

step() { printf '\n\033[1;32m▶ %s\033[0m\n' "$*"; }
fail() {
  printf '\n\033[1;38;5;208m✕ %s\033[0m\n' "$*"
  notify-send -i dialog-error "로봇 브링업" "$*" 2>/dev/null
  read -rp "Enter를 누르면 닫힙니다 " _
  exit 1
}

step "Pi 연결 (ssh $HOST)"
for i in $(seq 18); do
  "${SSH[@]}" true 2>/dev/null && break
  [[ $i == 18 ]] && fail "Pi에 연결할 수 없습니다. 로봇 전원과 Wi-Fi(핫스팟)를 확인해 주세요."
  echo "  기다리는 중… ($((i * 5)) s)"; sleep 1
done
echo "  연결됨  $("${SSH[@]}" 'vcgencmd measure_temp; vcgencmd get_throttled' | tr '\n' ' ')"

RESTART=0
if [[ -f $LAUNCH ]]; then
  if [[ "$(md5sum <"$LAUNCH")" != "$("${SSH[@]}" "md5sum <$PI_LAUNCH" 2>/dev/null)" ]]; then
    step "follow.launch.py가 바뀌었다: Pi에 올리고 mr_bringup을 다시 빌드"
    scp -q "$LAUNCH" "$HOST:$PI_LAUNCH" || fail "launch 파일을 올리지 못했습니다."
    "${SSH[@]}" 'set +u; source /opt/ros/jazzy/setup.bash; cd ~/ros2_ws && colcon build --packages-select mr_bringup --parallel-workers 2 2>&1 | tail -3' \
      || fail "Pi에서 mr_bringup 빌드가 실패했습니다."
    RESTART=1
  fi
fi

step "Pi 스택을 내리고 다시 띄운다 (mr-booth.target)"
OUT=$("$ROOT/scripts/robot.sh" ensure 2>&1); echo "$OUT" | sed 's/^/  /'
# 파일이 달라 새로 설치했으면 그때 이미 다시 떴다. 같았으면 여기서 다시 띄운다.
if ! grep -q '새로 올리고' <<<"$OUT"; then
  "$ROOT/scripts/robot.sh" restart 2>&1 | sed 's/^/  /'
  echo "  다시 띄웠다"
fi

step "유닛이 뜨기를 기다린다"
for i in $(seq 40); do
  STATE=$("${SSH[@]}" "for u in $UNITS; do echo \"\$u=\$(systemctl is-active mr-booth@\$u)\"; done" | tr '\n' ' ')
  grep -qv -e '=active' <<<"$(tr ' ' '\n' <<<"$STATE" | grep .)" || break
  sleep 1
done
"$ROOT/scripts/robot.sh" status | sed 's/^/  /'
DOWN=$(tr ' ' '\n' <<<"$STATE" | grep . | grep -v '=active' | tr '\n' ' ')

step "대시보드를 내리고 다시 띄운다"
# 브리지, 전방 카메라, 팔 데몬까지 내린다(팔은 그 자세로 버틴다). 열려 있던 창도 닫는다.
"$ROOT/scripts/stop.sh" --arm | sed 's/^/  /'
pkill -f -- "--user-data-dir=$HOME/.cache/booth-dashboard-chrome" 2>/dev/null && sleep 1
# 8>&-: 대시보드가 브링업 잠금을 물려받으면 다음 브링업이 못 들어온다
setsid "$ROOT/scripts/desktop.sh" >/dev/null 2>&1 </dev/null 8>&- &
for _ in $(seq 120); do curl -sf -o /dev/null --max-time 1 http://127.0.0.1:8080/ && break; sleep 0.5; done
curl -sf -o /dev/null --max-time 1 http://127.0.0.1:8080/ || fail "대시보드가 뜨지 않았습니다. 로그: ~/.cache/booth-dashboard/run.log"
echo "  떴다  http://127.0.0.1:8080/"

[[ -n $DOWN ]] && fail "안 뜬 유닛이 있습니다: $DOWN (scripts/robot.sh log 이름 으로 확인)"
printf '\n\033[1;32m● 준비 끝.\033[0m 이 창은 5초 뒤에 닫힙니다.\n'
notify-send -i "$ROOT/scripts/bringup-icon.svg" "로봇 브링업" "준비 끝" 2>/dev/null
sleep 5
