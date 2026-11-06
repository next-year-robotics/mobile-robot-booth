#!/usr/bin/env bash
# Pi 시연 스택. 부팅하면 systemd(mr-booth.target)가 알아서 띄운다. 이 스크립트는 설치와 손 조작용이다. ssh rpi5로 돈다.
#
#   scripts/robot.sh install       robot/ 파일을 Pi ~/booth에 올리고 systemd 유닛을 설치·enable한다(sudo)
#   scripts/robot.sh up            스택 올리기 (부팅 때는 저절로 뜬다)
#   scripts/robot.sh down          스택 내리기
#   scripts/robot.sh restart [NAME]  전부 또는 하나만 (follow, phone_bridge, mission, telemetry, pad)
#   scripts/robot.sh status        유닛 상태, 전원, 최근 경고
#   scripts/robot.sh places        config/places.yaml을 앱 장소 목록으로 올리고 폰 브리지를 다시 띄운다
#   scripts/robot.sh log NAME      로그 따라 보기
#   scripts/robot.sh mcu-reset     ST-Link로 MCU 리셋 (지령을 안 받는 fault latch 복구)
#   scripts/robot.sh poweroff      Pi 끄기. LED가 꺼진 뒤 배터리 스위치를 내린다
#
# follow는 꺼진 채 뜬다. 패드 A·앱·대시보드에서 출발하면 임무 노드가 켠다. 패드는 LB를 쥐는 동안 넘겨받는다.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HOST=rpi5
CMD="${1:-status}"
NAME="${2:-}"
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=6 "$HOST")

case "$CMD" in
log)
  exec ssh -t "$HOST" "journalctl -n 60 -f -o cat -u mr-booth@${NAME:?이름: follow, phone_bridge, mission, telemetry, pad}" ;;
install)
  # booth.yaml stack 절·aruco.marker_size_m → booth.env (stack.sh가 읽는다)
  python3 - "$ROOT/config/booth.yaml" >"$ROOT/robot/booth.env" <<'PY' || exit 1
import sys, yaml
c = yaml.safe_load(open(sys.argv[1], encoding='utf-8'))
s = c['stack']
print('# scripts/robot.sh install이 config/booth.yaml에서 만든다. 여기를 고치지 말 것.')
print(f"MARKER_ID={int(s['marker_id'])}\nMARKER_SIZE={float(c['aruco']['marker_size_m'])}")
print(f"TRIP_FROM={s['trip_from']}\nTRIP_TO={s['trip_to']}")
PY
  "${SSH[@]}" 'mkdir -p ~/booth' || exit 1
  scp -q "$ROOT/robot/stack.sh" "$ROOT/robot/pad_trip.py" "$ROOT/robot/booth.env" \
    "$ROOT/robot/mr-booth@.service" "$ROOT/robot/mr-booth.target" "$HOST:booth/" || exit 1
  ;;
places)
  # 앱이 바뀐 목록으로 알아보게 revision을 지금 시각으로 새로 붙인다
  python3 - "$ROOT/config/places.yaml" <<'PY' | "${SSH[@]}" 'mkdir -p ~/booth && cat > ~/booth/places.yaml' || exit 1
import sys, datetime, yaml
doc = yaml.safe_load(open(sys.argv[1], encoding='utf-8'))
now = datetime.datetime.now(datetime.timezone.utc)
doc['revision'] = now.strftime('%Y-%m-%dT%H:%M:%S.') + f'{now.microsecond // 1000:03d}Z'
sys.stdout.write(yaml.safe_dump(doc, allow_unicode=True, sort_keys=False, default_flow_style=False))
PY
  ;;
esac

"${SSH[@]}" 'bash -s' -- "$CMD" "$NAME" <<'REMOTE'
CMD="$1"; NAME="$2"
B=~/booth
UNITS=(follow phone_bridge mission telemetry pad)
unit() { echo "mr-booth@$1.service"; }

# 예전 방식(setsid + PGID 파일)으로 띄운 스택. 설치할 때 한 번 내린다.
legacy_down() {
  local boot g s found=0
  boot=$(cat /proc/sys/kernel/random/boot_id)
  for s in mission phone_bridge telemetry follow; do
    [[ -f "$B/$s.pgid" ]] || continue
    found=1
    read -r g b <"$B/$s.pgid" || continue
    if [[ $b == "$boot" ]] && pgrep -g "$g" >/dev/null; then
      kill -INT -"$g"
      for _ in $(seq 40); do pgrep -g "$g" >/dev/null || break; sleep 0.25; done
      pgrep -g "$g" >/dev/null && kill -KILL -"$g"
      echo "예전 방식 스택 내림: $s"
    fi
    rm -f "$B/$s.pgid"
  done
  # systemd follow 유닛의 에이전트는 건드리지 않는다. 예전 방식 스택이 있었을 때만 남은 것을 지운다.
  if [[ $found == 1 ]]; then
    for p in $(pgrep -x micro_ros_agent); do echo "남은 micro_ros_agent $p 정리"; kill -TERM "$p"; done
  fi
}

case "$CMD" in
install)
  chmod +x "$B/stack.sh" "$B/pad_trip.py"
  legacy_down
  sudo install -m 644 "$B/mr-booth@.service" "$B/mr-booth.target" /etc/systemd/system/
  sudo systemctl daemon-reload
  sudo systemctl enable mr-booth.target
  sudo systemctl restart mr-booth.target
  echo "설치·enable 끝. 부팅하면 mr-booth.target이 뜬다."
  ;;
up) sudo systemctl start mr-booth.target ;;
down) sudo systemctl stop mr-booth.target ;;
restart)
  if [[ -z "$NAME" || "$NAME" == all ]]; then sudo systemctl restart mr-booth.target
  else sudo systemctl restart "$(unit "$NAME")"; fi
  ;;
places)
  # 브리지가 파일 주인이라 내린 상태에서 바꾼다. 앱에서 고친 목록은 .booth-bak에 남긴다.
  P=~/ros2_ws/config/places.yaml; mkdir -p "$(dirname "$P")"
  sudo systemctl stop "$(unit phone_bridge)"
  [[ -f "$P" ]] && cp "$P" "$P.booth-bak"
  mv "$B/places.yaml" "$P"
  sudo systemctl start "$(unit phone_bridge)"
  sleep 8; journalctl -o cat -u "$(unit phone_bridge)" --since -15s | grep -a 'places:' | tail -1 | cut -c1-120
  ;;
mcu-reset) st-flash reset 2>&1 | tail -2 ;;
poweroff) echo "Pi를 끈다. 초록 LED가 꺼지면 배터리 스위치를 내린다."; sudo systemctl poweroff ;;
status)
  for s in "${UNITS[@]}"; do
    st=$(systemctl is-active "$(unit "$s")")
    case $st in active) m="●";; *) m="○";; esac
    echo "$m $s  $st"
    # 템플릿의 After=mr-booth@follow가 follow 자신에게 붙어 버려졌다는 경고는 정상이다
    journalctl -o cat -u "$(unit "$s")" -b -p warning --no-pager 2>/dev/null | grep -v 'is dropped' | tail -2 | cut -c1-160 | sed 's/^/    /'
  done
  echo "enabled $(systemctl is-enabled mr-booth.target 2>/dev/null)  throttled $(vcgencmd get_throttled | cut -d= -f2)  $(vcgencmd measure_temp)  load $(cut -d' ' -f1 /proc/loadavg)"
  ;;
*) echo "install | up | down | restart [NAME] | status | places | log NAME | mcu-reset | poweroff"; exit 2 ;;
esac
REMOTE
