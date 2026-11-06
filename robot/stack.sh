#!/usr/bin/env bash
# mr-booth@NAME.service가 부르는 실행기. Pi ~/booth/stack.sh에 놓인다(scripts/robot.sh install).
#
#   follow        베이스(micro-ROS 에이전트·오도메트리)·카메라·ArUco·twist_mux·추종(꺼진 채)·Xbox 패드 수동
#   phone_bridge  폰 앱 WebSocket
#   mission       앱·패드의 출발/멈춤 → follow 켜고 끄기
#   telemetry     보호자 대시보드(Firebase)
#   pad           패드 A 출발 · B 멈춤 · View+Menu 3 s Pi 끄기, 마커 크기 맞추기, 시계 점프 복구 (pad_trip.py)
#
# 끌 때는 systemd가 이 유닛의 cgroup을 통째로 정리한다. ssh로 띄울 때처럼 고아 노드나 에이전트가 남지 않는다.
set -uo pipefail
NAME="${1:?이름: follow, phone_bridge, mission, telemetry, pad}"
B="$HOME/booth"
set +u
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
set -u
export ROS_DOMAIN_ID=0
export PYTHONUNBUFFERED=1
# MARKER_ID, MARKER_SIZE, TRIP_FROM, TRIP_TO (config/booth.yaml에서 만든다)
[[ -f "$B/booth.env" ]] && source "$B/booth.env"

# DDS는 노드가 뜰 때 있던 네트워크 주소만 알린다. Wi-Fi 주소 없이 뜨면 나중에 핫스팟이 붙어도 노트북에서
# 토픽이 안 보인다. 핫스팟이 늦게 켜졌으면 대시보드에서 스택 재시작.
for _ in $(seq 60); do ip -4 -o addr show wlan0 2>/dev/null | grep -q inet && break; sleep 1; done
ip -4 -o addr show wlan0 2>/dev/null | grep -q inet || echo "wlan0 주소 없이 띄운다 (Wi-Fi가 붙은 뒤 재시작해야 노트북에서 보인다)"

# Pi에는 RTC 전지가 없어 부팅 시계가 수십 분 밀려 있고, Wi-Fi가 붙으면 chrony가 한 번에 옮긴다(makestep).
# usb_cam은 벽시계-단조시계 차이를 기동할 때 한 번만 재므로, 그 전에 뜨면 영상 stamp가 옮긴 만큼 과거에
# 찍히고 follow가 모든 마커 표본을 낡았다고 버린다(2026-10-31 A를 눌러도 안 따라감, 3141 s). 맞춰질 때까지 기다린다.
# 인터넷이 없어 끝내 못 맞추면 그냥 띄운다. 나중에 시계가 뛰면 pad_trip.py가 follow를 다시 띄운다.
# pad도 기다리는 이유: 동기화 전에 뜨면 기동 중의 정상 동기화를 점프로 보고 follow를 괜히 한 번 더 띄운다.
if [[ $NAME == follow || $NAME == pad ]]; then
  chronyc waitsync 30 0.5 0 1 >/dev/null 2>&1 || echo "시계 동기화 안 됨 (30 s), 그대로 띄운다"
fi

# 베이스와 카메라가 장치와 CPU를 먼저 잡게 둔다
[[ $NAME != follow ]] && sleep 4

case "$NAME" in
follow)
  # 손으로 띄운 에이전트가 남아 있으면 한 시리얼에 둘이 붙는다
  for p in $(pgrep -x micro_ros_agent); do echo "남은 micro_ros_agent $p 정리"; kill -TERM "$p"; done
  exec ros2 launch mr_bringup follow.launch.py marker_id:="${MARKER_ID:-2}" ;;
phone_bridge)
  exec ros2 launch mr_phone_bridge phone_bridge.launch.py ;;
mission)
  # 부스에서는 앱 끊김 자동 정지(link_lost_stop_s 30 s)를 끈다. 폰 앱이 안 붙어 있어도 패드로 출발한 주행이
  # 이어진다. 정지는 패드 B·LB와 대시보드가 맡는다. SOS(마커 10 s 놓침)는 mission.yaml 그대로다.
  # ros2 run을 거치면 systemd의 INT가 노드까지 안 가서 끌 때마다 12 s를 기다렸다. 실행 파일을 바로 띄운다.
  exec "$HOME/ros2_ws/install/mr_mission/lib/mr_mission/mission" --ros-args \
    --params-file "$HOME/ros2_ws/install/mr_mission/share/mr_mission/config/mission.yaml" \
    -p link_lost_stop_s:=0.0 ;;
telemetry)
  if [[ ! -s ~/.config/mr_care_telemetry/service-account.json ]]; then
    echo "서비스 계정 없음: 텔레메트리 건너뜀 (패드 대시보드에 로봇이 안 뜬다)"; exit 0
  fi
  exec ros2 launch mr_care_telemetry care_telemetry.launch.py ;;
pad)
  exec python3 "$B/pad_trip.py" --ros-args \
    -p from_place:="${TRIP_FROM:-place_home}" -p to_place:="${TRIP_TO:-place_senior_center}" \
    -p marker_size:="${MARKER_SIZE:-0.0}" ;;
*)
  echo "모르는 이름: $NAME"; exit 2 ;;
esac
