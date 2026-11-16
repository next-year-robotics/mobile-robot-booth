#!/usr/bin/env bash
# 전방 카메라(C920) + ArUco 검출을 노트북에서 띄우고 내린다. config/booth.yaml stack.front_camera가 laptop일 때만 쓴다.
#
#   scripts/front_camera.sh start    이미 떠 있으면 그대로 둔다
#   scripts/front_camera.sh stop
#   scripts/front_camera.sh status
#
# 로그는 ~/.cache/booth-dashboard/front_camera.log. run.sh와 desktop.sh가 start를 부른다.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CACHE="$HOME/.cache/booth-dashboard"; mkdir -p "$CACHE"
PGID_FILE="$CACHE/front_camera.pgid"
running() { [[ -s $PGID_FILE ]] && kill -0 -- "-$(cat "$PGID_FILE")" 2>/dev/null; }
where() { python3 -c "import sys,yaml;print(yaml.safe_load(open(sys.argv[1]))['stack'].get('front_camera','pi'))" "$ROOT/config/booth.yaml"; }

case "${1:-status}" in
start)
  [[ $(where) == laptop ]] || { echo "front_camera: pi (노트북에서는 띄우지 않는다)"; exit 0; }
  running && { echo "전방 카메라 이미 실행 중"; exit 0; }
  # 손으로 띄운 것이 남아 있으면 장치를 못 연다
  pkill -INT -f 'ros/front_camera.launch.py' 2>/dev/null && sleep 1
  set +u
  source /opt/ros/jazzy/setup.bash
  source "$ROOT/ros_ws/install/setup.bash"
  set -u
  export ROS_DOMAIN_ID=0
  SIZE=$(python3 -c "import sys,yaml;print(float(yaml.safe_load(open(sys.argv[1]))['aruco']['marker_size_m']))" "$ROOT/config/booth.yaml")
  setsid nohup ros2 launch "$ROOT/ros/front_camera.launch.py" marker_size:="$SIZE" >"$CACHE/front_camera.log" 2>&1 &
  echo $! >"$PGID_FILE"
  echo "전방 카메라 기동 (로그 $CACHE/front_camera.log)" ;;
stop)
  if running; then kill -INT -- "-$(cat "$PGID_FILE")"; sleep 1; running && kill -TERM -- "-$(cat "$PGID_FILE")"; fi
  pkill -INT -f 'ros/front_camera.launch.py' 2>/dev/null
  rm -f "$PGID_FILE"; echo "전방 카메라 내림" ;;
status)
  if running; then echo "실행 중 (pgid $(cat "$PGID_FILE"))"; else echo "꺼짐"; exit 1; fi ;;
*) echo "start | stop | status"; exit 2 ;;
esac
