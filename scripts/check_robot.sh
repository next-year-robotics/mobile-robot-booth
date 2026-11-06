#!/usr/bin/env bash
# 부스 시작 전 점검. 읽기만 하고 아무것도 바꾸지 않는다.
#
#   scripts/check_robot.sh             노트북에서 보이는 토픽 + Pi 상태(ssh rpi5)
#   scripts/check_robot.sh --peer IP   멀티캐스트가 막힌 네트워크
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HOST=rpi5
[[ "${1:-}" == "--peer" ]] && export ROS_STATIC_PEERS="$2"
set +u
source /opt/ros/jazzy/setup.bash
source "$ROOT/ros_ws/install/setup.bash"
set -u
export ROS_DOMAIN_ID=0

ok()   { printf '  \033[37m●\033[0m %s\n' "$*"; }
bad()  { printf '  \033[38;5;208m●\033[0m %s\n' "$*"; }

echo "[노트북에서 보이는 토픽]  ROS_DOMAIN_ID=0 ${ROS_STATIC_PEERS:+ROS_STATIC_PEERS=$ROS_STATIC_PEERS}"
ros2 daemon stop >/dev/null 2>&1
LIST="$(timeout 10 ros2 topic list --no-daemon 2>/dev/null)"
for t in /joint_states /base/status /imu/data_raw /odom /camera/image_raw/compressed /camera/camera_info \
         /aruco_detections /marker_track /cmd_vel /phone/link_status /robot/app_status; do
  if grep -qx "$t" <<<"$LIST"; then
    n="$(timeout 6 ros2 topic info "$t" --no-daemon 2>/dev/null | awk '/Publisher count/ {print $3}')"
    if [[ "${n:-0}" -gt 0 ]]; then ok "$t  발행자 $n"; else bad "$t  이름만 있고 발행자 0 (도메인이 갈렸거나 노드가 꺼짐)"; fi
  else
    case "$t" in
      /camera/image_raw/compressed) bad "$t  없음 → Pi에 ros-jazzy-image-transport-plugins 필요(아래 Pi 점검)";;
      /phone/link_status|/robot/app_status) bad "$t  없음 → 폰 브리지·임무 노드가 안 떠 있음";;
      *) bad "$t  없음";;
    esac
  fi
done

echo
echo "[노트북 커널]"
# 영상 한 장(약 50 KB)이 IP 조각 37개로 온다. 조각 하나만 빠져도 남은 조각이 ipfrag_time 동안 메모리를 잡고,
# 기본값(30 s, 4 MB)이면 덜 온 프레임 24장에 꽉 차서 그 뒤 조각을 전부 버린다. 영상이 1~3 fps로 주저앉는다.
ft=$(sysctl -n net.ipv4.ipfrag_time); fh=$(sysctl -n net.ipv4.ipfrag_high_thresh)
if [[ $ft -le 5 && $fh -ge 33554432 ]]; then ok "IP 조각 재조립 ipfrag_time=${ft}s high_thresh=$((fh >> 20))MB"
else bad "IP 조각 재조립 ipfrag_time=${ft}s high_thresh=$((fh >> 20))MB → 영상이 끊긴다. 아래 한 줄을 실행:"
  echo "      printf 'net.ipv4.ipfrag_time = 3\\nnet.ipv4.ipfrag_high_thresh = 33554432\\n' | sudo tee /etc/sysctl.d/90-booth-ipfrag.conf && sudo sysctl -p /etc/sysctl.d/90-booth-ipfrag.conf"
fi

echo
echo "[Pi]  ssh $HOST"
if ! timeout 6 ssh -o BatchMode=yes -o ConnectTimeout=4 "$HOST" true 2>/dev/null; then
  bad "ssh 안 됨 (rpi5.local mDNS가 막힌 네트워크면 ~/.ssh/config의 HostName을 IP로)"
  exit 0
fi
ssh -o BatchMode=yes "$HOST" 'bash -s' <<'REMOTE'
ok()  { printf '  \033[37m●\033[0m %s\n' "$*"; }
bad() { printf '  \033[38;5;208m●\033[0m %s\n' "$*"; }
ip -4 -o addr show wlan0 2>/dev/null | awk '{print "  · wlan0 " $4}'
t=$(vcgencmd get_throttled 2>/dev/null | cut -d= -f2); c=$(vcgencmd measure_temp 2>/dev/null | cut -d= -f2)
if [[ "$t" == "0x0" ]]; then ok "전원·온도 정상 ($c)"; else bad "throttled=$t ($c)  0x1 저전압 지금, 0x10000 저전압 이력"; fi
if dpkg -l ros-jazzy-compressed-image-transport 2>/dev/null | grep -q '^ii'; then ok "compressed_image_transport 설치됨"
else bad "compressed_image_transport 없음 → sudo apt install ros-jazzy-image-transport-plugins (설치 뒤 카메라 launch 재시작)"; fi
if [[ -s ~/.config/mr_care_telemetry/service-account.json ]]; then ok "텔레메트리 서비스 계정 있음"
else bad "텔레메트리 서비스 계정 없음 → 패드 대시보드가 로봇을 오프라인으로 그린다 (노트북 ~/.config/mr_care_telemetry/service-account.json 복사)"; fi
en=$(systemctl is-enabled mr-booth.target 2>/dev/null)
[[ "$en" == enabled ]] && ok "mr-booth.target 자동 기동 enabled" || bad "mr-booth.target $en → scripts/robot.sh install"
for u in follow phone_bridge mission telemetry pad; do
  st=$(systemctl is-active "mr-booth@$u" 2>/dev/null)
  [[ "$st" == active ]] && ok "mr-booth@$u active" || bad "mr-booth@$u $st → scripts/robot.sh log $u"
done
source /opt/ros/jazzy/setup.bash; source ~/ros2_ws/install/setup.bash; export ROS_DOMAIN_ID=0
id=$(timeout 8 ros2 param get /marker_track target_marker_id 2>/dev/null | awk '{print $NF}')
want=$(sed -n 's/^MARKER_ID=//p' ~/booth/booth.env 2>/dev/null)
[[ -n "$id" && "$id" == "${want:-2}" ]] && ok "marker_track 대상 ID $id" || bad "marker_track 대상 ID ${id:-?} (booth.env ${want:-?})"
if bluetoothctl devices Connected 2>/dev/null | grep -qi xbox; then ok "Xbox 패드 연결"; else bad "Xbox 패드 안 붙음 → 패드 가운데 Xbox 버튼"; fi
for p in micro_ros_agent usb_cam_node_exe aruco_tracker marker_track phone_bridge mission care_telemetry pad_trip; do
  n=$(pgrep -fc "$p" 2>/dev/null); n=$((n));
  if [[ $n -gt 0 ]]; then ok "$p 실행 중 ($n)"; else bad "$p 안 떠 있음"; fi
done
[[ $(pgrep -xc micro_ros_agent) -gt 1 ]] && bad "micro_ros_agent가 둘 이상 (한 시리얼에 둘이 붙음)"
REMOTE
