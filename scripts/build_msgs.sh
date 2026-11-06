#!/usr/bin/env bash
# 노트북이 Pi 토픽을 디코드하려면 사용자 메시지 타입이 있어야 한다.
# mobile-robot의 메시지 패키지 셋을 이 리포의 ros_ws/install에 빌드한다(mobile-robot의 install은 건드리지 않는다).
# 메시지 정의가 바뀌면(예: BaseStatus에 필드 추가) 다시 돌린다.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="${MOBILE_ROBOT:-$HOME/dev/mobile-robot}/software/ros_pkgs"
set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "$ROOT/ros_ws"
colcon build \
  --paths "$SRC/mr_msgs" "$SRC/mr_aruco_msgs" "$SRC/mr_care_msgs" \
  --build-base build --install-base install \
  --cmake-args -DBUILD_TESTING=OFF
