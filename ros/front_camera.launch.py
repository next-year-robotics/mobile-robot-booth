"""전방 카메라(C920)와 ArUco 검출을 노트북에서 띄운다.

카메라가 Pi가 아니라 노트북 USB에 꽂혀 있을 때 쓴다. 영상은 노트북 안에서만 돌고, Pi로는
/aruco_detections(작은 메시지)만 유선으로 넘어간다. 마커 선택(marker_track)과 추종은 Pi에 그대로 있다.
TF(base_footprint → camera_optical_frame)는 Pi의 robot_state_publisher가 낸 것을 받는다.

설정 파일은 mobile-robot 소스에서 그대로 읽는다(정본이 한 곳에 있게).
usb_cam은 ros_ws/src의 패치본(epoch 변환 오류 수정)을 쓴다. apt 0.8.1 그대로면 stamp가 늦어 추종이 표본을 버린다.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, TimerAction, LogInfo
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_DEVICE = '/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920-video-index0'


def _setup(context, *args, **kwargs):
    del args, kwargs
    src = os.path.expanduser(LaunchConfiguration('mobile_robot').perform(context))
    pkgs = os.path.join(src, 'software', 'ros_pkgs')
    # usb_cam은 심볼릭 링크를 '/dev/' + 링크 내용으로 풀어서 by-id 경로(../../videoN)를 못 연다. 여기서 풀어 준다.
    path = LaunchConfiguration('video_device').perform(context)
    if not os.path.exists(path):
        return [TimerAction(period=1.0, actions=[OpaqueFunction(function=_setup)])]
    device = os.path.realpath(path)
    marker_size = float(LaunchConfiguration('marker_size').perform(context))
    profile = os.path.join(pkgs, 'mr_bringup', 'config', 'fastdds_camera.xml')
    camera = Node(
        package='usb_cam', executable='usb_cam_node_exe', name='usb_cam', namespace='camera',
        output='screen',
        parameters=[os.path.join(pkgs, 'mr_bringup', 'config', 'c920.yaml'), {
            'video_device': device,
            'camera_info_url': 'file://' + os.path.join(pkgs, 'mr_description', 'config', 'c920_calibration.yaml'),
        }],
        additional_env={'FASTDDS_DEFAULT_PROFILES_FILE': profile, 'FASTRTPS_DEFAULT_PROFILES_FILE': profile},
    )
    aruco_params = [os.path.join(pkgs, 'mr_aruco', 'config', 'aruco_tracker.yaml')]
    if marker_size > 0:
        aruco_params.append({'marker_size': marker_size})
    aruco = Node(
        package='aruco_opencv', executable='aruco_tracker_autostart', name='aruco_tracker',
        output='screen', parameters=aruco_params,
    )
    # C920은 AF를 끈 다음 focus_absolute를 따로 넣어야 받는다(mr_bringup launch_common.py와 같은 순서).
    af_off = ExecuteProcess(cmd=['v4l2-ctl', '--device', device, '--set-ctrl', 'focus_automatic_continuous=0'], output='screen')
    # 어두우면 C920이 노출을 늘리느라 15 fps로 떨어진다. 따라가기는 30 fps가 필요해서 그 기능을 끈다(영상은 조금 어두워진다).
    focus = ExecuteProcess(cmd=['v4l2-ctl', '--device', device, '--set-ctrl', 'focus_absolute=0,exposure_dynamic_framerate=0'], output='screen')
    return [
        RegisterEventHandler(OnProcessExit(target_action=af_off, on_exit=[focus])),
        RegisterEventHandler(OnProcessExit(target_action=focus, on_exit=[camera])),
        af_off, aruco,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('video_device', default_value=DEFAULT_DEVICE),
        DeclareLaunchArgument('mobile_robot', default_value='~/dev/mobile-robot'),
        DeclareLaunchArgument('marker_size', default_value='0.0', description='0이면 aruco_tracker.yaml 값'),
        LogInfo(msg=['전방 카메라 장치 대기: ', LaunchConfiguration('video_device')]),
        OpaqueFunction(function=_setup),
    ])
