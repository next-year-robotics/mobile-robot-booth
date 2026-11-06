"""오버레이 투영이 검출기와 같은 규약인지 확인한다.

핵심 시험은 왕복이다. 알고 있는 자세의 진짜 ArUco 마커를 C920 모델로 그려 OpenCV로 검출·자세 추정하고,
그 자세를 base_footprint로 옮겨 다시 투영했을 때 검출 모서리와 겹쳐야 한다. 모서리 순서나 광학 좌표
변환이 틀리면 여기서 깨진다.
"""
import json
import math
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml

from booth_bridge.overlay import CameraModel, Frame, FrameSync, marker_overlay, quat_matrix

CFG = yaml.safe_load((Path(__file__).resolve().parents[2] / 'config' / 'booth.yaml').read_text())
SIZE = CFG['aruco']['marker_size_m']
# 광학 → base 축: x_b = z_o, y_b = -x_o, z_b = -y_o
OPT_TO_BASE = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)


@pytest.fixture
def cam():
    return CameraModel(CFG['camera'])


def test_project_matches_opencv(cam):
    rng = np.random.default_rng(1)
    pts = np.column_stack([rng.uniform(0.8, 4.0, 50), rng.uniform(-1, 1, 50), rng.uniform(0.0, 1.2, 50)])
    ours = cam.project(pts)
    opt = cam.base_to_optical(pts)
    ref, _ = cv2.projectPoints(opt.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), cam.K, cam.D)
    assert np.abs(ours - ref.reshape(-1, 2)).max() < 1e-6


def test_behind_camera_is_none(cam):
    assert cam.project(np.array([[0.2, 0.0, 0.4]])) is None


def _marker_frame(normal_xy):
    z = np.array([normal_xy[0], normal_xy[1], 0.0]); z /= np.linalg.norm(z)
    y = np.array([0.0, 0.0, 1.0])
    return np.stack([np.cross(y, z), y, z], axis=1)


def _quat(R):
    w = math.sqrt(max(0.0, 1 + np.trace(R))) / 2
    return [(R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w), w]


@pytest.mark.parametrize('pos,normal', [
    ((1.5, 0.0, 0.55), (-1.0, 0.0)),
    ((2.2, 0.5, 1.1), (-1.0, -0.4)),
    ((1.2, -0.35, 0.5), (-0.8, 0.5)),
])
def test_roundtrip_with_real_detector(cam, pos, normal):
    p = np.array(pos)
    R = _marker_frame(np.array(normal))
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker = np.full((240, 240), 255, np.uint8)
    marker[30:210, 30:210] = cv2.aruco.drawMarker(d, 7, 180)
    h = SIZE / 2 * 240 / 180
    outer = p + np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]]) @ R.T
    H = cv2.getPerspectiveTransform(np.float32([[0, 0], [240, 0], [240, 240], [0, 240]]),
                                    cam.project(outer).astype(np.float32))
    img = cv2.warpPerspective(marker, H, (cam.width, cam.height), borderValue=90)

    params = cv2.aruco.DetectorParameters_create()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    corners, ids, _ = cv2.aruco.detectMarkers(img, d, parameters=params)
    assert ids is not None and int(ids[0][0]) == 7
    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, SIZE, cam.K, cam.D)
    R_opt, _ = cv2.Rodrigues(rvec[0])
    p_base = cam.origin + OPT_TO_BASE @ tvec[0].ravel()
    R_base = OPT_TO_BASE @ R_opt

    ov = marker_overlay(cam, [(7, p_base.tolist(), _quat(R_base))], SIZE)
    assert len(ov) == 1
    got = np.array(ov[0]['corners'])
    assert np.abs(got - corners[0].reshape(4, 2)).max() < 1.5
    # 축과 부호가 맞는지 본다. 50 px짜리 합성 마커의 PnP 거리 잡음은 거리의 2 %까지 허용한다
    assert np.linalg.norm(p_base - p) < 0.02 * np.linalg.norm(p - cam.origin)
    assert math.degrees(math.acos(np.clip(R_base[:, 2] @ R[:, 2], -1, 1))) < 3.0
    json.dumps(ov)


def test_quat_identity():
    assert np.allclose(quat_matrix([0, 0, 0, 1]), np.eye(3))


def test_frame_sync_pairs_by_stamp():
    s = FrameSync(hold_s=0.25)
    s.add_overlay(1, 0.0, [])            # 검출이 흐르는 중
    s.add_frame(Frame(10, 0.00, b'a', 'image/jpeg'))
    s.add_frame(Frame(20, 0.03, b'b', 'image/jpeg'))
    assert s.pop_ready(0.05) is None      # 둘 다 검출을 기다린다
    s.add_overlay(20, 0.06, [{'id': 0}])
    f, ov = s.pop_ready(0.06)
    assert f.stamp_ns == 20 and ov == [{'id': 0}]   # 더 새 프레임이 짝을 찾으면 앞 것은 버린다
    assert s.pop_ready(0.07) is None


def test_frame_sync_gives_up_after_hold_and_without_detector():
    s = FrameSync(hold_s=0.25)
    s.add_overlay(1, 0.0, [])
    s.add_frame(Frame(10, 0.0, b'a', 'image/jpeg'))
    assert s.pop_ready(0.1) is None
    f, ov = s.pop_ready(0.3)
    assert f.stamp_ns == 10 and ov is None
    s.add_frame(Frame(30, 5.0, b'c', 'image/jpeg'))   # 검출이 1 s 넘게 없으면 기다리지 않는다
    f, ov = s.pop_ready(5.0)
    assert f.stamp_ns == 30 and ov is None
