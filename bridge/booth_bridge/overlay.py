"""카메라 영상 위에 로봇이 실제로 검출한 마커를 다시 그린다.

/aruco_detections에는 모서리 픽셀이 없고 base_footprint 기준 자세만 있다. 그 자세를 카메라 광학
좌표로 되돌려 C920 캘리브레이션(plumb_bob)으로 투영하면 검출기가 본 모서리가 다시 나온다.
노트북에서 따로 검출하지 않으니 화면의 사각형은 로봇의 결과 그대로다.

영상(/camera/image_raw/compressed)과 검출(/aruco_detections)은 따로 도착한다. 둘 다 원본 영상의
header.stamp를 달고 있으므로 FrameSync가 같은 stamp끼리 묶어 내보낸다.
"""
from __future__ import annotations

import math
import threading
from collections import OrderedDict, deque
from dataclasses import dataclass

import numpy as np


def quat_matrix(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class CameraModel:

    def __init__(self, cam_cfg: dict):
        self.origin = np.array(cam_cfg['optical_origin_m'], dtype=float)
        self.lock = threading.Lock()
        self.set_intrinsics(cam_cfg['K'], cam_cfg['D'], cam_cfg['width'], cam_cfg['height'])
        self.from_camera_info = False

    def set_intrinsics(self, K, D, width: int, height: int) -> None:
        with self.lock:
            self.K = np.array(K, dtype=float).reshape(3, 3)
            d = list(D) + [0.0] * (5 - len(D))
            self.D = np.array(d[:5], dtype=float)
            self.width, self.height = int(width), int(height)

    def base_to_optical(self, pts: np.ndarray) -> np.ndarray:
        """base_footprint → camera_optical_frame. URDF의 카메라 장착 rpy가 0이라는 전제다.

        광학 좌표: x 오른쪽 = −y_base, y 아래 = −z_base, z 전방 = x_base.
        """
        d = np.atleast_2d(pts) - self.origin
        return np.stack([-d[:, 1], -d[:, 2], d[:, 0]], axis=1)

    def project(self, pts_base: np.ndarray) -> np.ndarray | None:
        """OpenCV projectPoints와 같은 plumb_bob 식. 카메라 뒤(z ≤ 5 cm)가 있으면 None."""
        p = self.base_to_optical(pts_base)
        if np.any(p[:, 2] <= 0.05):
            return None
        with self.lock:
            K, (k1, k2, p1, p2, k3) = self.K, self.D
        x = p[:, 0] / p[:, 2]
        y = p[:, 1] / p[:, 2]
        r2 = x * x + y * y
        radial = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
        xd = x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
        yd = y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
        u = K[0, 0] * xd + K[0, 1] * yd + K[0, 2]
        v = K[1, 1] * yd + K[1, 2]
        return np.stack([u, v], axis=1)


def marker_overlay(cam: CameraModel, markers, size_m: float, pnp_scale: float = 1.0) -> list[dict]:
    """원시 검출 [(id, p, q), ...] → 화면 좌표 사각형·축.

    ArUco 마커 좌표계의 모서리 순서(좌상, 우상, 우하, 좌하)와 축(x 오른쪽, y 위, z 바깥)을 따른다.
    투영은 원시값으로 해야 검출 모서리와 맞는다. 이름표 거리만 marker_track과 같은 보정을 건
    로봇 중심(base_footprint) 기준 수평 거리다.
    """
    h = size_m / 2.0
    local = np.array([
        [-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0],       # 모서리
        [0, 0, 0], [h, 0, 0], [0, h, 0], [0, 0, h],           # 중심과 축 끝
    ])
    out = []
    for mid, p, q in markers:
        pts = np.asarray(p, dtype=float) + local @ quat_matrix(q).T
        uv = cam.project(pts)
        if uv is None:
            continue
        uv = np.round(uv, 1)
        corrected = cam.origin + pnp_scale * (np.asarray(p, dtype=float) - cam.origin)
        out.append({
            'id': int(mid),
            'corners': uv[:4].tolist(),
            'axes': uv[4:].tolist(),
            'dist': round(float(math.hypot(corrected[0], corrected[1])), 3),
        })
    return out


@dataclass
class Frame:
    stamp_ns: int
    recv: float
    data: bytes
    mime: str


class FrameSync:
    """같은 stamp의 영상과 검출을 묶는다.

    검출이 흐르는 동안에는 영상을 sync_hold_s까지 붙잡고 기다린다. 그 사이 검출이 오면 같이 내보내고,
    안 오면(검출기가 그 프레임을 건너뜀) 오버레이 없이 내보낸다. 검출이 1 s 넘게 안 오면 기다리지 않는다.
    """

    def __init__(self, hold_s: float):
        self.hold_s = hold_s
        self.lock = threading.Lock()
        self.frames: deque[Frame] = deque(maxlen=16)
        self.overlays: OrderedDict[int, list] = OrderedDict()
        self.last_det: float | None = None
        self.latest: Frame | None = None          # 내보냈든 아니든 가장 최근 영상(앞 보기가 쓴다)

    def add_frame(self, frame: Frame) -> None:
        with self.lock:
            self.frames.append(frame)
            self.latest = frame

    def add_overlay(self, stamp_ns: int, now: float, overlay: list) -> None:
        with self.lock:
            self.last_det = now
            self.overlays[stamp_ns] = overlay
            while len(self.overlays) > 64:
                self.overlays.popitem(last=False)

    def pop_ready(self, now: float) -> tuple[Frame, list | None] | None:
        """내보낼 수 있는 가장 최근 프레임 하나. 그보다 오래된 프레임은 버린다."""
        with self.lock:
            det_active = self.last_det is not None and now - self.last_det < 1.0
            chosen = None
            for i, f in enumerate(self.frames):
                ov = self.overlays.get(f.stamp_ns)
                if ov is not None or not det_active or now - f.recv > self.hold_s:
                    chosen = (i, ov)
            if chosen is None:
                return None
            i, ov = chosen
            frame = self.frames[i]
            for _ in range(i + 1):
                self.frames.popleft()
            return frame, ov
