"""작업대 높이를 작업대 위에 올려 둔 ArUco 마커로 잰다.

작업대마다 높이가 다르다. 한 변 size_m인 마커(기본 ID 10, 8 cm)를 작업대 위에 눕혀 두면 탑뷰 카메라가 그 마커까지의
거리를 크기로 재고(PnP), 보정해 둔 카메라 자세로 팔 좌표의 z를 구한다. 그 z에서 마커 판 두께를 빼면 작업대 윗면 높이다.

- 마커가 없는 작업대도 있다. 안 보이면 None을 돌려주고, 쓰는 쪽(skills.py)이 현재 장면으로 받침면 높이를 추정한다.
- 한 장으로는 거리 잡음이 크다(8 cm 마커가 화면에서 50 px 남짓이라 0.5 px이 약 8 mm). 여러 장의 가운데값을 쓴다.
- 눕혀 둔 마커만 믿는다. 세워져 있거나(다른 용도의 마커) 높이가 말이 안 되면 버린다.
"""
from __future__ import annotations

import logging
import math
import time

import cv2
import numpy as np

log = logging.getLogger('arm.table')

DICTS = {'4X4_50': cv2.aruco.DICT_4X4_50, '4X4_100': cv2.aruco.DICT_4X4_100, '5X5_50': cv2.aruco.DICT_5X5_50}


class TableMarker:

    def __init__(self, cfg: dict, view):
        """cfg: workspace.table_marker. view: cameras.TopView(K, dist, C, R)."""
        self.view = view
        self.id = int(cfg.get('id', 10))
        self.size = float(cfg.get('size_m', 0.08))
        self.thickness = float(cfg.get('thickness_m', 0.0))
        self.offset = float(cfg.get('z_offset_m', 0.0))
        self.z_range = tuple(float(v) for v in cfg.get('z_range_m', (-0.08, 0.30)))
        self.max_tilt = math.radians(float(cfg.get('max_tilt_deg', 20.0)))
        self.frames = int(cfg.get('frames', 7))
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(DICTS.get(str(cfg.get('dictionary', '4X4_50')), cv2.aruco.DICT_4X4_50)), params)
        h = self.size / 2
        self._obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float64)

    def detect(self, frame: np.ndarray) -> dict | None:
        """한 장에서 마커를 찾는다. {'z', 'xy', 'tilt_deg', 'corners'} 또는 None."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return None
        for c, i in zip(corners, ids.ravel()):
            if int(i) != self.id:
                continue
            px = c.reshape(4, 2).astype(np.float64)
            ok, rvec, tvec = cv2.solvePnP(self._obj, px, self.view.K, self.view.dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                return None
            p = self.view.C + self.view.R @ tvec.ravel()
            normal = self.view.R @ cv2.Rodrigues(rvec)[0][:, 2]       # 마커 면에서 나오는 쪽
            tilt = math.acos(max(-1.0, min(1.0, abs(float(normal[2])))))
            return {'z': float(p[2]) - self.thickness + self.offset, 'xy': (float(p[0]), float(p[1])), 'normal': normal, 'tilt_deg': math.degrees(tilt), 'corners': px}
        return None

    def valid(self, d: dict | None) -> bool:
        return d is not None and math.radians(d['tilt_deg']) <= self.max_tilt and self.z_range[0] <= d['z'] <= self.z_range[1]

    def measure(self, camera, timeout: float = 1.2) -> dict | None:
        """새 프레임 여러 장으로 잰다. 절반 넘게 보여야 믿는다. {'z', 'xy', 'n', 'spread_m', 'corners'} 또는 None."""
        seen, tried, last = [], 0, None
        seq = getattr(camera, 'seq', 0)
        end = time.monotonic() + timeout
        while tried < self.frames and time.monotonic() < end:
            frame = camera.latest()
            if frame is None:
                break
            tried += 1
            d = self.detect(frame)
            if self.valid(d):
                seen.append(d)
                last = d
            elif d is not None:
                log.info('작업대 마커를 버린다: 기울기 %.0f°, z %.3f', d['tilt_deg'], d['z'])
            if tried == 2 and not seen:
                break                                              # 없는 작업대다. 오래 붙들지 않는다
            seq = camera.wait_next(seq, 0.3)
        if not seen or len(seen) * 2 < tried:
            return None
        zs = sorted(d['z'] for d in seen)
        slopes = np.array([-d['normal'][:2] / d['normal'][2] for d in seen])
        slope = np.median(slopes, axis=0)
        # 평면 자세의 두 해 사이에서 흔들리면 높이만 사용한다.
        consistent = len(seen) >= 3 and np.max(np.linalg.norm(slopes - slope, axis=1)) < 0.10
        return {'slope': slope.tolist() if consistent else None, 'z': round(zs[len(zs) // 2], 4), 'xy': last['xy'], 'n': len(seen), 'spread_m': round(zs[-1] - zs[0], 4),
                'corners': last['corners']}
