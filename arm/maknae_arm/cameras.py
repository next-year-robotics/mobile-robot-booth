"""탑뷰·손목 카메라. 스레드가 최신 프레임만 붙들고 있고, 달라는 쪽이 가져간다."""
from __future__ import annotations

import logging
import os
import subprocess
import threading
import time
from pathlib import Path

import cv2
import numpy as np

log = logging.getLogger('arm.camera')


class Camera:

    def __init__(self, name: str, cfg: dict):
        self.name, self.cfg = name, cfg
        self.frame: np.ndarray | None = None
        self.stamp = 0.0
        self.seq = 0
        self._jpeg: tuple[int, bytes] | None = None
        self._stop = threading.Event()
        self.cond = threading.Condition()
        threading.Thread(target=self._loop, name=f'cam-{name}', daemon=True).start()

    def _open(self):
        dev = os.path.realpath(os.path.expanduser(str(self.cfg['device'])))
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.cfg.get('fourcc', 'MJPG')))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.cfg['width']))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.cfg['height']))
        cap.set(cv2.CAP_PROP_FPS, int(self.cfg.get('fps', 30)))
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not cap.isOpened():
            return None
        controls = self.cfg.get('v4l2_controls')
        if controls:
            settings = ','.join(f'{key}={value}' for key, value in controls.items())
            try:
                subprocess.run(['v4l2-ctl', '-d', dev, '-c', settings], capture_output=True,
                               text=True, check=True, timeout=3)
            except (OSError, subprocess.SubprocessError) as e:
                log.warning('%s 카메라 설정 실패: %s', self.name, getattr(e, 'stderr', None) or e)
        return cap

    def _loop(self) -> None:
        cap = None
        while not self._stop.is_set():
            if cap is None:
                cap = self._open()
                if cap is None:
                    time.sleep(2.0)
                    continue
                log.info('%s 카메라 열림 (%s)', self.name, self.cfg['device'])
            ok, frame = cap.read()
            if not ok:
                log.warning('%s 카메라 읽기 실패, 다시 연다', self.name)
                cap.release()
                cap = None
                time.sleep(1.0)
                continue
            with self.cond:
                self.frame, self.stamp = frame, time.monotonic()
                self.seq += 1
                self.cond.notify_all()
        if cap is not None:
            cap.release()

    def close(self) -> None:
        self._stop.set()

    @property
    def age(self) -> float:
        return time.monotonic() - self.stamp if self.stamp else float('inf')

    def latest(self) -> np.ndarray | None:
        with self.cond:
            return None if self.frame is None or self.age > 2.0 else self.frame.copy()

    def jpeg(self, quality: int = 80) -> bytes | None:
        with self.cond:
            if self.frame is None:
                return None
            if self._jpeg is None or self._jpeg[0] != self.seq:
                ok, buf = cv2.imencode('.jpg', self.frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
                if not ok:
                    return None
                self._jpeg = (self.seq, buf.tobytes())
            return self._jpeg[1]

    def wait_next(self, seq: int, timeout: float = 1.0) -> int:
        with self.cond:
            self.cond.wait_for(lambda: self.seq != seq, timeout)
            return self.seq


class TopView:
    """탑뷰 픽셀 ↔ 팔 좌표. 광선이 작업대 평면과 만나는 점을 쓴다: P = C + (z − Cz) / rz · r"""

    def __init__(self, cfg: dict):
        fs = cv2.FileStorage(str(Path(cfg['intrinsics']).expanduser()), cv2.FILE_STORAGE_READ)
        self.K = fs.getNode('K').mat()
        self.dist = fs.getNode('dist').mat().ravel()
        fs.release()
        self.set_pose(cfg['xyz_m'], cfg['rotation'])

    def set_pose(self, xyz, rotation) -> None:
        self.C = np.asarray(xyz, float)
        self.R = np.asarray(rotation, float)          # 열이 카메라 축의 팔 좌표 방향

    def ray(self, u: float, v: float) -> np.ndarray:
        n = cv2.undistortPoints(np.array([[[u, v]]], np.float64), self.K, self.dist).ravel()
        r = self.R @ np.array([n[0], n[1], 1.0])
        return r / np.linalg.norm(r)

    def to_plane(self, u: float, v: float, z: float) -> np.ndarray:
        r = self.ray(u, v)
        if abs(r[2]) < 1e-6:
            raise ValueError('광선이 평면과 평행하다')
        return self.C + (z - self.C[2]) / r[2] * r

    def project(self, p) -> tuple[float, float]:
        pc = self.R.T @ (np.asarray(p, float) - self.C)
        uv, _ = cv2.projectPoints(pc.reshape(1, 1, 3), np.zeros(3), np.zeros(3), self.K, self.dist)
        return float(uv[0, 0, 0]), float(uv[0, 0, 1])

    def align_screws(self, frame, templates, centers) -> float:
        """고정 나사 세 점의 기준 광선과 현재 광선을 맞춘다. 카메라 중심은 고정이다."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if len(centers) != 3 or templates is None or templates.shape != (21, 63):
            raise ValueError('나사 기준 영상이 없다')
        points = []
        for i, (x, y) in enumerate(centers):
            left, top = max(0, x - 50), max(0, y - 95)
            roi = gray[top:y + 96, left:x + 51]
            patch = templates[:, i * 21:(i + 1) * 21]
            if min(roi.shape) < 21:
                raise ValueError('나사가 화면 밖에 있다')
            scores = cv2.matchTemplate(roi, patch, cv2.TM_CCOEFF_NORMED)
            _, score, _, (u, v) = cv2.minMaxLoc(scores)
            if score < 0.75:
                raise ValueError(f'나사 {i + 1} 매칭 불확실 ({score:.2f})')
            points.append((left + u + 10, top + v + 10))
        old = np.array([self.ray(*p) for p in centers])
        new = np.array([self.ray(*p) for p in points])
        u, _, vt = np.linalg.svd(old.T @ new)
        delta = u @ np.diag([1, 1, np.linalg.det(u @ vt)]) @ vt
        angle = float(np.linalg.norm(cv2.Rodrigues(delta)[0]) * 180 / np.pi)
        residual = np.linalg.norm(new @ delta.T - old, axis=1).max() * max(self.K[0, 0], self.K[1, 1])
        if residual > 3 or angle > 8:
            raise ValueError(f'나사 위치 불일치 ({residual:.1f}px, {angle:.1f}도)')
        self.R = delta @ self.R
        return angle
