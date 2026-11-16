"""2D 라이다 스캔을 3D 화면에 그릴 모양으로 다듬는다. ROS를 import하지 않는다.

한 바퀴(/scan)마다:
1. 거리 한계 밖과 로봇 자기 몸에 맞은 점을 버린다.
2. 이웃 셋의 중앙값으로 튀는 점 하나짜리 잡음을 지운다.
3. laser_link → base_footprint로 옮긴다(config lidar.xyz_m, rpy_deg의 yaw).
4. 이웃 점 사이가 벌어지면 끊어 덩어리로 나누고, 덩어리마다 꺾은선으로 줄인다(Douglas-Peucker).
   화면은 이 꺾은선을 세워 벽으로 그린다. 점이 몇 개 안 되는 덩어리(다리, 의자 다리)는 기둥으로 남는다.

브라우저에는 mm 단위 int16로 보낸다(720점 한 바퀴가 3 kB쯤).
"""
from __future__ import annotations

import math

import numpy as np


class ScanShaper:

    def __init__(self, cfg: dict):
        xyz = cfg.get('xyz_m', [0.0, 0.0, 0.25])
        self.x, self.y, self.z = float(xyz[0]), float(xyz[1]), float(xyz[2])
        self.yaw = math.radians(float(cfg.get('rpy_deg', [0, 0, 0])[2]))
        self.self_radius = float(cfg.get('self_radius_m', 0.20))
        # 차체 틀(기둥)에 맞은 점. base_footprint 기준 상자 [x0, x1, y0, y1] 안이면 버린다
        box = cfg.get('self_box_m')
        self.self_box = None if not box else tuple(float(v) for v in box)
        self.max_range = float(cfg.get('max_range_m', 8.0))
        self.max_points = int(cfg.get('max_points', 720))
        self.gap_m = float(cfg.get('gap_m', 0.12))             # 이웃 점이 이보다 멀면 다른 물체
        self.gap_ratio = float(cfg.get('gap_ratio', 0.06))     # 멀리서는 점 간격이 벌어지므로 거리에 비례해 늘린다
        self.simplify_m = float(cfg.get('simplify_m', 0.03))
        self.min_wall_points = int(cfg.get('min_wall_points', 4))

    def shape(self, ranges, angle_min: float, angle_increment: float,
              range_min: float, range_max: float) -> dict:
        """→ {'points': (N,2) float m, 'lines': [꺾은선 (K,2)...], 'posts': [(x,y)...], 'nearest': (d, x, y) | None}"""
        r = np.asarray(ranges, dtype=float)
        n = len(r)
        if n == 0:
            return {'points': np.zeros((0, 2)), 'lines': [], 'posts': [], 'nearest': None}
        if n > self.max_points:                                 # 촘촘한 라이다는 솎는다
            step = int(math.ceil(n / self.max_points))
            r, angle_increment, n = r[::step], angle_increment * step, len(r[::step])
        hi = min(range_max, self.max_range)
        ok = np.isfinite(r) & (r > max(range_min, self.self_radius)) & (r < hi)
        r = np.where(ok, r, np.nan)
        # 중앙값 3. 한 점만 튄 것을 이웃 값으로 누른다. 가장자리(이웃이 없는 점)는 그대로 둔다.
        stack = np.stack([np.roll(r, 1), r, np.roll(r, -1)])
        valid = np.isfinite(stack).sum(axis=0)
        med = np.where(valid == 3, np.median(stack, axis=0), r)
        # 이웃이 둘 다 없는 외톨이 점은 잡음으로 본다
        r = np.where(valid >= 2, med, np.nan)

        a = angle_min + angle_increment * np.arange(n) + self.yaw
        idx = np.flatnonzero(np.isfinite(r))
        rr = r[idx]
        pts = np.stack([self.x + rr * np.cos(a[idx]), self.y + rr * np.sin(a[idx])], axis=1)
        if self.self_box is not None and len(pts):
            x0, x1, y0, y1 = self.self_box
            out = ~((pts[:, 0] > x0) & (pts[:, 0] < x1) & (pts[:, 1] > y0) & (pts[:, 1] < y1))
            idx, rr, pts = idx[out], rr[out], pts[out]

        lines, posts = [], []
        if len(idx):
            # 덩어리 나누기: 각도 번호가 건너뛰었거나 이웃 점이 멀면 끊는다
            step_d = np.hypot(*(pts[1:] - pts[:-1]).T) if len(pts) > 1 else np.zeros(0)
            limit = np.maximum(self.gap_m, self.gap_ratio * np.minimum(rr[1:], rr[:-1])) if len(pts) > 1 else np.zeros(0)
            cut = np.flatnonzero((np.diff(idx) > 3) | (step_d > limit)) + 1
            groups = np.split(np.arange(len(pts)), cut)
            # 한 바퀴의 처음과 끝이 이어지면 붙인다
            if len(groups) > 1 and idx[0] + n - idx[-1] <= 3 and \
                    np.hypot(*(pts[0] - pts[-1])) <= max(self.gap_m, self.gap_ratio * min(rr[0], rr[-1])):
                groups[0] = np.concatenate([groups.pop(), groups[0]])
            for g in groups:
                if len(g) >= self.min_wall_points:
                    lines.append(simplify(pts[g], self.simplify_m))
                else:
                    posts.append(tuple(pts[g].mean(axis=0)))
        nearest = None
        if len(pts):
            d = np.hypot(pts[:, 0], pts[:, 1])
            i = int(np.argmin(d))
            nearest = (float(d[i]), float(pts[i, 0]), float(pts[i, 1]))
        return {'points': pts, 'lines': lines, 'posts': posts, 'nearest': nearest}

    def pack(self, shaped: dict, pose: tuple[float, float, float] | None,
             odom: tuple[float, float, float] | None = None) -> tuple[dict, bytes]:
        """브라우저로 보낼 머리(JSON)와 본문(int16 mm: 점 x y …).

        pose는 흔적을 쌓을 좌표(EKF)의 로봇 자세, odom은 같은 순간 화면이 로봇을 그리는 /odom 자세다.
        """
        r3 = lambda p: None if p is None else [round(p[0], 4), round(p[1], 4), round(p[2], 5)]  # noqa: E731
        mm = lambda v: int(round(float(v) * 1000))  # noqa: E731
        header = {
            'type': 'scan', 'n': len(shaped['points']), 'z': self.z,
            'pose': r3(pose), 'odom': r3(odom if odom is not None else pose),
            'lines': [[mm(v) for v in line.flatten()] for line in shaped['lines']],
            'posts': [[mm(x), mm(y)] for x, y in shaped['posts']],
            'nearest': None if shaped['nearest'] is None else [round(v, 3) for v in shaped['nearest']],
        }
        body = np.clip(np.round(shaped['points'] * 1000), -32768, 32767).astype('<i2').tobytes()
        return header, body


def simplify(points: np.ndarray, eps: float) -> np.ndarray:
    """Douglas-Peucker. 순서가 있는 점열에서 직선에서 eps 넘게 벗어난 점만 남긴다."""
    if len(points) <= 2:
        return points
    keep = np.zeros(len(points), dtype=bool)
    keep[0] = keep[-1] = True
    stack = [(0, len(points) - 1)]
    while stack:
        i, j = stack.pop()
        if j - i < 2:
            continue
        a, b = points[i], points[j]
        seg = b - a
        length = float(np.hypot(*seg))
        inner = points[i + 1:j]
        if length < 1e-9:
            dist = np.hypot(*(inner - a).T)
        else:
            dist = np.abs(seg[0] * (inner[:, 1] - a[1]) - seg[1] * (inner[:, 0] - a[0])) / length
        k = int(np.argmax(dist))
        if dist[k] > eps:
            k += i + 1
            keep[k] = True
            stack += [(i, k), (k, j)]
    return points[keep]
