"""팔·카메라 없이 심부름 흐름을 끝까지 돌려 보는 가짜 장면 (--fake).

작업대 위에 물건 몇 개를 두고, 탑뷰 카메라가 볼 그림을 그려 주고, Argon 대신 답한다.
집게가 물건 위에서 닫히면 물건이 집힌 것으로 치고, 건네면(공중에서 열면) 잠시 뒤 제자리에 다시 놓인다.
화면 개발과 음성 → 집기 → 건네기 리허설에 쓴다.
"""
from __future__ import annotations

import threading
import time

import cv2
import numpy as np

from .cameras import TopView

# label, x, y [m], 반지름 또는 (길이, 폭) [m], 높이 [m], BGR, 긴 축 각도 [deg]
OBJECTS = [
    ('귤', 0.24, 0.10, 0.028, 0.05, (40, 150, 245), 0),
    ('약통', 0.30, -0.06, 0.022, 0.07, (235, 235, 235), 0),
    ('리모컨', 0.19, -0.15, (0.13, 0.04), 0.02, (50, 50, 55), 20),
    ('안경집', 0.33, 0.13, (0.14, 0.055), 0.04, (150, 70, 40), -20),
    ('물병', 0.20, 0.28, 0.033, 0.18, (220, 170, 90), 0),          # 보이지만 팔이 닿지 않는 곳
]
ALIASES = {'귤': ['오렌지', '과일', '밀감'], '약통': ['약', '약병', '비타민'], '리모컨': ['리모콘', '리모트'],
           '안경집': ['안경', '안경통', '케이스'], '물병': ['물', '생수', '병']}


class Thing:

    def __init__(self, label, x, y, size, h, color, angle):
        self.label, self.home, self.size, self.h, self.color, self.angle = label, (x, y), size, h, color, angle
        self.xy = np.array([x, y], float)
        self.held = False
        self.gone_until = 0.0            # 건네받은 사람이 들고 있는 동안은 보이지 않는다

    @property
    def visible(self) -> bool:
        return not self.held and time.monotonic() >= self.gone_until

    @property
    def radius(self) -> float:
        return self.size if isinstance(self.size, float) else max(self.size) / 2

    def outline(self, z: float) -> np.ndarray:
        if isinstance(self.size, float):
            a = np.linspace(0, 2 * np.pi, 24, endpoint=False)
            pts = np.stack([np.cos(a), np.sin(a)], 1) * self.size
        else:
            l, w = self.size[0] / 2, self.size[1] / 2
            t = np.radians(self.angle)
            R = np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])
            pts = np.array([[l, w], [-l, w], [-l, -w], [l, -w]]) @ R.T
        return np.column_stack([pts + self.xy, np.full(len(pts), z)])


class Scene:
    """가짜 작업대. 팔 상태를 보고 물건이 집혔는지·놓였는지 정한다."""

    def __init__(self, cfg: dict, arm, view: TopView):
        self.cfg, self.arm, self.view = cfg, arm, view
        self.things = [Thing(*o) for o in OBJECTS]
        self.z0 = float(cfg['workspace']['table_z_m'])
        self.lock = threading.Lock()
        self._prev_grip = arm.joints()['gripper']

    def step(self) -> None:
        arm = self.arm
        grip = arm.target.get('gripper', 0.0)
        tip = arm.tip()
        now = time.monotonic()
        with self.lock:
            held = next((t for t in self.things if t.held), None)
            if held is None and grip < 12 <= self._prev_grip:
                for t in self.things:
                    if t.visible and np.linalg.norm(tip[:2] - t.xy) < max(0.03, t.radius * 0.6) and tip[2] < self.z0 + t.h + 0.03:
                        t.held = True
                        arm.bus.grip_floor = 14.0
                        break
            elif held is not None and grip > 40:
                held.held = False
                arm.bus.grip_floor = 0.0
                if tip[2] < self.z0 + held.h + 0.06:
                    held.xy = np.array(tip[:2], float)          # 작업대에 내려놓았다
                else:
                    held.xy = np.array(held.home, float)        # 사람이 받아 갔다가 조금 뒤 다시 놓는다
                    held.gone_until = now + 6.0
            self._prev_grip = grip

    def render(self) -> np.ndarray:
        w, h = int(self.cfg['cameras']['top']['width']), int(self.cfg['cameras']['top']['height'])
        img = np.full((h, w, 3), (96, 118, 132), np.uint8)
        noise = np.random.default_rng(int(time.monotonic() * 3)).integers(-3, 4, (h // 8, w // 8, 1), np.int16)
        img = np.clip(img.astype(np.int16) + cv2.resize(noise.astype(np.float32), (w, h))[..., None].astype(np.int16), 0, 255).astype(np.uint8)

        def px(p):
            try:
                u, v = self.view.project(p)
                return [int(round(u)), int(round(v))]
            except cv2.error:
                return [-999, -999]

        table = [(0.06, -0.45), (0.75, -0.45), (0.75, 0.45), (0.06, 0.45)]
        cv2.fillPoly(img, [np.array([px([x, y, self.z0]) for x, y in table], np.int32)], (150, 185, 205))
        deck = [(-0.10, -0.12), (0.07, -0.12), (0.07, 0.12), (-0.10, 0.12)]
        cv2.fillPoly(img, [np.array([px([x, y, self.z0 - 0.02]) for x, y in deck], np.int32)], (120, 160, 190))
        with self.lock:
            things = [(t, t.visible, t.held) for t in self.things]
        for t, visible, _ in things:
            if not visible:
                continue
            base = np.array([px(p) for p in t.outline(self.z0)], np.int32)
            top = np.array([px(p) for p in t.outline(self.z0 + t.h)], np.int32)
            cv2.fillPoly(img, [cv2.convexHull(np.vstack([base, top]))], tuple(int(c * 0.6) for c in t.color))
            cv2.fillPoly(img, [top], t.color)
        # 팔: 관절 원점을 이은 굵은 선
        pts = self.arm.kin.points(self.arm.q())
        line = np.array([px(p) for p in pts], np.int32)
        cv2.polylines(img, [line], False, (245, 245, 245), 16, cv2.LINE_AA)
        cv2.polylines(img, [line], False, (200, 200, 200), 2, cv2.LINE_AA)
        for t, _, held in things:
            if held:
                tip = self.arm.tip()
                cv2.circle(img, tuple(px(tip)), 12, t.color, -1, cv2.LINE_AA)
        cv2.putText(img, 'SIM', (w - 62, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (40, 60, 70), 2, cv2.LINE_AA)
        return img

    def detections(self) -> list[dict]:
        """Argon이 돌려줄 법한 답 (픽셀 좌표)"""
        out = []
        with self.lock:
            for t in self.things:
                if not t.visible:
                    continue
                u, v = self.view.project([t.xy[0], t.xy[1], self.z0 + t.h])
                top = np.array([self.view.project(p) for p in t.outline(self.z0 + t.h)])
                base = np.array([self.view.project(p) for p in t.outline(self.z0)])
                both = np.vstack([top, base])
                out.append({'label': t.label, 'u': float(u), 'v': float(v), 'height_m': t.h,
                            'box': [float(both[:, 0].min()), float(both[:, 1].min()), float(both[:, 0].max()), float(both[:, 1].max())]})
        return out


class SimCamera:
    """cameras.Camera와 같은 모양. 장면을 15 Hz로 그린다."""

    def __init__(self, scene: Scene):
        self.name, self.scene = 'top', scene
        self.frame: np.ndarray | None = None
        self.stamp = 0.0
        self.seq = 0
        self._jpeg: tuple[int, bytes] | None = None
        self.cond = threading.Condition()
        self._stop = threading.Event()
        threading.Thread(target=self._loop, name='cam-sim', daemon=True).start()

    def _loop(self) -> None:
        while not self._stop.is_set():
            self.scene.step()
            frame = self.scene.render()
            with self.cond:
                self.frame, self.stamp = frame, time.monotonic()
                self.seq += 1
                self.cond.notify_all()
            time.sleep(1 / 15)

    def close(self) -> None:
        self._stop.set()

    @property
    def age(self) -> float:
        return time.monotonic() - self.stamp if self.stamp else float('inf')

    def latest(self) -> np.ndarray | None:
        with self.cond:
            return None if self.frame is None else self.frame.copy()

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


class SimArgon:
    """vision.Argon 대역. 실제 모델처럼 몇 초 걸리는 척한다."""

    def __init__(self, scene: Scene, delay_s: float = 2.5):
        self.scene, self.delay = scene, delay_s
        self.calls = 0
        self.last_s = 0.0

    def _wait(self) -> None:
        time.sleep(self.delay)
        self.calls += 1
        self.last_s = self.delay

    def list(self, frame) -> list[dict]:
        self._wait()
        return self.scene.detections()

    def find(self, frame, name: str) -> dict:
        self._wait()
        items = self.scene.detections()
        key = name.replace(' ', '')
        for i in items:
            words = [i['label']] + ALIASES.get(i['label'], [])
            if any(w in key or key in w for w in words):
                return {'found': True, 'reason': '', **i}
        return {'found': False, 'reason': '보이는 것: ' + ', '.join(i['label'] for i in items)}
