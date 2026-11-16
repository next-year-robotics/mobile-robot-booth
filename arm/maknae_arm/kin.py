"""SO-101 기구학. URDF에서 체인을 읽어 FK와 수치 IK를 푼다. numpy만 쓴다.

관절각은 여기서 전부 URDF 라디안이다. LeRobot 도(deg)와의 변환은 Kinematics.to_urdf / to_lerobot이 한다.
팔은 5자유도라 끝점 자세를 다 정할 수 없다. IK 목표는 위치(3) + 그리퍼가 향하는 방향(접근 벡터)이고,
손목 회전(wrist_roll)은 따로 준다.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np

ARM_JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def transform(R: np.ndarray, p) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, p
    return T


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    T = np.eye(4)
    T[0, 0], T[0, 1], T[1, 0], T[1, 1] = c, -s, s, c
    return T


@dataclass
class Joint:
    name: str
    origin: np.ndarray      # 관절각 0일 때 부모 → 자식
    revolute: bool
    lower: float = -math.pi
    upper: float = math.pi


def load_chain(urdf: Path, base: str, tip: str, prefix: str = 'arm_') -> list[Joint]:
    """base → tip 순서의 관절 목록. SO-101 URDF의 회전축은 전부 자식 좌표계 z다."""
    root = ET.parse(urdf).getroot()
    by_child = {}
    for j in root.findall('joint'):
        by_child[j.find('child').get('link')] = j
    chain: list[Joint] = []
    link = tip
    while link != base:
        j = by_child[link]
        o = j.find('origin')
        xyz = [float(v) for v in (o.get('xyz') or '0 0 0').split()]
        rpy = [float(v) for v in (o.get('rpy') or '0 0 0').split()]
        revolute = j.get('type') == 'revolute'
        if revolute:
            axis = [float(v) for v in j.find('axis').get('xyz').split()]
            if not np.allclose(axis, [0, 0, 1]):
                raise ValueError(f"{j.get('name')}: 회전축이 z가 아니다 {axis}")
        lim = j.find('limit')
        name = j.get('name')
        chain.append(Joint(name[len(prefix):] if name.startswith(prefix) else name,
                           transform(rpy_matrix(*rpy), xyz), revolute,
                           float(lim.get('lower')) if revolute else 0.0,
                           float(lim.get('upper')) if revolute else 0.0))
        link = j.find('parent').get('link')
    chain.reverse()
    return chain


class Kinematics:

    def __init__(self, urdf, base: str = 'arm_base_link', tip: str = 'arm_gripper_frame_link',
                 offset_deg: dict[str, float] | None = None):
        self.chain = load_chain(Path(urdf).expanduser(), base, tip)
        names = [j.name for j in self.chain if j.revolute]
        if names != ARM_JOINTS:
            raise ValueError(f'관절 순서가 예상과 다르다: {names}')
        self.lower = np.array([j.lower for j in self.chain if j.revolute])
        self.upper = np.array([j.upper for j in self.chain if j.revolute])
        self.offset = np.radians([float((offset_deg or {}).get(n, 0.0)) for n in ARM_JOINTS])
        # 접근 벡터: 끝 좌표계에서 손가락이 뻗는 방향. 손목에서 끝점으로 가는 방향과 가장 가까운 축을 고른다.
        T, frames = self._fk_frames(np.zeros(5))
        d = T[:3, 3] - frames[-2][:3, 3]
        d = T[:3, :3].T @ (d / np.linalg.norm(d))
        k = int(np.argmax(np.abs(d)))
        self.approach_local = np.zeros(3)
        self.approach_local[k] = 1.0 if d[k] > 0 else -1.0

    # ---------------------------------------------------------------- 단위 변환
    def to_urdf(self, deg: dict[str, float]) -> np.ndarray:
        return np.radians([float(deg[n]) for n in ARM_JOINTS]) + self.offset

    def to_lerobot(self, q: np.ndarray) -> dict[str, float]:
        return {n: float(v) for n, v in zip(ARM_JOINTS, np.degrees(np.asarray(q) - self.offset))}

    # ---------------------------------------------------------------- FK
    def _fk_frames(self, q) -> tuple[np.ndarray, list[np.ndarray]]:
        T = np.eye(4)
        frames = []
        i = 0
        for j in self.chain:
            T = T @ j.origin
            if j.revolute:
                T = T @ rot_z(float(q[i]))
                i += 1
            frames.append(T.copy())
        return T, frames

    def fk(self, q) -> np.ndarray:
        return self._fk_frames(q)[0]

    def tip(self, q) -> np.ndarray:
        return self.fk(q)[:3, 3]

    def approach(self, q) -> np.ndarray:
        return self.fk(q)[:3, :3] @ self.approach_local

    def points(self, q) -> np.ndarray:
        """관절 원점들과 끝점 (N, 3). 충돌 검사와 그림에 쓴다."""
        T, frames = self._fk_frames(q)
        return np.array([f[:3, 3] for f in frames])

    # ---------------------------------------------------------------- IK
    def _residual(self, q, pos, approach, w_dir):
        T = self.fk(q)
        r = T[:3, 3] - pos
        if approach is None:
            return r
        return np.concatenate([r, w_dir * (T[:3, :3] @ self.approach_local - approach)])

    def ik(self, pos, approach=None, roll: float | None = None, seed=None,
           w_dir: float = 0.08, tol_m: float = 0.0015, margin: float = math.radians(2.0)):
        """끝점 위치 pos[m]와 접근 방향 approach(단위 벡터)를 맞추는 관절각을 찾는다. 못 찾으면 None.

        wrist_roll은 위치에 거의 영향이 없어 풀지 않고 roll 값(없으면 seed 값)으로 고정한다.
        방향 오차는 w_dir[m/단위] 가중치로 섞는다. 위치가 tol_m 안에 들어와야 해로 인정한다.
        """
        pos = np.asarray(pos, float)
        approach = None if approach is None else np.asarray(approach, float) / np.linalg.norm(approach)
        lo, hi = self.lower + margin, self.upper - margin
        seeds = []
        if seed is not None:
            seeds.append(np.asarray(seed, float))
        pan = math.atan2(pos[1], pos[0] - 0.0388)
        # 팔꿈치를 든 자세들에서 출발한다(작업대에 팔꿈치가 닿는 해를 피한다)
        for lift, elbow, flex in ((0.0, 0.3, 1.2), (-0.6, 0.9, 1.2), (0.6, -0.4, 1.3), (-1.0, 1.4, 0.6), (0.2, 0.8, 0.3)):
            seeds.append(np.array([pan, lift, elbow, flex, 0.0]))
        best = None
        for s in seeds:
            q = np.clip(s.copy(), lo, hi)
            if roll is not None:
                q[4] = roll
            lam = 1e-3
            err = self._residual(q, pos, approach, w_dir)
            for _ in range(80):
                J = np.zeros((len(err), 4))
                for k in range(4):
                    dq = q.copy()
                    dq[k] += 1e-5
                    J[:, k] = (self._residual(dq, pos, approach, w_dir) - err) / 1e-5
                step = np.linalg.solve(J.T @ J + lam * np.eye(4), -J.T @ err)
                step = np.clip(step, -0.35, 0.35)
                trial = q.copy()
                trial[:4] = np.clip(q[:4] + step, lo[:4], hi[:4])
                e2 = self._residual(trial, pos, approach, w_dir)
                if e2 @ e2 < err @ err:
                    q, err, lam = trial, e2, max(lam * 0.5, 1e-6)
                    if np.linalg.norm(step) < 1e-7:
                        break
                else:
                    lam *= 4.0
                    if lam > 1e3:
                        break
            perr = float(np.linalg.norm(self.tip(q) - pos))
            derr = 0.0 if approach is None else float(np.linalg.norm(self.approach(q) - approach))
            cost = perr + 0.02 * derr
            if seed is not None:
                cost += 0.002 * float(np.linalg.norm(q[:4] - np.asarray(seed, float)[:4]))
            if perr <= tol_m and (best is None or cost < best[0]):
                best = (cost, q, perr, derr)
        if best is None:
            return None
        return best[1]
