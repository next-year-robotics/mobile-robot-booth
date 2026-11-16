"""장기자랑 팔춤. 화면 속 막내가 권루트 챌린지("L을 가져가")를 출 때 팔이 같은 음악에 맞춰 춘다.

음악은 150 BPM(한 박 0.4 s)이고 4박(1.6 s)마다 L 손 ↔ 주먹이 바뀐다. 구간 시각은 아바타 모션
(web/public/motions/keonroot-challenge.vrma)의 왼손 검지 트랙에서, 박자는 음원의 onset에서 뽑았다.

- L: 집게를 끝까지 벌리면 두 집게가 L자가 된다. 그 자세로 2박마다 좌우로 흔들고 박마다 손목을 끄덕인다.
- 주먹: 집게를 닫고 박마다 팔을 들어 올린다.
- V: 마지막 V 사인. 집게를 반쯤 벌려 V자를 만들고 멈춘다.

시각은 전부 음악 시각(초)이다. 데몬은 브라우저가 알려 준 '음악 0초의 벽시계 시각'에 맞춰 목표를 미리 줄 세운다.
"""
from __future__ import annotations

import math

from .kin import ARM_JOINTS

MOTORS = ARM_JOINTS + ['gripper']


def smoothstep(x: float) -> float:
    x = min(1.0, max(0.0, x))
    return x * x * (3 - 2 * x)


def min_jerk(s: float) -> float:
    s = min(1.0, max(0.0, s))
    return 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5


class Choreography:

    def __init__(self, cfg: dict):
        d = cfg
        self.beat = float(d.get('beat_s', 0.4))
        self.beat0 = float(d.get('beat_offset_s', 0.0))
        self.pose = {m: float(d['pose'][m]) for m in ARM_JOINTS}
        self.sway = float(d.get('sway_deg', 18))
        self.bob = float(d.get('bob_deg', 8))
        self.pump = float(d.get('pump_deg', 10))
        self.grip = {'L': float(d.get('l_open', 100)), 'fist': float(d.get('fist', 0)), 'V': float(d.get('v_open', 45))}
        self.sections = [(float(a), float(b), str(k)) for a, b, k in d['sections']]
        self.blend = float(d.get('blend_s', 0.16))
        for a, b, k in self.sections:
            if k not in self.grip or b <= a:
                raise ValueError(f'춤 구간이 이상하다: {a}–{b} {k}')
        self.end = self.sections[-1][1]

    # ------------------------------------------------------------------ 한 구간의 동작
    def _move(self, kind: str, m: float) -> dict[str, float]:
        b = (m - self.beat0) / self.beat                 # 박 수(정수가 박)
        on = 0.5 * (1 + math.cos(2 * math.pi * b))       # 박에서 1, 엇박에서 0
        g = dict(self.pose)
        if kind == 'L':
            # 2박에 한 번 왼쪽, 다음 2박에 오른쪽. 짝수 박에서 가운데를 지나므로 구간이 바뀌는 4박째에는 가운데다
            g['shoulder_pan'] += self.sway * math.sin(math.pi * b / 2)
            g['wrist_flex'] += self.bob * (on - 0.5) * 2
        elif kind == 'fist':
            # 박마다 팔을 위로 뻗었다가 돌아온다(주먹 펌프)
            g['elbow_flex'] -= self.pump * on
            g['wrist_flex'] -= 0.5 * self.pump * on
        g['gripper'] = self.grip[kind]
        return g

    def at(self, m: float) -> dict[str, float]:
        """음악 시각 m[s]의 관절 목표. 구간 경계에서는 blend_s 동안 섞어 튀지 않게 한다."""
        secs = self.sections
        m = min(max(m, secs[0][0]), self.end)
        i = next((k for k, (a, b, _) in enumerate(secs) if m < b), len(secs) - 1)
        g = self._move(secs[i][2], m)
        h = self.blend / 2
        # 다음 구간으로 넘어가는 중
        if i + 1 < len(secs) and m > secs[i][1] - h:
            w = smoothstep((m - (secs[i][1] - h)) / self.blend)
            n = self._move(secs[i + 1][2], m)
            g = {k: (1 - w) * g[k] + w * n[k] for k in MOTORS}
        # 앞 구간에서 막 넘어온 중
        elif i > 0 and m < secs[i][0] + h:
            w = smoothstep((m - (secs[i][0] - h)) / self.blend)
            p = self._move(secs[i - 1][2], m)
            g = {k: (1 - w) * p[k] + w * g[k] for k in MOTORS}
        return g

    # ------------------------------------------------------------------ 궤적
    def plan(self, start: dict[str, float], m0: float, rate_hz: float, enter_s: float,
             hold_s: float = 0.0, enter_deg_s: float = 140.0) -> list[tuple[float, dict[str, float]]]:
        """음악 시각 m0부터 끝까지 (음악 시각, 목표) 목록. 음악 enter_s초까지 지금 자세(start)에서 춤으로 들어간다.

        늦게 시작했으면 들어가는 시간을 늘려, 가장 많이 도는 관절이 평균 enter_deg_s를 넘지 않게 한다
        (접힌 팔이 급하게 튀어 오르지 않게).
        """
        dt = 1.0 / rate_hz
        dist = max(abs(self.pose[j] - float(start[j])) for j in ARM_JOINTS)
        span = max(self.sections[0][0] + enter_s - m0, 1.875 * dist / enter_deg_s, 0.5)
        join = m0 + span
        grip_span = min(span, 0.6)                       # 집게는 먼저 벌려 L을 일찍 보여 준다
        out = []
        n = int(math.ceil((self.end + hold_s - m0) / dt))
        for k in range(1, n + 1):
            m = m0 + k * dt
            g = self.at(m)
            if m < join:
                w = min_jerk((m - m0) / span)
                wg = min_jerk((m - m0) / grip_span)
                g = {j: (1 - w) * float(start[j]) + w * g[j] for j in ARM_JOINTS} | {
                    'gripper': (1 - wg) * float(start['gripper']) + wg * g['gripper']}
            out.append((m, g))
        return out


def peak_speed(steps: list[tuple[float, dict[str, float]]]) -> dict[str, float]:
    """관절마다 가장 빠른 속도 [deg/s]. 시험과 로그용."""
    top = {m: 0.0 for m in MOTORS}
    for (t0, a), (t1, b) in zip(steps, steps[1:]):
        for m in MOTORS:
            top[m] = max(top[m], abs(b[m] - a[m]) / (t1 - t0))
    return top
