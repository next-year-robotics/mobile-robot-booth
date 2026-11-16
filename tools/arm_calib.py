#!/usr/bin/env python3
"""탑뷰 카메라와 팔의 위치 관계를 실측으로 맞춘다. 팔 데몬(scripts/arm.sh start)이 떠 있어야 한다.

    python tools/arm_calib.py collect        팔을 여러 자세로 보내며 사진과 관절각을 모은다 (약 4분, 팔이 움직인다)
    python tools/arm_calib.py solve          모은 자료로 카메라 자세를 풀어 config/arm.local.yaml에 쓴다
    python tools/arm_calib.py check          지금 자세의 FK 뼈대를 탑뷰에 겹쳐 그린다 (눈으로 확인)
    python tools/arm_calib.py table          끝점을 작업대에 대 놓은 상태에서 부르면 그 높이를 작업대 높이로 적는다

방법: 집게에 달린 손목 카메라 기판은 위에서 또렷이 보인다. 자세마다 그 기판 중심의 픽셀을 Gemini 4 Argon에게
물어 받고, '카메라 자세 6개 + 기판이 집게 좌표계의 어디 있는지 3개'를 재투영 오차가 최소가 되게 한꺼번에 푼다.
마커를 붙이지 않아도 된다. 틀린 점 몇 개는 Huber 손실이 눌러 준다.

LeRobot 환경의 파이썬으로 돌린다: ~/miniconda3/envs/lerobot/bin/python tools/arm_calib.py …
"""
from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'arm'))
from maknae_arm.cameras import TopView  # noqa: E402
from maknae_arm.kin import Kinematics  # noqa: E402
from maknae_arm.vision import Argon  # noqa: E402
from maknae_arm.__main__ import merge  # noqa: E402

DATA = Path('~/.local/share/maknae/calib').expanduser()
PROMPT = """로봇팔 집게 끝 쪽에 작은 정사각형 전자 기판(손목 카메라 모듈, 짙은 녹색·검정, 네 귀퉁이에 구멍)이 보인다.
그 기판의 정중앙을 가리켜라. JSON 객체로 답한다: {"found": true/false, "point": [y, x]} 좌표는 0~1000 정규화.
기판이 가려져 안 보이면 found를 false로 한다."""


def load_cfg() -> dict:
    cfg = yaml.safe_load((ROOT / 'config' / 'arm.yaml').read_text())
    local = ROOT / 'config' / 'arm.local.yaml'
    if local.exists():
        merge(cfg, yaml.safe_load(local.read_text()) or {})
    return cfg


def save_local(update: dict) -> None:
    path = ROOT / 'config' / 'arm.local.yaml'
    cur = yaml.safe_load(path.read_text()) if path.exists() else {}
    cur = cur or {}
    merge(cur, update)
    path.write_text('# tools/arm_calib.py가 쓴 실측 보정값. arm.yaml 위에 덮어쓴다. git에 올리지 않는다.\n'
                    + yaml.safe_dump(cur, allow_unicode=True, sort_keys=False))
    print('저장:', path)


class Daemon:

    def __init__(self, cfg):
        self.url = f"http://{cfg['server']['host']}:{cfg['server']['port']}"

    def state(self) -> dict:
        return json.loads(urllib.request.urlopen(self.url + '/state', timeout=3).read())

    def cmd(self, **msg) -> dict:
        req = urllib.request.Request(self.url + '/cmd', data=json.dumps(msg).encode())
        return json.loads(urllib.request.urlopen(req, timeout=10).read())

    def image(self) -> np.ndarray:
        data = urllib.request.urlopen(self.url + '/cam/top.jpg', timeout=3).read()
        return cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)

    def settle(self, timeout=20.0) -> dict:
        t0 = time.time()
        time.sleep(0.3)
        while time.time() - t0 < timeout:
            s = self.state()
            if not s['moving'] and s['errand']['state'] in ('idle', 'holding', 'error'):
                time.sleep(0.8)
                return self.state()
            time.sleep(0.1)
        raise SystemExit('팔이 멈추지 않는다')


def collect(args) -> None:
    cfg = load_cfg()
    d = Daemon(cfg)
    argon = Argon(cfg['vision'])
    DATA.mkdir(parents=True, exist_ok=True)
    targets = []
    k = 0
    z0 = float(cfg['workspace']['table_z_m'])
    for z in (z0 + 0.04, z0 + 0.11, z0 + 0.19):          # 작업대 위 세 높이. 높이가 달라야 관절 영점이 풀린다
        for x in (0.17, 0.23, 0.29):
            for y in (-0.15, -0.075, 0.0, 0.075, 0.15):
                targets.append((x, y, z, (0, 20, -20)[k % 3]))
                k += 1
    samples = []
    for i, (x, y, z, roll) in enumerate(targets):
        r = d.cmd(cmd='reach', pos=[x, y, z], roll_deg=roll)
        if not r.get('ok'):
            print(f'{i:2d} ({x:.2f},{y:+.3f},{z:.2f}) 건너뜀: {r.get("error")}')
            continue
        s = d.settle()
        if s['errand']['state'] == 'error':
            print(f'{i:2d} 이동 실패: {s["errand"]["detail"]}')
            d.cmd(cmd='home')
            d.settle()
            continue
        img = d.image()
        cv2.imwrite(str(DATA / f'{i:02d}.jpg'), img)
        try:
            ans = argon._ask(cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 92])[1].tobytes(), PROMPT)
        except Exception as e:
            print(f'{i:2d} Argon 실패: {e}')
            continue
        if isinstance(ans, list):
            ans = ans[0] if ans else {}
        if not ans.get('found'):
            print(f'{i:2d} 기판 안 보임')
            continue
        v, u = ans['point'][0] / 1000 * img.shape[0], ans['point'][1] / 1000 * img.shape[1]
        samples.append({'i': i, 'joints': s['joints'], 'u': u, 'v': v})
        print(f'{i:2d} ({x:.2f},{y:+.3f},{z:.2f}) roll {roll:+d} → 기판 ({u:.0f}, {v:.0f})  {argon.last_s:.1f} s', flush=True)
        (DATA / 'samples.json').write_text(json.dumps(samples))
    d.cmd(cmd='home')
    print(f'{len(samples)}개 모음 → {DATA}/samples.json')


PROMPT2 = """로봇팔을 위에서 내려다본 사진이다. 흰 로봇팔에는 검은색 직육면체 서보 모터가 여러 개 박혀 있다.
그중 집게(팔 끝, 작은 전자 기판이 달린 쪽)에 가장 가까운 검은 서보 모터 하나를 골라 그 보이는 면의 정중앙을 가리켜라.
JSON 객체로 답한다: {"found": true/false, "point": [y, x]} 좌표는 0~1000 정규화. 안 보이면 found를 false로 한다."""


def annotate(args) -> None:
    """모아 둔 사진에서 둘째 특징점(손목 쪽 검은 서보)을 찍는다. 팔은 움직이지 않는다.

    기판 하나만으로는 '카메라가 돌아간 것'과 '어깨·손목 회전 영점이 어긋난 것'이 구별되지 않아 끝점이 옆으로 몇 cm 빗나간다.
    손목 회전과 무관한 점을 하나 더 보면 풀린다.
    """
    from concurrent.futures import ThreadPoolExecutor
    cfg = load_cfg()
    samples = json.loads((DATA / 'samples.json').read_text())

    def one(s):
        img = cv2.imread(str(DATA / f"{s['i']:02d}.jpg"))
        try:
            ans = Argon(cfg['vision'])._ask(cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 92])[1].tobytes(), PROMPT2)
        except Exception as e:
            return s['i'], None, str(e)
        if isinstance(ans, list):
            ans = ans[0] if ans else {}
        if not ans.get('found'):
            return s['i'], None, '안 보임'
        return s['i'], (ans['point'][1] / 1000 * img.shape[1], ans['point'][0] / 1000 * img.shape[0]), ''

    with ThreadPoolExecutor(4) as ex:
        got = {i: (uv, why) for i, uv, why in ex.map(one, samples)}
    n = 0
    for s in samples:
        uv, why = got[s['i']]
        if uv is None:
            s.pop('u2', None); s.pop('v2', None)
            print(f"{s['i']:2d} 건너뜀: {why}")
        else:
            s['u2'], s['v2'] = uv
            n += 1
    (DATA / 'samples.json').write_text(json.dumps(samples))
    print(f'{n}/{len(samples)}개에 둘째 점을 찍었다')


PROMPT3 = """로봇팔을 위에서 내려다본 사진이다. 흰 로봇팔 끝에 흰색 집게가 달려 있고, 집게의 두 손가락은 닫혀서 뾰족하게 모여 있다.
집게 손가락이 모여 끝나는 가장 끝점(팔 몸통에서 가장 먼 뾰족한 끝)을 가리켜라.
JSON 객체로 답한다: {"found": true/false, "point": [y, x]} 좌표는 0~1000 정규화. 끝이 가려져 안 보이면 found를 false로 한다."""


def tips(args) -> None:
    """집게를 닫고 앞으로 기울여 끝이 탑뷰에 보이게 한 뒤 끝점을 찍는다. 끝점은 위치를 아는 점이라 보정을 단단히 묶는다."""
    cfg = load_cfg()
    d = Daemon(cfg)
    argon = Argon(cfg['vision'])
    kin = Kinematics(cfg['arm']['urdf'], offset_deg=cfg.get('joint_offset_deg'))
    z = float(cfg['workspace']['table_z_m']) + args.height
    out = []
    d.cmd(cmd='gripper', value=0)
    d.settle()
    d.cmd(cmd='pose', name='ready')
    d.settle()
    targets = [(x, y) for x in args.x for y in args.y]
    for i, (x, y) in enumerate(targets):
        radial = np.array([x - 0.0388, y, 0.0])
        radial /= np.linalg.norm(radial)
        q = None
        for tilt in (50.0, 40.0, 60.0, 30.0):
            t = np.radians(tilt)
            q = kin.ik([x, y, z], np.sin(t) * radial + np.array([0, 0, -np.cos(t)]), roll=0.0, seed=kin.to_urdf(d.state()['joints']))
            if q is not None:
                break
        if q is None:
            print(f'{i:2d} ({x:.2f},{y:+.2f}) IK 실패')
            continue
        r = d.cmd(cmd='joints', goal=kin.to_lerobot(q))
        if not r.get('ok'):
            print(f'{i:2d} ({x:.2f},{y:+.2f}) 건너뜀: {r.get("error")}')
            continue
        s = d.settle()
        if s['errand']['state'] == 'error':
            print(f'{i:2d} 이동 실패: {s["errand"]["detail"]}')
            d.cmd(cmd='home'); d.settle(); d.cmd(cmd='pose', name='ready'); d.settle()
            continue
        img = d.image()
        cv2.imwrite(str(DATA / f'tip_{i:02d}.jpg'), img)
        try:
            ans = argon._ask(cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 92])[1].tobytes(), PROMPT3)
        except Exception as e:
            print(f'{i:2d} Argon 실패: {e}')
            continue
        if isinstance(ans, list):
            ans = ans[0] if ans else {}
        if not ans.get('found'):
            print(f'{i:2d} 끝이 안 보임')
            continue
        v, u = ans['point'][0] / 1000 * img.shape[0], ans['point'][1] / 1000 * img.shape[1]
        out.append({'i': i, 'joints': s['joints'], 'u': u, 'v': v})
        print(f'{i:2d} ({x:.2f},{y:+.2f}) → 끝 ({u:.0f}, {v:.0f})', flush=True)
        (DATA / 'tips.json').write_text(json.dumps(out))
    d.cmd(cmd='pose', name='ready'); d.settle()
    d.cmd(cmd='home')
    print(f'{len(out)}개 → {DATA}/tips.json')


def solve(args) -> None:
    """카메라 자세(6) + 기판 위치(3) + 관절 영점 어긋남(어깨 회전·어깨 들기·팔꿈치·손목 굽힘)을 같이 푼다.

    LeRobot 각도의 0은 '캘리브레이션 때 손으로 잡아 준 가운데'라 URDF의 0과 몇 도씩 다르다. 그 차이를 안 풀면
    카메라 자세가 대신 틀어져서, 표본을 모은 높이에서는 맞아도 작업대 높이에서는 끝점이 몇 cm 빗나간다.
    기판 하나로는 카메라가 돌아간 것과 어깨 회전 영점이 어긋난 것이 구별되지 않는다. annotate로 둘째 점(손목 서보)을
    찍어 두면 그것까지 푼다. 손목 회전 영점은 기판 위치와 구별되지 않아 풀지 않는다(끝점 위치에는 영향이 없다).
    """
    from scipy.optimize import least_squares
    cfg = load_cfg()
    base = yaml.safe_load((ROOT / 'config' / 'arm.yaml').read_text())
    kin = Kinematics(cfg['arm']['urdf'], offset_deg=base.get('joint_offset_deg'))
    view = TopView(base['cameras']['top'])                 # 출발점은 URDF 값
    samples = json.loads((DATA / 'samples.json').read_text())
    Q = np.array([kin.to_urdf(s['joints']) for s in samples])
    uv = np.array([[s['u'], s['v']] for s in samples])
    has2 = np.array(['u2' in s for s in samples])
    uv2 = np.array([[s.get('u2', 0.0), s.get('v2', 0.0)] for s in samples])
    r0, _ = cv2.Rodrigues(view.R)
    free = not args.no_offsets
    two = bool(has2.sum() >= 12) and free            # 둘째 점(손목 서보)이 있으면 어깨 회전 영점도 같이 푼다
    FRAME2 = 3                                       # 손목 굽힘 관절 좌표계: 손목 회전과 무관하다

    def offsets(p):
        return np.array([p[13] if two else 0.0, p[9], p[10], p[11], 0.0]) if free else np.zeros(5)

    def pix(R, C, pts):
        pc = (R.T @ (np.asarray(pts) - C).T).T
        px, _ = cv2.projectPoints(pc.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), view.K, view.dist)
        return px.reshape(-1, 2)

    def project(p, Q):
        R, _ = cv2.Rodrigues(p[:3])
        C, f, d = p[3:6], p[6:9], offsets(p)
        return pix(R, C, [(kin.fk(q + d) @ np.append(f, 1.0))[:3] for q in Q])

    def project2(p, Q):
        R, _ = cv2.Rodrigues(p[:3])
        C, g, d = p[3:6], p[14:17], offsets(p)
        return pix(R, C, [(kin._fk_frames(q + d)[1][FRAME2] @ np.append(g, 1.0))[:3] for q in Q])

    def cost(p, Q, uv, uv2, has2):
        r = (project(p, Q) - uv).ravel()
        if not free:
            return r
        parts = [r, p[9:12] / np.radians(20.0)]                # 영점 어긋남은 20° 안팎이라는 약한 믿음
        if two:
            # 둘째 점은 모델이 서보의 어느 면을 찍느냐에 따라 흔들려서 무게를 덜 준다
            parts += [0.5 * (project2(p, Q[has2]) - uv2[has2]).ravel(), [p[13] / np.radians(20.0)]]
        return np.concatenate([np.ravel(x) for x in parts])

    p0 = np.concatenate([r0.ravel(), view.C, [0.0, -0.06, -0.03], [0.0, 0.0, 0.0, 0.0], [0.0], [0.0, 0.0, 0.03]])
    fit = least_squares(cost, p0, args=(Q, uv, uv2, has2), loss='huber', f_scale=4.0)
    res = np.linalg.norm(project(fit.x, Q) - uv, axis=1)
    keep = res < max(8.0, 2.5 * np.median(res))
    if two:
        res2 = np.linalg.norm(project2(fit.x, Q) - uv2, axis=1)
        has2 = has2 & (res2 < max(14.0, 2.5 * np.median(res2[has2])))      # 엉뚱한 서보를 찍은 사진은 뺀다
    if keep.sum() >= 8:                                     # 크게 틀린 점을 빼고 한 번 더
        Q, uv, uv2, has2 = Q[keep], uv[keep], uv2[keep], has2[keep]
        fit = least_squares(cost, fit.x, args=(Q, uv, uv2, has2), loss='huber', f_scale=4.0)
        res = np.linalg.norm(project(fit.x, Q) - uv, axis=1)
    if two:
        res2 = np.linalg.norm(project2(fit.x, Q[has2]) - uv2[has2], axis=1)
        print(f'둘째 점 {int(has2.sum())}개, 재투영 오차 RMS {np.sqrt(np.mean(res2 ** 2)):.1f} px, 손목 좌표 {fit.x[14:17].round(3)}')
    R, _ = cv2.Rodrigues(fit.x[:3])
    C, f = fit.x[3:6], fit.x[6:9]
    off = np.degrees([fit.x[13] if two else 0.0, *fit.x[9:12]]) if free else np.zeros(4)
    print(f'표본 {len(uv)}개 (제외 {int((~keep).sum())}개), 재투영 오차 RMS {np.sqrt(np.mean(res ** 2)):.1f} px, 최대 {res.max():.1f} px')
    print('카메라 위치 [m]:', C.round(4), ' (URDF', view.C.round(4), ', 차이', ((C - view.C) * 1000).round(0), 'mm)')
    tilt = np.degrees(np.arccos(np.clip(-R[2, 2], -1, 1)))
    print(f'광축이 수직에서 기운 각: {tilt:.1f}°')
    print('기판 중심의 집게 좌표 [m]:', f.round(4))
    print('관절 영점 어긋남 [deg] 어깨 회전 %+.1f, 어깨 들기 %+.1f, 팔꿈치 %+.1f, 손목 굽힘 %+.1f' % tuple(off))
    if args.dry:
        return
    b = base.get('joint_offset_deg') or {}
    save_local({'cameras': {'top': {'xyz_m': [round(float(v), 5) for v in C],
                                    'rotation': [[round(float(v), 6) for v in row] for row in R]}},
                'joint_offset_deg': {m: round(float(b.get(m, 0.0)) + float(o), 2)
                                     for m, o in zip(('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex'), off)},
                'calib': {'date': time.strftime('%Y-%m-%d %H:%M'), 'samples': int(len(uv)),
                          'rms_px': round(float(np.sqrt(np.mean(res ** 2))), 2),
                          'board_in_gripper_m': [round(float(v), 4) for v in f]}})
    print('팔 데몬을 다시 띄우면 적용된다: scripts/arm.sh restart')


def check(args) -> None:
    cfg = load_cfg()
    d = Daemon(cfg)
    kin = Kinematics(cfg['arm']['urdf'], offset_deg=cfg.get('joint_offset_deg'))
    view = TopView(cfg['cameras']['top'])
    s, img = d.state(), d.image()
    prev = None
    for name, p in zip(('pan', 'lift', 'elbow', 'wrist', 'roll', 'tip'), kin.points(kin.to_urdf(s['joints']))):
        c = tuple(int(round(v)) for v in view.project(p))
        cv2.circle(img, c, 5, (0, 0, 255), -1)
        cv2.putText(img, name, (c[0] + 7, c[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
        if prev:
            cv2.line(img, prev, c, (0, 0, 255), 1)
        prev = c
    board = (cfg.get('calib') or {}).get('board_in_gripper_m')
    if board:
        c = tuple(int(round(v)) for v in view.project((kin.fk(kin.to_urdf(s['joints'])) @ np.append(board, 1.0))[:3]))
        cv2.drawMarker(img, c, (0, 255, 0), cv2.MARKER_CROSS, 16, 2)
    out = Path(args.out).expanduser()
    cv2.imwrite(str(out), img)
    print('저장:', out, '(빨강: FK 뼈대, 초록 십자: 기판 예측 위치)')


def table(args) -> None:
    cfg = load_cfg()
    s = Daemon(cfg).state()
    z = round(float(s['tip'][2]) + args.offset, 4)
    save_local({'workspace': {'table_z_m': z}})
    print(f'작업대 높이 {z:+.4f} m (팔 받침 기준). scripts/arm.sh restart로 적용')


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='cmd', required=True)
    sub.add_parser('collect')
    sub.add_parser('annotate')
    p = sub.add_parser('tips'); p.add_argument('--height', type=float, default=0.03)
    p.add_argument('--x', type=float, nargs='+', default=[0.24, 0.30, 0.35]); p.add_argument('--y', type=float, nargs='+', default=[-0.14, -0.07, 0.0, 0.07, 0.14])
    p = sub.add_parser('solve'); p.add_argument('--dry', action='store_true'); p.add_argument('--no-offsets', action='store_true', help='관절 영점은 풀지 않는다')
    p = sub.add_parser('check'); p.add_argument('--out', default='~/.cache/booth-dashboard/arm_check.jpg')
    p = sub.add_parser('table'); p.add_argument('--offset', type=float, default=0.0, help='끝점과 작업대 사이 틈 [m]')
    a = ap.parse_args()
    {'collect': collect, 'annotate': annotate, 'tips': tips, 'solve': solve, 'check': check, 'table': table}[a.cmd](a)
