"""탑뷰 영상에서 물건 찾기. Gemini 4 Argon에게 사진과 이름을 주고 점·상자를 받는다(zero-shot).

모델은 '어디에 무엇이 있는지'만 답한다. 그 점을 팔 좌표로 바꾸고 집을 수 있는지 판단하는 것은 skills.py다.
좌표는 Gemini 규약대로 0~1000으로 정규화된 [y, x]로 받아 픽셀로 바꾼다.
"""
from __future__ import annotations

import base64
import json
import logging
import time
import urllib.error
import urllib.request
from pathlib import Path

import cv2
import numpy as np

log = logging.getLogger('arm.vision')

FIND = """이 사진은 로봇 위에서 작업대를 내려다본 것이다. 아래쪽의 흰 로봇팔과 나무판은 로봇 자신이니 물건으로 치지 않는다.
찾을 물건: "{name}"
사진 전체에서 이 말에 맞는 물건을 찾는다. 작업대 밖이나 사람의 손·몸 위에 있어도 보이면 found=true다.
point는 물체 윗면의 집기 중심이고 box_2d는 물체 자체의 경계다. 받침의 종류와 관계없이 같은 기준으로 찍는다.
테이프 롤이 눕혀져 있고 고정 집게 손가락이 들어갈 빈 구멍이 위에서 뚜렷이 보이면 tape_hole=true로 하고 point를 그 구멍 정중앙에 찍는다. 그 밖에는 false다.
same_board는 물건과 흑백 높이 마커가 같은 단단한 판 위에 나란히 놓였을 때만 true다. 천·쿠션·손 위이거나 모르면 false다.
grasp_clear는 집게가 그 중심으로 내려가 닫힐 위치에 손가락이나 몸이 겹치지 않는지 나타낸다. 물건의 다른 쪽을 손으로 잡고 있는 것만으로 false가 되지는 않는다. 겹치거나 판단할 수 없으면 false다.
{{"found": true/false, "grasp_clear": true/false, "tape_hole": true/false, "same_board": true/false, "label": "물건 이름(한국어, 짧게)", "point": [y, x], "box_2d": [ymin, xmin, ymax, xmax], "height_cm": 숫자, "reason": "한 문장"}}
좌표는 0~1000으로 정규화한다.
height_cm는 받치고 있는 손이나 받침을 제외한 물체 자체의 높이다. 맞는 물건이 없으면 found를 false로 하고 reason에 보이는 물건을 적는다."""

LIST = """이 사진은 로봇 위에서 작업대를 내려다본 것이다. 아래쪽의 흰 로봇팔과 나무판, 전선은 로봇 자신이니 빼고,
화면 전체에서 보여주는 물건과 작업면 위의 작은 물건을 찾아 JSON 배열로 답한다. 손에 들거나 손바닥·무릎 위에 놓인 물건도 포함한다. 없으면 [].
[{"label": "물건 이름(한국어, 짧게. 같은 종류가 여럿이면 색이나 위치로 구분)", "point": [y, x], "box_2d": [ymin, xmin, ymax, xmax], "height_cm": 숫자}]
point는 위에서 집기 좋은 물건 윗면의 가운데다. 좌표는 0~1000으로 정규화한다."""


class VisionError(RuntimeError):
    pass


def read_key(path: str) -> str:
    for line in Path(path).expanduser().read_text().splitlines():
        if line.startswith('GEMINI_API_KEY='):
            return line.split('=', 1)[1].strip().strip('"\'')
    raise VisionError(f'{path}에 GEMINI_API_KEY가 없다')


class Argon:

    def __init__(self, cfg: dict):
        self.model = cfg['model']
        self.timeout = float(cfg.get('timeout_s', 20))
        self.key_file = cfg['key_env_file']
        self.calls = 0
        self.last_s = 0.0

    def _ask(self, jpeg: bytes, prompt: str):
        body = {
            'contents': [{'parts': [
                {'inlineData': {'mimeType': 'image/jpeg', 'data': base64.b64encode(jpeg).decode()}},
                {'text': prompt}]}],
            'generationConfig': {'temperature': 0, 'responseMimeType': 'application/json',
                                 'thinkingConfig': {'thinkingLevel': 'low'}},
        }
        req = urllib.request.Request(
            f'https://generativelanguage.googleapis.com/v1beta/models/{self.model}:generateContent',
            data=json.dumps(body).encode(),
            headers={'x-goog-api-key': read_key(self.key_file), 'Content-Type': 'application/json'})
        t0 = time.monotonic()
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as r:
                data = json.loads(r.read())
        except urllib.error.HTTPError as e:
            raise VisionError(f'Gemini HTTP {e.code}') from None
        except (urllib.error.URLError, TimeoutError, OSError) as e:
            raise VisionError(f'Gemini 연결 실패: {type(e).__name__}') from None
        self.calls += 1
        self.last_s = time.monotonic() - t0
        try:
            text = data['candidates'][0]['content']['parts'][0]['text']
            return json.loads(text)
        except (KeyError, IndexError, ValueError):
            raise VisionError('Gemini 응답을 읽지 못했다') from None

    @staticmethod
    def _pixels(item: dict, w: int, h: int) -> dict | None:
        try:
            y, x = (float(v) for v in item['point'])
            out = {'label': str(item.get('label', ''))[:40], 'u': x / 1000 * w, 'v': y / 1000 * h,
                   'height_m': float(np.clip(float(item.get('height_cm') or 3.0), 0.5, 30.0)) / 100}
            if item.get('support_depth_cm') is not None:
                try:
                    out['support_depth_m'] = float(item['support_depth_cm']) / 100
                except (ValueError, TypeError):
                    pass
            box = item.get('box_2d')
            if isinstance(box, list) and len(box) == 4:
                y0, x0, y1, x1 = (float(v) for v in box)
                out['box'] = [x0 / 1000 * w, y0 / 1000 * h, x1 / 1000 * w, y1 / 1000 * h]
            return out
        except (KeyError, TypeError, ValueError):
            return None

    def find(self, frame: np.ndarray, name: str, hint: str = '') -> dict:
        """{'found': bool, 'label', 'u', 'v', 'box', 'height_m', 'reason'}. hint: 작업대 정보(높이 마커, 카메라까지 거리)"""
        h, w = frame.shape[:2]
        ans = self._ask(encode(frame), FIND.format(name=name.replace('"', ' ')[:60]) + (f'\n{hint}' if hint else ''))
        if isinstance(ans, list):
            ans = ans[0] if ans else {'found': False}
        if not ans.get('found'):
            return {'found': False, 'reason': str(ans.get('reason', ''))[:120]}
        item = self._pixels(ans, w, h)
        if item is None:
            return {'found': False, 'reason': '좌표가 없는 응답'}
        return {'found': True, 'same_board': ans.get('same_board') is True, 'tape_hole': ans.get('tape_hole') is True,
                'grasp_clear': ans.get('grasp_clear') is True, 'reason': str(ans.get('reason', ''))[:120], **item}

    def list(self, frame: np.ndarray, hint: str = '') -> list[dict]:
        h, w = frame.shape[:2]
        ans = self._ask(encode(frame), LIST + (f'\n{hint}' if hint else ''))
        if not isinstance(ans, list):
            return []
        return [p for i in ans if isinstance(i, dict) and (p := self._pixels(i, w, h))]


def encode(frame: np.ndarray, quality: int = 90) -> bytes:
    ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise VisionError('JPEG 인코딩 실패')
    return buf.tobytes()


def grasp_axis(frame: np.ndarray, box: list[float] | None, u: float, v: float) -> tuple[float, float, float, float] | None:
    """상자 안에서 물건을 배경과 갈라(GrabCut) 좁은 쪽 방향을 구한다.

    반환: (중심 u, 중심 v, 긴 축의 영상 각도[rad], 짧은 변[px]) 또는 None. 집게는 긴 축에 수직으로 다문다.
    둥글거나 정사각형에 가까우면 각도는 의미가 없어 None을 준다.
    """
    if not box:
        return None
    h, w = frame.shape[:2]
    x0, y0, x1, y1 = box
    pad = 0.25 * max(x1 - x0, y1 - y0) + 6
    rx0, ry0 = int(max(0, x0 - pad)), int(max(0, y0 - pad))
    rx1, ry1 = int(min(w, x1 + pad)), int(min(h, y1 + pad))
    if rx1 - rx0 < 12 or ry1 - ry0 < 12:
        return None
    roi = frame[ry0:ry1, rx0:rx1]
    mask = np.zeros(roi.shape[:2], np.uint8)
    rect = (int(x0 - rx0), int(y0 - ry0), max(2, int(x1 - x0)), max(2, int(y1 - y0)))
    try:
        cv2.grabCut(roi, mask, rect, np.zeros((1, 65)), np.zeros((1, 65)), 4, cv2.GC_INIT_WITH_RECT)
    except cv2.error:
        return None
    fg = ((mask == cv2.GC_FGD) | (mask == cv2.GC_PR_FGD)).astype(np.uint8)
    fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    cs, _ = cv2.findContours(fg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cs:
        return None
    # 모델이 찍은 점에 가장 가까운 덩어리
    def dist(c):
        return -cv2.pointPolygonTest(c, (float(u - rx0), float(v - ry0)), True)
    c = min((c for c in cs if cv2.contourArea(c) > 30), key=dist, default=None)
    if c is None:
        return None
    (cx, cy), (rw, rh), ang = cv2.minAreaRect(c)
    long_, short = max(rw, rh), min(rw, rh)
    if short < 2 or long_ / short < 1.25:
        return None
    theta = np.radians(ang if rw >= rh else ang + 90.0)
    return cx + rx0, cy + ry0, float(theta), short
