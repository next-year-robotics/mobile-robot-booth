#!/usr/bin/env python3
"""부스에서 쓸 ArUco 마커를 A4 PDF로 만든다.

    python3 tools/make_markers.py            → docs/aruco_markers.pdf

- 사전은 DICT_4X4_50, 검은 변 길이는 config/booth.yaml aruco.marker_size_m(8 cm)이다. 검출기가 크기를 하나로만
  가정하고 거리를 계산하므로 용도가 달라도 크기는 전부 같다. 다른 크기로 뽑으면 거리가 그 비율만큼 틀린다.
- 인쇄는 반드시 "실제 크기(100 %)"로 한다. 쪽마다 있는 10 cm 자를 재서 확인한다.
- 카드(점선) 안쪽의 흰 여백 2 cm는 자르지 않는다. 여백이 없으면 어두운 바탕에서 검출이 안 된다.
"""
from pathlib import Path

import cv2
import numpy as np
import yaml
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
CFG = yaml.safe_load((ROOT / 'config' / 'booth.yaml').read_text(encoding='utf-8'))
DPI = 600
PX = DPI / 25.4                                   # px per mm
W, H = round(210 * PX), round(297 * PX)
SIZE_MM = float(CFG['aruco']['marker_size_m']) * 1000
CARD_MM = SIZE_MM + 40                            # 흰 여백 2 cm씩
FONT = '/usr/share/fonts/truetype/nanum/NanumSquare_acR.ttf'
DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

BADGE = int(CFG['stack']['marker_id'])
PLACES = {int(p['marker_id']): p['label'] for p in (CFG.get('nav', {}).get('places') or {}).values()}
# (번호, 쓰임) — 납작한 카드
FLAT = [(BADGE, '따라가기 배지 ("따라와")'), (BADGE, '따라가기 배지 · 예비'),
        *[(i, f'장소: {label} ("{label}으로 가")') for i, label in PLACES.items()],
        (3, '번호 따라가기 ("3번 마커 따라와")'), (4, '번호 따라가기 · 예비'),
        (7, '가서 정렬 ("7번 마커 앞으로 가")'), (8, '예비 장소 A'), (9, '예비 장소 B'),
        (10, '작업대 높이 (작업대 위에 눕혀 둔다. 팔 카메라가 잰다)')]      # config/arm.yaml workspace.table_marker
# 반으로 접어 세우는 것. 양면에 같은 마커가 온다
TENT = [(i, label) for i, label in PLACES.items()] + [(7, '가서 정렬')]


def font(mm: float):
    return ImageFont.truetype(FONT, round(mm * PX))


def marker(mid: int) -> Image.Image:
    side = round(SIZE_MM * PX)
    draw = getattr(cv2.aruco, 'generateImageMarker', None) or cv2.aruco.drawMarker      # OpenCV 4.7에서 이름이 바뀌었다
    bits = draw(DICT, mid, 6)                                    # 4x4 + 테두리 = 6칸
    return Image.fromarray(cv2.resize(bits, (side, side), interpolation=cv2.INTER_NEAREST))


def dashed(d: ImageDraw.ImageDraw, box, step_mm=3.0):
    x0, y0, x1, y1 = box
    s = step_mm * PX
    for a, b, horizontal in ((x0, x1, True), (y0, y1, False)):
        t = a
        while t < b:
            e = min(t + s / 2, b)
            for fixed in ((y0, y1) if horizontal else (x0, x1)):
                d.line([(t, fixed), (e, fixed)] if horizontal else [(fixed, t), (fixed, e)], fill=150, width=3)
            t += s


def ruler(d: ImageDraw.ImageDraw, x_mm: float, y_mm: float):
    """10 cm 자. 인쇄 배율 확인용."""
    x, y = x_mm * PX, y_mm * PX
    d.line([(x, y), (x + 100 * PX, y)], fill=0, width=4)
    for i in range(11):
        d.line([(x + i * 10 * PX, y - 2.5 * PX), (x + i * 10 * PX, y)], fill=0, width=4)
    d.text((x + 103 * PX, y - 3.5 * PX), '← 정확히 10 cm여야 한다 (인쇄: 실제 크기 100 %)', font=font(3.2), fill=0)


def card(mid: int, rotate=False) -> Image.Image:
    """흰 여백까지 들어간 카드 한 장. 귀퉁이에 작은 번호(여백 가장자리 4 mm 안)."""
    side = round(CARD_MM * PX)
    im = Image.new('L', (side, side), 255)
    m = marker(mid)
    im.paste(m, ((side - m.width) // 2,) * 2)
    d = ImageDraw.Draw(im)
    d.text((2 * PX, 1.2 * PX), f'ID {mid} ▲위', font=font(3.0), fill=120)
    return im.rotate(180) if rotate else im


def flat_page(items) -> Image.Image:
    page = Image.new('L', (W, H), 255)
    d = ImageDraw.Draw(page)
    d.text((15 * PX, 9 * PX), f'막내 부스 ArUco 마커 · DICT_4X4_50 · 검은 변 {SIZE_MM / 10:g} cm · 점선을 따라 자른다(흰 여백은 남긴다)', font=font(3.4), fill=0)
    for k, (mid, use) in enumerate(items):
        x, y = (W - CARD_MM * PX) / 2, (22 + k * (CARD_MM + 14)) * PX
        c = card(mid)
        page.paste(c, (round(x), round(y)))
        dashed(d, (round(x), round(y), round(x) + c.width, round(y) + c.height))
        d.text((x, y + c.height + 2 * PX), f'ID {mid} — {use}', font=font(4.6), fill=0)
    ruler(d, 15, 288)
    return page


def tent_page(mid: int, use: str) -> Image.Image:
    """가운데를 접어 ㅅ자로 세운다. 위쪽 절반은 뒤집어 찍어서 접으면 양면이 다 바로 선다."""
    page = Image.new('L', (W, H), 255)
    d = ImageDraw.Draw(page)
    mid_y = H // 2
    c = card(mid)
    x = (W - c.width) // 2
    low_y = mid_y + round(7 * PX)
    page.paste(c, (x, low_y))
    page.paste(card(mid, rotate=True), (x, mid_y - round(7 * PX) - c.height))
    for t in range(0, W, round(4 * PX)):
        d.line([(t, mid_y), (t + round(2 * PX), mid_y)], fill=110, width=3)
    d.text((15 * PX, mid_y + 2 * PX), f'↑ 여기를 접어 세운다 (ID {mid} — {use} · 세움형, 양면)', font=font(3.6), fill=0)
    d.text((15 * PX, low_y + c.height + 3 * PX), '아래 끝을 2 cm쯤 안으로 접어 테이프로 바닥에 붙이면 덜 쓰러진다. 면은 되도록 곧게 세운다.', font=font(3.2), fill=0)
    ruler(d, 15, 12)
    return page


def main() -> None:
    pages = [flat_page(FLAT[i:i + 2]) for i in range(0, len(FLAT), 2)] + [tent_page(i, u) for i, u in TENT]
    out = ROOT / 'docs' / 'aruco_markers.pdf'
    pages[0].save(out, save_all=True, append_images=pages[1:], resolution=DPI)
    # 다시 읽어 검출해 본다: 번호와 변 길이
    for n, page in enumerate(pages, 1):
        small = cv2.resize(np.array(page), None, fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)
        corners, ids, _ = (cv2.aruco.ArucoDetector(DICT, cv2.aruco.DetectorParameters()).detectMarkers(small)
                           if hasattr(cv2.aruco, 'ArucoDetector') else cv2.aruco.detectMarkers(small, DICT))
        sides = [np.linalg.norm(c[0][0] - c[0][1]) / 0.25 / PX for c in corners]
        print(f'{n}쪽: ID {sorted(int(i) for i in ids.flatten())}, 변 {", ".join(f"{s:.1f}" for s in sides)} mm')
    print(f'{out} ({len(pages)}쪽)')


if __name__ == '__main__':
    main()
