import math

import numpy as np

from booth_bridge.scan import ScanShaper, simplify


def room_scan(n=720, half=2.0, noise=0.0):
    """정사각형 방 한가운데에서 본 한 바퀴."""
    a = -math.pi + 2 * math.pi * np.arange(n) / n
    r = half / np.maximum(np.abs(np.cos(a)), np.abs(np.sin(a)))
    return r + np.random.default_rng(1).normal(0, noise, n), a


def shape(r, cfg=None, **kw):
    n = len(r)
    return ScanShaper({'xyz_m': [0, 0, 0.25], 'rpy_deg': [0, 0, 0], **(cfg or {})}).shape(
        r, -math.pi, 2 * math.pi / max(n, 1), 0.05, 12.0, **kw)


def test_square_room_becomes_one_closed_outline_with_four_corners():
    r, _ = room_scan(noise=0.004)
    out = shape(r)
    assert len(out['points']) == 720
    assert len(out['lines']) == 1 and not out['posts']
    line = out['lines'][0]
    assert 5 <= len(line) <= 14                      # 720점이 꼭짓점 몇 개로 줄었다
    assert np.abs(np.abs(line).max(axis=1) - 2.0).max() < 0.05    # 모든 꼭짓점이 벽 위에 있다
    assert abs(out['nearest'][0] - 2.0) < 0.02


def test_mount_yaw_and_offset_are_applied():
    r = np.full(360, np.inf)
    r[180] = 1.0                                     # 라이다 기준 각도 0
    r[179] = r[181] = 1.0
    s = ScanShaper({'xyz_m': [0.1, 0.0, 0.25], 'rpy_deg': [0, 0, 180.0]})
    out = s.shape(r, -math.pi, 2 * math.pi / 360, 0.05, 12.0)
    assert np.allclose(out['points'][1], [0.1 - 1.0, 0.0], atol=1e-6)     # 뒤를 보고 달렸다


def test_lone_spike_and_self_hits_are_dropped():
    r, _ = room_scan()
    r[100] = 0.6                                     # 한 점만 튐
    r[300:310] = 0.1                                 # 로봇 자기 몸
    out = shape(r)
    d = np.hypot(*out['points'].T)
    assert d.min() > 1.9
    assert len(out['points']) == 710


def test_legs_become_posts_and_split_the_wall():
    r, a = room_scan()
    for centre in (350, 372):                        # 앞쪽 1 m에 가는 다리 둘
        r[centre:centre + 3] = 1.0
    out = shape(r)
    assert len(out['posts']) == 2
    assert all(abs(math.hypot(x, y) - 1.0) < 0.02 for x, y in out['posts'])
    assert out['nearest'][0] < 1.01


def test_pack_is_int16_millimetres():
    r, _ = room_scan()
    s = ScanShaper({'xyz_m': [0, 0, 0.25], 'rpy_deg': [0, 0, 0]})
    shaped = s.shape(r, -math.pi, 2 * math.pi / 720, 0.05, 12.0)
    header, body = s.pack(shaped, (1.0, 2.0, 0.5))
    assert header['type'] == 'scan' and header['n'] == 720 and header['pose'] == [1.0, 2.0, 0.5]
    pts = np.frombuffer(body, '<i2').reshape(-1, 2) / 1000
    assert np.abs(pts - shaped['points']).max() < 0.001
    assert all(len(line) % 2 == 0 and len(line) >= 4 for line in header['lines'])


def test_empty_and_all_invalid_scans():
    assert shape([])['nearest'] is None
    out = shape(np.full(100, np.inf))
    assert len(out['points']) == 0 and out['lines'] == [] and out['nearest'] is None


def test_simplify_keeps_corner():
    pts = np.array([[0, 0], [1, 0.001], [2, 0], [2, 1], [2, 2]], float)
    assert simplify(pts, 0.03).tolist() == [[0, 0], [2, 0], [2, 2]]


def test_cart_frame_posts_inside_the_self_box_are_dropped():
    r, a = room_scan()
    for centre in (90, 270, 450, 630):               # 네 모서리 기둥 0.26 m
        r[centre:centre + 4] = 0.26
    out = shape(r, {'self_box_m': [-0.25, 0.31, -0.25, 0.25]})
    assert not out['posts'] and np.hypot(*out['points'].T).min() > 1.9
    assert np.hypot(*shape(r)['points'].T).min() < 0.3            # 상자가 없으면 그대로 남는다
