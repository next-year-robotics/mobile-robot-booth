// 카메라 패널. 받은 JPEG를 그리고, 같은 프레임에 로봇이 검출한 마커를 다시 그린다.
import type { FrameHeader, OverlayMarker, State } from './types';
import { C, el, fmt, font, surface } from './ui';

export class CameraPanel {
  private canvas = el<HTMLCanvasElement>('cam-canvas');
  private empty = el('cam-empty');
  private meta = el('cam-meta');
  private bitmap: ImageBitmap | null = null;
  private overlay: OverlayMarker[] = [];
  private overlayAt = 0;
  private imgW = 1280; private imgH = 720;
  private decoding = false;
  private pending: { h: FrameHeader; blob: Blob } | null = null;
  private frameTimes: number[] = [];
  private lastFrameAt = 0;
  private targetId = 0;
  private trackStatus: 'lost' | 'tracking' | 'holding' = 'lost';

  // 판독 칸
  private status = el('mk-status'); private mid = el('mk-id'); private range = el('mk-range');
  private bearing = el('mk-bearing'); private xy = el('mk-xy'); private yaw = el('mk-yaw');
  private bigBox = this.range.parentElement!;
  private lastText = 0;

  onFrame(h: FrameHeader, blob: Blob) {
    // 디코드가 밀리면 가장 최근 것 하나만 남긴다. 쌓아 두면 지연이 늘기만 한다.
    if (this.decoding) { this.pending = { h, blob }; return; }
    this.decode(h, blob);
  }

  private decode(h: FrameHeader, blob: Blob) {
    this.decoding = true;
    createImageBitmap(blob).then((bmp) => {
      this.bitmap?.close();
      this.bitmap = bmp;
      this.imgW = h.w || bmp.width; this.imgH = h.h || bmp.height;
      const now = performance.now();
      if (h.overlay) { this.overlay = h.overlay; this.overlayAt = now; }
      this.frameTimes.push(now);
      while (this.frameTimes.length && now - this.frameTimes[0] > 2000) this.frameTimes.shift();
      this.lastFrameAt = now;
    }).catch(() => {}).finally(() => {
      this.decoding = false;
      const p = this.pending; this.pending = null;
      if (p) this.decode(p.h, p.blob);
    });
  }

  onState(s: State) {
    const m = s.marker;
    this.targetId = m?.id ?? 0;
    this.trackStatus = m?.status ?? 'lost';
    const now = performance.now();
    if (now - this.lastText < 120) return;      // 숫자는 8 Hz로만 바꾼다. 30 Hz로 바뀌면 못 읽는다
    this.lastText = now;
    const st = m?.status ?? 'lost';
    this.status.className = 'chip ' + (st === 'lost' ? '' : st);
    this.status.textContent = !m ? '추적 없음' : st === 'tracking' ? (m.locked ? '추적 고정' : '추적 중')
      : st === 'holding' ? `유지 ${fmt(m.hold_s, 1)} s` : '마커 찾는 중';
    this.mid.textContent = m ? `대상 ID ${m.id}` : '';
    this.bigBox.className = 'ro-big ' + (st === 'lost' ? '' : st);
    const show = m && st !== 'lost';
    this.range.textContent = show ? fmt(m!.range, 2) : '-';
    this.bearing.textContent = show ? `${fmt(m!.bearing, 1, true)}°` : '-';
    this.xy.textContent = show ? `${fmt(m!.x, 2)}  ${fmt(m!.y, 2, true)} m` : '-';
    this.yaw.textContent = show && m!.yaw_valid ? `${fmt(((m!.yaw! * 180 / Math.PI) + 540) % 360 - 180, 0, true)}°` : '-';
  }

  draw() {
    const now = performance.now();
    const fps = this.frameTimes.length / 2;
    const live = now - this.lastFrameAt < 1500;
    this.empty.style.display = live && this.bitmap ? 'none' : 'grid';
    this.meta.textContent = live ? `${this.imgW}x${this.imgH}  ${fps.toFixed(0)} fps` : '신호 없음';

    const sf = surface(this.canvas);
    if (!sf) return;
    const { ctx, w, h } = sf;
    if (!this.bitmap || !live) return;
    // 화면에 맞춰 잘림 없이 그린다. 오버레이 좌표도 같은 변환을 탄다.
    const k = Math.min(w / this.imgW, h / this.imgH);
    const ox = (w - this.imgW * k) / 2, oy = (h - this.imgH * k) / 2;
    ctx.drawImage(this.bitmap, ox, oy, this.imgW * k, this.imgH * k);

    // 동기 오버레이가 없는 프레임(검출기가 건너뜀)에는 150 ms 안의 직전 것을 쓴다
    if (now - this.overlayAt > 150) return;
    const P = (p: [number, number]) => [ox + p[0] * k, oy + p[1] * k] as const;
    for (const mk of this.overlay) {
      const isTarget = mk.id === this.targetId;
      const col = isTarget && this.trackStatus !== 'lost' ? C.accent : C.title;
      const c = mk.corners.map(P);
      ctx.save();
      ctx.shadowColor = 'rgba(0,0,0,0.6)'; ctx.shadowBlur = 6;
      ctx.strokeStyle = col; ctx.lineWidth = isTarget ? 3 : 2; ctx.lineJoin = 'round';
      ctx.beginPath(); c.forEach(([x, y], i) => (i ? ctx.lineTo(x, y) : ctx.moveTo(x, y))); ctx.closePath(); ctx.stroke();
      ctx.fillStyle = isTarget ? 'rgba(124,255,79,0.10)' : 'rgba(242,244,245,0.06)'; ctx.fill();
      ctx.restore();
      // 모서리 0번(좌상)을 표시해 마커 방향이 보이게 한다
      ctx.fillStyle = col;
      ctx.beginPath(); ctx.arc(c[0][0], c[0][1], 4, 0, Math.PI * 2); ctx.fill();
      // 축: x·y는 흐리게, 법선(z)은 강조
      const [o, ax, ay, az] = mk.axes.map(P);
      ctx.lineWidth = 2;
      ctx.strokeStyle = C.sub;
      for (const e of [ax, ay]) { ctx.beginPath(); ctx.moveTo(o[0], o[1]); ctx.lineTo(e[0], e[1]); ctx.stroke(); }
      ctx.strokeStyle = col; ctx.lineWidth = 3;
      ctx.beginPath(); ctx.moveTo(o[0], o[1]); ctx.lineTo(az[0], az[1]); ctx.stroke();
      // 이름표
      const top = c.reduce((a, b) => (b[1] < a[1] ? b : a));
      const label = `ID ${mk.id}  ${mk.dist.toFixed(2)} m`;
      ctx.font = font(700, 14, true);
      const tw = ctx.measureText(label).width;
      const lx = Math.min(Math.max(top[0] - tw / 2 - 8, 4), w - tw - 20), ly = Math.max(top[1] - 34, 4);
      ctx.fillStyle = 'rgba(14,18,22,0.85)';
      ctx.fillRect(lx, ly, tw + 16, 24);
      ctx.fillStyle = col; ctx.textBaseline = 'middle';
      ctx.fillText(label, lx + 8, ly + 12.5);
    }
  }
}
