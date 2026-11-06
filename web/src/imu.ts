// IMU 패널. 주황은 IMU에서 나온 값, 밝은 회색은 바퀴에서 나온 값(트윈 경로 색과 같다).
import type { State } from './types';
import { C, drawChart, el, fmt, font, Series, surface } from './ui';

const SPAN = 12;
const RAD2DEG = 180 / Math.PI;

export class ImuPanel {
  private g = el<HTMLCanvasElement>('gmeter');
  private cy = el<HTMLCanvasElement>('chart-yaw');
  private cs = el<HTMLCanvasElement>('chart-speed');
  private meta = el('imu-meta');
  private yawGyro = new Series(SPAN); private yawWheel = new Series(SPAN);
  private vWheel = new Series(SPAN); private vImu = new Series(SPAN);
  private tail: { t: number; x: number; y: number }[] = [];
  private s: State | null = null;
  private shown: State['imu'] = null; private shownAt = 0;

  onState(s: State) {
    this.s = s;
    const t = performance.now() / 1000;
    const m = s.imu;
    this.yawGyro.push(t, m?.gyro_z != null ? m.gyro_z * RAD2DEG : null);
    this.yawWheel.push(t, m?.odom_w != null ? m.odom_w * RAD2DEG : s.odom ? s.odom.w * RAD2DEG : null);
    this.vWheel.push(t, m?.v_wheel ?? null);
    this.vImu.push(t, m?.v_imu ?? null);
    if (m?.ax != null && m.ay != null) {
      this.tail.push({ t, x: m.ax, y: m.ay });
      while (this.tail.length && t - this.tail[0].t > 1.2) this.tail.shift();
    }
    this.meta.textContent = `${fmt(s.rates.imu, 0)} Hz  ICM-42688-P`;
    if (performance.now() - this.shownAt > 150) { this.shown = m; this.shownAt = performance.now(); }
  }

  draw() {
    this.drawG();
    this.drawYaw();
    this.drawSpeed();
  }

  private drawG() {
    const sf = surface(this.g);
    if (!sf) return;
    const { ctx, w, h } = sf;
    const m = this.shown;
    ctx.font = font(400, 13); ctx.fillStyle = C.sub; ctx.textBaseline = 'top'; ctx.textAlign = 'left';
    ctx.fillText('가속도  전후 · 좌우', 0, 2);
    const R = Math.min(w / 2 - 10, (h - 110) / 2);
    const cx = w / 2, cy = 26 + R;
    const MAX = 0.6;   // m/s²
    ctx.strokeStyle = C.line; ctx.lineWidth = 1;
    for (const r of [0.2, 0.4, 0.6]) { ctx.beginPath(); ctx.arc(cx, cy, (r / MAX) * R, 0, Math.PI * 2); ctx.stroke(); }
    ctx.beginPath(); ctx.moveTo(cx - R, cy); ctx.lineTo(cx + R, cy); ctx.moveTo(cx, cy - R); ctx.lineTo(cx, cy + R); ctx.stroke();
    ctx.font = font(400, 11); ctx.fillStyle = C.sub; ctx.textAlign = 'center'; ctx.textBaseline = 'bottom';
    ctx.fillText('가속', cx, cy - R - 1);
    ctx.textBaseline = 'top'; ctx.fillText('감속', cx, cy + R + 1);
    ctx.font = font(400, 10, true); ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
    ctx.fillText('0.6', cx + R * 0.72, cy + R * 0.72);

    const now = performance.now() / 1000;
    // 화면 위 = 전진 가속(+x), 왼쪽 = 왼쪽 가속(+y)
    const P = (ax: number, ay: number) => {
      const k = R / MAX, mag = Math.hypot(ax, ay), lim = mag > MAX ? MAX / mag : 1;
      return [cx - ay * lim * k, cy - ax * lim * k];
    };
    for (let i = 1; i < this.tail.length; i++) {
      const a = this.tail[i - 1], b = this.tail[i];
      const alpha = Math.max(0, 1 - (now - b.t) / 1.2);
      ctx.strokeStyle = `rgba(255,122,47,${(alpha * 0.8).toFixed(3)})`; ctx.lineWidth = 2;
      const [x0, y0] = P(a.x, a.y), [x1, y1] = P(b.x, b.y);
      ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(x1, y1); ctx.stroke();
    }
    const last = this.tail[this.tail.length - 1];
    if (last) {
      const [x, y] = P(last.x, last.y);
      ctx.fillStyle = C.warn; ctx.beginPath(); ctx.arc(x, y, 6, 0, Math.PI * 2); ctx.fill();
    }

    // 판독
    let ty = cy + R + 22;
    const row = (k: string, v: string) => {
      ctx.font = font(400, 12); ctx.fillStyle = C.sub; ctx.textAlign = 'left'; ctx.textBaseline = 'top';
      ctx.fillText(k, 0, ty);
      ctx.font = font(400, 12, true); ctx.fillStyle = C.title; ctx.textAlign = 'right';
      ctx.fillText(v, w, ty);
      ty += 19;
    };
    row('전후 · 좌우', m ? `${fmt(m.ax, 2, true)}  ${fmt(m.ay, 2, true)}` : '-');
    row('기울기 앞뒤 · 좌우', m ? `${fmt(m.pitch, 1, true)}°  ${fmt(m.roll, 1, true)}°` : '-');
    const hd = (r: number | null | undefined) => (r == null ? '-' : `${((r * RAD2DEG) % 360 + 360) % 360 | 0}°`);
    row('방위 자이로 · 바퀴', m ? `${hd(m.heading_gyro)}  ${hd(this.s?.odom?.yaw)}` : '-');
  }

  /** 차트 머리: 제목과 단위는 왼쪽, 색 범례는 오른쪽. 한글은 산세리프, 단위는 고정폭. */
  private head(ctx: CanvasRenderingContext2D, w: number, title: string, unit: string,
               legend: [string, string][]) {
    ctx.textBaseline = 'top'; ctx.textAlign = 'left';
    ctx.font = font(400, 13); ctx.fillStyle = C.sub;
    ctx.fillText(title, 0, 2);
    const tw = ctx.measureText(title).width;
    ctx.font = font(400, 11, true);
    ctx.fillText(unit, tw + 6, 4);
    ctx.textAlign = 'right'; ctx.font = font(600, 12);
    let x = w;
    for (const [name, color] of [...legend].reverse()) {
      ctx.fillStyle = color; ctx.fillText(name, x, 3);
      x -= ctx.measureText(name).width + 12;
    }
  }

  private drawYaw() {
    const sf = surface(this.cy);
    if (!sf) return;
    const { ctx, w, h } = sf;
    this.head(ctx, w, '회전 속도', '°/s', [['자이로', C.warn], ['바퀴', C.body]]);
    const all = [...this.yawGyro.y, ...this.yawWheel.y].filter((v): v is number => v !== null);
    const peak = Math.max(30, ...all.map(Math.abs));
    const lim = Math.ceil(peak / 30) * 30;
    drawChart(ctx, 34, 26, w - 36, h - 32,
      [{ s: this.yawWheel, color: C.body }, { s: this.yawGyro, color: C.warn }],
      -lim, lim, performance.now() / 1000, SPAN, [-lim, 0, lim], (v) => `${v}`);
  }

  private drawSpeed() {
    const sf = surface(this.cs);
    if (!sf) return;
    const { ctx, w, h } = sf;
    this.head(ctx, w, '전진 속도', 'm/s', [['IMU', C.warn], ['바퀴', C.body]]);
    const all = [...this.vWheel.y, ...this.vImu.y].filter((v): v is number => v !== null);
    const peak = Math.max(0.3, ...all.map(Math.abs));
    const lim = Math.ceil(peak * 10) / 10;
    drawChart(ctx, 34, 26, w - 36, h - 32,
      [{ s: this.vWheel, color: C.body }, { s: this.vImu, color: C.warn }],
      -lim, lim, performance.now() / 1000, SPAN, [-lim, 0, lim], (v) => v.toFixed(1));
  }
}
