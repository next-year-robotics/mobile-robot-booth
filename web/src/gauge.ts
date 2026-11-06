// 바퀴 하나의 속도계. 자동차 계기판처럼 240° 눈금에 바늘, 바깥 삼각형은 지령에서 역산한 목표 속도.
import type { Wheel } from './types';
import { C, clamp, el, fmt, font, Spring, surface } from './ui';

const LO = -0.4, HI = 0.6;                 // m/s. 빠른 모드 최대 0.5 m/s + 제자리 회전 여유
const A0 = (150 * Math.PI) / 180, SWEEP = (240 * Math.PI) / 180;
const ang = (v: number) => A0 + ((clamp(v, LO, HI) - LO) / (HI - LO)) * SWEEP;

export class Gauge {
  private canvas: HTMLCanvasElement;
  private needle = new Spring(12);
  private target = new Spring(10);
  private w: Wheel | null = null;
  private shown = { v: 0, rpm: 0, at: 0 };

  constructor(id: string, private label: string) { this.canvas = el<HTMLCanvasElement>(id); }

  set(w: Wheel) { this.w = w; }

  draw(dt: number) {
    const sf = surface(this.canvas);
    if (!sf) return;
    const { ctx, w, h } = sf;
    const wh = this.w;
    const v = this.needle.step(wh?.v ?? 0, dt);
    const hasTarget = wh?.target !== null && wh?.target !== undefined;
    const tv = this.target.step(hasTarget ? wh!.target! : 0, dt);

    // 눈금 숫자는 호 바깥, 큰 숫자는 호가 열린 아래쪽에 둔다(서로 겹치지 않게).
    const R = Math.max(40, Math.min(w / 2 - 30, (h - 92) / 1.55));
    const cx = w / 2, cy = R + 40;

    ctx.font = font(600, 15);
    ctx.fillStyle = C.body; ctx.textBaseline = 'top'; ctx.textAlign = 'left';
    ctx.fillText(this.label, 4, 2);

    ctx.lineCap = 'butt';
    ctx.strokeStyle = C.line; ctx.lineWidth = 10;
    ctx.beginPath(); ctx.arc(cx, cy, R, A0, A0 + SWEEP); ctx.stroke();
    const sat = !!wh?.sat;
    ctx.strokeStyle = sat ? C.warn : C.title;
    const a0 = ang(0), av = ang(v);
    ctx.beginPath(); ctx.arc(cx, cy, R, Math.min(a0, av), Math.max(a0, av)); ctx.stroke();

    for (let i = 0; i <= Math.round((HI - LO) / 0.05); i++) {
      const val = LO + i * 0.05;
      const major = Math.abs(val * 10 - Math.round(val * 10)) < 1e-6;
      const a = ang(val), r1 = R - 9, r2 = r1 - (major ? 10 : 5);
      ctx.strokeStyle = Math.abs(val) < 1e-6 ? C.title : major ? C.sub : C.line2;
      ctx.lineWidth = major ? 2 : 1;
      ctx.beginPath();
      ctx.moveTo(cx + Math.cos(a) * r1, cy + Math.sin(a) * r1);
      ctx.lineTo(cx + Math.cos(a) * r2, cy + Math.sin(a) * r2);
      ctx.stroke();
      if (major && Math.abs(Math.round(val * 10)) % 2 === 0) {
        const rl = R + 17;
        ctx.font = font(400, 12, true); ctx.fillStyle = Math.abs(val) < 1e-6 ? C.title : C.sub;
        ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
        ctx.fillText(Math.abs(val) < 1e-6 ? '0' : val.toFixed(1), cx + Math.cos(a) * rl, cy + Math.sin(a) * rl);
      }
    }

    // 목표 속도 (twist_mux 출력 → MCU 역기구학). 호를 가로지르는 짧은 막대
    if (hasTarget) {
      const a = ang(tv);
      ctx.strokeStyle = C.body; ctx.lineWidth = 4; ctx.lineCap = 'round';
      ctx.beginPath();
      ctx.moveTo(cx + Math.cos(a) * (R - 8), cy + Math.sin(a) * (R - 8));
      ctx.lineTo(cx + Math.cos(a) * (R + 8), cy + Math.sin(a) * (R + 8));
      ctx.stroke();
    }

    // 바늘
    ctx.strokeStyle = sat ? C.warn : C.title; ctx.lineWidth = 3; ctx.lineCap = 'round';
    ctx.beginPath();
    ctx.moveTo(cx - Math.cos(av) * 12, cy - Math.sin(av) * 12);
    ctx.lineTo(cx + Math.cos(av) * (R - 16), cy + Math.sin(av) * (R - 16));
    ctx.stroke();
    ctx.fillStyle = C.panel; ctx.strokeStyle = C.title; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(cx, cy, 6, 0, Math.PI * 2); ctx.fill(); ctx.stroke();

    // 큰 숫자는 10 Hz로만 갱신
    const now = performance.now();
    if (now - this.shown.at > 100) { this.shown = { v: wh?.v ?? 0, rpm: wh?.rpm ?? 0, at: now }; }
    const live = wh?.v !== null && wh?.v !== undefined;
    ctx.textAlign = 'center'; ctx.textBaseline = 'alphabetic';
    ctx.font = font(900, Math.round(Math.min(46, R * 0.46)));
    ctx.fillStyle = live ? C.title : C.line2;
    const txt = live ? fmt(this.shown.v, 2) : '-';
    ctx.save(); ctx.translate(cx, cy + R * 0.58);
    (ctx as CanvasRenderingContext2D & { letterSpacing: string }).letterSpacing = '-1.5px';
    ctx.fillText(txt, 0, 0);
    ctx.restore();
    ctx.font = font(400, 12, true); ctx.fillStyle = C.sub;
    ctx.fillText(`m/s   ${live ? fmt(this.shown.rpm, 0) : '-'} rpm`, cx, cy + R * 0.58 + 18);

    // PWM 듀티
    const by = h - 14, bw = Math.min(w - 16, 2 * R), bx = cx - bw / 2;
    const duty = wh?.duty ?? null;
    ctx.fillStyle = C.line; ctx.fillRect(bx, by, bw, 4);
    ctx.fillStyle = C.line2; ctx.fillRect(cx - 1, by - 3, 2, 10);
    if (duty !== null) {
      ctx.fillStyle = sat ? C.warn : C.body;
      const dw = clamp(duty, -1, 1) * bw / 2;
      ctx.fillRect(Math.min(cx, cx + dw), by, Math.abs(dw), 4);
    }
    ctx.font = font(400, 11, true); ctx.fillStyle = sat ? C.warn : C.sub;
    ctx.textAlign = 'left'; ctx.textBaseline = 'bottom';
    ctx.fillText('PWM', bx, by - 4);
    ctx.textAlign = 'right';
    ctx.fillText(duty === null ? '-' : `${sat ? 'SAT ' : ''}${fmt(duty * 100, 0, true)} %`, bx + bw, by - 4);
    if (hasTarget) {
      // 한글은 고정폭 글꼴에 넣지 않는다(템플릿 규칙). 숫자와 낱말을 따로 그린다.
      ctx.textBaseline = 'top'; ctx.fillStyle = C.sub; ctx.textAlign = 'right';
      ctx.font = font(400, 12, true);
      const num = fmt(wh!.target, 2);
      ctx.fillText(num, w - 4, 4);
      const nw = ctx.measureText(num).width;
      ctx.font = font(400, 12);
      ctx.fillText('목표', w - 10 - nw, 3);
    }
  }
}
