// 캔버스 공통 도구. 캔버스 안의 크기는 전부 "설계 px"(1920x1200 화면 기준)로 쓰고 ui()를 곱한다.

export const C = {
  bg: '#0E1216', panel: '#18212A', line: '#26313B', line2: '#3A4652',
  title: '#F2F4F5', body: '#C9D3DB', sub: '#9AA7B2', accent: '#7CFF4F', warn: '#FF7A2F',
};
export const SANS = "'Pretendard', Arial, sans-serif";
export const MONO = "'JetBrains Mono', 'Courier New', monospace";

/** 1 설계 px가 몇 CSS px인가. 루트 글꼴 크기 16 px이 기준이다. */
export function ui(): number {
  return parseFloat(getComputedStyle(document.documentElement).fontSize) / 16;
}

export interface Surface { ctx: CanvasRenderingContext2D; w: number; h: number; u: number }

/** 캔버스 백버퍼를 표시 크기 × devicePixelRatio에 맞추고, 설계 px 단위로 그릴 수 있게 변환을 건다. */
export function surface(canvas: HTMLCanvasElement): Surface | null {
  const rect = canvas.getBoundingClientRect();
  if (rect.width < 2 || rect.height < 2) return null;
  const dpr = window.devicePixelRatio || 1;
  const bw = Math.round(rect.width * dpr), bh = Math.round(rect.height * dpr);
  if (canvas.width !== bw || canvas.height !== bh) { canvas.width = bw; canvas.height = bh; }
  const ctx = canvas.getContext('2d')!;
  const u = ui();
  ctx.setTransform(dpr * u, 0, 0, dpr * u, 0, 0);
  ctx.clearRect(0, 0, rect.width / u, rect.height / u);
  return { ctx, w: rect.width / u, h: rect.height / u, u };
}

export function font(weight: number, sizePx: number, mono = false): string {
  return `${weight} ${sizePx}px ${mono ? MONO : SANS}`;
}

export function fmt(x: number | null | undefined, digits = 2, signed = false): string {
  if (x === null || x === undefined || !Number.isFinite(x)) return '-';
  const s = x.toFixed(digits);
  if (signed && x >= 0) return '+' + s;
  // -0.00은 0.00으로
  return Number(s) === 0 ? (0).toFixed(digits) : s;
}

export const clamp = (x: number, a: number, b: number) => Math.min(b, Math.max(a, x));

/** 임계 감쇠 스프링. 바늘이 30 Hz 값 사이를 부드럽게 따라간다. */
export class Spring {
  x = 0; v = 0;
  constructor(public omega = 14) {}
  step(target: number, dt: number): number {
    const w = this.omega;
    const f = 1 + 2 * dt * w, oo = w * w, hoo = dt * oo, hhoo = dt * hoo;
    const det = 1 / (f + hhoo);
    const x = det * (f * this.x + dt * this.v + hhoo * target);
    this.v = det * (this.v + hoo * (target - this.x));
    this.x = x;
    return x;
  }
}

/** 고정 길이 시계열. 차트용. */
export class Series {
  t: number[] = []; y: (number | null)[] = [];
  constructor(public span_s: number) {}
  push(t: number, y: number | null) {
    this.t.push(t); this.y.push(y);
    while (this.t.length && t - this.t[0] > this.span_s) { this.t.shift(); this.y.shift(); }
  }
}

export interface ChartLine { s: Series; color: string; width?: number }

/** 시간축 꺾은선. 좌표는 설계 px. */
export function drawChart(ctx: CanvasRenderingContext2D, x: number, y: number, w: number, h: number,
                          lines: ChartLine[], lo: number, hi: number, now: number, span: number,
                          ticks: number[], tickFmt: (v: number) => string) {
  ctx.save();
  ctx.font = font(400, 11, true);
  ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
  for (const tv of ticks) {
    const ty = y + h - (tv - lo) / (hi - lo) * h;
    ctx.strokeStyle = tv === 0 ? C.line2 : C.line; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(x, ty); ctx.lineTo(x + w, ty); ctx.stroke();
    ctx.fillStyle = C.sub; ctx.fillText(tickFmt(tv), x - 6, ty);
  }
  ctx.beginPath(); ctx.rect(x, y - 2, w, h + 4); ctx.clip();
  for (const ln of lines) {
    ctx.strokeStyle = ln.color; ctx.lineWidth = ln.width ?? 2; ctx.lineJoin = 'round';
    ctx.beginPath();
    let pen = false;
    for (let i = 0; i < ln.s.t.length; i++) {
      const v = ln.s.y[i];
      if (v === null) { pen = false; continue; }
      const px = x + w - (now - ln.s.t[i]) / span * w;
      const py = y + h - (clamp(v, lo, hi) - lo) / (hi - lo) * h;
      if (pen) ctx.lineTo(px, py); else ctx.moveTo(px, py);
      pen = true;
    }
    ctx.stroke();
  }
  ctx.restore();
}

export function el<T extends HTMLElement = HTMLElement>(id: string): T {
  return document.getElementById(id) as T;
}

export function hms(ms: number): string {
  const d = new Date(ms);
  return [d.getHours(), d.getMinutes(), d.getSeconds()].map((n) => String(n).padStart(2, '0')).join(':');
}

/** 경과 시간. 고정폭 글꼴에 들어가므로 단위는 영문이다. */
export function ago(sec: number | null | undefined): string {
  if (sec === null || sec === undefined || !Number.isFinite(sec)) return '-';
  if (sec < 90) return `${Math.round(sec)} s`;
  if (sec < 5400) return `${Math.round(sec / 60)} min`;
  if (sec < 172800) return `${Math.round(sec / 3600)} h`;
  return `${Math.round(sec / 86400)} d`;
}

/** 오늘이면 시:분:초, 아니면 월-일 시:분. */
export function stamp(ms: number): string {
  const d = new Date(ms), now = new Date();
  if (d.toDateString() === now.toDateString()) return hms(ms);
  const p = (n: number) => String(n).padStart(2, '0');
  return `${p(d.getMonth() + 1)}-${p(d.getDate())} ${p(d.getHours())}:${p(d.getMinutes())}`;
}
