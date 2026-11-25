// 탑뷰 지도. 라이다 스캔을 이동 노드의 좌표(EKF)로 쌓아 벽을 그리고, 그 위에 로봇·시작 위치·장소·가는 경로를 얹는다.
// 지도(SLAM)는 아니다: 지금까지 본 점을 잠깐 기억해 둘 뿐이다. 배치 패널(layout.ts)과 아바타 화면의 왼쪽 창이 같이 쓴다.
//
//   MapStore   스캔과 상태를 받아 쌓아 둔다(화면 하나에 하나)
//   MiniMap    캔버스 하나에 그린다. 위쪽이 시작 위치에서 본 앞이다
import type { ScanHeader } from './lidar';
import type { Hello, Place, State } from './types';

type Pose = [number, number, number];
const CELL = 0.05;
const BODY = { front: 0.31, rear: 0.25, half: 0.25 };

function compose(a: Pose, x: number, y: number): [number, number] {
  const c = Math.cos(a[2]), s = Math.sin(a[2]);
  return [a[0] + c * x - s * y, a[1] + s * x + c * y];
}
function relative(a: Pose, x: number, y: number): [number, number] {
  const c = Math.cos(a[2]), s = Math.sin(a[2]), dx = x - a[0], dy = y - a[1];
  return [c * dx + s * dy, -s * dx + c * dy];
}

export class MapStore {
  /** 칸 → [x, y, 본 횟수, 마지막으로 본 시각 ms] */
  cells = new Map<number, [number, number, number, number]>();
  trail: [number, number][] = [];
  pose: Pose | null = null;
  state: State | null = null;
  places: Place[] = [];
  private scanPose: Pose | null = null;
  private decayed = 0;

  onHello(h: Hello) { this.places = h.places ?? []; }

  onScan(h: ScanHeader, mm: Int16Array) {
    if (!h.pose) return;
    this.scanPose = h.pose;
    const now = performance.now(), c = Math.cos(h.pose[2]), s = Math.sin(h.pose[2]);
    for (let i = 0; i + 1 < mm.length; i += 2) {
      const bx = mm[i] / 1000, by = mm[i + 1] / 1000;
      if (bx * bx + by * by > 36) continue;                       // 6 m 밖은 점이 성겨 지저분하다
      const x = h.pose[0] + c * bx - s * by, y = h.pose[1] + s * bx + c * by;
      const key = Math.round(x / CELL) * 100003 + Math.round(y / CELL);
      const cell = this.cells.get(key);
      if (cell) { cell[2] = Math.min(cell[2] + 1, 120); cell[3] = now; } else this.cells.set(key, [x, y, 1, now]);
    }
    // 지금 안 보이는 칸은 조금씩 잊는다. 지나간 사람(몇 번만 찍힌 칸)은 금방 사라지고, 오래 본 벽·가구는 1분쯤 남는다
    if (now - this.decayed > 1000) {
      this.decayed = now;
      for (const [k, c] of this.cells) if (now - c[3] > 1500 && (c[2] -= 2) <= 0) this.cells.delete(k);
    }
  }

  onState(s: State) {
    this.state = s;
    this.pose = s.ekf ?? this.scanPose ?? (s.odom ? [s.odom.x, s.odom.y, s.odom.yaw] : null);
    const p = this.pose, last = this.trail[this.trail.length - 1];
    if (p && (!last || Math.hypot(p[0] - last[0], p[1] - last[1]) > 0.05)) {
      // 오도메트리가 0에서 다시 시작하면(로봇을 다시 켰다) 쌓아 둔 것이 전부 어긋난다. 버린다
      if (last && Math.hypot(p[0] - last[0], p[1] - last[1]) > 1.5) { this.trail = []; this.cells.clear(); }
      this.trail.push([p[0], p[1]]);
      if (this.trail.length > 600) this.trail.splice(0, this.trail.length - 600);
    }
  }

  get start(): Pose { return this.state?.tasks?.layout?.start ?? [0, 0, 0]; }

  /** 이동 노드가 지금 가는 곳(EKF 좌표) */
  goal(): [number, number] | null {
    const nav = this.state?.nav;
    if (!nav?.goal || !this.pose || ['idle', 'arrived', 'failed', 'cancelled'].includes(nav.state)) return null;
    return compose(this.pose, nav.goal[0], nav.goal[1]);
  }
}

export class MiniMap {
  /** 지도를 누르면 시작 좌표(앞, 왼쪽 m)로 알려 준다 */
  onPick?: (x: number, y: number) => void;
  /** 고른 장소(강조해서 그린다) */
  selected = '';
  private ctx: CanvasRenderingContext2D;
  private view = { cx: 0, cy: 0, scale: 60 };       // 시작 좌표의 화면 중심과 px/m
  private first = true;
  private raf = 0;
  private colors: Record<string, string> = {};

  constructor(private canvas: HTMLCanvasElement, private store: MapStore) {
    this.ctx = canvas.getContext('2d')!;
    canvas.addEventListener('click', e => {
      if (!this.onPick) return;
      const r = canvas.getBoundingClientRect();
      const [x, y] = this.toStart(e.clientX - r.left, e.clientY - r.top, r.width, r.height);
      this.onPick(x, y);
    });
  }

  /** 보일 때만 그린다 */
  setActive(on: boolean) {
    cancelAnimationFrame(this.raf);
    if (!on) return;
    this.first = true;
    const loop = () => { this.draw(); this.raf = requestAnimationFrame(loop); };
    this.raf = requestAnimationFrame(loop);
  }

  private toScreen(wx: number, wy: number, w: number, h: number): [number, number] {
    const [x, y] = relative(this.store.start, wx, wy);
    return [w / 2 - (y - this.view.cy) * this.view.scale, h / 2 - (x - this.view.cx) * this.view.scale];
  }
  private toStart(px: number, py: number, w: number, h: number): [number, number] {
    return [this.view.cx - (py - h / 2) / this.view.scale, this.view.cy - (px - w / 2) / this.view.scale];
  }

  /** 로봇·시작·장소·목표가 다 들어오게 맞춘다(천천히 따라간다) */
  private fit(w: number, h: number) {
    const st = this.store, pts: [number, number][] = [[0, 0]];
    const add = (wx: number, wy: number) => pts.push(relative(st.start, wx, wy));
    if (st.pose) add(st.pose[0], st.pose[1]);
    const g = st.goal(); if (g) add(g[0], g[1]);
    for (const p of Object.values(st.state?.tasks?.layout?.places ?? {})) add(p.pose[0], p.pose[1]);
    let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
    for (const [x, y] of pts) { x0 = Math.min(x0, x); x1 = Math.max(x1, x); y0 = Math.min(y0, y); y1 = Math.max(y1, y); }
    const spanX = Math.max(6.0, x1 - x0 + 3.0), spanY = Math.max(6.0, y1 - y0 + 3.0);
    const scale = Math.min(h / spanX, w / spanY), cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
    const k = this.first ? 1 : 0.06;
    this.view.cx += (cx - this.view.cx) * k; this.view.cy += (cy - this.view.cy) * k; this.view.scale += (scale - this.view.scale) * k;
    this.first = false;
  }

  draw() {
    const cv = this.canvas, dpr = Math.min(window.devicePixelRatio || 1, 2);
    const w = cv.clientWidth, h = cv.clientHeight;
    if (!w || !h) return;
    if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(h * dpr)) { cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr); }
    if (!this.colors.bg) {
      const css = getComputedStyle(document.documentElement);
      for (const n of ['bg', 'panel', 'line', 'line-2', 'title', 'body', 'sub', 'accent', 'warn']) this.colors[n] = css.getPropertyValue(`--${n}`).trim();
    }
    const C = this.colors, g = this.ctx, st = this.store, s = st.state, now = performance.now();
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    g.fillStyle = C.bg; g.fillRect(0, 0, w, h);
    this.fit(w, h);
    const sc = this.view.scale;
    const P = (wx: number, wy: number) => this.toScreen(wx, wy, w, h);

    // 1 m 격자(시작 좌표)
    g.strokeStyle = C.line; g.lineWidth = 1; g.beginPath();
    const [tx, ty] = this.toStart(0, 0, w, h), [bx, by] = this.toStart(w, h, w, h);
    for (let x = Math.ceil(Math.min(tx, bx)); x <= Math.max(tx, bx); x++) { const py = h / 2 - (x - this.view.cx) * sc; g.moveTo(0, py); g.lineTo(w, py); }
    for (let y = Math.ceil(Math.min(ty, by)); y <= Math.max(ty, by); y++) { const px = w / 2 - (y - this.view.cy) * sc; g.moveTo(px, 0); g.lineTo(px, h); }
    g.stroke();

    // 라이다로 쌓은 점
    const size = Math.max(1.5, CELL * sc);
    for (const c of st.cells.values()) {
      const [px, py] = P(c[0], c[1]);
      if (px < -4 || py < -4 || px > w + 4 || py > h + 4) continue;
      if (c[2] < 3) continue;                                   // 한두 번 찍힌 점은 잡음이거나 지나가는 사람이다
      g.globalAlpha = Math.min(1, 0.2 + c[2] / 12) * (now - c[3] > 3000 ? 0.75 : 1);
      g.fillStyle = '#58C7FF';
      g.fillRect(px - size / 2, py - size / 2, size, size);
    }
    g.globalAlpha = 1;

    // 지나온 길
    if (st.trail.length > 1) {
      g.strokeStyle = C.sub; g.globalAlpha = 0.55; g.lineWidth = 2; g.beginPath();
      st.trail.forEach(([x, y], i) => { const [px, py] = P(x, y); if (i) g.lineTo(px, py); else g.moveTo(px, py); });
      g.stroke(); g.globalAlpha = 1;
    }

    const text = (label: string, px: number, py: number, color: string, weight = 600) => {
      g.font = `${weight} 13px Pretendard, sans-serif`; g.textAlign = 'center';
      g.lineWidth = 4; g.strokeStyle = C.bg; g.strokeText(label, px, py); g.fillStyle = color; g.fillText(label, px, py);
    };
    const heading = (px: number, py: number, yaw: number, len: number, color: string) => {
      // 화면에서 위쪽이 시작 좌표의 +x다
      const a = yaw - st.start[2];
      g.strokeStyle = color; g.lineWidth = 2; g.beginPath(); g.moveTo(px, py);
      g.lineTo(px - Math.sin(a) * len, py - Math.cos(a) * len); g.stroke();
    };

    // 시작 위치
    const layout = s?.tasks?.layout, start = st.start;
    {
      const [px, py] = P(start[0], start[1]);
      g.strokeStyle = C.title; g.lineWidth = 2; g.beginPath(); g.arc(px, py, 9, 0, Math.PI * 2); g.stroke();
      heading(px, py, start[2], 16, C.title);
      text(layout?.start_set ? '시작 위치' : '시작(켠 자리)', px, py + 26, C.title);
    }
    // 장소
    for (const pl of st.places) {
      const spot = layout?.places?.[pl.key];
      if (!spot) continue;
      const [px, py] = P(spot.pose[0], spot.pose[1]), on = this.selected === pl.key;
      g.fillStyle = on ? C.accent : C.panel; g.strokeStyle = C.accent; g.lineWidth = 2;
      g.beginPath(); g.rect(px - 8, py - 8, 16, 16); g.fill(); g.stroke();
      heading(px, py, spot.pose[2], 18, C.accent);
      text(pl.label, px, py + 26, C.accent, 700);
    }

    // 가는 길 (Nav2 전역 경로 /plan이 있으면 곡선 경로, 없으면 직선 점선)
    const goal = st.goal(), pose = st.pose, task = s?.tasks?.task, running = task?.state === 'running';
    const planPts = s?.plan?.pts && s.plan.pts.length >= 2 ? s.plan.pts : null;
    if (planPts) {
      const end = planPts[planPts.length - 1];
      const [gx, gy] = P(end[0], end[1]);
      g.strokeStyle = 'rgba(124, 255, 79, 0.28)'; g.lineWidth = 8; g.lineCap = 'round'; g.lineJoin = 'round';
      g.beginPath();
      planPts.forEach(([x, y], i) => { const [px, py] = P(x, y); if (i) g.lineTo(px, py); else g.moveTo(px, py); });
      g.stroke();
      g.strokeStyle = s?.nav?.blocked || s?.nav?.guard?.hold ? C.warn : C.accent; g.lineWidth = 3;
      g.beginPath();
      planPts.forEach(([x, y], i) => { const [px, py] = P(x, y); if (i) g.lineTo(px, py); else g.moveTo(px, py); });
      g.stroke();
      const pulse = 9 + 3 * Math.sin(now / 250);
      g.strokeStyle = C.accent; g.lineWidth = 2.5; g.beginPath(); g.arc(gx, gy, pulse, 0, Math.PI * 2); g.stroke();
      g.fillStyle = C.accent; g.beginPath(); g.arc(gx, gy, 3.5, 0, Math.PI * 2); g.fill();
      const mid = planPts[Math.floor(planPts.length / 2)];
      const [mx, my] = P(mid[0], mid[1]);
      text(`Nav2 ${s!.plan!.len_m.toFixed(1)} m`, mx, my - 10, C.accent, 700);
    } else if (goal && pose) {
      const [rx, ry] = P(pose[0], pose[1]), [gx, gy] = P(goal[0], goal[1]);
      g.strokeStyle = s?.nav?.blocked || s?.nav?.guard?.hold ? C.warn : C.accent; g.lineWidth = 3;
      g.setLineDash([8, 7]); g.lineDashOffset = -(now / 40) % 15;
      g.beginPath(); g.moveTo(rx, ry); g.lineTo(gx, gy);
      // 다녀오는 심부름이면 돌아오는 길도 흐리게 잇는다
      if (running && task!.kind === 'fetch_from' && task!.step < task!.steps) {
        g.stroke(); g.globalAlpha = 0.4; g.beginPath(); g.moveTo(gx, gy);
        const [hx, hy] = layout?.start_set || !s?.nav?.home ? P(start[0], start[1]) : P(...compose(pose, s.nav.home[0], s.nav.home[1]));
        g.lineTo(hx, hy);
      }
      g.stroke(); g.setLineDash([]); g.globalAlpha = 1;
      const pulse = 9 + 3 * Math.sin(now / 250);
      g.strokeStyle = C.accent; g.lineWidth = 2.5; g.beginPath(); g.arc(gx, gy, pulse, 0, Math.PI * 2); g.stroke();
      g.fillStyle = C.accent; g.beginPath(); g.arc(gx, gy, 3.5, 0, Math.PI * 2); g.fill();
      if (s?.nav?.dist_m != null) text(`${s.nav.dist_m.toFixed(1)} m`, (rx + gx) / 2, (ry + gy) / 2 - 8, C.title);
    }

    // 로봇
    if (pose) {
      const [px, py] = P(pose[0], pose[1]), a = pose[2] - start[2];
      g.save(); g.translate(px, py); g.rotate(-a);
      // 캔버스의 위쪽(-y)이 로봇의 앞이다
      const stopped = !!(s?.nav?.blocked || s?.nav?.guard?.hold);
      g.fillStyle = C.panel; g.strokeStyle = stopped ? C.warn : C.title; g.lineWidth = 2;
      g.beginPath(); g.roundRect(-BODY.half * sc, -BODY.front * sc, 2 * BODY.half * sc, (BODY.front + BODY.rear) * sc, 4); g.fill(); g.stroke();
      g.fillStyle = stopped ? C.warn : C.accent; g.beginPath();
      g.moveTo(0, -BODY.front * sc + 3); g.lineTo(-0.1 * sc, -0.08 * sc); g.lineTo(0.1 * sc, -0.08 * sc); g.closePath(); g.fill();
      if (stopped) {                                            // 막힌 쪽을 표시한다
        g.strokeStyle = C.warn; g.lineWidth = 4; g.beginPath();
        g.arc(0, 0, (BODY.front + 0.2) * sc, -Math.PI * 0.75, -Math.PI * 0.25); g.stroke();
      }
      g.restore();
    } else text('로봇 위치를 모릅니다', w / 2, h / 2, C.sub);

    // 축척 1 m
    g.strokeStyle = C.sub; g.lineWidth = 2; g.beginPath();
    g.moveTo(12, h - 12); g.lineTo(12 + sc, h - 12); g.moveTo(12, h - 16); g.lineTo(12, h - 8); g.moveTo(12 + sc, h - 16); g.lineTo(12 + sc, h - 8); g.stroke();
    g.font = '400 11px "JetBrains Mono", monospace'; g.textAlign = 'left'; g.fillStyle = C.sub; g.fillText('1 m', 16, h - 18);
  }
}
