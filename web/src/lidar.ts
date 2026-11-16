// 2D 라이다를 디지털 트윈에 겹쳐 그린다. 스캔은 한 높이의 단면뿐이라, 그대로 찍으면 바닥에 뜬 점선으로만 보인다.
// 그래서 브리지(scan.py)가 묶어 준 꺾은선을 위아래로 세워 반투명 벽으로 그리고, 지나온 스캔은 바닥에 흔적으로 남긴다.
//
//   실시간(로봇 좌표)   벽 띠 + 단면 선 + 점, 가는 물체(다리 같은 것)는 기둥, 빈 바닥 부채꼴, 가장 가까운 물체
//   흔적               스캔을 받을 때의 로봇 자세로 바닥에 찍어 두고 MEMORY_S 동안 서서히 지운다.
//                      자세는 EKF(바퀴 + 자이로) 것을 쓴다. 화면의 로봇을 그리는 /odom은 회전을 덜 재서 돌 때마다 번진다.
//                      지도(SLAM)가 아니라서 오래 두면 어긋난다. 그래서 쌓아 두지 않고 지운다.
import * as THREE from 'three';
import { CSS2DObject } from 'three/examples/jsm/renderers/CSS2DRenderer.js';
import { fmt } from './ui';

export interface ScanHeader {
  type: 'scan'; n: number; z: number; pose: [number, number, number] | null; odom?: [number, number, number] | null;
  lines: number[][]; posts: [number, number][]; nearest: [number, number, number] | null;
}

export const SCAN_COLOR = '#58C7FF';
const MAX_POINTS = 1024, MAX_SEGMENTS = 1024, MAX_POSTS = 128;
const WALL_UP = 0.32;                  // 단면 위로 세우는 높이 [m]
const FADE_RANGE = 12;                 // 이 거리에서 벽이 다 흐려진다
const MEMORY = 24000, MEMORY_S = 25, CELL = 0.05;
const NEAR_SHOW = 1.5;                 // 가장 가까운 물체는 이 안일 때만 표시한다

export type LidarMode = 'all' | 'live' | 'off';

export class LidarViz {
  /** 로봇(base_footprint)에 붙인다 */
  readonly live = new THREE.Group();
  /** 장면(odom)에 붙인다 */
  readonly trace: THREE.Points;
  mode: LidarMode = 'all';
  /** 이동 목적지(로봇 기준). 가장 가까운 물체가 목적지 자체면 이름표를 겹쳐 띄우지 않는다 */
  avoid: [number, number] | null = null;
  lastAt = 0;
  count = 0;
  private col = new THREE.Color(SCAN_COLOR);
  private points: THREE.Points;
  private walls: THREE.Mesh;
  private edges: THREE.LineSegments;
  private posts: THREE.LineSegments;
  private floor: THREE.Mesh;
  private near = new THREE.Group();
  private nearLabel: CSS2DObject;
  private origin = new THREE.Vector2();
  // 흔적
  private cells = new Map<number, number>();
  private slotKey = new Float64Array(MEMORY).fill(NaN);
  private born = new Float32Array(MEMORY);
  private next = 0;
  private fadeAt = 0;

  constructor() {
    const dyn = (n: number, size: number) => { const a = new THREE.BufferAttribute(new Float32Array(n * size), size); a.setUsage(THREE.DynamicDrawUsage); return a; };
    const geom = (attrs: Record<string, THREE.BufferAttribute>) => { const g = new THREE.BufferGeometry(); for (const k in attrs) g.setAttribute(k, attrs[k]); g.setDrawRange(0, 0); return g; };

    this.points = new THREE.Points(geom({ position: dyn(MAX_POINTS, 3) }),
      new THREE.PointsMaterial({ color: SCAN_COLOR, size: 0.022, sizeAttenuation: true, transparent: true, opacity: 0.95 }));
    // 벽: 선분마다 아래 띠(바닥 → 단면)와 위 띠(단면 → 위) 두 장. 꼭짓점 색의 알파로 단면에서 가장 진하게 한다.
    this.walls = new THREE.Mesh(geom({ position: dyn(MAX_SEGMENTS * 12, 3), color: dyn(MAX_SEGMENTS * 12, 4) }),
      new THREE.MeshBasicMaterial({ vertexColors: true, transparent: true, side: THREE.DoubleSide, depthWrite: false }));
    this.edges = new THREE.LineSegments(geom({ position: dyn(MAX_SEGMENTS * 2, 3), color: dyn(MAX_SEGMENTS * 2, 4) }),
      new THREE.LineBasicMaterial({ vertexColors: true, transparent: true }));
    this.posts = new THREE.LineSegments(geom({ position: dyn(MAX_POSTS * 2, 3) }),
      new THREE.LineBasicMaterial({ color: SCAN_COLOR, transparent: true, opacity: 0.8 }));
    this.floor = new THREE.Mesh(geom({ position: dyn(MAX_SEGMENTS * 3, 3) }),
      new THREE.MeshBasicMaterial({ color: SCAN_COLOR, transparent: true, opacity: 0.045, side: THREE.DoubleSide, depthWrite: false }));
    for (const o of [this.points, this.walls, this.edges, this.posts, this.floor]) o.frustumCulled = false;
    this.walls.renderOrder = 2; this.floor.renderOrder = 1;

    const ring = new THREE.Mesh(new THREE.RingGeometry(0.05, 0.065, 40),
      new THREE.MeshBasicMaterial({ color: SCAN_COLOR, transparent: true, opacity: 0.9, side: THREE.DoubleSide }));
    ring.position.z = 0.004;
    const d = document.createElement('div'); d.className = 'label3d scan';
    this.nearLabel = new CSS2DObject(d); this.nearLabel.position.set(0, 0, 0.12);
    this.near.add(ring, this.nearLabel); this.near.visible = false;
    this.live.add(this.floor, this.walls, this.edges, this.posts, this.points, this.near);

    const tg = geom({ position: dyn(MEMORY, 3), color: dyn(MEMORY, 3) });
    (tg.getAttribute('position').array as Float32Array).fill(1e6);         // 안 쓴 칸은 화면 밖
    this.trace = new THREE.Points(tg, new THREE.PointsMaterial({
      vertexColors: true, size: 0.03, sizeAttenuation: true, transparent: true, depthWrite: false, blending: THREE.AdditiveBlending }));
    this.trace.frustumCulled = false;
    tg.setDrawRange(0, MEMORY);
  }

  setOrigin(xy: [number, number]) { this.origin.set(xy[0], xy[1]); }

  setMode(m: LidarMode) {
    this.mode = m;
    this.live.visible = m !== 'off';
    this.trace.visible = m === 'all';
    if (m === 'off') (this.nearLabel.element as HTMLElement).style.display = 'none';
  }

  clearTrace() {
    this.cells.clear(); this.slotKey.fill(NaN); this.next = 0;
    const p = this.trace.geometry.getAttribute('position') as THREE.BufferAttribute;
    (p.array as Float32Array).fill(1e6); p.needsUpdate = true;
  }

  onScan(h: ScanHeader, mm: Int16Array, now = performance.now()) {
    this.lastAt = now; this.count = h.n;
    const z = h.z, n = Math.min(h.n, MAX_POINTS, mm.length >> 1);
    const pos = this.points.geometry.getAttribute('position') as THREE.BufferAttribute;
    for (let i = 0; i < n; i++) pos.setXYZ(i, mm[2 * i] / 1000, mm[2 * i + 1] / 1000, z);
    pos.needsUpdate = true; this.points.geometry.setDrawRange(0, n);

    // 벽·단면 선·바닥 부채꼴
    const wp = this.walls.geometry.getAttribute('position') as THREE.BufferAttribute;
    const wc = this.walls.geometry.getAttribute('color') as THREE.BufferAttribute;
    const ep = this.edges.geometry.getAttribute('position') as THREE.BufferAttribute;
    const ec = this.edges.geometry.getAttribute('color') as THREE.BufferAttribute;
    const fp = this.floor.geometry.getAttribute('position') as THREE.BufferAttribute;
    const { r, g, b } = this.col, ox = this.origin.x, oy = this.origin.y;
    let seg = 0;
    const fade = (x: number, y: number) => Math.max(0.3, 1 - Math.hypot(x, y) / FADE_RANGE);
    for (const line of h.lines) {
      for (let i = 0; i + 3 < line.length && seg < MAX_SEGMENTS; i += 2, seg++) {
        const ax = line[i] / 1000, ay = line[i + 1] / 1000, bx = line[i + 2] / 1000, by = line[i + 3] / 1000;
        const fa = fade(ax, ay), fb = fade(bx, by);
        let v = seg * 12;
        const quad = (z0: number, z1: number, a0: number, a1: number) => {
          // a(z0) b(z0) b(z1) / a(z0) b(z1) a(z1)
          const put = (x: number, y: number, zz: number, al: number) => { wp.setXYZ(v, x, y, zz); wc.setXYZW(v, r, g, b, al); v++; };
          put(ax, ay, z0, a0 * fa); put(bx, by, z0, a0 * fb); put(bx, by, z1, a1 * fb);
          put(ax, ay, z0, a0 * fa); put(bx, by, z1, a1 * fb); put(ax, ay, z1, a1 * fa);
        };
        quad(0, z, 0.08, 0.42);
        quad(z, z + WALL_UP, 0.42, 0);
        ep.setXYZ(seg * 2, ax, ay, z); ep.setXYZ(seg * 2 + 1, bx, by, z);
        ec.setXYZW(seg * 2, r, g, b, fa); ec.setXYZW(seg * 2 + 1, r, g, b, fb);
        fp.setXYZ(seg * 3, ox, oy, 0.003); fp.setXYZ(seg * 3 + 1, ax, ay, 0.003); fp.setXYZ(seg * 3 + 2, bx, by, 0.003);
      }
    }
    for (const a of [wp, wc, ep, ec, fp]) a.needsUpdate = true;
    this.walls.geometry.setDrawRange(0, seg * 12);
    this.edges.geometry.setDrawRange(0, seg * 2);
    this.floor.geometry.setDrawRange(0, seg * 3);

    const pp = this.posts.geometry.getAttribute('position') as THREE.BufferAttribute;
    const posts = Math.min(h.posts.length, MAX_POSTS);
    for (let i = 0; i < posts; i++) {
      const x = h.posts[i][0] / 1000, y = h.posts[i][1] / 1000;
      pp.setXYZ(i * 2, x, y, 0); pp.setXYZ(i * 2 + 1, x, y, z + WALL_UP * 0.6);
    }
    pp.needsUpdate = true; this.posts.geometry.setDrawRange(0, posts * 2);

    const near = h.nearest;
    const show = !!near && near[0] < NEAR_SHOW && this.mode !== 'off'
      && !(this.avoid && Math.hypot(near[1] - this.avoid[0], near[2] - this.avoid[1]) < 0.45);
    this.near.visible = show;
    (this.nearLabel.element as HTMLElement).style.display = show ? '' : 'none';
    if (near && show) {
      this.near.position.set(near[1], near[2], 0);
      (this.nearLabel.element as HTMLElement).textContent = `가장 가까운 물체 ${fmt(near[0], 2)} m`;
    }

    if (h.pose) {
      this.remember(h.pose, mm, n, now);
      // 흔적은 EKF 좌표로 쌓았다. 화면의 로봇은 /odom에 있으니 두 자세의 차이만큼 흔적 전체를 옮겨 로봇 자리에서 맞춘다.
      const o = h.odom ?? h.pose, dyaw = o[2] - h.pose[2], c = Math.cos(dyaw), s = Math.sin(dyaw);
      this.trace.rotation.z = dyaw;
      this.trace.position.set(o[0] - (c * h.pose[0] - s * h.pose[1]), o[1] - (s * h.pose[0] + c * h.pose[1]), 0);
    }
  }

  /** 스캔 점을 그때의 로봇 자세로 바닥(odom)에 남긴다. 같은 칸(5 cm)은 시각만 새로 고친다. */
  private remember(pose: [number, number, number], mm: Int16Array, n: number, now: number) {
    const [px, py, th] = pose, c = Math.cos(th), s = Math.sin(th);
    const pos = this.trace.geometry.getAttribute('position') as THREE.BufferAttribute;
    for (let i = 0; i < n; i += 2) {                       // 절반만 남겨도 벽은 이어져 보인다
      const bx = mm[2 * i] / 1000, by = mm[2 * i + 1] / 1000;
      const x = px + c * bx - s * by, y = py + s * bx + c * by;
      const key = Math.round(x / CELL) * 100003 + Math.round(y / CELL);
      let slot = this.cells.get(key);
      if (slot === undefined) {
        slot = this.next; this.next = (this.next + 1) % MEMORY;
        const old = this.slotKey[slot];
        if (!Number.isNaN(old)) this.cells.delete(old);
        this.slotKey[slot] = key; this.cells.set(key, slot);
        pos.setXYZ(slot, x, y, 0.012);
      }
      this.born[slot] = now;
    }
    pos.needsUpdate = true;
  }

  /** 흔적을 나이에 따라 흐리게 한다. 스캔이 끊기면 실시간 층도 지운다. */
  draw(now = performance.now()) {
    if (this.mode !== 'off' && now - this.lastAt > 1500 && this.count) {
      this.count = 0;
      for (const o of [this.points, this.walls, this.edges, this.posts, this.floor]) (o as THREE.Mesh).geometry.setDrawRange(0, 0);
      this.near.visible = false; (this.nearLabel.element as HTMLElement).style.display = 'none';
    }
    if (now - this.fadeAt < 250 || this.mode !== 'all') return;
    this.fadeAt = now;
    const pos = this.trace.geometry.getAttribute('position') as THREE.BufferAttribute;
    const col = this.trace.geometry.getAttribute('color') as THREE.BufferAttribute;
    const { r, g, b } = this.col;
    let moved = false;
    for (let i = 0; i < MEMORY; i++) {
      const key = this.slotKey[i];
      if (Number.isNaN(key)) continue;
      const k = 1 - (now - this.born[i]) / (MEMORY_S * 1000);
      if (k <= 0) {
        this.cells.delete(key); this.slotKey[i] = NaN; pos.setXYZ(i, 1e6, 1e6, 1e6); moved = true;
        continue;
      }
      const a = 0.42 * k * k;
      col.setXYZ(i, r * a, g * a, b * a);
    }
    col.needsUpdate = true;
    if (moved) pos.needsUpdate = true;
  }
}
