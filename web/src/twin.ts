// 디지털 트윈. ROS 좌표(x 전방, y 왼쪽, z 위)를 그대로 쓴다.
// 카트 자세는 /odom, 마커는 base_footprint 기준 검출값을 카트에 붙여 그린다.
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import { CSS2DObject, CSS2DRenderer } from 'three/examples/jsm/renderers/CSS2DRenderer.js';
import { LidarViz, type LidarMode, type ScanHeader } from './lidar';
import type { Hello, State } from './types';
import { C, el, fmt } from './ui';

THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

// mr_description robot_params.yaml
const BASE_LINK_Z = 0.05;
const CART_MESH_OFFSET = new THREE.Vector3(0.025, 0, -0.043482687);
const WHEEL_Y = 0.264 / 2;
const CAM_BOX = { x: 0.25, z: 0.36, size: [0.024, 0.094, 0.029] as const };
const HFOV = (67.6 * Math.PI) / 180, VFOV = (41.3 * Math.PI) / 180;   // c920_calibration.yaml
const FOLLOW_RANGE = 1.1;                                              // follow.yaml target_range_m
const TRAIL_MAX = 6000;

type View = 'follow' | 'orbit' | 'top';

class Trail {
  readonly line: THREE.Line;
  private pos = new Float32Array(TRAIL_MAX * 3);
  private n = 0;
  private geom = new THREE.BufferGeometry();
  private last: [number, number] | null = null;
  constructor(color: string, opacity: number, private z: number) {
    this.geom.setAttribute('position', new THREE.BufferAttribute(this.pos, 3));
    this.geom.setDrawRange(0, 0);
    this.line = new THREE.Line(this.geom, new THREE.LineBasicMaterial({ color, transparent: true, opacity }));
    this.line.frustumCulled = false;
  }
  add(x: number, y: number) {
    if (this.last && Math.hypot(x - this.last[0], y - this.last[1]) < 0.015) return;
    if (this.n >= TRAIL_MAX) {                 // 오래된 절반을 버린다
      this.pos.copyWithin(0, (TRAIL_MAX / 2) * 3);
      this.n = TRAIL_MAX / 2;
    }
    this.pos.set([x, y, this.z], this.n * 3);
    this.n++;
    this.last = [x, y];
    this.geom.setDrawRange(0, this.n);
    (this.geom.attributes.position as THREE.BufferAttribute).needsUpdate = true;
  }
  clear() { this.n = 0; this.last = null; this.geom.setDrawRange(0, 0); }
}

function label(text: string, cls = ''): CSS2DObject {
  const d = document.createElement('div');
  d.className = 'label3d ' + cls;
  d.textContent = text;
  return new CSS2DObject(d);
}

/** ArUco처럼 보이는 텍스처. 실제 비트 패턴은 아니고 id로 정한 4x4 무늬다. */
function markerTexture(id: number): THREE.CanvasTexture {
  const cv = document.createElement('canvas');
  cv.width = cv.height = 96;
  const g = cv.getContext('2d')!;
  g.fillStyle = '#F2F4F5'; g.fillRect(0, 0, 96, 96);
  g.fillStyle = '#0E1216'; g.fillRect(8, 8, 80, 80);
  let s = id * 2654435761 + 12345;
  g.fillStyle = '#F2F4F5';
  for (let r = 0; r < 4; r++) for (let c = 0; c < 4; c++) {
    s = (s * 1103515245 + 12345) & 0x7fffffff;
    if ((s >> 16) & 1) g.fillRect(20 + c * 14, 20 + r * 14, 14, 14);
  }
  const t = new THREE.CanvasTexture(cv);
  t.colorSpace = THREE.SRGBColorSpace;
  t.magFilter = THREE.NearestFilter;
  return t;
}

class MarkerViz {
  readonly group = new THREE.Group();        // base_footprint 기준 마커 자세
  readonly ground = new THREE.Group();
  private plane: THREE.Mesh;
  private outline: THREE.LineLoop;
  private ray: THREE.Line;
  private drop: THREE.Line;
  private idLabel: CSS2DObject;
  private distLabel: CSS2DObject;
  private outlineMat = new THREE.LineBasicMaterial({ color: C.accent, transparent: true });
  private rayMat = new THREE.LineDashedMaterial({ color: C.accent, dashSize: 0.05, gapSize: 0.035, transparent: true });
  private dropMat = new THREE.LineDashedMaterial({ color: C.sub, dashSize: 0.03, gapSize: 0.03, transparent: true });
  private ringMat = new THREE.MeshBasicMaterial({ color: C.accent, transparent: true, side: THREE.DoubleSide });
  private planeMat: THREE.MeshBasicMaterial;
  id = -1;
  opacity = 0;

  constructor(parent: THREE.Object3D, private camOrigin: THREE.Vector3, size: number) {
    this.planeMat = new THREE.MeshBasicMaterial({ transparent: true, side: THREE.DoubleSide });
    this.plane = new THREE.Mesh(new THREE.PlaneGeometry(size * 1.33, size * 1.33), this.planeMat);
    const h = size / 2 + 0.006;
    this.outline = new THREE.LineLoop(new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(-h, h, 0.002), new THREE.Vector3(h, h, 0.002),
      new THREE.Vector3(h, -h, 0.002), new THREE.Vector3(-h, -h, 0.002)]), this.outlineMat);
    const normal = new THREE.Line(new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, 0.12)]), this.outlineMat);
    this.group.add(this.plane, this.outline, normal);
    this.idLabel = label('', 'accent');
    this.idLabel.position.set(0, size * 0.9, 0);
    this.group.add(this.idLabel);

    this.ray = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3(1, 0, 0)]), this.rayMat);
    this.drop = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3(0, 0, 1)]), this.dropMat);
    this.ray.frustumCulled = this.drop.frustumCulled = false;
    const ring = new THREE.Mesh(new THREE.RingGeometry(0.07, 0.085, 48), this.ringMat);
    this.ground.add(ring);
    this.distLabel = label('', 'dist');
    parent.add(this.group, this.ground, this.ray, this.drop, this.distLabel);
  }

  name = '';
  setId(id: number, target: boolean, name = '') {
    if (id !== this.id || name !== this.name) {
      this.name = name;
      this.id = id;
      this.planeMat.map?.dispose();
      this.planeMat.map = markerTexture(id);
      this.planeMat.needsUpdate = true;
    }
    const col = target ? C.accent : C.body;
    this.outlineMat.color.set(col); this.rayMat.color.set(col); this.ringMat.color.set(col);
    (this.idLabel.element as HTMLElement).className = 'label3d' + (target ? ' accent' : '');
    (this.idLabel.element as HTMLElement).textContent = name ? `${name} · ID ${id}` : `ID ${id}`;
  }

  setPose(p: THREE.Vector3, q: THREE.Quaternion) {
    this.group.position.copy(p);
    this.group.quaternion.copy(q);
    this.ground.position.set(p.x, p.y, 0.003);
    const rp = this.ray.geometry.attributes.position as THREE.BufferAttribute;
    rp.setXYZ(0, this.camOrigin.x, this.camOrigin.y, this.camOrigin.z); rp.setXYZ(1, p.x, p.y, p.z); rp.needsUpdate = true;
    this.ray.computeLineDistances();
    const dp = this.drop.geometry.attributes.position as THREE.BufferAttribute;
    dp.setXYZ(0, p.x, p.y, 0); dp.setXYZ(1, p.x, p.y, p.z); dp.needsUpdate = true;
    this.drop.computeLineDistances();
    // 거리는 바닥 고리 옆에 둔다. 광선 중간에 두면 마커가 가까울 때 ID 이름표와 겹친다.
    this.distLabel.position.set(p.x, p.y - 0.16, 0.01);
    (this.distLabel.element as HTMLElement).textContent = `${fmt(Math.hypot(p.x, p.y), 2)} m`;
  }

  setOpacity(a: number) {
    this.opacity = a;
    const vis = a > 0.02;
    for (const o of [this.group, this.ground, this.ray, this.drop, this.distLabel]) o.visible = vis;
    this.planeMat.opacity = a; this.outlineMat.opacity = a; this.rayMat.opacity = a * 0.9;
    this.dropMat.opacity = a * 0.6; this.ringMat.opacity = a * 0.8;
    (this.idLabel.element as HTMLElement).style.opacity = String(a);
    (this.distLabel.element as HTMLElement).style.opacity = String(a);
  }
}

export class Twin {
  private host = el('twin');
  private renderer = new THREE.WebGLRenderer({ antialias: true });
  private labels = new CSS2DRenderer();
  private scene = new THREE.Scene();
  private camera = new THREE.PerspectiveCamera(42, 1, 0.05, 80);
  private controls: OrbitControls;
  private robot = new THREE.Group();          // base_footprint (odom 기준)
  private wheels: THREE.Group[] = [];
  private fusedMark = new THREE.Group();
  private odomTrail = new Trail(C.body, 0.75, 0.004);
  private fusedTrail = new Trail(C.warn, 0.9, 0.006);
  private markers: MarkerViz[] = [];
  private camOrigin = new THREE.Vector3(0.25, 0, 0.41);
  private markerSize = 0.15;
  private target = { x: 0, y: 0, yaw: 0, wl: 0, wr: 0 };
  private fused = { x: 0, y: 0, yaw: 0 };
  private haveOdom = false;
  private view: View = 'orbit';
  private idleUntil = 0;
  private meta = el('twin-meta');
  private lastState: State | null = null;
  private lastTrack: { p: THREE.Vector3; q: THREE.Quaternion; id: number } | null = null;
  private lidar = new LidarViz();
  private placeName = new Map<number, string>();
  private goal = new THREE.Group();
  private home = new THREE.Group();
  private goalLine!: THREE.Line;
  private goalLabel = label('', 'accent');
  private planGroup = new THREE.Group();
  private planGlow!: THREE.Mesh;
  private planCore!: THREE.Mesh;
  private planDots!: THREE.Points;
  private planEnd = new THREE.Group();
  private planLabel = label('', 'plan');

  constructor(private onReset: () => void) {
    const r = this.renderer;
    r.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    r.setClearColor(C.bg);
    r.outputColorSpace = THREE.SRGBColorSpace;
    this.host.appendChild(r.domElement);
    this.labels.domElement.style.position = 'absolute';
    this.labels.domElement.style.inset = '0';
    this.labels.domElement.style.pointerEvents = 'none';
    this.host.appendChild(this.labels.domElement);

    this.scene.fog = new THREE.Fog(C.bg, 7, 18);
    this.scene.add(new THREE.HemisphereLight(0xc9d3db, 0x0e1216, 1.6));
    const key = new THREE.DirectionalLight(0xf2f4f5, 2.0); key.position.set(2.5, -3, 6); this.scene.add(key);
    const rim = new THREE.DirectionalLight(0x9aa7b2, 0.8); rim.position.set(-3, 2, 3); this.scene.add(rim);
    this.scene.add(this.grid());
    this.scene.add(this.odomTrail.line, this.fusedTrail.line);

    // 자이로 경로의 현재 끝. 원뿔 축(+y)을 진행 방향(+x)으로 눕힌다.
    const cone = new THREE.Mesh(new THREE.ConeGeometry(0.05, 0.14, 3), new THREE.MeshBasicMaterial({ color: C.warn }));
    cone.rotation.z = -Math.PI / 2;
    this.fusedMark.add(cone);
    this.scene.add(this.fusedMark);

    this.scene.add(this.robot);
    this.buildRobot();
    this.scene.add(this.lidar.trace);
    this.robot.add(this.lidar.live);
    this.buildNav();

    this.camera.position.set(-2.4, -1.8, 1.7);
    this.controls = new OrbitControls(this.camera, r.domElement);
    this.controls.enableDamping = true;
    this.controls.target.set(0.4, 0, 0.3);
    this.controls.maxPolarAngle = Math.PI * 0.47;
    this.controls.minDistance = 0.8; this.controls.maxDistance = 12;
    this.controls.autoRotateSpeed = 0.6;
    this.controls.addEventListener('start', () => { this.idleUntil = performance.now() + 15000; });
    this.setView('orbit');

    window.addEventListener('keydown', (e) => {
      if (document.body.classList.contains('view-avatar')) return;
      if (e.key === '4') this.setView('follow');
      else if (e.key === '5') this.setView('orbit');
      else if (e.key === '6') this.setView('top');
      else if (e.key === 'r' || e.key === 'R') this.reset();
      else if ((e.key === 'f' || e.key === 'F') && !e.repeat && !e.ctrlKey && !e.metaKey && !e.altKey) this.toggleWide();
      else if (e.key === 'Escape') this.toggleWide(false);
      else if (e.key === 'l' || e.key === 'L') {
        const order: LidarMode[] = ['all', 'live', 'off'];
        this.lidar.setMode(order[(order.indexOf(this.lidar.mode) + 1) % order.length]);
      }
    });
    new ResizeObserver(() => this.resize()).observe(this.host);
    this.resize();
  }

  private grid(): THREE.Object3D {
    const g = new THREE.Group();
    const minor: number[] = [], major: number[] = [];
    const N = 12;
    for (let i = -N * 4; i <= N * 4; i++) {
      const v = i * 0.25;
      const arr = i % 4 === 0 ? major : minor;
      arr.push(v, -N, 0, v, N, 0, -N, v, 0, N, v, 0);
    }
    const mk = (arr: number[], color: string) => {
      const geo = new THREE.BufferGeometry();
      geo.setAttribute('position', new THREE.Float32BufferAttribute(arr, 3));
      return new THREE.LineSegments(geo, new THREE.LineBasicMaterial({ color }));
    };
    g.add(mk(minor, '#151c23'), mk(major, C.line));
    // odom 원점 축
    const axes = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(0, 0, 0.002), new THREE.Vector3(0.5, 0, 0.002),
      new THREE.Vector3(0, 0, 0.002), new THREE.Vector3(0, 0.5, 0.002)]);
    g.add(new THREE.LineSegments(axes, new THREE.LineBasicMaterial({ color: C.line2 })));
    const o = label('원점'); o.position.set(0, -0.12, 0.01); g.add(o);
    return g;
  }

  private buildRobot() {
    const base = new THREE.Group();
    base.position.z = BASE_LINK_Z;
    this.robot.add(base);

    // 바닥 그림자 대신 어두운 원판
    const cv = document.createElement('canvas'); cv.width = cv.height = 128;
    const g = cv.getContext('2d')!;
    const grad = g.createRadialGradient(64, 64, 8, 64, 64, 64);
    grad.addColorStop(0, 'rgba(0,0,0,0.55)'); grad.addColorStop(1, 'rgba(0,0,0,0)');
    g.fillStyle = grad; g.fillRect(0, 0, 128, 128);
    const shadow = new THREE.Mesh(new THREE.PlaneGeometry(0.9, 0.75),
      new THREE.MeshBasicMaterial({ map: new THREE.CanvasTexture(cv), transparent: true, depthWrite: false }));
    shadow.position.set(0.03, 0, 0.002);
    this.robot.add(shadow);

    const loader = new STLLoader();
    const frameMat = new THREE.MeshStandardMaterial({ color: 0x9aa7b2, metalness: 0.35, roughness: 0.55 });
    const edgeMat = new THREE.LineBasicMaterial({ color: C.body, transparent: true, opacity: 0.22 });
    loader.load('meshes/cart_frame.stl', (geo) => {
      const m = new THREE.Mesh(geo, frameMat);
      m.scale.setScalar(0.001);
      m.position.copy(CART_MESH_OFFSET);
      m.add(new THREE.LineSegments(new THREE.EdgesGeometry(geo, 35), edgeMat));
      base.add(m);
    });
    const wheelMat = new THREE.MeshStandardMaterial({ color: 0x26313b, metalness: 0.1, roughness: 0.85 });
    const wheelEdge = new THREE.LineBasicMaterial({ color: C.sub, transparent: true, opacity: 0.35 });
    loader.load('meshes/drive_wheel.stl', (geo) => {
      for (const side of [1, -1]) {
        const wg = new THREE.Group();
        wg.position.set(0, side * WHEEL_Y, 0);
        const m = new THREE.Mesh(geo, wheelMat);
        m.scale.setScalar(0.001);
        m.add(new THREE.LineSegments(new THREE.EdgesGeometry(geo, 25), wheelEdge));
        wg.add(m);
        base.add(wg);
        this.wheels[side === 1 ? 0 : 1] = wg;
      }
    });
    const camBox = new THREE.Mesh(new THREE.BoxGeometry(...CAM_BOX.size),
      new THREE.MeshStandardMaterial({ color: 0x0e1216, roughness: 0.5 }));
    camBox.position.set(CAM_BOX.x, 0, CAM_BOX.z);
    base.add(camBox);
    this.buildFrustum();
    this.buildFollowArc();
    for (let i = 0; i < 4; i++) this.markers.push(new MarkerViz(this.robot, this.camOrigin, this.markerSize));
  }

  private frustum = new THREE.Group();
  private buildFrustum() {
    this.frustum.clear();
    const far = 2.8, o = this.camOrigin;
    const hx = Math.tan(HFOV / 2) * far, hz = Math.tan(VFOV / 2) * far;
    const c = [[far, hx, hz], [far, -hx, hz], [far, -hx, -hz], [far, hx, -hz]].map(([x, y, z]) => new THREE.Vector3(o.x + x, o.y + y, o.z + z));
    const pts: THREE.Vector3[] = [];
    for (const p of c) pts.push(o.clone(), p);
    for (let i = 0; i < 4; i++) pts.push(c[i], c[(i + 1) % 4]);
    this.frustum.add(new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(pts),
      new THREE.LineBasicMaterial({ color: C.sub, transparent: true, opacity: 0.35 })));
    const faces = new THREE.BufferGeometry().setFromPoints([o, c[0], c[1], o, c[1], c[2], o, c[2], c[3], o, c[3], c[0]]);
    this.frustum.add(new THREE.Mesh(faces, new THREE.MeshBasicMaterial({
      color: C.body, transparent: true, opacity: 0.035, side: THREE.DoubleSide, depthWrite: false })));
    this.robot.add(this.frustum);
  }

  private buildFollowArc() {
    const pts: THREE.Vector3[] = [];
    const a = HFOV / 2;
    for (let i = 0; i <= 40; i++) {
      const t = -a + (2 * a * i) / 40;
      pts.push(new THREE.Vector3(Math.cos(t) * FOLLOW_RANGE, Math.sin(t) * FOLLOW_RANGE, 0.004));
    }
    const line = new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts),
      new THREE.LineDashedMaterial({ color: C.sub, dashSize: 0.04, gapSize: 0.03, transparent: true, opacity: 0.7 }));
    line.computeLineDistances();
    this.robot.add(line);
    const l = label(`추종 거리 ${FOLLOW_RANGE} m`);
    l.position.set(Math.cos(a) * FOLLOW_RANGE, -Math.sin(a) * FOLLOW_RANGE - 0.05, 0.01);
    this.robot.add(l);
  }

  /** 이동 목적지와 돌아올 자리. 이동 노드가 로봇 기준 위치로 알려 주므로 로봇에 붙여 그린다. */
  private buildNav() {
    const ring = (r: number, color: string) => new THREE.Mesh(new THREE.RingGeometry(r, r + 0.022, 48),
      new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.9, side: THREE.DoubleSide }));
    this.goal.add(ring(0.16, C.accent), ring(0.05, C.accent));
    this.goalLabel.position.set(0, 0, 0.3);
    this.goal.add(this.goalLabel);
    this.home.add(ring(0.12, C.body));
    const hl = label('돌아올 자리'); hl.position.set(0, -0.2, 0.01); this.home.add(hl);
    this.goalLine = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3(1, 0, 0)]),
      new THREE.LineDashedMaterial({ color: C.accent, dashSize: 0.07, gapSize: 0.05, transparent: true, opacity: 0.8 }));
    this.goalLine.frustumCulled = false;
    for (const o of [this.goal, this.home, this.goalLine]) { o.visible = false; this.robot.add(o); }

    // Nav2 전역 경로(/plan): WebGL 1 px 제한을 피하려고 3D 띠 메쉬(발광 외곽 + 중심선 + 웨이포인트 점)로 그린다.
    const mkRibbon = (color: string, opacity: number, order: number) => {
      const geo = new THREE.BufferGeometry();
      const attr = new THREE.BufferAttribute(new Float32Array(200 * 6 * 3), 3);
      attr.setUsage(THREE.DynamicDrawUsage);
      geo.setAttribute('position', attr);
      geo.setDrawRange(0, 0);
      const mesh = new THREE.Mesh(geo, new THREE.MeshBasicMaterial({
        color, transparent: true, opacity, side: THREE.DoubleSide, depthWrite: false,
      }));
      mesh.frustumCulled = false;
      mesh.renderOrder = order;
      return mesh;
    };
    this.planGlow = mkRibbon(C.accent, 0.30, 3);
    this.planCore = mkRibbon('#C6FF87', 0.96, 4);
    const dotGeo = new THREE.BufferGeometry();
    const dotAttr = new THREE.BufferAttribute(new Float32Array(200 * 3), 3);
    dotAttr.setUsage(THREE.DynamicDrawUsage);
    dotGeo.setAttribute('position', dotAttr);
    dotGeo.setDrawRange(0, 0);
    this.planDots = new THREE.Points(dotGeo, new THREE.PointsMaterial({
      color: C.title, size: 0.065, sizeAttenuation: true, transparent: true, opacity: 0.92, depthWrite: false,
    }));
    this.planDots.frustumCulled = false;
    this.planDots.renderOrder = 5;
    this.planEnd.add(ring(0.18, C.accent), ring(0.06, '#C6FF87'));
    this.planLabel.position.set(0, 0, 0.34);
    this.planEnd.add(this.planLabel);
    this.planGroup.add(this.planGlow, this.planCore, this.planDots, this.planEnd);
    this.planGroup.visible = false;
    (this.planLabel.element as HTMLElement).style.display = 'none';
    this.scene.add(this.planGroup);
  }

  private fillRibbon(mesh: THREE.Mesh, pts: [number, number][], halfWidth: number, z: number) {
    const n = Math.min(pts.length, 200);
    const pos = mesh.geometry.getAttribute('position') as THREE.BufferAttribute;
    const normals: [number, number][] = new Array(n);
    for (let i = 0; i < n; i++) {
      const a = pts[Math.max(0, i - 1)], b = pts[Math.min(n - 1, i + 1)];
      let dx = b[0] - a[0], dy = b[1] - a[1];
      const len = Math.hypot(dx, dy);
      if (len > 1e-6) { dx /= len; dy /= len; } else { dx = 1; dy = 0; }
      normals[i] = [-dy * halfWidth, dx * halfWidth];
    }
    let v = 0;
    for (let i = 0; i < n - 1; i++) {
      const [x0, y0] = pts[i], [x1, y1] = pts[i + 1];
      const [nx0, ny0] = normals[i], [nx1, ny1] = normals[i + 1];
      pos.setXYZ(v++, x0 + nx0, y0 + ny0, z);
      pos.setXYZ(v++, x0 - nx0, y0 - ny0, z);
      pos.setXYZ(v++, x1 + nx1, y1 + ny1, z);
      pos.setXYZ(v++, x0 - nx0, y0 - ny0, z);
      pos.setXYZ(v++, x1 - nx1, y1 - ny1, z);
      pos.setXYZ(v++, x1 + nx1, y1 + ny1, z);
    }
    pos.needsUpdate = true;
    mesh.geometry.setDrawRange(0, v);
  }

  private showNav(s: State) {
    const nav = s.nav, moving = !!nav && !['idle', 'arrived', 'failed', 'cancelled'].includes(nav.state);
    const g = moving ? nav!.goal : null;
    const plan = s.plan && s.plan.pts.length >= 2 ? s.plan : null;
    const hasPlan = !!plan;

    this.planGroup.visible = hasPlan;
    (this.planLabel.element as HTMLElement).style.display = hasPlan ? '' : 'none';
    if (plan) {
      // 라이다 흔적과 같은 보정: /plan은 EKF 좌표이므로 /odom 로봇 자리와 정확히 겹치게 옮긴다.
      if (s.ekf && s.odom) {
        const dyaw = s.odom.yaw - s.ekf[2], c = Math.cos(dyaw), sn = Math.sin(dyaw);
        this.planGroup.rotation.z = dyaw;
        this.planGroup.position.set(
          s.odom.x - (c * s.ekf[0] - sn * s.ekf[1]),
          s.odom.y - (sn * s.ekf[0] + c * s.ekf[1]),
          0,
        );
      } else {
        this.planGroup.rotation.z = 0;
        this.planGroup.position.set(0, 0, 0);
      }
      this.fillRibbon(this.planGlow, plan.pts, 0.085, 0.015);
      this.fillRibbon(this.planCore, plan.pts, 0.026, 0.018);
      const dotPos = this.planDots.geometry.getAttribute('position') as THREE.BufferAttribute;
      let dn = 0, acc = 0;
      for (let i = 1; i < plan.pts.length && dn < 200; i++) {
        acc += Math.hypot(plan.pts[i][0] - plan.pts[i - 1][0], plan.pts[i][1] - plan.pts[i - 1][1]);
        if (acc >= 0.35) {
          dotPos.setXYZ(dn++, plan.pts[i][0], plan.pts[i][1], 0.022);
          acc = 0;
        }
      }
      dotPos.needsUpdate = true;
      this.planDots.geometry.setDrawRange(0, dn);
      const end = plan.pts[plan.pts.length - 1];
      this.planEnd.position.set(end[0], end[1], 0.019);
      const name = s.tasks?.task?.kind === 'home' ? '시작 위치' : s.tasks?.task?.label ?? '목적지';
      (this.planLabel.element as HTMLElement).textContent =
        `Nav2 A* · ${name} ${fmt(plan.len_m, 2)} m${nav?.blocked ? ' · 앞이 막힘' : ''}`;
    }

    this.goal.visible = this.goalLine.visible = !!g && !hasPlan;
    this.lidar.avoid = g;
    (this.goalLabel.element as HTMLElement).style.display = g && !hasPlan ? '' : 'none';
    if (g && !hasPlan) {
      this.goal.position.set(g[0], g[1], 0.005);
      const p = this.goalLine.geometry.attributes.position as THREE.BufferAttribute;
      p.setXYZ(0, 0, 0, 0.006); p.setXYZ(1, g[0], g[1], 0.006); p.needsUpdate = true;
      this.goalLine.computeLineDistances();
      const name = s.tasks?.task?.kind === 'home' || nav!.kind === 'pose' ? '돌아갈 곳' : s.tasks?.task?.label ?? '목적지';
      (this.goalLabel.element as HTMLElement).textContent =
        `${name}${nav!.dist_m != null ? ` ${fmt(nav!.dist_m, 2)} m` : ''}${nav!.blocked ? ' · 앞이 막힘' : ''}`;
    }
    // 돌아올 자리는 떠나 있을 때만(30 cm 넘게) 보인다
    const h = nav?.home ?? null, away = !!h && Math.hypot(h[0], h[1]) > 0.3;
    this.home.visible = away;
    this.home.traverse(o => { if (o instanceof CSS2DObject) (o.element as HTMLElement).style.display = away ? '' : 'none'; });
    if (h && away) this.home.position.set(h[0], h[1], 0.004);
  }

  onHello(h: Hello) {
    this.camOrigin.set(...h.camera.origin);
    this.markerSize = h.marker_size;
    if (h.lidar) this.lidar.setOrigin(h.lidar.xy);
    this.placeName = new Map((h.places ?? []).map(p => [p.marker_id, p.label]));
  }

  onScan(h: ScanHeader, mm: Int16Array) { this.lidar.onScan(h, mm); }

  onState(s: State) {
    this.lastState = s;
    if (s.odom) {
      this.target.x = s.odom.x; this.target.y = s.odom.y; this.target.yaw = s.odom.yaw;
      if (!this.haveOdom) {
        this.haveOdom = true;
        this.robot.position.set(s.odom.x, s.odom.y, 0); this.robot.rotation.z = s.odom.yaw;
      }
      this.odomTrail.add(s.odom.x, s.odom.y);
    }
    if (s.fused) { this.fused = { ...s.fused }; this.fusedTrail.add(s.fused.x, s.fused.y); }
    if (s.wheels.l.pos != null) this.target.wl = s.wheels.l.pos;
    if (s.wheels.r.pos != null) this.target.wr = s.wheels.r.pos;

    // 마커: 지금 보이는 것은 검출값, 대상 마커가 잠깐 사라지면(holding) 마지막 자세를 흐리게 둔다
    const targetId = s.marker?.id ?? -1;
    const seen = s.markers3d.slice(0, this.markers.length);
    seen.forEach((m, i) => {
      const viz = this.markers[i];
      const p = new THREE.Vector3(...m.p), q = new THREE.Quaternion(...m.q);
      viz.setId(m.id, m.id === targetId, this.placeName.get(m.id) ?? '');
      viz.setPose(p, q);
      viz.setOpacity(1);
      if (m.id === targetId) this.lastTrack = { p, q, id: m.id };
    });
    let used = seen.length;
    const holding = s.marker?.status === 'holding';
    if (holding && this.lastTrack && !seen.some((m) => m.id === targetId) && used < this.markers.length) {
      const viz = this.markers[used++];
      viz.setId(this.lastTrack.id, true, this.placeName.get(this.lastTrack.id) ?? '');
      viz.setPose(this.lastTrack.p, this.lastTrack.q);
      viz.setOpacity(0.45);
    }
    for (let i = used; i < this.markers.length; i++) {
      const viz = this.markers[i];
      if (viz.opacity > 0) viz.setOpacity(Math.max(0, viz.opacity - 0.12));
    }
    const d = s.odom ? Math.hypot(s.odom.x, s.odom.y) : 0;
    this.showNav(s);
    const scan = s.lidar ? `  라이다 ${s.lidar.n}점 ${fmt(s.lidar.hz, 0)} Hz` : '';
    const planMeta = s.plan && s.plan.pts.length >= 2 ? `  Nav2 경로 ${fmt(s.plan.len_m, 2)} m` : '';
    this.meta.textContent = (s.odom
      ? `x ${fmt(s.odom.x, 2, true)}  y ${fmt(s.odom.y, 2, true)}  yaw ${fmt((s.odom.yaw * 180) / Math.PI, 0, true)}°  원점에서 ${fmt(d, 2)} m`
      : '오도메트리 없음') + scan + planMeta;
  }

  /** 디지털 트윈만 화면 가득 띄운다(F, 다시 누르거나 Esc로 돌아온다). 크기 변화는 ResizeObserver가 따라간다. */
  toggleWide(on = !document.body.classList.contains('twin-wide')) {
    document.body.classList.toggle('twin-wide', on);
  }

  private reset() {
    this.odomTrail.clear(); this.fusedTrail.clear(); this.lidar.clearTrace();
    this.onReset();
  }

  private setView(v: View) {
    this.view = v;
    this.controls.enabled = v === 'orbit';
    this.idleUntil = 0;
  }

  private resize() {
    const w = this.host.clientWidth, h = this.host.clientHeight;
    if (!w || !h) return;
    this.renderer.setSize(w, h, false);
    this.renderer.domElement.style.width = '100%';
    this.renderer.domElement.style.height = '100%';
    this.labels.setSize(w, h);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  draw(dt: number) {
    const k = 1 - Math.exp(-dt / 0.07);
    const rp = this.robot.position;
    rp.x += (this.target.x - rp.x) * k;
    rp.y += (this.target.y - rp.y) * k;
    let dy = this.target.yaw - this.robot.rotation.z;
    dy = Math.atan2(Math.sin(dy), Math.cos(dy));
    this.robot.rotation.z += dy * k;
    if (this.wheels[0]) this.wheels[0].rotation.y += (this.target.wl - this.wheels[0].rotation.y) * k;
    if (this.wheels[1]) this.wheels[1].rotation.y += (this.target.wr - this.wheels[1].rotation.y) * k;
    this.fusedMark.position.set(this.fused.x, this.fused.y, 0.008);
    this.fusedMark.rotation.z = this.fused.yaw;
    this.fusedMark.visible = !!this.lastState?.fused;
    this.lidar.draw();

    const focus = new THREE.Vector3(rp.x, rp.y, 0.3);
    const yaw = this.robot.rotation.z;
    const fwd = new THREE.Vector3(Math.cos(yaw), Math.sin(yaw), 0);
    if (this.view === 'orbit') {
      const delta = focus.clone().add(fwd.clone().multiplyScalar(0.35)).sub(this.controls.target);
      this.controls.target.add(delta);
      this.camera.position.add(delta);
      this.controls.autoRotate = performance.now() > this.idleUntil;
      this.controls.update(dt);
    } else if (this.view === 'top' && this.planGroup.visible) {
      // Nav2 경로가 보이면 로봇과 목적지 전체 곡선이 한 화면에 들어오도록 부감 높이와 중심을 맞춘다.
      const endW = new THREE.Vector3();
      this.planEnd.getWorldPosition(endW);
      const mid = focus.clone().lerp(endW, 0.56);
      const dist = focus.distanceTo(endW);
      const camZ = Math.min(9.6, Math.max(6.2, dist * 1.62 + 2.6));
      const want = new THREE.Vector3(mid.x - 1.3, mid.y - 1.6, camZ);
      this.camera.position.lerp(want, 1 - Math.exp(-dt / 0.45));
      const look = new THREE.Vector3(mid.x, mid.y, 0.15);
      this.controls.target.lerp(look, 1 - Math.exp(-dt / 0.35));
      this.camera.lookAt(this.controls.target);
    } else {
      const want = this.view === 'follow'
        ? focus.clone().add(fwd.clone().multiplyScalar(-1.9)).add(new THREE.Vector3(0, 0, 1.15))
        : focus.clone().add(fwd.clone().multiplyScalar(-0.6)).add(new THREE.Vector3(0, 0, 5.2));
      this.camera.position.lerp(want, 1 - Math.exp(-dt / 0.35));
      const look = focus.clone().add(fwd.clone().multiplyScalar(this.view === 'follow' ? 0.9 : 0.5));
      this.controls.target.lerp(look, 1 - Math.exp(-dt / 0.25));
      this.camera.lookAt(this.controls.target);
    }
    this.renderer.render(this.scene, this.camera);
    this.labels.render(this.scene, this.camera);
  }
}
