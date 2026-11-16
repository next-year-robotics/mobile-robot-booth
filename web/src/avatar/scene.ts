import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { VRMLoaderPlugin, VRMUtils, type VRM } from '@pixiv/three-vrm';
import type { DialogueFrame } from './dialogue';
import { Motion } from './motion';

export class AvatarScene {
  private renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  private scene = new THREE.Scene();
  private camera = new THREE.PerspectiveCamera(29, 1, 0.1, 20);
  private active = false;
  private last = 0;
  private raf = 0;
  private time = 0;
  private headHeight = 0;
  private framing = 0;
  private frames = 0;
  private measureAt = 0;
  motion?: Motion;
  constructor(private host: HTMLElement, private stats: HTMLElement, private voice: () => DialogueFrame, private introTime: () => number | null, private onFrame: (time: number | null) => void, private idleAllowed: () => boolean) {
    this.camera.up.set(0, 1, 0); // The robot twin sets THREE.DEFAULT_UP to Z. VRM uses Y.
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 1.5));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    host.append(this.renderer.domElement);
    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x8f9fae, 0.8));
    const sun = new THREE.DirectionalLight(0xffffff, 2.0);
    sun.position.set(1, 2, 3); this.scene.add(sun);
    new ResizeObserver(() => this.resize()).observe(host);
    this.renderer.domElement.addEventListener('webglcontextlost', (e) => {
      e.preventDefault(); this.setActive(false); stats.textContent = '그래픽 연결이 끊겼습니다. 새로고침해 주세요.';
    });
  }
  async load() {
    const loader = new GLTFLoader();
    loader.register(p => new VRMLoaderPlugin(p));
    const gltf = await loader.loadAsync(`${import.meta.env.BASE_URL}avatar/platinum.vrm`);
    const vrm = gltf.userData.vrm as VRM | undefined;
    if (!vrm) throw new Error('VRM 형식을 읽을 수 없습니다.');
    VRMUtils.removeUnnecessaryVertices(gltf.scene);
    VRMUtils.combineSkeletons(gltf.scene);
    this.motion = new Motion(vrm);
    VRMUtils.combineMorphs(vrm);
    VRMUtils.rotateVRM0(vrm);
    vrm.scene.traverse(o => { o.frustumCulled = false; });
    this.scene.add(vrm.scene);
    vrm.update(0);
    vrm.scene.updateMatrixWorld(true);
    const head = vrm.humanoid.getNormalizedBoneNode('head')!.getWorldPosition(new THREE.Vector3());
    this.headHeight = head.y;
    this.camera.position.set(0, head.y - 0.12, 1.9);
    this.camera.lookAt(0, head.y - 0.12, 0);
    const target = new THREE.Object3D(); target.position.copy(this.camera.position); this.scene.add(target);
    if (vrm.lookAt) vrm.lookAt.target = target;
    this.resize();
    // Optional asset failure preserves the authored wave fallback.
    await this.motion.greeting.load(vrm).catch(() => {});
  }
  private resize() {
    const w = this.host.clientWidth, h = this.host.clientHeight;
    if (!w || !h) return;
    this.renderer.setSize(w, h); this.camera.aspect = w / h; this.camera.updateProjectionMatrix();
  }
  setActive(active: boolean) {
    if (active !== this.active) this.motion?.greeting.resetIdle();
    this.active = active; cancelAnimationFrame(this.raf);
    if (active) { this.last = 0; this.measureAt = performance.now(); this.frames = 0; this.raf = requestAnimationFrame(this.draw); }
  }
  private draw = (now: number) => {
    if (!this.active) return;
    this.raf = requestAnimationFrame(this.draw);
    if (this.last && now - this.last < 1000 / 30 - 0.5) return;
    const dt = Math.min(0.1, this.last ? (now - this.last) / 1000 : 1 / 30);
    this.last = now; this.time += dt;
    const introTime = this.introTime(); this.onFrame(introTime);
    if (this.motion) { this.motion.update(dt, this.time, introTime, this.idleAllowed(), this.voice()); }
    if (this.headHeight) {
      this.framing += ((this.motion?.greeting.wideShot ?? 0) - this.framing) * (1 - Math.exp(-dt / 0.28));
      const y = this.headHeight - 0.12 - this.framing * (this.headHeight / 2 - 0.12);
      this.camera.position.set(0, y, 1.9 + this.framing * (this.headHeight * 2.85 - 1.9));
      this.camera.lookAt(0, y, 0);
    }
    const start = performance.now(); this.renderer.render(this.scene, this.camera); this.frames++;
    if (now - this.measureAt > 2000) {
      const info = this.renderer.info.render;
      this.stats.textContent = `${(this.frames * 1000 / (now - this.measureAt)).toFixed(0)} fps · ${(performance.now() - start).toFixed(1)} ms · ${info.calls} calls · ${info.triangles.toLocaleString()} triangles`;
      this.frames = 0; this.measureAt = now;
    }
  };
}
