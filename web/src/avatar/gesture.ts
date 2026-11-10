import { Quaternion, type Object3D, type Interpolant } from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { BVHLoader } from 'three/addons/loaders/BVHLoader.js';
import { createVRMAnimationHumanoidTracks, VRMAnimationLoaderPlugin, type VRMAnimation } from '@pixiv/three-vrm-animation';
import type { VRM, VRMHumanBoneName } from '@pixiv/three-vrm';
import { envelope } from './performance';
import { IdleSchedule } from './idle';

type Track = { node: Object3D; sample: Interpolant; origin?: Quaternion };
type Clip = { tracks: Track[]; duration: number };
/** Local optional assets. Root translation, legs and facial expressions stay untouched. */
export class GreetingGesture {
  private clips: Record<string, Clip> = {};
  private bases = new Map<Object3D, Quaternion>();
  private rotation = new Quaternion();
  private idle = new IdleSchedule();
  private idleWeight = 0;
  private lastIdle: { kind: string; time: number } | null = null;
  get ready() { return !!this.clips.greeting; }
  get idleSmile() { return this.idleWeight * 0.28; }
  resetIdle() { this.idle.reset(); this.lastIdle = null; this.idleWeight = 0; }
  async load(vrm: VRM) {
    const loader = new GLTFLoader(); loader.register(p => new VRMAnimationLoaderPlugin(p));
    const register = (key: string, clip: Clip) => {
      this.clips[key] = clip;
      for (const t of clip.tracks) if (!this.bases.has(t.node)) this.bases.set(t.node, t.node.quaternion.clone());
    };
    await Promise.allSettled(['greeting', 'peace'].map(async key => {
      const gltf = await loader.loadAsync(`${import.meta.env.BASE_URL}avatar/motions/${key}.vrma`);
      const animation = gltf.userData.vrmAnimations?.[0] as VRMAnimation | undefined;
      if (!animation) return;
      const tracks: Track[] = [];
      for (const [bone, track] of createVRMAnimationHumanoidTracks(animation, vrm.humanoid, vrm.meta.metaVersion).rotation) {
        if (/hips|leg|foot|toes|eye|jaw/i.test(bone)) continue;
        const node = vrm.humanoid.getNormalizedBoneNode(bone);
        if (node) tracks.push({ node, sample: track.InterpolantFactoryMethodLinear(new Float32Array(4)) });
      }
      register(key, { tracks, duration: animation.duration });
    }));
    try {
      const bvh = await new BVHLoader().loadAsync(`${import.meta.env.BASE_URL}avatar/motions/neutral-idle.bvh`);
      const tracks: Track[] = [];
      for (const track of bvh.clip.tracks) {
        const bone = track.name.replace(/\.quaternion$/, '');
        // Subtle relative upper-body movement avoids importing a foreign rest pose.
        if (!/^(spine|chest|upperChest|neck|head)$/.test(bone)) continue;
        const node = vrm.humanoid.getNormalizedBoneNode(bone as VRMHumanBoneName);
        if (!node) continue;
        const sample = track.InterpolantFactoryMethodLinear(new Float32Array(4));
        tracks.push({ node, sample, origin: new Quaternion().fromArray(sample.evaluate(0)).invert() });
      }
      register('neutral', { tracks, duration: bvh.clip.duration });
    } catch { /* Procedural breathing remains available without optional BVH. */ }
  }
  restore() { for (const [node, base] of this.bases) node.quaternion.copy(base); }
  private overlay(key: string, time: number, weight: number) {
    const clip = this.clips[key]; if (!clip || weight <= 0) return;
    const blend = weight * envelope(time, 0, clip.duration, key === 'neutral' ? 1.5 : 0.8);
    for (const track of clip.tracks) {
      this.rotation.fromArray(track.sample.evaluate(time));
      if (track.origin) {
        this.rotation.premultiply(track.origin);
        // BVH local rotation delta, blended onto the authored pose.
        this.rotation.premultiply(track.node.quaternion);
      }
      track.node.quaternion.slerp(this.rotation, blend);
    }
  }
  apply(time: number | null, dt: number, clock: number, allowed: boolean) {
    for (const [node, base] of this.bases) base.copy(node.quaternion);
    const idle = this.idle.update(dt, allowed && time === null,
      Object.fromEntries(Object.entries(this.clips).map(([k, v]) => [k, v.duration])));
    if (idle) this.lastIdle = idle;
    this.idleWeight += ((idle ? 1 : 0) - this.idleWeight) * (1 - Math.exp(-dt / 0.22));
    const neutral = this.clips.neutral;
    if (neutral && allowed && time === null) this.overlay('neutral', clock % neutral.duration, 0.35);
    if (this.lastIdle) this.overlay(this.lastIdle.kind, this.lastIdle.time, this.idleWeight);
    if (time !== null) this.overlay('greeting', time, 1);
  }
}
