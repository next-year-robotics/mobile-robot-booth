import { Quaternion, Vector3, type Object3D, type Interpolant } from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { BVHLoader } from 'three/addons/loaders/BVHLoader.js';
import { createVRMAnimationHumanoidTracks, VRMAnimationLoaderPlugin, type VRMAnimation } from '@pixiv/three-vrm-animation';
import type { VRM, VRMHumanBoneName } from '@pixiv/three-vrm';
import type { DialogueFrame } from './dialogue';
import { envelope } from './performance';
import { IdleSchedule } from './idle';

import { introMoves, recordingMotion, sharedMotions } from './choreography';

type Track = { node: Object3D; sample: Interpolant; origin?: Quaternion };
type Clip = { tracks: Track[]; duration: number; hips?: Interpolant };
/** Body tracks are layered onto the procedural pose. The face remains audio-driven. */
export class GreetingGesture {
  private clips: Record<string, Clip> = {};
  private bases = new Map<Object3D, Quaternion>();
  wideShot = 0;
  private hips?: Object3D;
  private hipsBase = new Vector3();
  private hipsOffset = new Vector3();
  private speechWeight = 0;
  private rotation = new Quaternion();
  private idle = new IdleSchedule();
  private idleWeight = 0;
  private lastIdle: { kind: string; time: number } | null = null;
  get talentReady() { return !!this.clips['keonroot-challenge']; }
  get ready() { return !!this.clips.greeting; }
  get idleSmile() { return this.idleWeight * 0.28; }
  resetIdle() { this.idle.reset(); this.lastIdle = null; this.idleWeight = 0; }
  async load(vrm: VRM) {
    const loader = new GLTFLoader(); loader.register(p => new VRMAnimationLoaderPlugin(p));
    const register = (key: string, clip: Clip) => {
      this.clips[key] = clip;
      for (const t of clip.tracks) if (!this.bases.has(t.node)) this.bases.set(t.node, t.node.quaternion.clone());
    };
    this.hips = vrm.humanoid.getNormalizedBoneNode('hips') ?? undefined;
    if (this.hips) this.hipsBase.copy(this.hips.position);
    await Promise.allSettled(['greeting', 'peace', 'pose', ...sharedMotions].map(async key => {
      const shared = sharedMotions.some(name => name === key);
      const directory = shared ? 'motions' : 'avatar/motions';
      const gltf = await loader.loadAsync(`${import.meta.env.BASE_URL}${directory}/${key}.vrma`);
      const animation = gltf.userData.vrmAnimations?.[0] as VRMAnimation | undefined;
      if (!animation) return;
      const tracks: Track[] = [];
      const humanoid = createVRMAnimationHumanoidTracks(animation, vrm.humanoid, vrm.meta.metaVersion);
      for (const [bone, track] of humanoid.rotation) {
        if (!shared && /hips|leg|foot|toes|eye|jaw/i.test(bone)) continue;
        const node = vrm.humanoid.getNormalizedBoneNode(bone);
        if (node) tracks.push({ node, sample: track.InterpolantFactoryMethodLinear(new Float32Array(4)) });
      }
      const hips = shared ? humanoid.translation.get('hips')?.InterpolantFactoryMethodLinear(new Float32Array(3)) : undefined;
      register(key, { tracks, duration: animation.duration, hips });
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
  restore() {
    for (const [node, base] of this.bases) node.quaternion.copy(base);
    this.hips?.position.copy(this.hipsBase);
    this.wideShot = 0;
  }
  private overlay(key: string, time: number, weight: number, span?: number) {
    const clip = this.clips[key]; if (!clip || weight <= 0) return;
    const duration = span ?? clip.duration;
    const fade = key === 'neutral' ? 1.5 : Math.min(0.45, duration / 3);
    const blend = weight * envelope(time, 0, duration, fade);
    if (blend <= 0) return;
    if (span && clip.hips) {
      // A pickup or handover happens once. Cyclic steps and dance beats keep their tempo.
      if (key === 'keonroot-challenge') time = Math.min(time, clip.duration);
      else if (key === 'pickup' || key === 'reach') time *= clip.duration / span;
      else time %= clip.duration;
    }
    if (clip.hips && this.hips) {
      this.hipsOffset.fromArray(clip.hips.evaluate(time));
      this.hips.position.lerp(this.hipsOffset, blend);
      if (key === 'keonroot-challenge' || key === 'dance' || key === 'cheer' || key === 'walk') this.wideShot = Math.max(this.wideShot, blend);
    }
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
  apply(time: number | null, dt: number, clock: number, allowed: boolean, voice: DialogueFrame) {
    for (const [node, base] of this.bases) base.copy(node.quaternion);
    const speaking = time === null && !voice.key && voice.level > 0.04;
    this.speechWeight += ((speaking ? 0.7 : 0) - this.speechWeight) * (1 - Math.exp(-dt / 0.24));
    const idle = this.idle.update(dt, allowed && time === null,
      Object.fromEntries(Object.entries(this.clips).map(([k, v]) => [k, k === 'dance' || k === 'cheer' ? v.duration * 3 : v.duration])));
    if (idle) this.lastIdle = idle;
    this.idleWeight += ((idle ? 1 : 0) - this.idleWeight) * (1 - Math.exp(-dt / 0.22));
    const neutral = this.clips.neutral;
    if (neutral && allowed && time === null) this.overlay('neutral', clock % neutral.duration, 0.35);
    if (this.lastIdle) {
      const { kind, time: elapsed } = this.lastIdle;
      const span = kind === 'dance' || kind === 'cheer' ? this.clips[kind].duration * 3 : undefined;
      this.overlay(kind, elapsed, this.idleWeight, span);
    }
    if (time !== null) {
      this.overlay('greeting', time, 1);
      const cue = introMoves.find(([start, end]) => time >= start && time < end);
      if (cue) this.overlay(cue[2], time - cue[0], 1, cue[1] - cue[0]);
    } else if (voice.time !== undefined && voice.duration && voice.key) {
      const key = recordingMotion[voice.key];
      if (key) this.overlay(key, voice.time, 1, voice.duration);
    } else if (this.clips.talk && this.speechWeight > 0.01) {
      this.overlay('talk', clock % this.clips.talk.duration, this.speechWeight);
    }
  }
}
