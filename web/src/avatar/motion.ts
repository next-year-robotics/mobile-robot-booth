import { VRMExpression, VRMExpressionMorphTargetBind, type VRM, type VRMHumanBoneName } from '@pixiv/three-vrm';
import { dialogueMood, type DialogueFrame } from './dialogue';
import { bindFaces, faceNames, introductionFace, recordedFace, type Face } from './faces';
import { Arms } from './arms';
import { GreetingGesture } from './gesture';
import { acting, speechShape, VOWELS } from './performance';
export const EXPRESSIONS = ['neutral', 'happy', 'sad', 'surprised', 'relaxed', ...faceNames] as const;
export type Expression = typeof EXPRESSIONS[number];

export class Motion {
  readonly greeting = new GreetingGesture();
  private emphasis = 0;
  debugVowel: string | null = null;
  expression: Expression = 'neutral';
  strength = 0.6;
  private arms: Arms;
  private nextBlink = 2;
  private blinkStart = -10;
  private weights: Record<string, number> = {};
  constructor(private vrm: VRM) {
    this.arms = new Arms(vrm);
    bindFaces(vrm);
    // Platinum's existing smile binds combine mouth, eyebrows and eyes. Separate
    // their runtime controls BEFORE combineMorphs; the purchased VRM stays intact.
    for (const name of ['happy', 'relaxed']) {
      const original = vrm.expressionManager?.getExpression(name);
      if (!original) continue;
      const mouth = new VRMExpression(`${name}Mouth`);
      for (const bind of [...original.binds]) {
        if (!(bind instanceof VRMExpressionMorphTargetBind)) continue;
        const names = bind.primitives.flatMap(p => Object.entries(p.morphTargetDictionary ?? {})
          .filter(([, i]) => i === bind.index).map(([n]) => n));
        if (names.some(n => ['にこ口', 'にやり口（開き）', 'わあ'].includes(n))) {
          original.deleteBind(bind); mouth.addBind(bind);
        }
      }
      vrm.expressionManager?.registerExpression(mouth);
    }
    this.pose('leftUpperArm', 0, 0, 1.15, 1);
    this.pose('rightUpperArm', 0, 0, -1.15, 1);
  }
  private pose(name: VRMHumanBoneName, x: number, y: number, z: number, blend: number) {
    const node = this.vrm.humanoid.getNormalizedBoneNode(name);
    if (!node) return;
    node.rotation.x += (x - node.rotation.x) * blend;
    node.rotation.y += (y - node.rotation.y) * blend;
    node.rotation.z += (z - node.rotation.z) * blend;
  }
  private weight(name: string, target: number, dt: number, tau = 0.18) {
    const old = this.weights[name] ?? 0;
    const value = old + (target - old) * (1 - Math.exp(-dt / tau));
    this.weights[name] = value;
    this.vrm.expressionManager?.setValue(name, value);
  }
  update(dt: number, t: number, introTime: number | null = null, idleAllowed = true, voice: DialogueFrame = { level: 0, vowel: 'aa' }) {
    this.greeting.restore();
    const a = acting(introTime), beat = introTime ?? t;
    const mouth = voice.level < 0.035 ? 0 : Math.min(1, Math.sqrt(voice.level));
    const speaking = introTime === null ? mouth : 0;
    this.emphasis += (speaking - this.emphasis) * (1 - Math.exp(-dt / (speaking > this.emphasis ? 0.12 : 0.34)));
    const mood = dialogueMood(voice.key);
    if (introTime === null) {
      const emphasis = this.emphasis;
      a.present = mood === 'happy' ? emphasis * (0.55 + 0.35 * Math.sin(t * 2.6)) : 0;
      a.shrug = mood === 'sad' ? emphasis * 0.75 : 0;
      a.bow = emphasis * (0.32 + 0.22 * Math.sin(t * 4));
      a.look = emphasis * 0.32;
    }
    const k = 1 - Math.exp(-dt / 0.12);
    if (t > this.nextBlink) { this.blinkStart = t; this.nextBlink = t + 3 + Math.random() * 3.5; }
    const blink = Math.max(0, 1 - Math.abs(t - this.blinkStart - 0.085) / 0.085);
    const emotion = this.expression;
    let face: Face | undefined;
    let intensity = 0.95;
    if (introTime !== null) face = introductionFace(introTime);
    else if (emotion !== 'neutral') {
      face = emotion === 'surprised' ? 'startled' : faceNames.find(name => name === emotion);
      intensity = this.strength;
    } else if (voice.key && voice.time !== undefined && voice.duration) {
      face = recordedFace(voice.key, voice.time, voice.duration);
    }
    for (const name of faceNames) {
      this.weight(name, face === name ? intensity : 0, dt, name === 'wink' ? 0.08 : 0.15);
      this.weight(name + 'Mouth', (this.weights[name] ?? 0) * (introTime !== null || mouth > 0 || voice.key ? 0.12 : 0.72), dt);
    }
    for (const name of ['happy', 'sad', 'relaxed', 'surprised'] as const) {
      let target = 0;
      if (introTime !== null) target = name === 'surprised' ? 0 : a[name];
      else if (emotion === name) target = this.strength;
      else if (emotion === 'neutral') {
        const smile = idleAllowed && name === 'happy' ? this.greeting.idleSmile : 0;
        target = Math.max(smile, name === mood ? this.emphasis * 0.55 : 0);
      }
      this.weight(name, face ? target * 0.15 : target, dt);
    }
    // Eyes no longer pump in response to every audio sample. Mouth smile is small
    // while talking, leaving room for the original aa/ih/ou/ee/oh visemes.
    this.weight('happyMouth', (this.weights.happy ?? 0) * (introTime !== null || mouth > 0 ? 0.13 : 0.45), dt);
    this.weight('relaxedMouth', 0, dt);
    this.vrm.expressionManager?.setValue('blink', blink * (1 - Math.max(this.weights.wink ?? 0, this.weights.sleepy ?? 0, (this.weights.happy ?? 0) * 0.5)));
    const speech = introTime === null ? null : speechShape(introTime)
      ?? (a.laugh > 0.2 ? { vowel: 'aa', weight: a.laugh } : null);
    for (const vowel of VOWELS) {
      const target = this.debugVowel ? Number(this.debugVowel === vowel)
        : introTime !== null
          ? (speech?.vowel === vowel ? speech.weight * (mouth === 0 ? 0 : Math.min(0.92, 0.26 + mouth * 0.8)) : 0)
          : (vowel === voice.vowel ? mouth * 0.95 : 0);
      this.weight(vowel, target, dt, target > (this.weights[vowel] ?? 0) ? 0.025 : 0.055);
    }
    const wave = this.greeting.ready ? 0 : a.wave, present = a.present, shrug = a.shrug;
    this.pose('leftShoulder', 0, 0, -0.08 * shrug, k);
    this.pose('rightShoulder', 0, 0, 0.08 * shrug, k);
    const chuckle = a.laugh * Math.sin(beat * 18) * 0.018;
    this.pose('spine', 0.14 * a.bow, Math.sin(t * 0.55) * 0.025, -0.025 * shrug, k);
    this.pose('chest', Math.sin(t * 1.8) * 0.012 + chuckle + 0.11 * a.bow, 0.035 * present, 0, k);
    this.pose('head', 0.11 * a.bow + chuckle - 0.035 * a.proud,
      a.look * Math.sin(beat * 3) * 0.22 + Math.sin(t * 0.65) * 0.025 + a.proud * Math.sin(beat * 3.5) * 0.045,
      Math.sin(t * 0.4) * 0.018 + 0.07 * shrug - 0.035 * wave - 0.055 * a.cross, k);
    this.arms.update(dt, a, beat);
    // Retain the simple wave only if the optional official clip is missing.
    if (wave > 0) {
      this.pose('leftUpperArm', -0.2 * wave, 0, 1.15 - 1.02 * wave, k);
      this.pose('leftLowerArm', 0, -0.12, -1.8 * wave, k);
      this.pose('leftHand', 0, -0.3 * wave, wave * Math.sin(beat * 9) * 0.3, k);
    }
    this.greeting.apply(introTime, dt, t, idleAllowed && !speaking, voice);
    this.vrm.update(dt);
  }
}
