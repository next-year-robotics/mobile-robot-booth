import { VRMExpression, VRMExpressionMorphTargetBind, type VRM, type VRMHumanBoneName } from '@pixiv/three-vrm';
import { Arms } from './arms';
import { GreetingGesture } from './gesture';
import { acting, speechShape, VOWELS } from './performance';
export const EXPRESSIONS = ['neutral', 'happy', 'sad', 'surprised', 'relaxed', 'thinking'] as const;
export type Expression = typeof EXPRESSIONS[number];

export class Motion {
  readonly greeting = new GreetingGesture();
  mouth = 0;
  debugVowel: string | null = null;
  expression: Expression = 'neutral';
  strength = 0.6;
  private arms: Arms;
  private nextBlink = 2;
  private blinkStart = -10;
  private weights: Record<string, number> = {};
  constructor(private vrm: VRM) {
    this.arms = new Arms(vrm);
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
  update(dt: number, t: number, introTime: number | null = null, idleAllowed = true) {
    this.greeting.restore();
    const a = acting(introTime), beat = introTime ?? t;
    const k = 1 - Math.exp(-dt / 0.12);
    if (t > this.nextBlink) { this.blinkStart = t; this.nextBlink = t + 3 + Math.random() * 3.5; }
    const blink = Math.max(0, 1 - Math.abs(t - this.blinkStart - 0.085) / 0.085);
    const emotion = this.expression === 'thinking' ? 'relaxed' : this.expression;
    for (const name of ['happy', 'sad', 'relaxed', 'surprised'] as const) {
      const target = introTime === null ? (emotion === name ? this.strength : idleAllowed && emotion === 'neutral' && name === 'happy' ? this.greeting.idleSmile : 0) : (name === 'surprised' ? 0 : a[name]);
      this.weight(name, target, dt);
    }
    // Eyes no longer pump in response to every audio sample. Mouth smile is small
    // while talking, leaving room for the original aa/ih/ou/ee/oh visemes.
    this.weight('happyMouth', (this.weights.happy ?? 0) * (introTime !== null || this.mouth > 0.04 ? 0.13 : 0.45), dt);
    this.weight('relaxedMouth', 0, dt);
    this.vrm.expressionManager?.setValue('blink', blink * (1 - (this.weights.happy ?? 0) * 0.5));
    const speech = introTime === null ? null : speechShape(introTime)
      ?? (a.laugh > 0.2 ? { vowel: 'aa', weight: a.laugh } : null);
    for (const vowel of VOWELS) {
      const target = this.debugVowel ? Number(this.debugVowel === vowel)
        : introTime !== null
          ? (speech?.vowel === vowel ? speech.weight * (this.mouth < 0.035 ? 0 : Math.min(0.92, 0.26 + Math.sqrt(this.mouth) * 0.8)) : 0)
          : (vowel === 'aa' ? this.mouth * 0.85 : 0);
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
    this.greeting.apply(introTime, dt, t, idleAllowed);
    this.vrm.update(dt);
  }
}
