import { INTRO_WORDS } from './intro-words';
export type Vowel = 'aa' | 'ih' | 'ou' | 'ee' | 'oh';
export const VOWELS: Vowel[] = ['aa', 'ih', 'ou', 'ee', 'oh'];
export const smooth = (x: number) => { x = Math.max(0, Math.min(1, x)); return x * x * (3 - 2 * x); };
export const envelope = (t: number, start: number, end: number, fade = 0.35) =>
  smooth((t - start) / fade) * smooth((end - t) / fade);

// Hangul medial vowel -> the five existing VRM mouth shapes.
const MEDIAL: Vowel[] = ['aa','ee','aa','ee','oh','ee','oh','ee','oh','aa','ee','ee','oh','ou','oh','ee','ih','ou','ih','ih','ih'];
function vowels(word: string): Vowel[] {
  const result: Vowel[] = [];
  for (const char of word) {
    const n = char.charCodeAt(0) - 0xac00;
    if (n >= 0 && n < 11172) result.push(MEDIAL[Math.floor(n / 28) % 21]);
  }
  if (result.length) return result;
  const english: Record<string, Vowel[]> = {
    mobile: ['oh','aa','ih'], assistant: ['oh','ih','oh'], for: ['oh'],
    kins: ['ih'], needs: ['ih'], and: ['ee'], errands: ['ee','oh'], maknae: ['aa','ee'],
  };
  return english[word.toLowerCase().replace(/[^a-z]/g, '')] ?? ['aa'];
}
export const VISEMES = INTRO_WORDS.flatMap(word => {
  const shapes = vowels(word.word), step = (word.end - word.start) / shapes.length;
  return shapes.map((vowel, i) => ({ start: word.start + i * step, end: word.start + (i + 1) * step, vowel }));
});
export function speechShape(t: number): { vowel: Vowel; weight: number } | null {
  const cue = VISEMES.find(c => t >= c.start && t < c.end);
  if (!cue) return null;
  return { vowel: cue.vowel, weight: envelope(t, cue.start, cue.end, Math.min(0.045, (cue.end - cue.start) / 3)) };
}

// Times are audio.currentTime, never wall-clock timers: stop/restart cannot drift.
export const NAME_WORDS = [
  { word: 'Mobile', at: 4.0, initial: true },
  { word: 'Assistant', at: 4.34, initial: true },
  { word: 'for', at: 4.90, initial: false },
  { word: "Kin’s", at: 5.50, initial: true },
  { word: 'Needs', at: 6.12, initial: true },
  { word: 'And', at: 6.38, initial: true },
  { word: 'Errands', at: 6.58, initial: true },
];
export const INTRO_END = 42.840813;
export function nameCard(t: number | null) {
  return {
    opacity: t === null ? 0 : smooth((t - NAME_WORDS[0].at) / 0.2) * smooth((INTRO_END - 0.25 - t) / 0.85),
    shown: t === null ? 0 : NAME_WORDS.filter(w => t >= w.at).length,
  };
}
export function acting(t: number | null) {
  if (t === null) return { wave: 0, present: 0, shrug: 0, proud: 0, laugh: 0, bow: 0, cross: 0, look: 0, happy: 0, relaxed: 0, sad: 0 };
  return {
    wave: envelope(t, 0.1, 3.7, 0.5),
    present: envelope(t, 4.0, 8.2, 0.65),
    shrug: envelope(t, 10.2, 15.8, 0.7),
    proud: envelope(t, 17, 23, 0.8) + envelope(t, 30, 35.2, 0.6),
    laugh: envelope(t, 8.1, 9.0, 0.3) + envelope(t, 27.7, 28.7, 0.35),
    look: envelope(t, 32.0, 33.4, 0.4),
    cross: envelope(t, 30.0, 32.05, 0.65),
    bow: envelope(t, 41.0, 42.8, 0.65),
    happy: 0.22 + envelope(t, 8.1, 9.0, 0.3) * 0.48 + envelope(t, 27.7, 28.7, 0.35) * 0.5,
    relaxed: envelope(t, 10.5, 15.5, 0.6) * 0.28,
    sad: envelope(t, 0, 0.75, 0.2) * 0.2,
  };
}
