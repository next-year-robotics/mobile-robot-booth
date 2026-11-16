import { vowels, type Vowel } from './performance';

export interface DialogueFrame {
  level: number;
  vowel: Vowel;
  key?: string;
  time?: number;
  duration?: number;
}

// Distribute caption syllables over audible 20 ms windows so a pause doesn't advance the lips.
export function clipVowels(text: string, samples: Float32Array, rate: number) {
  const shapes = vowels(text);
  const hop = Math.max(1, Math.round(rate * 0.02));
  const progress = new Uint32Array(Math.ceil(samples.length / hop));
  let active = 0;
  for (let i = 0; i < progress.length; i++) {
    const start = i * hop, end = Math.min(start + hop, samples.length);
    let energy = 0;
    for (let j = start; j < end; j++) energy += samples[j] ** 2;
    progress[i] = active;
    if (energy > (end - start) * 0.012 ** 2) active++;
  }
  return (seconds: number) => {
    const frame = Math.min(progress.length - 1, Math.max(0, Math.floor(seconds * rate / hop)));
    return shapes[Math.min(shapes.length - 1, Math.floor(progress[frame] * shapes.length / Math.max(1, active)))] ?? 'aa';
  };
}

// Approximate the first two vowel resonances from the live spectrum.
export function liveVowel(bins: Float32Array, hzPerBin: number): Vowel {
  function peak(low: number, high: number) {
    let best = low, strongest = -Infinity;
    for (let i = Math.ceil(low / hzPerBin); i <= Math.floor(high / hzPerBin); i++) {
      const power = (bins[i - 1] + 2 * bins[i] + bins[i + 1]) / 4 + 6 * Math.log2(i);
      if (power > strongest) { strongest = power; best = i * hzPerBin; }
    }
    return best;
  }
  const first = peak(280, 1000), second = peak(1050, 3100);
  if (second > 1850) return first < 510 ? 'ih' : 'ee';
  if (first > 640) return 'aa';
  return first < 440 ? 'ou' : 'oh';
}

export function dialogueMood(key?: string) {
  if (['marker_lost', 'not_found', 'unreachable', 'missed', 'failed', 'battery_low', 'nav_back_empty', 'nav_not_found', 'nav_blocked', 'nav_failed'].includes(key ?? '')) return 'sad';
  if (['sos_sent', 'stopped', 'follow_stop', 'wait_close'].includes(key ?? '')) return 'relaxed';
  return 'happy';
}
