import assert from 'node:assert/strict';
import { test } from 'node:test';
import { clipVowels, liveVowel } from '../src/avatar/dialogue';

test('caption vowels advance over speech and hold through silent lead-in and pauses', () => {
  const samples = new Float32Array(1200);
  samples.fill(0.2, 100, 500);
  samples.fill(0.2, 900, 1100);
  const shape = clipVowels('아 이 우', samples, 1000);
  assert.deepEqual([0, 0.15, 0.35, 0.5, 0.85, 1.04].map(shape), ['aa', 'aa', 'ih', 'ou', 'ou', 'ou']);
});

test('live vowel estimate responds to resonance changes at equal level', () => {
  const spectra = [[800, 1450], [350, 2450], [350, 1100], [600, 2250], [550, 1250]];
  const shapes = spectra.map(pair => {
    const bins = new Float32Array(256).fill(-90);
    for (const frequency of pair) bins.fill(-24, frequency / 50 - 1, frequency / 50 + 2);
    return liveVowel(bins, 50);
  });
  assert.deepEqual(shapes, ['aa', 'ih', 'ou', 'ee', 'oh']);
});
