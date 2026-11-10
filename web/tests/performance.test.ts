import { test } from 'node:test';
import assert from 'node:assert/strict';
import { acting, nameCard, NAME_WORDS, speechShape, VISEMES, INTRO_END } from '../src/avatar/performance';

test('name typography follows audio time, resets and fades at the end', () => {
  assert.equal(nameCard(null).shown, 0);
  assert.equal(nameCard(3.8).shown, 0);
  NAME_WORDS.forEach((w, i) => assert.equal(nameCard(w.at + 0.001).shown, i + 1));
  assert.equal(nameCard(15).opacity, 1);
  assert.ok(nameCard(INTRO_END - 0.7).opacity < 1);
  assert.equal(nameCard(INTRO_END).opacity, 0);
  assert.equal(nameCard(0).shown, 0); // replay/seek backwards
});

test('word-derived lip shapes include all five vowels and close in the pauses', () => {
  assert.deepEqual(new Set(VISEMES.map(v => v.vowel)), new Set(['aa', 'ih', 'ou', 'ee', 'oh']));
  assert.equal(speechShape(15.7), null);
  assert.equal(speechShape(42.7), null);
  for (const cue of VISEMES) {
    assert.ok(cue.end > cue.start);
    const shape = speechShape((cue.start + cue.end) / 2);
    assert.equal(shape?.vowel, cue.vowel);
    assert.ok(shape!.weight > 0 && shape!.weight <= 1);
  }
});

test('acting is stable during speech and relaxes after playback', () => {
  assert.equal(acting(null).wave, 0);
  assert.equal(acting(null).bow, 0);
  assert.ok(acting(41.8).bow > 0.5);
  assert.ok(acting(28.2).laugh > 0.5);
  assert.equal(acting(18).happy, acting(20).happy); // no audio-amplitude dependency
});
