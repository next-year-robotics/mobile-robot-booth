import { test } from 'node:test';
import assert from 'node:assert/strict';
import { IdleSchedule } from '../src/avatar/idle';

test('idle actions wait, avoid repeating, and cancel/rearm for conversation', () => {
  const idle = new IdleSchedule(() => 0.5);
  const lengths = { greeting: 7, peace: 11 };
  assert.equal(idle.update(9, true, lengths), null);
  assert.equal(idle.update(1.1, true, lengths)?.kind, 'greeting');
  assert.equal(idle.update(1, false, lengths), null);
  assert.equal(idle.update(1, true, lengths), null);
  assert.equal(idle.update(30, true, lengths)?.kind, 'peace');
  assert.equal(idle.update(12, true, lengths), null);
  idle.reset();
  assert.equal(idle.update(1, true, lengths), null);
});

test('missing optional animations leave breathing available without invalid selection', () => {
  const idle = new IdleSchedule(() => 0.5);
  assert.equal(idle.update(30, true, {}), null);
});

test('a single surviving clip can still play when another asset is absent', () => {
  const idle = new IdleSchedule(() => 0.5);
  assert.equal(idle.update(30, true, { peace: 11 })?.kind, 'peace');
});
