import { test } from 'node:test';
import assert from 'node:assert/strict';
import { expressionArgs, robotStatus } from '../src/avatar/tools';
import { LiveVoice } from '../src/avatar/live';
import type { State } from '../src/types';

globalThis.window = globalThis as unknown as Window & typeof globalThis;

test('status never discloses stale state and omits visitor/trip identifiers', () => {
  const state = { sim: true, app: { age_s: 0.5, mission: 'active', tracking: 'tracking', trip_id: 'private' },
    battery: { soc: 70, zone: 'ok', v: 13.2 }, marker: { age_s: 4, status: 'tracking' }, mux: { active: 'teleop' } } as State;
  const fresh = robotStatus(state, true);
  assert.equal(fresh.mission, 'active'); assert.equal(fresh.simulation, true);
  assert.equal(fresh.marker, null); assert.equal(JSON.stringify(fresh).includes('private'), false);
  const old = robotStatus(state, false);
  assert.equal(old.available, false); assert.equal(old.battery, null); assert.equal(old.input, null);
  state.app!.age_s = 4; assert.equal(robotStatus(state, true).mission, null);
});

test('expression tools reject arbitrary commands and invalid intensity', () => {
  for (const args of [null, {name:'op:start',strength:1}, {name:'happy',strength:NaN}, {name:'happy',strength:2}, {name:'happy',strength:'1'}]) assert.equal(expressionArgs(args), null);
  assert.deepEqual(expressionArgs({name:'happy',strength:0.5}), {name:'happy',strength:0.5});
});

test('release while connecting initializes with muted microphone', async () => {
  let resolve!: (s: string) => void;
  const token = new Promise<string>(r => { resolve = r; });
  const muted: boolean[] = []; let ended = 0;
  const fake = { setMicMuted: (v: boolean) => muted.push(v), endSession: async () => { ended++; } };
  const live = new LiveVoice(() => token, () => {}, () => {}, {}, async options => {
    options.onConversationCreated?.(fake as never); return fake as never;
  });
  const pending = live.press(); live.release(); resolve('signed'); await pending;
  assert.ok(muted.length > 0 && muted.every(Boolean));
  await live.stop(); assert.equal(ended, 1);
});

test('leaving view while token is pending prevents opening microphone', async () => {
  let resolve!: (s: string) => void;
  const token = new Promise<string>(r => { resolve = r; }); let opened = 0;
  const live = new LiveVoice(() => token, () => {}, () => {}, {}, async () => { opened++; return {} as never; });
  const pending = live.press(); await live.stop(); resolve('signed'); await pending;
  assert.equal(opened, 0);
});

test('session completing after cancellation is immediately closed', async () => {
  let resolve!: (s: never) => void;
  const session = new Promise<never>(r => { resolve = r; }); let ended = 0;
  const live = new LiveVoice(async () => 'signed', () => {}, () => {}, {}, () => session);
  const pending = live.press(); await Promise.resolve(); await live.stop();
  resolve({endSession: async () => { ended++; }} as never); await pending;
  assert.equal(ended, 1);
});
