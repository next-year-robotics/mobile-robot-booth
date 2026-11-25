import { test } from 'node:test';
import assert from 'node:assert/strict';
import { buildTools, expressionArgs, moveArgs, num, robotStatus } from '../src/avatar/tools';
import { persona } from '../src/avatar/persona';
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
  assert.deepEqual(expressionArgs({name:'sad'}), {name:'sad',strength:0.7});
});

// ---- Gemini Live 클라이언트: 가짜 소켓과 가짜 마이크로 순서만 본다 ----
class FakeSocket {
  static OPEN = 1;
  readyState = 1; binaryType = ''; sent: any[] = []; closed = false;
  onopen: (() => void) | null = null; onmessage: ((e: { data: string }) => void) | null = null;
  onclose: (() => void) | null = null; onerror: (() => void) | null = null;
  send(data: string) { this.sent.push(JSON.parse(data)); }
  close() { this.closed = true; }
  server(msg: unknown) { this.onmessage?.({ data: JSON.stringify(msg) }); }
}
(globalThis as any).WebSocket = FakeSocket;
class FakeMic { started = 0; stopped = 0; onChunk: (b: ArrayBuffer) => void = () => {};
  async start() { this.started++; } stop() { this.stopped++; } }
const tick = () => new Promise(r => setTimeout(r, 0));
function make(extra: Record<string, unknown> = {}) {
  const sockets: FakeSocket[] = []; const mic = new FakeMic(); const states: string[] = [];
  const live = new LiveVoice({
    token: async () => ({ token: 'auth_tokens/t', model: 'gemini-3.8-live', voice: 'Leda' }),
    system: () => 'persona', tools: () => ({ decls: [{ name: 'give_object', description: '' }], handlers: { give_object: () => ({ ok: false, error: '들고 있는 물건이 없어요.' }) } }),
    status: s => states.push(s), caption: () => {}, mic: mic as never,
    socket: () => { const s = new FakeSocket(); sockets.push(s); return s as never; }, ...extra });
  return { live, sockets, mic, states };
}

test('setup turns off automatic voice detection and audio is sent only between activity marks', async () => {
  const { live, sockets, mic, states } = make();
  const pressed = live.press(); await pressed; await tick();
  const ws = sockets[0]; ws.onopen!();
  const setup = ws.sent[0].setup;
  assert.equal(setup.model, 'models/gemini-3.8-live');
  assert.equal(setup.realtimeInputConfig.automaticActivityDetection.disabled, true);
  assert.equal(JSON.stringify(ws.sent).includes('auth_tokens'), false);
  mic.onChunk(new Int16Array(4).buffer);                    // 연결 전 발화는 모아 둔다
  assert.equal(ws.sent.length, 1);
  ws.server({ setupComplete: {} });
  assert.deepEqual(ws.sent[1], { realtimeInput: { activityStart: {} } });
  assert.ok(ws.sent[2].realtimeInput.audio.data);
  live.release();
  assert.deepEqual(ws.sent.at(-1), { realtimeInput: { activityEnd: {} } });
  mic.onChunk(new Int16Array(4).buffer);                    // 뗀 뒤의 소리는 보내지 않는다
  assert.deepEqual(ws.sent.at(-1), { realtimeInput: { activityEnd: {} } });
  assert.deepEqual(states.filter(s => ['연결 중', '듣는 중', '생각 중'].includes(s)), ['연결 중', '듣는 중', '생각 중']);
  live.stop();
});

test('tool refusal is returned to the model verbatim, unknown tools are refused', async () => {
  const { live, sockets } = make();
  await live.press(); await tick();
  const ws = sockets[0]; ws.onopen!(); ws.server({ setupComplete: {} });
  ws.server({ toolCall: { functionCalls: [{ id: 'a', name: 'give_object', args: {} }, { id: 'b', name: 'rm_rf', args: {} }] } });
  await tick();
  const responses = ws.sent.at(-1).toolResponse.functionResponses;
  assert.deepEqual(responses[0], { id: 'a', name: 'give_object', response: { ok: false, error: '들고 있는 물건이 없어요.' } });
  assert.equal(responses[1].response.ok, false);
  live.stop();
});

test('stopping while the token is pending never opens a socket', async () => {
  let resolve!: (t: { token: string; model: string; voice: string }) => void;
  const { live, sockets, mic } = make({ token: () => new Promise(r => { resolve = r; }) });
  const pressed = live.press(); await tick(); live.stop();
  resolve({ token: 'auth_tokens/t', model: 'm', voice: 'v' }); await pressed; await tick();
  assert.equal(sockets.length, 0); assert.ok(mic.stopped > 0);
});

test('robot notices reach the model only while a session is open and nobody is talking', async () => {
  const { live, sockets } = make();
  assert.equal(live.notify('집었다'), false);
  await live.press(); await tick();
  const ws = sockets[0]; ws.onopen!(); ws.server({ setupComplete: {} });
  assert.equal(live.notify('집었다'), false);                // 누르고 있는 동안은 끼어들지 않는다
  live.release();
  assert.equal(live.notify('집었다'), true);
  assert.equal(ws.sent.at(-1).clientContent.turns[0].parts[0].text, '[로봇 알림] 집었다');
  live.stop();
});

test('introduction request starts local playback and answers the Live tool silently', async () => {
  let starts = 0;
  const tools = buildTools({
    bridge: {} as never, state: () => null, fresh: () => false, setExpression: () => false,
    playIntroduction: async () => { starts++; return true; },
  });
  const { live, sockets } = make({ tools: () => tools });
  try {
    await live.press(); await tick();
    const ws = sockets[0]; ws.onopen!(); ws.server({ setupComplete: {} });
    live.release();
    ws.server({ toolCall: { functionCalls: [{ id: 'intro', name: 'play_introduction', args: {} }] } });
    await tick();
    assert.equal(starts, 1);
    assert.deepEqual(ws.sent.at(-1).toolResponse.functionResponses, [
      { id: 'intro', name: 'play_introduction', response: { ok: true }, scheduling: 'SILENT' },
    ]);
  } finally { live.stop(); }
});

// ---- 장소 이동 도구 ----
const PLACES = [{ key: 'desk', label: '책상', marker_id: 5, aliases: ['테이블'] }, { key: 'door', label: '현관', marker_id: 6, aliases: [] }];
function moveTools(reply: Record<string, unknown>, clips: string[] = []) {
  const asked: unknown[] = [];
  const bridge = { task: async (m: unknown) => { asked.push(m); return reply; }, send: () => {}, arm: async () => ({ ok: true }) };
  const t = buildTools({ bridge: bridge as never, state: () => ({}) as State, fresh: () => true, playIntroduction: async () => true,
    setExpression: () => true, places: () => PLACES, hasClip: k => clips.includes(k) });
  return { ...t, asked };
}

test('movement tools exist only when places are configured and list them for the model', () => {
  const none = buildTools({ bridge: {} as never, state: () => null, fresh: () => true, playIntroduction: async () => true, setExpression: () => true });
  assert.equal(none.decls.some(d => d.name === 'go_to_place'), false);
  const { decls } = moveTools({ ok: true });
  const go = decls.find(d => d.name === 'go_to_place')!;
  const place = (go.parameters as any).properties.place;
  assert.deepEqual(place.enum, ['desk', 'door']);
  assert.ok(place.description.includes('desk=책상/테이블'));
  assert.ok(decls.some(d => d.name === 'fetch_from_place') && decls.some(d => d.name === 'return_home'));
});

test('movement tools pass the request to the bridge, stay silent when a clip speaks, and relay refusals', async () => {
  const quiet = moveTools({ ok: true }, ['nav_go']);
  assert.deepEqual(await quiet.handlers.go_to_place({ place: 'desk' }), { ok: true, silent: true });
  assert.deepEqual(quiet.asked[0], { cmd: 'goto', place: 'desk' });
  const spoken = moveTools({ ok: true });
  const r = await spoken.handlers.fetch_from_place({ place: 'desk', name: ' 약통 ' }) as Record<string, unknown>;
  assert.equal(r.ok, true); assert.equal(r.silent, undefined); assert.ok(typeof r.note === 'string');
  assert.deepEqual(spoken.asked[0], { cmd: 'fetch_from', place: 'desk', name: '약통' });
  assert.equal((await spoken.handlers.fetch_from_place({ place: 'desk' }) as any).ok, false);
  assert.equal(spoken.asked.length, 1);
  const refused = moveTools({ ok: false, error: '이동 노드가 꺼져 있어요.' }, ['nav_return']);
  assert.deepEqual(await refused.handlers.return_home({}), { ok: false, error: '이동 노드가 꺼져 있어요.' });
});

// ---- 여러 동작을 한 문장으로 ----
function chainTools(taskReply: (m: any) => Record<string, unknown>, state: Partial<State> = {}) {
  const asked: any[] = [], armed: any[] = [];
  let turn = 1;
  const bridge = { task: async (m: any) => { asked.push(m); return taskReply(m); }, send: () => {}, arm: async (m: any) => { armed.push(m); return { ok: true }; } };
  const s = { arm: { errand: { state: 'idle', object: '', events: [], inventory: [] } }, ...state } as unknown as State;
  const t = buildTools({ bridge: bridge as never, state: () => s, fresh: () => true, playIntroduction: async () => true,
    setExpression: () => true, places: () => PLACES, nav: () => ({ enabled: true, follow_marker_id: 2 }), turn: () => turn, hasClip: () => true });
  return { ...t, asked, armed, next: () => { turn++; } };
}

test('fetch_from_place takes a marker number or a place and can hand over on return', async () => {
  const t = chainTools(() => ({ ok: true }));
  await t.handlers.fetch_from_place({ marker_id: '7번', name: '귤', hand_over: true });
  assert.deepEqual(t.asked[0], { cmd: 'fetch_from', marker_id: 7, name: '귤', hand_over: true, turn: 1 });
  await t.handlers.fetch_from_place({ place: 'desk', marker_id: 5, name: '귤' });
  assert.deepEqual(t.asked[1], { cmd: 'fetch_from', place: 'desk', name: '귤', turn: 1 });
  assert.equal((await t.handlers.fetch_from_place({ name: '귤' }) as any).ok, false);
  assert.equal(t.asked.length, 2);
});

test('pick and hand-over go through the sequencer while a trip runs, and straight to the arm otherwise', async () => {
  // 가는 중: "가서, 집고, 돌아와"를 도구 셋으로 불렀다. 집기는 팔에 바로 가지 않고 순서기 뒤에 붙는다
  const busy = chainTools(m => m.cmd === 'goto' ? { ok: true } : { ok: true, chained: true, same_turn: true });
  await busy.handlers.go_to_place({ place: 'desk' });
  assert.deepEqual(await busy.handlers.fetch_object({ name: '약통' }), { ok: true, silent: true });
  assert.deepEqual(await busy.handlers.return_home({}), { ok: true, silent: true });
  assert.deepEqual(busy.asked.map(m => [m.cmd, m.turn]), [['goto', 1], ['fetch', 1], ['home', 1]]);
  assert.equal(busy.armed.length, 0);
  // 돌아오는 길에 따로 "줘"라고 했다: 도착하면 건넨다고 말한다
  const later = chainTools(() => ({ ok: true, chained: true, same_turn: false }));
  const r = await later.handlers.give_object({}) as any;
  assert.equal(r.ok, true); assert.ok(r.note.includes('도착하면')); assert.equal(later.armed.length, 0);
  // 하는 일이 없다: 팔에 바로 시킨다. 같은 말의 "돌아와"는 팔이 끝난 뒤에 출발한다
  const idle = chainTools(m => ['fetch', 'give'].includes(m.cmd) ? { ok: false, direct: true } : { ok: true });
  assert.deepEqual(await idle.handlers.fetch_object({ name: '약통' }), { ok: true, silent: true });
  assert.deepEqual(idle.armed, [{ cmd: 'fetch', name: '약통' }]);
  await idle.handlers.return_home({});
  assert.deepEqual(idle.asked.at(-1), { cmd: 'home', turn: 1, after_arm: true });
  idle.next();
  await idle.handlers.return_home({});
  assert.deepEqual(idle.asked.at(-1), { cmd: 'home', turn: 2 });
  // 순서기가 거절하면(따라가는 중) 팔에 보내지 않는다
  const no = chainTools(() => ({ ok: false, error: '따라가는 중이에요.' }));
  assert.deepEqual(await no.handlers.fetch_object({ name: '귤' }), { ok: false, error: '따라가는 중이에요.' });
  assert.equal(no.armed.length, 0);
});

test('the table height marker is not a place to go, and its measurement reaches the model', async () => {
  const state = { mux: { active: null }, markers3d: [{ id: 10 }, { id: 9 }], arm: { errand: { state: 'idle', object: '', events: [], inventory: [],
    table: { z: 0.0912, source: 'marker', spread_m: 0.002, marker_id: 10 } } } } as unknown as State;
  const t = chainTools(() => ({ ok: true }), state);
  await t.handlers.go_to_marker({});
  assert.equal(t.asked[0].marker_id, 9);
  assert.deepEqual((robotStatus(state, true, PLACES) as any).work_table, { source: 'marker', height_cm: 9.1, measured_by_marker: true });
});

test('status tells the model about the errand, places in view and obstacles', () => {
  const state = { sim: true, mux: { active: null }, markers3d: [{ id: 5 }], lidar: { nearest_m: 0.7 },
    nav: { visible: [6], home: [1.2, 0], blocked: true },
    tasks: { task: { title: '책상(으)로 가기', state: 'running', detail: '다가가는 중', problem: '' }, events: [] } } as unknown as State;
  const s = robotStatus(state, true, PLACES) as any;
  assert.deepEqual(s.places_in_view, ['책상', '현관']);
  assert.equal(s.errand.doing, '책상(으)로 가기'); assert.equal(s.away_from_start, true);
  assert.equal(s.nearest_obstacle_m, 0.7); assert.equal(s.path_blocked, true);
});

// ---- 조금 움직이기·번호 마커·던지기 ----
const NAV = { enabled: true, follow_marker_id: 2, nudge_m: 0.3, turn_deg: 90 };
function navTools(reply: Record<string, unknown>, state: Record<string, unknown> = {}, clips: string[] = [], places = PLACES) {
  const asked: unknown[] = [], armed: unknown[] = [], sent: unknown[] = [];
  const bridge = { task: async (m: unknown) => { asked.push(m); return reply; }, send: (m: unknown) => { sent.push(m); },
    arm: async (m: unknown) => { armed.push(m); return reply; } };
  const t = buildTools({ bridge: bridge as never, state: () => state as unknown as State, fresh: () => true, playIntroduction: async () => true,
    setExpression: () => true, places: () => places, nav: () => NAV, hasClip: k => clips.includes(k) });
  return { ...t, asked, armed, sent };
}

test('spoken directions become signed distances and angles with small defaults', () => {
  assert.deepEqual(moveArgs({ direction: 'back' }), { dist_m: -0.3, turn_deg: 0 });
  assert.deepEqual(moveArgs({ direction: 'forward', distance_cm: '50cm' }), { dist_m: 0.5, turn_deg: 0 });
  assert.deepEqual(moveArgs({ direction: 'back', distance_cm: -500 }), { dist_m: -1, turn_deg: 0 });
  assert.deepEqual(moveArgs({ direction: 'left' }), { dist_m: 0, turn_deg: 90 });
  assert.deepEqual(moveArgs({ direction: 'right', angle_deg: 45 }), { dist_m: 0, turn_deg: -45 });
  assert.deepEqual(moveArgs({ direction: 'around' }), { dist_m: 0, turn_deg: 180 });
  assert.equal(moveArgs({ direction: 'up' }), null);
  assert.equal(num('7번'), 7); assert.equal(num(''), null); assert.equal(num(undefined), null); assert.equal(num('몰라'), null);
});

test('movement tools exist without places, and place tools only with places', () => {
  const bare = navTools({ ok: true }, {}, [], []);
  const names = bare.decls.map(d => d.name);
  for (const n of ['move_robot', 'go_to_marker', 'follow_marker', 'return_home', 'throw_object']) assert.ok(names.includes(n), n);
  assert.equal(names.includes('go_to_place'), false);
  assert.ok(navTools({ ok: true }).decls.some(d => d.name === 'go_to_place'));
  const text = persona([], [], true);
  for (const n of ['move_robot', 'go_to_marker', 'follow_marker', 'throw_object', 'return_home']) assert.ok(text.includes(n), n);
  assert.equal(text.includes('go_to_place를'), false);
  assert.equal(persona([], [], false).includes('move_robot을'), false);
});

test('move, marker and follow tools send the right task and relay refusals', async () => {
  const t = navTools({ ok: true }, { nav: { visible: [2, 7] }, marker: { status: 'tracking' } }, ['nav_go', 'follow_start']);
  const moved = await t.handlers.move_robot({ direction: 'back' }) as Record<string, unknown>;
  assert.equal(moved.ok, true); assert.ok(typeof moved.note === 'string');
  assert.deepEqual(t.asked[0], { cmd: 'move', dist_m: -0.3, turn_deg: 0 });
  assert.equal((await t.handlers.move_robot({}) as any).ok, false);
  assert.deepEqual(await t.handlers.go_to_marker({ marker_id: '5' }), { ok: true, silent: true });
  assert.deepEqual(t.asked[1], { cmd: 'goto_marker', marker_id: 5 });
  // 번호를 말하지 않았다: 배지(2)를 빼고 하나만 보이면 그것으로 간다
  assert.deepEqual(await t.handlers.go_to_marker({}), { ok: true, silent: true });
  assert.deepEqual(t.asked[2], { cmd: 'goto_marker', marker_id: 7 });
  assert.deepEqual(await t.handlers.follow_marker({ marker_id: 3 }), { ok: true, silent: true });
  assert.deepEqual(t.asked[3], { cmd: 'follow_marker', marker_id: 3 });
  // 배지 번호를 말하면 평소 따라가기(추종 노드)를 쓴다
  assert.deepEqual(await t.handlers.follow_marker({ marker_id: 2 }), { ok: true, silent: true });
  assert.deepEqual(t.sent, [{ cmd: 'op:start' }]); assert.equal(t.asked.length, 4);
  assert.equal((await t.handlers.follow_marker({}) as any).ok, false);
  const many = navTools({ ok: true }, { nav: { visible: [5, 6] } });
  assert.match((await many.handlers.go_to_marker({}) as any).error, /5, 6번/);
  const none = navTools({ ok: true }, {});
  assert.match((await none.handlers.go_to_marker({}) as any).error, /보이는 마커가 없어요/);
  const refused = navTools({ ok: false, error: '앞 카메라가 꺼져 있어서 마커를 볼 수 없어요.' });
  assert.deepEqual(await refused.handlers.go_to_marker({ marker_id: 5 }), { ok: false, error: '앞 카메라가 꺼져 있어서 마커를 볼 수 없어요.' });
});

test('throw goes to the arm, speaks through the model when no clip is recorded, and relays refusal', async () => {
  const arm = { arm: { errand: { state: 'holding', object: '귤', inventory: [] } } };
  const t = navTools({ ok: true }, arm);
  const r = await t.handlers.throw_object({}) as Record<string, unknown>;
  assert.deepEqual(t.armed, [{ cmd: 'throw' }]); assert.equal(r.ok, true); assert.ok(typeof r.note === 'string');
  assert.deepEqual(await navTools({ ok: true }, arm, ['throwing']).handlers.throw_object({}), { ok: true, silent: true });
  assert.deepEqual(await navTools({ ok: false, error: '지금은 들고 있는 물건이 없어요.' }, arm).handlers.throw_object({}),
    { ok: false, error: '지금은 들고 있는 물건이 없어요.' });
  assert.equal((await navTools({ ok: true }, {}).handlers.throw_object({}) as any).ok, false);
});

test('status reports markers in view and a stop for a person nearby', () => {
  const state = { sim: true, mux: { active: null }, markers3d: [{ id: 7 }], nav: { visible: [2], home: null, blocked: false, guard: { hold: true, near: [0.3, 0.3] } } } as unknown as State;
  const s = robotStatus(state, true, []) as any;
  assert.deepEqual(s.markers_in_view, [2, 7]); assert.equal(s.stopped_for_person_nearby, true);
});
