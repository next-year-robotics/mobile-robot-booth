// 실제 Gemini Live에 한국어 문장을 글로 넣어 보고, 어떤 도구를 어떤 인자로 부르는지 본다. 로봇은 움직이지 않는다
// (도구는 가짜 브리지가 받아서 적어 두기만 한다). 인터넷과 ~/.config/mobile-robot/gemini.env가 필요하다.
//
//   cd web && npm run voice-eval            전체
//   cd web && npm run voice-eval -- 던져     "던져"가 들어간 문장만
//
// 대시보드와 같은 페르소나·도구 선언(src/avatar)을 그대로 쓴다. 말을 바꿔 가며 모델이 헷갈리는 표현을 찾을 때 쓴다.
import { execFileSync } from 'node:child_process';
import { resolve } from 'node:path';
import { persona } from '../src/avatar/persona';
import { buildTools } from '../src/avatar/tools';
import type { State } from '../src/types';

const HOST = 'wss://generativelanguage.googleapis.com/ws/google.ai.generativelanguage.v1beta.GenerativeService.BidiGenerateContentConstrained';
const ROOT = resolve(import.meta.dirname, '../..');
const PLACES = [{ key: 'desk', label: '책상', marker_id: 5, aliases: ['테이블', '작업대'] }, { key: 'door', label: '현관', marker_id: 6, aliases: ['문', '입구'] }];
const NAV = { enabled: true, follow_marker_id: 2, nudge_m: 0.3, turn_deg: 90 };

type Call = { name: string; args: Record<string, unknown> };
interface Case { say: string; want: string; args?: Record<string, unknown>; holding?: string; visible?: number[] }
// want: 불러야 할 도구. '-'면 로봇을 움직이는 도구를 부르면 안 된다.
const CASES: Case[] = [
  { say: '막내야 장기자랑 해줘', want: 'play_talent_show' },
  { say: '막내야 따라와', want: 'follow_me' },
  { say: '나랑 같이 가자', want: 'follow_me' },
  { say: '멈춰', want: 'stop_moving' },
  { say: '그만 서', want: 'stop_moving' },
  { say: '책상으로 가', want: 'go_to_place', args: { place: 'desk' } },
  { say: '저기 보이는 테이블로 이동해', want: 'go_to_place', args: { place: 'desk' } },
  { say: '현관 쪽으로 가 봐', want: 'go_to_place', args: { place: 'door' } },
  { say: '책상에 가서 귤 좀 가져와', want: 'fetch_from_place', args: { place: 'desk' } },
  { say: '이제 돌아와', want: 'return_home' },
  { say: '원래 있던 자리로 가', want: 'return_home' },
  { say: '뒤로 쫌만 가', want: 'move_robot', args: { direction: 'back' } },
  { say: '뒤로 오십 센티만 물러나', want: 'move_robot', args: { direction: 'back', distance_cm: 50 } },
  { say: '앞으로 조금만 와 봐', want: 'move_robot', args: { direction: 'forward' } },
  { say: '왼쪽으로 돌아', want: 'move_robot', args: { direction: 'left' } },
  { say: '오른쪽으로 사십오 도 돌아 봐', want: 'move_robot', args: { direction: 'right', angle_deg: 45 } },
  { say: '뒤돌아 봐', want: 'move_robot', args: { direction: 'around' } },
  { say: '삼 번 아루코 마커 따라와', want: 'follow_marker', args: { marker_id: 3 } },
  { say: '7번 마커를 따라가', want: 'follow_marker', args: { marker_id: 7 } },
  { say: '오 번 마커 앞으로 가', want: 'go_to_marker', args: { marker_id: 5 } },
  { say: '아루코 마커 앞으로 가서 정렬해', want: 'go_to_marker', visible: [9] },
  { say: '8번 마커 앞에 가서 똑바로 서', want: 'go_to_marker', args: { marker_id: 8 } },
  { say: '귤 가져와 줘', want: 'fetch_object' },
  { say: '그거 이리 줘', want: 'give_object', holding: '귤' },
  { say: '던져 줘', want: 'throw_object', holding: '귤' },
  { say: '그냥 던져', want: 'throw_object', holding: '테이프' },
  { say: '나한테 던져 봐', want: 'throw_object', holding: '귤' },
  { say: '고마워 잘 받을게', want: 'give_object', holding: '귤' },
  { say: '도와줘 보호자 좀 불러 줘', want: 'call_for_help' },
  { say: '엘을 가져가 춤 춰 줘', want: 'play_talent_show' },
  { say: '앞에 뭐가 보여?', want: 'look_front' },
  { say: '내가 지금 손에 들고 있는 거 뭔지 맞혀 봐', want: 'look_front' },
  { say: '막내야 내 옷 무슨 색이야?', want: 'look_front' },
  { say: '작업대 위에 뭐 있어?', want: '-' },
  { say: '너는 누가 만들었어?', want: '-' },
  { say: '오늘 날씨 어때 보여?', want: '-' },
];
const MOVERS = new Set(['play_talent_show', 'follow_me', 'stop_moving', 'go_to_place', 'fetch_from_place', 'return_home', 'move_robot', 'go_to_marker', 'follow_marker',
  'fetch_object', 'give_object', 'throw_object', 'play_motion', 'call_for_help',
  'look_front']);                      // 움직이지는 않지만 사진 한 장(토큰)이 드니 필요할 때만 불러야 한다

function token(): { token: string; model: string } {
  const py = `import json,yaml;from booth_bridge.live_token import issue_token
print(json.dumps(issue_token(yaml.safe_load(open('${ROOT}/config/booth.yaml'))['live'])))`;
  const r = JSON.parse(execFileSync('python3', ['-c', py], { cwd: `${ROOT}/bridge`, encoding: 'utf8' }));
  if (!r.ok) throw new Error(`토큰 발급 실패: ${r.error}`);
  return r;
}

function state(c: Case): State {
  const errand = { state: c.holding ? 'holding' : 'idle', object: c.holding ?? '', inventory: [{ label: '귤', reachable: true }, { label: '테이프', reachable: true }],
    inventory_age_s: 3, motions: [], events: [] };
  return { sim: false, mux: { active: null }, marker: { status: 'tracking', age_s: 0.1, range: 1.2 }, markers3d: [],
    nav: { state: 'idle', visible: c.visible ?? [], home: null, blocked: false, guard: { hold: false, near: null } },
    arm: { errand }, tasks: { enabled: true, task: null, events: [] } } as unknown as State;
}

async function ask(c: Case): Promise<Call[]> {
  const calls: Call[] = [];
  const bridge = { task: async () => ({ ok: true }), arm: async () => ({ ok: true }), send: () => {},
    look: async () => ({ ok: true, answer: '파란 옷을 입은 분이 손에 귤을 들고 서 계세요.' }) };
  const s = state(c);
  const { decls, handlers } = buildTools({ bridge: bridge as never, state: () => s, fresh: () => true, playIntroduction: async () => true, playTalentShow: async () => true,
    setExpression: () => true, places: () => PLACES, nav: () => NAV, look: () => true, hasClip: () => true });
  const t = token();
  const ws = new WebSocket(`${HOST}?access_token=${encodeURIComponent(t.token)}`);
  ws.binaryType = 'arraybuffer';
  return await new Promise<Call[]>((done, fail) => {
    let quiet: NodeJS.Timeout;
    const finish = () => { clearTimeout(hard); clearTimeout(quiet); try { ws.close(); } catch { /* 이미 닫혔다 */ } done(calls); };
    const hard = setTimeout(finish, 25_000);
    ws.onerror = () => { clearTimeout(hard); fail(new Error('연결 실패')); };
    ws.onopen = () => ws.send(JSON.stringify({ setup: {
      model: `models/${t.model}`, generationConfig: { responseModalities: ['AUDIO'] },
      systemInstruction: { parts: [{ text: persona([], PLACES, true, true) }] }, tools: [{ functionDeclarations: decls }],
      realtimeInputConfig: { automaticActivityDetection: { disabled: true } }, outputAudioTranscription: {},
    } }));
    ws.onmessage = async ev => {
      const msg = JSON.parse(typeof ev.data === 'string' ? ev.data : new TextDecoder().decode(ev.data as ArrayBuffer));
      if (msg.setupComplete) {
        const context = c.holding ? `[로봇 알림] 작업대에 보이는 물건: 테이프. 지금 ${c.holding}을(를) 들고 있다. 이 알림에는 대답하지 않는다.`
          : '[로봇 알림] 작업대에 보이는 물건: 귤, 테이프. 이 알림에는 대답하지 않는다.';
        ws.send(JSON.stringify({ clientContent: { turns: [{ role: 'user', parts: [{ text: context }] }], turnComplete: false } }));
        ws.send(JSON.stringify({ clientContent: { turns: [{ role: 'user', parts: [{ text: c.say }] }], turnComplete: true } }));
      }
      for (const call of msg.toolCall?.functionCalls ?? []) {
        calls.push({ name: call.name, args: call.args ?? {} });
        const r = await handlers[call.name]?.(call.args ?? {}) as Record<string, unknown> | undefined;
        const { silent, ...body } = r ?? { ok: false };
        ws.send(JSON.stringify({ toolResponse: { functionResponses: [{ id: call.id, name: call.name, response: body, ...(silent ? { scheduling: 'SILENT' } : {}) }] } }));
        clearTimeout(quiet); quiet = setTimeout(finish, 2500);       // 도구를 부른 뒤 더 부르는지 잠깐 본다
      }
      if (msg.serverContent?.turnComplete) { clearTimeout(quiet); quiet = setTimeout(finish, calls.length ? 300 : 1500); }
    };
  });
}

function judge(c: Case, calls: Call[]): string {
  const moved = calls.filter(k => MOVERS.has(k.name));
  if (c.want === '-') return moved.length ? `움직이는 도구를 불렀다: ${moved.map(k => k.name).join(', ')}` : '';
  if (!moved.length) return '도구를 부르지 않았다';
  if (moved.length > 1 || moved[0].name !== c.want) return `다른 도구: ${moved.map(k => `${k.name}(${JSON.stringify(k.args)})`).join(', ')}`;
  for (const [k, v] of Object.entries(c.args ?? {})) {
    const got = moved[0].args[k];
    if (String(got) !== String(v) && Number(got) !== Number(v)) return `${k}가 ${JSON.stringify(got)} (바란 값 ${JSON.stringify(v)})`;
  }
  return '';
}

const filter = process.argv[2];
let bad = 0, n = 0;
for (const c of CASES.filter(k => !filter || k.say.includes(filter))) {
  n++;
  let why: string, calls: Call[] = [];
  try { calls = await ask(c); why = judge(c, calls); } catch (e) { why = e instanceof Error ? e.message : String(e); }
  if (why) bad++;
  const shown = calls.filter(k => MOVERS.has(k.name)).map(k => `${k.name}${Object.keys(k.args).length ? JSON.stringify(k.args) : ''}`).join(' ') || '(없음)';
  console.log(`${why ? '✗' : '✓'} "${c.say}"${c.holding ? ` [${c.holding} 들고 있음]` : ''} → ${shown}${why ? `   ← ${why}` : ''}`);
}
console.log(`\n${n - bad}/${n} 통과`);
process.exit(bad ? 1 : 0);
