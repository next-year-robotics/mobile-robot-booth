// 음성 모델이 부를 수 있는 도구. 모델은 '무엇을'만 정하고, 실제 동작은 브리지와 팔 데몬의 상태머신이
// 지금 해도 되는지 다시 검사한 뒤 실행한다. 여기서 거절되면 모델은 그 이유를 말로 전할 뿐이다.
// 로봇을 움직이는 도구는 성공하면 silent로 답한다: 모델은 아무 말도 하지 않고, 결과 대사는 녹음해 둔 목소리(Hina)로 나온다.
import type { Place, State } from '../types';
import type { Bridge } from '../ws';
import type { ToolDecl, ToolHandler } from './live';
import { EXPRESSIONS, type Expression } from './motion';

/** 지금 카메라에 마커가 보이는 장소 이름 */
export function visiblePlaces(s: State | null, places: Place[]): string[] {
  const seen = new Set([...(s?.nav?.visible ?? []), ...(s?.markers3d ?? []).map(m => m.id)]);
  return places.filter(p => seen.has(p.marker_id)).map(p => p.label);
}

/** 지금 카메라에 보이는 마커 번호 */
export function visibleMarkers(s: State | null): number[] {
  return [...new Set([...(s?.nav?.visible ?? []), ...(s?.markers3d ?? []).map(m => m.id)])].sort((a, b) => a - b);
}

/** 모델이 준 숫자(문자열로 오기도 한다)를 읽는다. 없거나 숫자가 아니면 null */
export function num(v: unknown): number | null {
  if (v === null || v === undefined || v === '') return null;
  if (typeof v === 'number') return Number.isFinite(v) ? v : null;
  const digits = String(v).replace(/[^\d.+-]/g, '');          // "7번", "50cm"
  const n = digits ? Number(digits) : NaN;
  return Number.isFinite(n) ? n : null;
}

/** "뒤로 조금만", "왼쪽으로 돌아" → 브리지에 보낼 거리 [m]와 각도 [deg]. 못 알아들으면 null */
export function moveArgs(args: Record<string, unknown>, nudge_m = 0.3, turn_deg = 90): { dist_m: number; turn_deg: number } | null {
  const cm = num(args.distance_cm), deg = num(args.angle_deg);
  const dist = cm === null ? nudge_m : Math.min(1, Math.abs(cm) / 100);
  const turn = deg === null ? turn_deg : Math.min(360, Math.abs(deg));
  switch (args.direction) {
    case 'forward': return { dist_m: dist, turn_deg: 0 };
    case 'back': return { dist_m: -dist, turn_deg: 0 };
    case 'left': return { dist_m: 0, turn_deg: turn };
    case 'right': return { dist_m: 0, turn_deg: -turn };
    case 'around': return { dist_m: 0, turn_deg: deg === null ? 180 : turn };
    default: return null;
  }
}

export function robotStatus(s: State | null, fresh: boolean, places: Place[] = []) {
  if (!fresh || !s) return { available: false, mission: null, tracking: null, battery: null, marker: null, input: null, simulation: null, arm: null };
  const t = s.tasks?.task ?? null;
  const appFresh = s.app?.age_s != null && s.app.age_s < 3;
  return { available: true, simulation: s.sim,
    mission: appFresh ? s.app?.mission : null, tracking: appFresh ? s.app?.tracking : null,
    battery: s.battery ? { percent: s.battery.soc, zone: s.battery.zone, voltage: s.battery.v } : null,
    marker: s.marker && s.marker.age_s != null && s.marker.age_s < 3 ? s.marker.status : null,
    input: s.mux.active,
    arm: s.arm ? { state: s.arm.errand.state, holding: s.arm.errand.state === 'holding' ? s.arm.errand.object : null } : null,
    // 말로 시킨 이동·심부름
    errand: t ? { doing: t.title, state: t.state, detail: t.detail || t.problem || null } : null,
    can_move_to_places: !!s.nav, places_in_view: visiblePlaces(s, places), markers_in_view: visibleMarkers(s),
    stopped_for_person_nearby: !!s.nav?.guard?.hold,
    away_from_start: !!s.nav?.home && Math.hypot(s.nav.home[0], s.nav.home[1]) > 0.3,
    nearest_obstacle_m: s.lidar?.nearest_m ?? null, path_blocked: s.nav?.blocked ?? null };
}

export function expressionArgs(args: unknown): { name: Expression; strength: number } | null {
  if (!args || typeof args !== 'object') return null;
  const a = args as Record<string, unknown>;
  const strength = a.strength === undefined ? 0.7 : a.strength;
  if (!EXPRESSIONS.includes(a.name as Expression) || typeof strength !== 'number' || !Number.isFinite(strength) || strength < 0 || strength > 1) return null;
  return { name: a.name as Expression, strength };
}

const str = (v: unknown, max = 40) => (typeof v === 'string' ? v.trim().slice(0, max) : '');

export interface ToolContext {
  bridge: Bridge;
  state: () => State | null;
  fresh: () => boolean;
  playTalentShow?: () => Promise<boolean>;
  playIntroduction: () => Promise<boolean>;
  setExpression: (name: Expression, strength: number) => boolean;
  /** 갈 수 있는 장소(config/booth.yaml nav.places). 없으면 이동 도구가 빠진다 */
  places?: () => Place[];
  /** 그 녹음 대사가 있는가. 있으면 로봇이 직접 말하므로 모델은 조용히 있고, 없으면 모델이 대신 알린다 */
  hasClip?: (key: string) => boolean;
  /** 이동 노드 설정(브리지 hello.nav). enabled가 아니면 이동 도구가 빠진다 */
  nav?: () => { enabled: boolean; follow_marker_id?: number; nudge_m?: number; turn_deg?: number } | undefined;
  /** 앞 보기(브리지 hello.look)가 켜져 있는가. 꺼져 있으면 look_front가 빠진다 */
  look?: () => boolean;
}

export function buildTools(ctx: ToolContext): { decls: ToolDecl[]; handlers: Record<string, ToolHandler> } {
  const arm = () => ctx.fresh() ? ctx.state()?.arm ?? null : null;
  const noArm = { ok: false, error: '로봇팔이 연결돼 있지 않아요.' };
  const places = ctx.places?.() ?? [];
  const placeEnum = { type: 'STRING', enum: places.map(p => p.key),
    description: `장소. ${places.map(p => `${p.key}=${[p.label, ...p.aliases].join('/')}`).join(', ')}` };
  const decls: ToolDecl[] = [
    { name: 'play_talent_show', behavior: 'NON_BLOCKING', description: '장기자랑이나 L을 가져와(가져가) 밈 춤 요청에 화면 속 막내가 음악에 맞춰 춤을 추고, 로봇팔도 집게를 L자로 벌려 함께 춘다.' },
    { name: 'play_introduction', behavior: 'NON_BLOCKING', description: '막내에게 자기소개를 부탁하면 아바타 화면에서 준비된 자기소개 음성과 모션을 함께 재생한다.' },
    { name: 'get_robot_status', description: '로봇의 지금 상태(배터리, 주행 임무, 마커 추적, 로봇팔)를 읽는다.' },
    { name: 'set_expression', description: '화면 속 캐릭터의 표정을 바꾼다.',
      parameters: { type: 'OBJECT', properties: { name: { type: 'STRING', enum: [...EXPRESSIONS] }, strength: { type: 'NUMBER' } }, required: ['name'] } },
    { name: 'look_at_table', description: '팔의 탑뷰 카메라에 보여 준 물건(손에 든 물건 포함)과 작업대 위 물건 목록을 돌려준다. 무엇이 있는지 물을 때 쓴다.' },
    { name: 'fetch_object', behavior: 'NON_BLOCKING', description: '작업대에서 물건을 찾아 집어 든다. 끝나면 [로봇 알림]으로 결과가 온다.',
      parameters: { type: 'OBJECT', properties: { name: { type: 'STRING', description: '가져올 물건을 가리키는 말. 예: 약통, 귤, 빨간 뚜껑 병' } }, required: ['name'] } },
    { name: 'give_object', behavior: 'NON_BLOCKING', description: '들고 있는 물건을 놓아 건넨다. 사람이 달라고 말했을 때만 부른다.' },
    { name: 'throw_object', behavior: 'NON_BLOCKING', description: '들고 있는 물건을 앞으로 던진다. 팔을 뒤로 뺐다가 빠르게 앞으로 휘두르며 놓는다. 사람이 "던져 줘"라고 말했을 때만 부른다.' },
    { name: 'play_motion', behavior: 'NON_BLOCKING', description: '미리 녹화해 둔 로봇팔 동작(재주)을 재생한다.',
      parameters: { type: 'OBJECT', properties: { name: { type: 'STRING' } }, required: ['name'] } },
    { name: 'follow_me', behavior: 'NON_BLOCKING', description: '배지 마커를 단 사람을 따라 걷기 시작한다.' },
    { name: 'stop_moving', behavior: 'NON_BLOCKING', description: '바퀴와 로봇팔을 즉시 멈춘다.' },
    { name: 'call_for_help', behavior: 'NON_BLOCKING', description: '보호자에게 SOS와 위치를 보낸다.' },
    { name: 'cancel_help', behavior: 'NON_BLOCKING', description: '보낸 SOS를 취소한다.' },
  ];
  if (ctx.look?.()) decls.push(
    { name: 'look_front', description: '로봇 앞 카메라로 지금 앞을 사진 한 장 찍어 보고, 질문에 대한 답(seen)을 받는다. 1~2초 걸린다. 앞에 무엇이 있는지, 관람객이 보이는지, 옷 색이나 손에 든 물건, 손가락 개수처럼 눈으로 봐야 답할 수 있을 때만 부른다.',
      parameters: { type: 'OBJECT', properties: { question: { type: 'STRING', description: '관람객이 물은 말 그대로. 예: 앞에 뭐가 보여?, 내가 손에 든 거 뭐야?' } }, required: ['question'] } });
  const nav = ctx.nav?.();
  const moving = nav ? nav.enabled : places.length > 0;
  const markerId = { type: 'INTEGER', description: '마커 번호(0~49). 사람이 번호를 말하지 않았으면 비운다.' };
  if (moving) decls.push(
    { name: 'move_robot', behavior: 'NON_BLOCKING', description: '지금 자리에서 조금 움직인다. "뒤로 조금만 가", "앞으로 30센티 와", "왼쪽으로 돌아", "뒤돌아 봐" 같은 말에 쓴다. 거리나 각도를 말하지 않았으면 비운다(조금만 움직인다).',
      parameters: { type: 'OBJECT', properties: {
        direction: { type: 'STRING', enum: ['forward', 'back', 'left', 'right', 'around'], description: 'forward 앞으로, back 뒤로, left 왼쪽으로 돌기, right 오른쪽으로 돌기, around 뒤돌기' },
        distance_cm: { type: 'NUMBER', description: 'forward·back일 때 갈 거리(센티미터, 최대 100). 1미터는 100.' },
        angle_deg: { type: 'NUMBER', description: 'left·right일 때 돌 각도(도, 최대 360). 한 바퀴는 360.' } }, required: ['direction'] } },
    { name: 'go_to_marker', behavior: 'NON_BLOCKING', description: '번호로 말한 ArUco 마커를 찾아 그 앞으로 가서, 마커 정면에 똑바로 맞춰 선다. "5번 마커 앞으로 가", "마커 앞에 가서 정렬해"에 쓴다. 안 보이면 제자리에서 돌며 찾는다.',
      parameters: { type: 'OBJECT', properties: { marker_id: markerId } } },
    { name: 'follow_marker', behavior: 'NON_BLOCKING', description: '번호로 말한 ArUco 마커를 계속 따라간다. "3번 마커 따라와"에 쓴다. 번호 없이 "따라와"라고만 하면 이 도구가 아니라 follow_me다.',
      parameters: { type: 'OBJECT', properties: { marker_id: markerId }, required: ['marker_id'] } },
    { name: 'return_home', behavior: 'NON_BLOCKING', description: '말로 시킨 이동(장소, 마커, 조금 움직이기)을 시작하기 전에 있던 자리로 돌아온다.' },
  );
  if (moving && places.length) decls.push(
    { name: 'go_to_place', behavior: 'NON_BLOCKING', description: '그 장소에 붙여 둔 마커를 찾아 그 앞까지 스스로 간다. 마커가 안 보이면 제자리에서 돌며 찾는다. 도착하면 [로봇 알림]이 온다.',
      parameters: { type: 'OBJECT', properties: { place: placeEnum }, required: ['place'] } },
    { name: 'fetch_from_place', behavior: 'NON_BLOCKING', description: '장소로 가서 그곳의 물건을 로봇팔로 집고, 출발한 자리로 돌아온다. 한 번에 끝까지 한다.',
      parameters: { type: 'OBJECT', properties: { place: placeEnum, name: { type: 'STRING', description: '가져올 물건을 가리키는 말' } }, required: ['place', 'name'] } },
  );
  const task = async (msg: Record<string, unknown>, clip: string, note: string) => {
    if (!ctx.fresh()) return { ok: false, error: '로봇과 연결이 끊겼어요.' };
    const r = await ctx.bridge.task(msg);
    if (!r.ok) return { ok: false, error: r.error ?? '실패' };
    return clip && ctx.hasClip?.(clip) ? { ok: true, silent: true } : { ok: true, note };
  };
  const armCall = async (msg: Record<string, unknown>) => {
    if (!arm()) return noArm;
    const r = await ctx.bridge.arm(msg);
    return r.ok ? { ok: true, silent: true } : { ok: false, error: r.error ?? '실패' };
  };
  const follow = () => {
    const s = ctx.state();
    if (!ctx.fresh() || !s) return { ok: false, error: '로봇과 연결이 끊겼어요.' };
    if (s.arm && !['idle', 'holding'].includes(s.arm.errand.state)) return { ok: false, error: '팔이 움직이는 중이라 지금은 출발할 수 없어요.' };
    if (!s.marker || s.marker.status === 'lost') return { ok: false, error: '배지 마커가 안 보여요. 카메라 앞에서 마커를 보여 주세요.' };
    ctx.bridge.send({ cmd: 'op:start' });
    return { ok: true, silent: true };
  };
  const handlers: Record<string, ToolHandler> = {
    move_robot: args => {
      const m = moveArgs(args, nav?.nudge_m, nav?.turn_deg);
      return m ? task({ cmd: 'move', ...m }, '', '움직이기 시작했다. "네" 정도로 아주 짧게만 답해라.')
        : { ok: false, error: '어느 쪽으로 움직일지 알아듣지 못했어요.' };
    },
    go_to_marker: args => {
      let id = num(args.marker_id);
      if (id === null) {
        // 번호를 말하지 않았다. 보이는 마커가 하나뿐이면 그것으로 간다(따라가는 배지는 뺀다)
        const seen = visibleMarkers(ctx.fresh() ? ctx.state() : null).filter(i => i !== nav?.follow_marker_id);
        if (seen.length !== 1) return { ok: false, error: seen.length ? `마커가 여러 개 보여요(${seen.join(', ')}번). 몇 번으로 갈지 말해 주세요.` : '지금 보이는 마커가 없어요. 몇 번 마커로 갈지 말해 주세요.' };
        id = seen[0];
      }
      return task({ cmd: 'goto_marker', marker_id: Math.round(id) }, 'nav_go', '출발했다. 가 보겠다고 한 문장으로만 말해라.');
    },
    follow_marker: args => {
      const id = num(args.marker_id);
      if (id === null) return { ok: false, error: '몇 번 마커를 따라갈지 말해 주세요.' };
      if (Math.round(id) === nav?.follow_marker_id) return follow();       // 배지 번호면 평소 따라가기다
      return task({ cmd: 'follow_marker', marker_id: Math.round(id) }, 'follow_start', '따라가기 시작했다. 따라가겠다고 한 문장으로만 말해라.');
    },
    throw_object: async () => {
      if (!arm()) return noArm;
      const r = await ctx.bridge.arm({ cmd: 'throw' });
      if (!r.ok) return { ok: false, error: r.error ?? '실패' };
      return ctx.hasClip?.('throwing') ? { ok: true, silent: true } : { ok: true, note: '던지기 시작했다. "던질게요, 받으세요!" 한마디만 해라.' };
    },
    play_talent_show: async () => await ctx.playTalentShow?.()
      ? { ok: true, silent: true } : { ok: false, error: '장기자랑을 재생할 수 없어요.' },
    play_introduction: async () => await ctx.playIntroduction()
      ? { ok: true, silent: true } : { ok: false, error: '자기소개를 재생할 수 없어요.' },
    get_robot_status: () => robotStatus(ctx.state(), ctx.fresh(), places),
    look_front: async args => {
      if (!ctx.fresh()) return { ok: false, error: '로봇과 연결이 끊겼어요.' };
      const r = await ctx.bridge.look(str(args.question, 120));
      return r.ok && r.answer ? { ok: true, seen: r.answer, note: '방금 앞을 보고 얻은 답이다. 막내 말투(존댓말)로 한두 문장 전해라.' }
        : { ok: false, error: r.error ?? '앞을 보지 못했어요.' };
    },
    go_to_place: args => task({ cmd: 'goto', place: str(args.place) }, 'nav_go', '출발했다. 다녀오겠다고 한 문장으로만 말해라.'),
    fetch_from_place: args => {
      const name = str(args.name);
      return name ? task({ cmd: 'fetch_from', place: str(args.place), name }, 'nav_go', '출발했다. 가서 가져오겠다고 한 문장으로만 말해라.')
        : { ok: false, error: '무엇을 가져올지 이름이 없다.' };
    },
    return_home: () => task({ cmd: 'home' }, 'nav_return', '돌아가기 시작했다. 한 문장으로만 말해라.'),
    set_expression: args => {
      const valid = expressionArgs(args);
      return { ok: !!valid && ctx.setExpression(valid.name, valid.strength) };
    },
    look_at_table: async () => {
      const a = arm();
      if (!a) return noArm;
      const seq = a.errand.events.at(-1)?.seq ?? 0;
      const r = await ctx.bridge.arm({ cmd: 'scan' });
      if (!r.ok) return { ok: false, error: r.error ?? '작업대를 살필 수 없어요.' };
      const until = Date.now() + 25000;
      while (Date.now() < until) {
        const e = arm()?.errand;
        if (!e) return noArm;
        const done = e.events.find(v => v.seq > seq && ['scan_done', 'scan_failed'].includes(v.type));
        if (done?.type === 'scan_failed') return { ok: false, error: done.text };
        if (done) return { ok: true, objects: e.inventory.map(i => ({ name: i.label, reachable: i.reachable })),
          seconds_since_look: e.inventory_age_s, holding: e.state === 'holding' ? e.object : null };
        await new Promise(resolve => setTimeout(resolve, 200));
      }
      return { ok: false, error: '작업대 인식 결과를 받지 못했어요.' };
    },
    fetch_object: args => {
      const name = str(args.name);
      return name ? armCall({ cmd: 'fetch', name }) : { ok: false, error: '무엇을 가져올지 이름이 없다.' };
    },
    give_object: () => armCall({ cmd: 'give' }),
    play_motion: args => armCall({ cmd: 'motion', name: str(args.name, 32) }),
    follow_me: follow,
    stop_moving: () => {
      // op:stop은 따라가기와 장소 이동(순서기·이동 노드)을 함께 끝낸다
      ctx.bridge.send({ cmd: 'op:stop' });
      if (arm()) void ctx.bridge.arm({ cmd: 'stop' });
      return { ok: true, silent: true };
    },
    call_for_help: () => { ctx.bridge.send({ cmd: 'op:sos' }); return { ok: true, silent: true }; },
    cancel_help: () => { ctx.bridge.send({ cmd: 'op:sos_cancel' }); return { ok: true, note: '취소했다고 짧게 알려라.' }; },
  };
  return { decls, handlers };
}
