// 막내의 목소리 전체. 대시보드·아바타 어느 화면에서든 한 벌만 돈다.
//
//   실시간 대화   Gemini Live (avatar/live.ts). V를 누르고 말한다.
//   고정 대사     로봇 상태가 바뀔 때 녹음 파일을 바로 튼다(출발, 마커 놓침, 지나갈게요 …).
//   심부름 결과   대화 세션이 열려 있으면 모델에게 알려 말하게 하고, 없으면 녹음 대사를 튼다.
import { LiveVoice } from '../avatar/live';
import type { Expression } from '../avatar/motion';
import { persona } from '../avatar/persona';
import { buildTools, visiblePlaces } from '../avatar/tools';
import type { ArmEvent, State, TaskEvent } from '../types';
import type { Bridge } from '../ws';
import { Clips, type Clip } from './clips';
import { settings } from '../settings';
import { player } from './player';

export interface VoiceUi { state: string; detail: string; who: 'user' | 'maknae' | ''; text: string }
type Listener = (ui: VoiceUi) => void;

// 심부름 사건 → [녹음 대사 키, 모델에게 줄 알림(없으면 모델은 말하지 않는다)]
const ERRAND: Record<string, (e: ArmEvent) => [string, string | null]> = {
  looking: () => ['looking', null],
  picking: () => ['picking', null],
  holding: e => ['holding', `"${e.text}"을(를) 집어서 앞으로 내밀고 있다. 가져왔다고 알리고, 받으려면 "줘"라고 말해 달라고 안내해라.`],
  released: e => ['released', `"${e.text}"을(를) 놓아 건넸다. 짧게 한마디 해라.`],
  throwing: e => ['throwing', `"${e.text}"을(를) 앞으로 던졌다.`],
  not_found: e => ['not_found', `"${e.text}"을(를) 작업대에서 찾지 못했다.${e.reason ? ` 보이는 것: ${e.reason}` : ''} 못 찾았다고 알리고 다른 물건을 권해라.`],
  unreachable: e => ['unreachable', `"${e.text}"이(가) 보이지만 팔이 닿지 않는 곳에 있다. ${e.reason || '더 가까이 놓아 주세요.'}라고 안내해라.`],
  missed: e => ['missed', `"${e.text}"을(를) 집으려다 놓쳤다. 사과하고 다시 시켜 달라고 해라.`],
  failed: e => ['failed', `동작 중 문제가 생겨 멈췄다(${e.text}). 잠시 문제가 생겼다고만 알려라.`],
  stopped: () => ['stopped', null],
  motion: () => ['motion', null],
};

// 이동·다녀오는 심부름 사건(브리지 tasks.py) → [녹음 대사 키, 대사가 없을 때 모델에게 시킬 말]
const TRIP: Record<string, (e: TaskEvent) => [string, string] | null> = {
  go_start: e => ['nav_go', `${e.label}(으)로 출발했다.`],
  move_start: () => null,                                  // 조금 움직이는 것은 모델이 "네" 하고 만다
  moved: () => null,
  follow_start: e => ['follow_start', `${e.label}을(를) 따라가기 시작했다.`],
  arrived: e => ['nav_arrived', `${e.label}에 도착했다.`],
  fetching: () => null,                                    // 팔 사건(찾아볼게요, 집어 올게요)이 따로 말한다
  return_start: e => ['nav_return', e.task === 'home' ? '원래 자리로 돌아가기 시작했다.' : '이제 원래 자리로 돌아간다.'],
  returned: e => e.got ? ['holding', `${e.got}을(를) 가지고 돌아왔다. 받으려면 "줘"라고 말해 달라고 안내한다.`]
    : e.task === 'fetch_from' ? ['nav_back_empty', `돌아왔지만 ${e.object}은(는) 가져오지 못했다.`] : ['nav_back', '원래 자리로 돌아왔다.'],
  failed: e => [e.reason === 'not_found' ? 'nav_not_found' : e.reason === 'blocked' ? 'nav_blocked' : e.reason === 'lost' ? 'marker_lost' : 'nav_failed',
    `이동하지 못했다. ${e.say ?? ''}`],
  cancelled: () => ['stopped', '이동을 멈췄다.'],
};

export class Assistant {
  readonly live: LiveVoice;
  readonly clips = new Clips();
  ui: VoiceUi = { state: '대기', detail: '', who: '', text: '' };
  /** 아바타 화면이 떠 있으면 표정을 바꾸는 함수를 꽂는다 */
  expression: (name: Expression, strength: number) => boolean = () => false;
  attractPeriod = 0;
  private listeners = new Set<Listener>();
  private state: State | null = null;
  private received = 0;
  private linked = false;
  private sources = new Set<string>();
  private clipSequence = 0;
  private clipTimer = 0;
  private lastTouch = performance.now();
  // 상태 변화 감지용
  private armSeq = -1;
  private tripSeq = -1;
  private passing = false;
  private guardHeld = false;
  private navLostSince = 0;
  private navLostSaid = false;
  private opsSeq = -1;
  private mission = '';
  private lostSince = 0;
  private lostSaid = false;
  private stoppedSince = 0;
  private moved = false;
  private cooldown: Record<string, number> = {};
  private batteryZone = '';
  private asked: Record<string, number> = {};
  private lastResult = { text: '', at: 0 };

  performanceActive = false;
  stopPerformance?: () => void;
  constructor(bridge: Bridge, playPerformance: (kind: 'intro' | 'keonroot-challenge') => Promise<boolean>) {
    this.live = new LiveVoice({
      token: () => bridge.requestLiveToken(),
      localPlayback: () => this.performanceActive,
      system: () => persona(this.state?.arm?.errand.motions ?? [], bridge.hello?.places ?? [], bridge.hello?.nav?.enabled ?? (bridge.hello?.places ?? []).length > 0, !!bridge.hello?.look),
      context: () => this.context(),
      voice: () => settings.value.geminiVoice,
      tts: () => settings.value.chatVoice === 'hina' && bridge.hello?.live?.tts ? (text, chunk) => bridge.tts(text, chunk) : null,
      tools: () => {
        const t = buildTools({ bridge, playIntroduction: () => playPerformance('intro'), playTalentShow: () => playPerformance('keonroot-challenge'), state: () => this.state, fresh: () => this.fresh,
          setExpression: (n, s) => this.expression(n, s),
          places: () => bridge.hello?.places ?? [], nav: () => bridge.hello?.nav, look: () => !!bridge.hello?.look, hasClip: key => settings.value.clips && this.clips.has(key) });
        // 말로 시킨 일은 모델이 곧바로 대답한다. 같은 일의 녹음 대사·알림이 겹치지 않게 시각을 적어 둔다
        for (const name of ['fetch_object', 'give_object', 'throw_object', 'play_motion', 'stop_moving', 'go_to_place', 'fetch_from_place', 'return_home',
          'move_robot', 'go_to_marker', 'follow_marker']) {
          const fn = t.handlers[name];
          if (!fn) continue;
          t.handlers[name] = args => { this.asked[name] = performance.now(); if (name === 'stop_moving') this.sayOnce('stopped', 3, true); return fn(args); };
        }
        return t;
      },
      // 녹음 대사가 나오는 중에 모델 쪽이 '대기'로 돌아가도 화면은 '말하는 중'으로 둔다
      status: (state, detail = '') => this.set({ state: state === '대기' && player.speaking ? '말하는 중' : state, detail }),
      caption: (who, text) => this.set({ who, text }),
      event: (phase, seconds) => bridge.send({ cmd: 'live_event', phase, duration_s: Math.round(seconds) }),
    });
    const apply = () => {
      const v = settings.value;
      this.attractPeriod = v.attract; player.setVolume(v.volume); void player.setSink(v.sink);
    };
    apply();
    settings.subscribe((v, changed) => {
      apply();
      // 목소리나 말하기 방식은 세션을 열 때 정해진다. 바뀌면 새로 연다
      if (changed.some(k => k === 'geminiVoice' || k === 'handsFree')) { this.live.handsFree = v.handsFree; this.reconnect(); }
    });
    this.live.handsFree = settings.value.handsFree;
    this.places = () => bridge.hello?.places ?? [];
    setInterval(() => this.tick(), 500);
  }
  private places: () => import('../types').Place[] = () => [];

  get fresh() { return this.linked && performance.now() - this.received < 3000; }
  frame() { return this.live.frame(); }
  subscribe(fn: Listener) { this.listeners.add(fn); fn(this.ui); return () => this.listeners.delete(fn); }
  private set(patch: Partial<VoiceUi>) { this.ui = { ...this.ui, ...patch }; for (const fn of this.listeners) fn(this.ui); }

  /** 모델이 말을 듣기 전에 알아야 할 것: 작업대에 보이는 물건과 들고 있는 물건. 잘못 들은 이름을 바로잡는 데 쓴다. */
  private context(): string {
    const s = this.fresh ? this.state : null;
    if (!s) return '';
    const e = s.arm?.errand, parts: string[] = [];
    if (e) {
      const near = e.inventory.filter(i => i.reachable).map(i => i.label), far = e.inventory.filter(i => !i.reachable).map(i => i.label);
      if (e.inventory_age_s != null) parts.push(`마지막으로 작업대를 살폈을 때 보인 물건: ${near.join(', ') || '없음'}.`);
      if (far.length) parts.push(`보이지만 팔이 닿지 않는 물건: ${far.join(', ')}.`);
      if (e.state === 'holding') parts.push(`지금 ${e.object}을(를) 들고 있다.`);
    }
    // 이동: 지금 하는 일, 카메라에 보이는 장소, 떠나 있는지
    const t = s.tasks?.task;
    if (t?.state === 'running') parts.push(`지금 "${t.title}"을(를) 하는 중이다${t.detail ? `(${t.detail})` : ''}.`);
    if (s.nav) {
      const seen = visiblePlaces(s, this.places());
      if (seen.length) parts.push(`지금 카메라에 보이는 장소: ${seen.join(', ')}.`);
      if (s.nav.home && Math.hypot(s.nav.home[0], s.nav.home[1]) > 0.3 && t?.state !== 'running') parts.push('출발한 자리를 떠나 있다.');
    }
    if (this.lastResult.text && performance.now() - this.lastResult.at < 90_000) parts.push(`방금 있었던 일: ${this.lastResult.text}.`);
    return parts.length ? `${parts.join(' ')} 이 알림에는 대답하지 않는다.` : '';
  }

  // ------------------------------------------------------------ 누르고 말하기
  /** 여러 입력(키보드, 화면 버튼, 패드)이 겹쳐도 하나라도 눌려 있으면 듣는다. */
  input(source: string, down: boolean) {
    const was = this.sources.size > 0;
    if (down) this.sources.add(source); else this.sources.delete(source);
    const held = this.sources.size > 0;
    if (held && !was) { if (this.performanceActive) this.stopPerformance?.(); this.touch(); clearTimeout(this.clipTimer); void this.live.press(); }
    else if (!held && was) this.live.release();
  }
  clearInput() { this.sources.clear(); this.live.release(); }
  get held() { return this.sources.size > 0; }
  touch() { this.lastTouch = performance.now(); }

  // ------------------------------------------------------------ 고정 대사
  /** 녹음 대사를 튼다. 누르고 말하는 중이거나 막내가 말하는 중이면 틀지 않는다(force면 끊고 튼다). */
  say(key: string, force = false): boolean {
    if (this.held || !settings.value.clips) return false;
    if (player.speaking && !force) return false;
    const clip = this.clips.pick(key);
    if (!clip) return false;
    if (force) player.flush();
    this.playRecording(key, clip);
    return true;
  }

  preparePerformance() {
    this.interruptForRecording();
    ++this.clipSequence;
    clearTimeout(this.clipTimer);
    this.performanceActive = true;
    this.set({ state: '대기', who: '', text: '', detail: '' });
  }

  interruptForRecording() {
    if (this.ui.state === '생각 중' || (this.ui.state === '말하는 중' && !player.clipPlaying)) this.live.interruptReply();
    player.stopClip();
  }

  playRecording(key: string, clip: Clip) {
    const sequence = ++this.clipSequence;
    clearTimeout(this.clipTimer);
    this.set({ state: '말하는 중', detail: '', who: 'maknae', text: clip.text });
    void player.play(this.clips.url(clip), key, clip.text).catch(() => false).then(ok => {
      if (sequence !== this.clipSequence || this.ui.text !== clip.text) return;
      if (!ok) {
        this.set({ state: '재생 불가', who: '', text: '', detail: '녹음 파일을 재생할 수 없습니다.' });
        return;
      }
      this.set({ state: '대기' });
      clearTimeout(this.clipTimer);
      this.clipTimer = window.setTimeout(() => { if (this.ui.text === clip.text) this.set({ who: '', text: '' }); }, 2500);
    });
  }
  private sayOnce(key: string, every_s: number, force = false) {
    const now = performance.now();
    if (now < (this.cooldown[key] ?? 0)) return;
    if (this.say(key, force)) this.cooldown[key] = now + every_s * 1000;
  }

  // ------------------------------------------------------------ 로봇 상태 → 말
  onLink(up: boolean) { this.linked = up; if (!up) this.clearInput(); }

  onState(s: State) {
    const first = this.state === null;
    this.state = s; this.received = performance.now();
    const now = performance.now();

    // 심부름 사건
    const events = s.arm?.errand.events ?? [];
    const top = events.length ? events[events.length - 1].seq : (s.arm ? 0 : this.armSeq);
    if (this.armSeq >= 0 && s.arm) {
      for (const e of events) {
        if (e.seq <= this.armSeq) continue;
        if (e.type === 'shown') {
          this.touch();
          const text = [`${e.text} 보여요.`, e.reason].filter(Boolean).join(' ');
          this.lastResult = { text, at: now };
          this.set({ who: 'maknae', text });
          this.live.notify(`보여 준 물건을 확인했다. "${text}"라고만 말해라.`);
          continue;
        }
        const map = ERRAND[e.type]?.(e);
        if (!map) continue;
        this.touch();
        const [key, note] = map;
        // 로봇이 한 일은 언제나 녹음 대사(Hina)로 말한다. 모델에게는 다음에 말을 걸 때 context()로 알려 준다
        // (그때그때 알리면 모델이 대답하려 들어 목소리가 겹친다).
        if (note) this.lastResult = { text: note.split('.')[0], at: now };
        if (e.type === 'stopped') { this.sayOnce('stopped', 3, true); continue; }
        // 다녀오는 심부름 중에 집은 것은 멀리 있는 책상 앞이다. "가져왔어요"는 돌아와서 말한다
        if (e.type === 'holding' && s.tasks?.task?.state === 'running' && s.tasks.task.kind === 'fetch_from') continue;
        // 로봇 대사가 모델의 말보다 먼저다. '집어 올게요'만은 앞선 '찾아볼게요'를 끊지 않게 양보한다
        this.say(key, e.type !== 'picking');
      }
    }
    if (s.arm) this.armSeq = Math.max(top, 0);

    // 이동·다녀오는 심부름
    const trips = s.tasks?.events ?? [];
    if (this.tripSeq >= 0) {
      for (const e of trips) {
        if (e.seq <= this.tripSeq) continue;
        const map = TRIP[e.type]?.(e);
        if (!map) continue;
        this.touch();
        const [key, note] = map;
        this.lastResult = { text: note.split('.')[0], at: now };
        if (e.type === 'cancelled') { this.sayOnce('stopped', 3, true); continue; }
        // 말로 시킨 직후의 출발 알림은 모델이 이미 했다(녹음 대사가 없을 때 도구 결과의 note로)
        const justAsked = ['go_to_place', 'fetch_from_place', 'return_home', 'go_to_marker', 'follow_marker'].some(n => now - (this.asked[n] ?? -1e9) < 4000);
        if (settings.value.clips && this.clips.has(key)) {
          this.say(key, true);
          continue;
        }
        if ((e.type === 'go_start' || e.type === 'return_start' || e.type === 'follow_start') && justAsked) continue;
        this.live.notify(`${note} 이 내용을 말해 줘.`);
      }
    }
    this.tripSeq = trips.length ? trips[trips.length - 1].seq : 0;
    const passing = !!s.nav?.passing && s.tasks?.task?.state === 'running';
    if (passing && !this.passing) this.sayOnce('passing', 10, true);
    this.passing = passing;
    // 따라가는 중에 누가 바짝 다가와 로봇이 섰다. 떨어져 달라고 한다
    const following = s.app?.age_s != null && s.app.age_s < 3 && s.app.mission === 'active';
    const tailing = s.tasks?.task?.state === 'running' && s.tasks.task.kind === 'follow_marker';
    const held = !!s.nav?.guard?.hold && (following || tailing);
    if (held && !this.guardHeld) {
      this.touch();
      const key = this.clips.has('guard_close') ? 'guard_close' : 'wait_close';
      const until = this.cooldown[key] ?? 0;
      this.sayOnce(key, 12, true);
      // 녹음이 없으면 모델이 대신 말한다
      if (now >= until && !this.clips.has(key)) this.live.notify('누군가 너무 가까이 다가와서 멈췄다. 조금만 떨어져 달라고 말해 줘.');
      this.lastResult = { text: '누군가 너무 가까이 다가와서 잠깐 멈췄다', at: now };
    }
    this.guardHeld = held;
    // 번호 마커를 따라가다 놓쳐 서 있다
    if (tailing && s.nav?.state === 'following' && s.nav.detail === 'lost') {
      if (!this.navLostSince) this.navLostSince = now;
      if (!this.navLostSaid && now - this.navLostSince > 2500) this.navLostSaid = this.say('marker_lost');
    } else {
      if (this.navLostSaid && tailing) this.sayOnce('marker_found', 5);
      this.navLostSince = 0; this.navLostSaid = false;
    }

    // 운영 알림(SOS)
    const ops = s.ops;
    if (ops && this.opsSeq >= 0 && ops.seq !== this.opsSeq && ops.text === 'SOS 보냄') this.say('sos_sent', true);
    this.opsSeq = ops?.seq ?? 0;

    // 주행
    const mission = s.app?.age_s != null && s.app.age_s < 3 ? s.app.mission : '';
    const active = mission === 'active';
    if (!first && mission !== this.mission) {
      if (active) { this.say('follow_start', true); this.moved = false; this.lostSaid = false; this.lostSince = 0; }
      else if (this.mission === 'active' || this.mission === 'cancelling') { if (mission === 'idle' || mission === '') this.sayOnce('follow_stop', 3); }
    }
    this.mission = mission;
    const v = Math.abs(s.odom?.v ?? 0);
    if (active) {
      this.touch();
      const lost = !s.marker || s.marker.status === 'lost';
      if (lost) {
        if (!this.lostSince) this.lostSince = now;
        if (!this.lostSaid && now - this.lostSince > 2500) { this.lostSaid = this.say('marker_lost'); }
      } else {
        if (this.lostSaid) this.sayOnce('marker_found', 5);
        this.lostSince = 0; this.lostSaid = false;
      }
      if (v > 0.15) {
        this.stoppedSince = 0; this.moved = true;
      } else {
        if (v < 0.03 && !lost && this.moved && (s.marker?.range ?? 9) < 1.0) {
          if (!this.stoppedSince) this.stoppedSince = now;
          if (now - this.stoppedSince > 1500) { this.sayOnce('wait_close', 30); this.moved = false; }
        } else this.stoppedSince = 0;
      }
    } else { this.stoppedSince = 0; }

    // 배터리
    const zone = s.battery?.zone ?? '';
    if (!first && zone !== this.batteryZone && (zone === 'charge' || zone === 'stop')) this.sayOnce('battery_low', 600);
    this.batteryZone = zone;
  }

  /** 아무도 말을 걸지 않으면 가끔 호객한다(attractPeriod가 0이면 끈다). */
  private tick() {
    if (!this.attractPeriod || document.hidden) return;
    const busy = this.performanceActive || this.held || player.speaking || this.ui.state === '말하는 중'
      || this.ui.state === '생각 중' || (this.state?.arm && this.state.arm.errand.state !== 'idle')
      || this.state?.tasks?.task?.state === 'running';
    if (busy) { this.touch(); return; }
    if (performance.now() - this.lastTouch > this.attractPeriod * 1000) { this.touch(); this.say(Math.random() < 0.3 ? 'greet' : 'attract'); }
  }

  stop() { this.clearInput(); this.live.stop(); player.stopClip(); }
  /** 대화 세션을 새로 연다(설정을 바꿨거나 대화가 꼬였을 때) */
  reconnect() { this.clearInput(); this.live.stop(); this.live.warm(); }
}
