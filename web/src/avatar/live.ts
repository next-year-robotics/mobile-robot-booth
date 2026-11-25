// Gemini Live 음성 대화. 브라우저가 WebSocket으로 직접 붙고, 브리지는 한 번 쓰는 토큰만 준다.
//
// - 부스는 시끄러워서 기본은 누르고 말하기다(자동 음성 감지를 끄고 activityStart/End를 직접 보낸다).
//   열린 마이크 모드(handsFree)는 조용한 곳에서 쓴다. 모드는 세션을 열 때 정해진다.
// - 세션은 미리 열어 둔다(warm). 누른 뒤에 연결하면 첫마디가 1초 넘게 늦는다.
//   조용히 시간이 지나면 세션을 새로 열어 앞 관람객과의 대화 문맥을 비운다.
// - 관람객 음성과 글자는 어디에도 저장하지 않는다. 자막은 화면에만 잠깐 보인다.
import { Mic } from '../voice/mic';
import { player } from '../voice/player';

export interface LiveToken { token: string; model: string; voice: string }
export interface ToolDecl { name: string; description: string; parameters?: Record<string, unknown>; behavior?: 'NON_BLOCKING' }
export type ToolHandler = (args: Record<string, unknown>) => unknown | Promise<unknown>;
export interface LiveOptions {
  token: () => Promise<LiveToken>;
  system: () => string;
  localPlayback?: () => boolean;
  /** Gemini 목소리 이름을 바꿔 쓸 때 (없으면 토큰에 딸려 온 이름) */
  voice?: () => string;
  /** 함수를 돌려주면 모델의 소리는 버리고, 모델이 한 말(받아쓰기)을 문장마다 이 함수로 읽힌다(Hina 목소리). null이면 모델 소리를 그대로 낸다 */
  tts?: () => ((text: string, chunk: (pcm: Uint8Array, rate: number) => void) => Promise<boolean>) | null;
  /** 말을 듣기 직전에 모델에게 알려 둘 지금 상황. 빈 문자열이면 보내지 않는다 */
  context?: () => string;
  tools: () => { decls: ToolDecl[]; handlers: Record<string, ToolHandler> };
  status: (state: string, detail?: string) => void;
  caption: (who: 'user' | 'maknae' | '', text: string) => void;
  event?: (phase: 'start' | 'end', seconds: number) => void;
  socket?: (url: string) => WebSocket;            // 시험용
  mic?: Mic;
}

const HOST = 'wss://generativelanguage.googleapis.com/ws/google.ai.generativelanguage.v1beta.GenerativeService.BidiGenerateContentConstrained';
const IDLE_MS = 90_000;                           // 이만큼 조용하면 문맥을 비운다
const ERRORS: Record<string, string> = {
  no_key: 'Gemini API 키 파일이 없습니다. 자기소개와 고정 대사는 그대로 나옵니다.',
  disabled: '실시간 대화가 꺼져 있습니다.', bridge_offline: '브리지 연결을 확인해 주세요.',
  auth_error: 'Gemini API 키를 확인해 주세요.', network_error: '인터넷 연결을 확인해 주세요.',
  timeout: '연결 시간이 초과되었습니다.', rate_limited: '잠시 뒤 다시 눌러 주세요.',
};

const b64 = (buf: ArrayBuffer) => {
  const u = new Uint8Array(buf); let s = '';
  for (let i = 0; i < u.length; i += 0x8000) s += String.fromCharCode(...u.subarray(i, i + 0x8000));
  return btoa(s);
};
const unb64 = (s: string) => Uint8Array.from(atob(s), c => c.charCodeAt(0));

export class LiveVoice {
  handsFree = false;
  private ws?: WebSocket;
  private ready = false;
  private connecting = false;
  private generation = 0;
  private held = false;
  private talking = false;                        // activityStart를 보냈고 아직 End를 안 보냄
  private backlog: ArrayBuffer[] = [];            // 연결 중에 들어온 마이크 조각
  private mic: Mic;
  private idle = 0;
  private limit = 0;
  private started = 0;
  private retryAt = 0;
  private failures = 0;
  private keepWarm = false;
  private said = '';
  private heard = '';
  private turnOpen = false;
  private drain = 0;
  private handlers: Record<string, ToolHandler> = {};
  private sentContext = '';
  private speaker: ReturnType<NonNullable<LiveOptions['tts']>> = null;
  private pendingText = '';
  private speechGen = 0;
  private speaking = 0;                           // Hina로 읽히는 중인 문장 수
  private chain: Promise<void> = Promise.resolve();
  private skipReply = false;
  private mute = false;        // 무음 도구 응답 뒤에 모델이 내는 소리는 버린다(녹음 대사가 대신 말한다)
  /** 사람이 말을 걸 때마다 하나씩 는다. 같은 말에서 나온 도구 호출을 한 묶음으로 잇는 데 쓴다(브리지 tasks.py) */
  turn = 0;
  private turnDone = true;

  constructor(private o: LiveOptions) {
    this.mic = o.mic ?? new Mic();
    this.mic.onChunk = pcm => this.onMic(pcm);
  }

  get connected() { return this.ready; }
  frame() { return this.held ? { level: 0, vowel: 'aa' as const } : player.frame(); }
  /** 마이크 입력 크기 0~1 (컨트롤 패널 표시용) */
  get micLevel() { return this.mic.open ? this.mic.level : 0; }

  /** 세션을 미리 열어 둔다. 마이크는 처음 누를 때 연다. */
  warm() { this.keepWarm = true; if (!this.ws && !this.connecting && Date.now() >= this.retryAt) void this.connect(); }

  async press() {
    if (this.held) return;
    this.held = true; clearTimeout(this.idle);
    this.limit = window.setTimeout(() => this.release(), 30_000);
    player.flush(); player.stopClip();              // 말하는 중이면 끊고 듣는다
    this.cutSpeech();
    this.said = ''; this.heard = ''; this.o.caption('', '');
    try { await this.mic.start(); } catch (e) {
      this.held = false; clearTimeout(this.limit);
      this.o.status('대화 불가', e instanceof DOMException && e.name === 'NotAllowedError'
        ? '브라우저 마이크 권한을 허용해 주세요.' : '마이크를 찾지 못했습니다. 장치를 확인해 주세요.');
      return;
    }
    if (!this.held) return;
    if (this.ready) { this.beginTalk(); return; }
    if (Date.now() < this.retryAt) { this.release(); this.o.status('대화 불가', '잠시 뒤 다시 눌러 주세요.'); return; }
    this.o.status('연결 중', '그대로 말씀하셔도 됩니다.');
    if (!this.connecting) void this.connect();
  }

  release() {
    if (!this.held) return;
    this.held = false; clearTimeout(this.limit);
    if (this.talking) {
      this.talking = false;
      if (!this.handsFree) this.send({ realtimeInput: { activityEnd: {} } });
      this.o.status('생각 중'); this.armIdle();
    } else if (!this.ready && !this.connecting) this.o.status('대기');
  }

  private beginTalk() {
    this.talking = true;
    this.turn++; this.turnDone = false;
    this.skipReply = false;
    this.mute = false;
    // 말을 듣기 직전에 지금 상황(작업대 위 물건 등)을 조용히 알려 둔다. 바뀌었을 때만 보낸다.
    const ctx = this.o.context?.() ?? '';
    if (ctx && ctx !== this.sentContext) {
      this.sentContext = ctx;
      this.send({ clientContent: { turns: [{ role: 'user', parts: [{ text: `[로봇 알림] ${ctx}` }] }], turnComplete: false } });
    }
    if (!this.handsFree) this.send({ realtimeInput: { activityStart: {} } });
    for (const pcm of this.backlog.splice(0)) this.sendAudio(pcm);
    this.o.status('듣는 중');
  }

  private onMic(pcm: ArrayBuffer) {
    if (this.o.localPlayback?.()) return;
    if (this.handsFree && this.ready) { this.sendAudio(pcm); return; }
    if (!this.held) return;
    if (this.talking) this.sendAudio(pcm);
    else if (this.backlog.length < 400) this.backlog.push(pcm);   // 연결을 기다리는 동안 16 s까지
  }
  private sendAudio(pcm: ArrayBuffer) {
    this.send({ realtimeInput: { audio: { mimeType: 'audio/pcm;rate=16000', data: b64(pcm) } } });
  }
  private send(obj: unknown) { if (this.ws?.readyState === WebSocket.OPEN) this.ws.send(JSON.stringify(obj)); }

  /** 로봇에서 생긴 일을 모델에게 알려 한마디 하게 한다. 세션이 없으면 false(호출한 쪽이 녹음 대사를 튼다). */
  notify(text: string, speak = true): boolean {
    if (!this.ready || this.held) return false;
    this.send({ clientContent: { turns: [{ role: 'user', parts: [{ text: `[로봇 알림] ${text}` }] }], turnComplete: speak } });
    if (speak) { clearTimeout(this.idle); this.said = ''; }
    return true;
  }

  private async connect() {
    this.connecting = true;
    const generation = ++this.generation;
    try {
      const t = await this.o.token();
      if (generation !== this.generation) return;
      const { decls, handlers } = this.o.tools();
      this.handlers = handlers;
      const ws = (this.o.socket ?? (u => new WebSocket(u)))(`${HOST}?access_token=${encodeURIComponent(t.token)}`);
      ws.binaryType = 'arraybuffer';
      this.ws = ws;
      const timer = window.setTimeout(() => { if (!this.ready && generation === this.generation) this.fail('timeout'); }, 12_000);
      ws.onopen = () => ws.send(JSON.stringify({ setup: {
        model: `models/${t.model}`,
        generationConfig: { responseModalities: ['AUDIO'], speechConfig: { voiceConfig: { prebuiltVoiceConfig: { voiceName: this.o.voice?.() || t.voice } } } },
        systemInstruction: { parts: [{ text: this.o.system() }] },
        tools: [{ functionDeclarations: decls }],
        realtimeInputConfig: this.handsFree ? {} : { automaticActivityDetection: { disabled: true } },
        inputAudioTranscription: {}, outputAudioTranscription: {},
      } }));
      ws.onmessage = ev => {
        if (generation !== this.generation) return;
        let msg: Record<string, any>;
        try { msg = JSON.parse(typeof ev.data === 'string' ? ev.data : new TextDecoder().decode(ev.data as ArrayBuffer)); } catch { return; }
        if (msg.setupComplete) {
          clearTimeout(timer);
          this.ready = true; this.connecting = false; this.failures = 0;
          this.started = performance.now(); this.o.event?.('start', 0);
          if (this.held) this.beginTalk(); else { this.o.status('대기'); this.armIdle(); }
          if (this.handsFree) void this.mic.start().catch(() => this.o.status('대화 불가', '마이크를 열지 못했습니다.'));
        }
        if (msg.serverContent) this.onContent(msg.serverContent);
        if (msg.toolCall) void this.onTools(msg.toolCall.functionCalls ?? [], generation);
        if (msg.goAway) this.armIdle(3000);          // 서버가 곧 끊는다. 조용해지면 새로 연다
      };
      ws.onerror = () => { /* onclose가 이어서 온다 */ };
      ws.onclose = () => {
        clearTimeout(timer);
        if (generation !== this.generation) return;
        const was = this.ready;
        this.teardown();
        if (!was) { this.fail('network_error'); return; }
        this.o.status('대기'); this.o.caption('', '');
        if (this.keepWarm) window.setTimeout(() => this.warm(), 800);
      };
    } catch (e) {
      if (generation === this.generation) this.fail(e instanceof Error ? e.message : '');
    }
  }

  private fail(key: string) {
    this.teardown();
    this.held = false; clearTimeout(this.limit);
    this.retryAt = Date.now() + Math.min(30_000, 1000 * 2 ** this.failures++);
    this.o.status('대화 불가', ERRORS[key] ?? '인터넷과 Gemini 설정을 확인해 주세요.');
    if (this.keepWarm) window.setTimeout(() => this.warm(), this.retryAt - Date.now() + 50);
  }

  private teardown() {
    const ws = this.ws; this.ws = undefined;
    if (ws) { ws.onclose = ws.onmessage = ws.onerror = null; try { ws.close(); } catch { /* 이미 닫힘 */ } }
    if (this.started) { this.o.event?.('end', (performance.now() - this.started) / 1000); this.started = 0; }
    this.ready = false; this.connecting = false; this.talking = false; this.backlog = []; this.turnOpen = false;
    this.skipReply = false;
    this.sentContext = '';
    clearTimeout(this.idle); clearInterval(this.drain);
  }

  private onContent(c: Record<string, any>) {
    // 열린 마이크에서는 누르는 순간이 없다. 모델의 대답이 끝난 뒤 처음 들린 말부터 새 말로 센다
    if (c.inputTranscription?.text && this.handsFree && this.turnDone) { this.turn++; this.turnDone = false; }
    if (c.inputTranscription?.text) { this.heard += c.inputTranscription.text; this.o.caption('user', this.heard.trim()); }
    if (c.interrupted) { player.flush(); this.cutSpeech(); this.said = ''; }
    for (const part of c.modelTurn?.parts ?? []) {
      if (part.inlineData?.data && !this.held && !this.mute && !this.skipReply && !this.o.localPlayback?.()) {
        if (!this.turnOpen) { this.turnOpen = true; this.said = ''; this.heard = ''; this.o.status('말하는 중'); clearTimeout(this.idle); this.speaker = this.o.tts?.() ?? null; this.pendingText = ''; }
        if (!this.speaker) player.pcm(unb64(part.inlineData.data));
      }
    }
    if (c.outputTranscription?.text && !this.held && !this.mute && !this.skipReply && !this.o.localPlayback?.()) {
      this.said += c.outputTranscription.text; this.o.caption('maknae', this.said.trim());
      if (this.speaker) this.feed(c.outputTranscription.text);
    }
    if (c.turnComplete) {
      this.turnDone = true;
      this.skipReply = false;
      if (this.speaker && this.turnOpen) this.feed('', true);
      this.turnOpen = false;
      // 소리가 다 나온 뒤에 대기로 돌린다
      clearInterval(this.drain);
      this.drain = window.setInterval(() => {
        if (player.pending > 0.05 || this.held || this.speaking > 0) return;
        clearInterval(this.drain);
        if (!this.held && this.ready) { this.o.status(this.handsFree ? '듣는 중' : '대기'); this.armIdle(); }
      }, 120);
    }
  }

  /** Hina 모드: 모델이 한 말을 문장 단위로 잘라 차례로 읽힌다. */
  private feed(text: string, end = false) {
    this.pendingText += text;
    const out: string[] = [];
    for (;;) {
      const m = /^(.*?[.!?。…~]+["')\]]?)(\s+|$)/s.exec(this.pendingText);
      if (!m || (!m[2] && !end)) break;             // 문장이 끝났는지는 뒤에 공백이 와야 안다
      out.push(m[1]); this.pendingText = this.pendingText.slice(m[0].length);
    }
    if (end && this.pendingText.trim()) { out.push(this.pendingText); this.pendingText = ''; }
    const speak = this.speaker, gen = this.speechGen;
    for (const raw of out) {
      const sentence = raw.replace(/<[^>]*>|\{[^}]*\}/g, '').trim();     // <no speech> 같은 표시는 읽지 않는다
      if (!sentence || !speak) continue;
      this.speaking++;
      this.chain = this.chain.then(async () => {
        try {
          if (gen !== this.speechGen) return;
          const ok = await speak(sentence, (pcm, rate) => { if (gen === this.speechGen) player.pcm(pcm, rate); });
          if (!ok && gen === this.speechGen) this.o.status('말하는 중', 'Hina 목소리를 받지 못했습니다. 컨트롤 패널에서 Gemini 목소리로 바꿔 보세요.');
        } finally { this.speaking--; }
      });
    }
  }
  interruptReply() {
    this.skipReply = true;
    player.flush();
    this.cutSpeech();
  }

  private cutSpeech() { this.speechGen++; this.pendingText = ''; }

  private async onTools(calls: { id: string; name: string; args?: Record<string, unknown> }[], generation: number) {
    const functionResponses = [];
    let quiet = false, refused = false;
    for (const call of calls) {
      let response: unknown;
      try {
        const fn = this.handlers[call.name];
        response = fn ? await fn(call.args ?? {}) : { ok: false, error: '없는 기능' };
      } catch { response = { ok: false, error: '실행 중 오류' }; }
      // 도구가 silent를 붙여 돌려주면 모델은 그 결과를 말하지 않는다(녹음 대사가 대신 나온다)
      const body: Record<string, unknown> = (response && typeof response === 'object') ? { ...(response as object) } : { result: response };
      const silent = body.silent === true;
      if (silent) { delete body.silent; quiet = true; }
      if (body.ok === false && call.name !== 'set_expression') refused = true;
      functionResponses.push({ id: call.id, name: call.name, response: body, ...(silent ? { scheduling: 'SILENT' } : {}) });
    }
    if (quiet && !refused) {
      if (!this.handsFree) this.mute = true;
      this.interruptReply();
    }
    if (generation === this.generation) this.send({ toolResponse: { functionResponses } });
  }

  private armIdle(ms = IDLE_MS) {
    clearTimeout(this.idle);
    this.idle = window.setTimeout(() => {
      if (this.held || player.speaking) { this.armIdle(5000); return; }
      // 문맥을 비운다. 계속 쓸 거면 새 세션을 곧바로 열어 둔다
      const warm = this.keepWarm; this.reset(); this.o.caption('', '');
      if (warm) this.warm();
    }, ms);
  }

  private reset() { ++this.generation; this.teardown(); if (!this.handsFree) this.mic.stop(); }

  /** 대화를 끝낸다. warm도 푼다. */
  stop() {
    this.keepWarm = false; this.held = false; clearTimeout(this.limit);
    ++this.generation; this.teardown(); this.mic.stop(); player.flush(); this.cutSpeech();
    this.o.caption('', ''); this.o.status('대기');
  }
}
