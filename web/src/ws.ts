import type { LiveToken } from './avatar/live';
import type { ScanHeader } from './lidar';
import type { FrameHeader, Hello, State } from './types';

export interface Reply { ok: boolean; error?: string; [k: string]: unknown }

export interface Handlers {
  dpad(direction: 'left' | 'up' | 'right' | 'down'): void;
  hello(h: Hello): void;
  state(s: State): void;
  frame(h: FrameHeader, jpeg: Blob): void;
  scan?(h: ScanHeader, mm: Int16Array): void;
  link(up: boolean): void;
}

/** 브리지 WebSocket. 끊기면 1 s 간격으로 다시 붙는다(부스에서 브리지를 재시작해도 화면은 그대로 둔다). */
export class Bridge {
  private ws: WebSocket | null = null;
  private tokenRequest?: { resolve: (t: LiveToken) => void; reject: (e: Error) => void; timer: number };
  private replies = new Map<number, (r: Reply) => void>();
  private nextId = 1;
  hello: Hello | null = null;
  private speech = new Map<number, { chunk: (pcm: Uint8Array, rate: number) => void; done: (ok: boolean) => void }>();
  /** 문장 하나를 Hina 목소리(브리지 → ElevenLabs)로 읽힌다. PCM 조각이 오는 대로 chunk가 불리고, 끝나면 성공 여부로 resolve. */
  tts(text: string, chunk: (pcm: Uint8Array, rate: number) => void): Promise<boolean> {
    if (this.ws?.readyState !== WebSocket.OPEN) return Promise.resolve(false);
    const id = this.nextId++;
    return new Promise(resolve => {
      const timer = window.setTimeout(() => { this.speech.delete(id); resolve(false); }, 15000);
      this.speech.set(id, { chunk, done: ok => { clearTimeout(timer); this.speech.delete(id); resolve(ok); } });
      this.send({ cmd: 'tts', id, text });
    });
  }
  /** 팔 데몬 명령. 답(수락·거절)을 기다린다. 끊기거나 5초 넘으면 거절로 돌려준다. */
  arm(msg: Record<string, unknown>): Promise<Reply> { return this.ask('arm', msg); }
  /** 장소로 가기·가져오기·돌아오기(브리지 tasks.py). 수락·거절만 바로 오고, 진행은 state.tasks로 온다. */
  task(msg: Record<string, unknown>): Promise<Reply> { return this.ask('task', msg); }
  /** 부스 배치(시작 위치·장소 자리) 바꾸기 */
  layout(msg: Record<string, unknown>): Promise<Reply> { return this.ask('layout', msg); }
  /** 앞 보기: 전방 카메라 한 장을 Gemini에게 묻고 글 답(answer)을 받는다. 모델 호출이 있어 넉넉히 기다린다. */
  look(question: string): Promise<Reply & { answer?: string; seconds?: number }> { return this.ask('look', { question }, 15000); }
  private ask(cmd: string, msg: Record<string, unknown>, timeout_ms = 6000): Promise<Reply> {
    if (this.ws?.readyState !== WebSocket.OPEN) return Promise.resolve({ ok: false, error: '브리지 연결이 끊겼어요.' });
    const id = this.nextId++;
    return new Promise(resolve => {
      const timer = window.setTimeout(() => { this.replies.delete(id); resolve({ ok: false, error: '응답이 없어요.' }); }, timeout_ms);
      this.replies.set(id, r => { clearTimeout(timer); this.replies.delete(id); resolve(r); });
      this.send({ cmd, id, msg });
    });
  }
  requestLiveToken(): Promise<LiveToken> {
    if (this.ws?.readyState !== WebSocket.OPEN) return Promise.reject(new Error('bridge_offline'));
    if (this.tokenRequest) return Promise.reject(new Error('busy'));
    return new Promise((resolve, reject) => {
      const timer = window.setTimeout(() => {
        this.tokenRequest = undefined; reject(new Error('timeout'));
      }, 15000);
      this.tokenRequest = { resolve, reject, timer };
      this.send({ cmd: 'live_token' });
    });
  }
  constructor(private h: Handlers) { this.open(); }

  private open() {
    const ws = new WebSocket(`ws://${location.host}/ws`);
    ws.binaryType = 'arraybuffer';
    ws.onopen = () => this.h.link(true);
    ws.onclose = () => { this.h.link(false); this.ws = null;
      if (this.tokenRequest) { clearTimeout(this.tokenRequest.timer); this.tokenRequest.reject(new Error('bridge_offline')); this.tokenRequest = undefined; } setTimeout(() => this.open(), 1000); };
    ws.onerror = () => ws.close();
    ws.onmessage = (ev) => {
      if (typeof ev.data === 'string') {
        const msg = JSON.parse(ev.data);
        if (msg.type === 'state') this.h.state(msg as State);
        else if (msg.type === 'dpad') this.h.dpad(msg.direction);
        else if (msg.type === 'hello') { this.hello = msg as Hello; this.h.hello(msg as Hello); }
        else if (msg.type === 'reply') this.replies.get(msg.id)?.(msg as Reply);
        else if (msg.type === 'live_token' && this.tokenRequest) {
          const pending = this.tokenRequest; this.tokenRequest = undefined; clearTimeout(pending.timer);
          if (msg.ok && typeof msg.token === 'string') pending.resolve({ token: msg.token, model: msg.model, voice: msg.voice });
          else pending.reject(new Error(msg.error || 'invalid_response'));
        }
        return;
      }
      const buf = ev.data as ArrayBuffer;
      const n = new DataView(buf).getUint32(0, true);
      const any = JSON.parse(new TextDecoder().decode(new Uint8Array(buf, 4, n)));
      if (any.type === 'tts') {
        const h = this.speech.get(any.id);
        if (h) { if (buf.byteLength > 4 + n) h.chunk(new Uint8Array(buf, 4 + n), any.rate ?? 24000); if (any.last) h.done(!!any.ok); }
        return;
      }
      if (any.type === 'scan') {
        // int16는 짝수 위치에서만 볼 수 있어 본문을 잘라 낸다
        this.h.scan?.(any as ScanHeader, new Int16Array(buf.slice(4 + n, 4 + n + ((buf.byteLength - 4 - n) & ~1))));
        return;
      }
      const header = any as FrameHeader;
      this.h.frame(header, new Blob([new Uint8Array(buf, 4 + n)], { type: header.mime }));
    };
    this.ws = ws;
  }

  send(obj: unknown) {
    if (this.ws?.readyState === WebSocket.OPEN) this.ws.send(JSON.stringify(obj));
  }
}
