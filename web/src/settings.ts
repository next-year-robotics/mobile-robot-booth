// 부스에서 바로 바꿔 볼 설정. 컨트롤 패널(panel.ts, 키 C)이 고치고 이 브라우저(localStorage)에 남는다.
// 값을 안 정한 항목은 config/booth.yaml의 기본값(hello.live)을 따른다.
import type { Hello } from './types';

export interface Settings {
  chatVoice: 'gemini' | 'hina';     // 대화 대답 목소리
  geminiVoice: string;              // chatVoice가 gemini일 때 쓰는 Gemini 목소리 이름
  handsFree: boolean;               // true면 버튼 없이 말소리를 자동으로 잡는다(조용한 곳에서만)
  attract: number;                  // 호객 간격 [s]. 0이면 끈다
  clips: boolean;                   // 로봇 동작 대사(녹음)를 낼지
  volume: number;                   // 0~1
  sink: string;                     // 스피커 장치 ID. 빈 문자열이면 시스템 기본
}

const KEY = 'maknae.settings';
const DEFAULTS: Settings = { chatVoice: 'gemini', geminiVoice: 'Leda', handsFree: false, attract: 0, clips: true, volume: 1, sink: '' };
type Listener = (s: Settings, changed: (keyof Settings)[]) => void;

class Store {
  private saved: Partial<Settings> = {};
  private base: Settings = { ...DEFAULTS };
  private listeners = new Set<Listener>();
  constructor() {
    try { this.saved = JSON.parse(localStorage.getItem(KEY) ?? '{}') ?? {}; } catch { this.saved = {}; }
  }
  get value(): Settings { return { ...this.base, ...this.saved }; }
  /** 브리지가 알려 준 기본값. 패널에서 따로 정한 항목은 그대로 둔다 */
  defaults(live: Hello['live']) {
    if (!live) return;
    const before = this.value;
    this.base = { ...DEFAULTS, attract: live.attract_s ?? 0, geminiVoice: live.voice ?? 'Leda', chatVoice: live.chat_voice === 'hina' ? 'hina' : 'gemini' };
    this.emit(before);
  }
  set(patch: Partial<Settings>) {
    const before = this.value;
    this.saved = { ...this.saved, ...patch };
    try { localStorage.setItem(KEY, JSON.stringify(this.saved)); } catch { /* 저장 못 해도 이번에는 적용된다 */ }
    this.emit(before);
  }
  reset() { const before = this.value; this.saved = {}; try { localStorage.removeItem(KEY); } catch { /* 무시 */ } this.emit(before); }
  subscribe(fn: Listener) { this.listeners.add(fn); return () => this.listeners.delete(fn); }
  private emit(before: Settings) {
    const now = this.value;
    const changed = (Object.keys(now) as (keyof Settings)[]).filter(k => now[k] !== before[k]);
    if (changed.length) for (const fn of this.listeners) fn(now, changed);
  }
}

export const settings = new Store();
