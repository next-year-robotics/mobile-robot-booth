// 소리 출력 한 곳. 실시간 음성(PCM 조각)과 미리 녹음한 대사(mp3)가 같은 분석기를 지나가서
// 아바타 입 모양이 어느 쪽 소리든 따라간다.

import { clipVowels, liveVowel, type DialogueFrame } from '../avatar/dialogue';

export class VoicePlayer {
  private ctx?: AudioContext;
  private gain?: GainNode;
  private analyser?: AnalyserNode;
  private data?: Uint8Array<ArrayBuffer>;
  private spectrum?: Float32Array<ArrayBuffer>;
  private performance?: { key: string; start: number; duration: number; vowel: ReturnType<typeof clipVowels> };
  private clipRequest = 0;
  private next = 0;                               // 다음 PCM 조각을 붙일 시각
  private sources = new Set<AudioBufferSourceNode>();
  private clip?: AudioBufferSourceNode;
  private cache = new Map<string, Promise<AudioBuffer | null>>();

  context() {
    if (!this.ctx) {
      this.ctx = new AudioContext({ latencyHint: 'interactive' });
      this.gain = this.ctx.createGain();
      this.analyser = this.ctx.createAnalyser(); this.analyser.fftSize = 512;
      this.data = new Uint8Array(this.analyser.fftSize);
      this.spectrum = new Float32Array(this.analyser.frequencyBinCount);
      this.analyser.smoothingTimeConstant = 0.25;
      this.analyser.connect(this.gain); this.gain.connect(this.ctx.destination);
      this.gain.gain.value = this.volume;
      if (this.sink) void this.setSink(this.sink);
    }
    if (this.ctx.state === 'suspended') void this.ctx.resume();
    return this.ctx;
  }

  private volume = 1;
  private sink = '';
  setVolume(v: number) { this.volume = Math.min(1, Math.max(0, v)); if (this.gain) this.gain.gain.value = this.volume; }
  /** 소리를 낼 스피커. 빈 문자열이면 시스템 기본 장치 */
  async setSink(id: string) {
    this.sink = id;
    const ctx = this.ctx as (AudioContext & { setSinkId?: (id: string) => Promise<void> }) | undefined;
    if (ctx?.setSinkId) await ctx.setSinkId(id).catch(() => {});
  }
  static async sinks() { return (await navigator.mediaDevices.enumerateDevices()).filter(d => d.kind === 'audiooutput'); }

  /** 24 kHz 16비트 모노 PCM 조각을 끊김 없이 이어 붙인다. */
  pcm(bytes: Uint8Array, rate = 24000) {
    const ctx = this.context();
    const n = bytes.byteLength >> 1;
    if (!n) return;
    const view = new DataView(bytes.buffer, bytes.byteOffset, n * 2);
    const buf = ctx.createBuffer(1, n, rate);
    const ch = buf.getChannelData(0);
    for (let i = 0; i < n; i++) ch[i] = view.getInt16(i * 2, true) / 32768;
    const src = ctx.createBufferSource(); src.buffer = buf; src.connect(this.analyser!);
    this.next = Math.max(this.next, ctx.currentTime + 0.06);
    src.start(this.next); this.next += buf.duration;
    this.sources.add(src); src.onended = () => this.sources.delete(src);
  }

  /** 지금 나오는 실시간 음성을 끊는다(끼어들기). */
  flush() {
    for (const s of this.sources) { try { s.stop(); } catch { /* 이미 끝남 */ } }
    this.sources.clear(); this.next = 0;
  }

  get speaking() { return !!this.ctx && (this.ctx.currentTime < this.next || !!this.clip); }
  get clipPlaying() { return !!this.clip; }
  /** 실시간 음성이 다 나올 때까지 남은 시간 [s] */
  get pending() { return this.ctx ? Math.max(0, this.next - this.ctx.currentTime) : 0; }

  private load(url: string) {
    let p = this.cache.get(url);
    if (!p) {
      p = fetch(url).then(r => r.ok ? r.arrayBuffer() : Promise.reject(new Error(String(r.status))))
        .then(b => this.context().decodeAudioData(b)).catch(() => null);
      this.cache.set(url, p);
    }
    return p;
  }
  preload(url: string) { void this.load(url); }

  /** 녹음한 대사를 튼다. 끝나면(또는 못 틀면) resolve. 앞선 대사는 끊는다. */
  async play(url: string, key: string, text: string): Promise<boolean> {
    const request = ++this.clipRequest;
    const buf = await this.load(url);
    if (request !== this.clipRequest || !buf) return false;
    this.stopClip();
    const ctx = this.context();
    const src = ctx.createBufferSource(); src.buffer = buf; src.connect(this.analyser!);
    this.performance = { key, start: ctx.currentTime, duration: buf.duration, vowel: clipVowels(text, buf.getChannelData(0), buf.sampleRate) };
    this.clip = src;
    return new Promise(resolve => {
      src.onended = () => { if (this.clip === src) this.clip = undefined; resolve(true); };
      src.start();
    });
  }
  stopClip() { this.clipRequest++; const c = this.clip; this.clip = undefined; if (c) { try { c.stop(); } catch { /* 이미 끝남 */ } } }

  /** 0~1. 입 벌림에 쓴다. */
  frame(): DialogueFrame {
    const level = this.level();
    if (this.clip && this.performance && this.ctx) {
      const { start, duration, key, vowel } = this.performance;
      const time = this.ctx.currentTime - start;
      return { level, vowel: vowel(time), time, duration, key };
    }
    if (level < 0.035 || !this.analyser || !this.spectrum || !this.ctx) return { level: 0, vowel: 'aa' };
    this.analyser.getFloatFrequencyData(this.spectrum);
    return { level, vowel: liveVowel(this.spectrum, this.ctx.sampleRate / this.analyser.fftSize) };
  }

  private level() {
    if (!this.analyser || !this.data || !this.speaking) return 0;
    this.analyser.getByteTimeDomainData(this.data);
    let sum = 0; for (const v of this.data) sum += ((v - 128) / 128) ** 2;
    return Math.min(1, Math.sqrt(sum / this.data.length) * 5);
  }
}

export const player = new VoicePlayer();
