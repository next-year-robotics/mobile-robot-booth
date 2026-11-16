// 마이크 → 16 kHz 16비트 모노 PCM 조각. Gemini Live가 받는 형식이다.
// 장치는 고를 수 있다(블루투스 이어폰을 손 마이크처럼 쓸 때). 고른 장치는 localStorage에 남는다.

const WORKLET = `
class Pcm16 extends AudioWorkletProcessor {
  constructor() { super(); this.ratio = sampleRate / 16000; this.acc = 0; this.sum = 0; this.n = 0; this.out = new Int16Array(640); this.len = 0; }
  process(inputs) {
    const ch = inputs[0] && inputs[0][0];
    if (!ch) return true;
    for (let i = 0; i < ch.length; i++) {
      this.sum += ch[i]; this.n++; this.acc += 1;
      if (this.acc >= this.ratio) {
        this.acc -= this.ratio;
        const v = Math.max(-1, Math.min(1, this.sum / this.n)); this.sum = 0; this.n = 0;
        this.out[this.len++] = v < 0 ? v * 32768 : v * 32767;
        if (this.len === this.out.length) { this.port.postMessage(this.out.slice().buffer); this.len = 0; }
      }
    }
    return true;
  }
}
registerProcessor('pcm16', Pcm16);`;

const KEY = 'maknae.mic';

export class Mic {
  private stream?: MediaStream;
  private node?: AudioWorkletNode;
  private source?: MediaStreamAudioSourceNode;
  private ctx?: AudioContext;
  private rms = 0;
  onChunk: (pcm: ArrayBuffer) => void = () => {};

  static get deviceId() { try { return localStorage.getItem(KEY) ?? ''; } catch { return ''; } }
  static set deviceId(id: string) { try { localStorage.setItem(KEY, id); } catch { /* 저장 못 해도 동작에는 지장 없다 */ } }
  static async devices() {
    return (await navigator.mediaDevices.enumerateDevices()).filter(d => d.kind === 'audioinput');
  }

  get open() { return !!this.stream; }
  /** 최근 조각의 음량 0~1 (듣는 중 표시용) */
  get level() { return this.rms; }

  async start() {
    if (this.stream) return;
    const id = Mic.deviceId;
    const audio: MediaTrackConstraints = { channelCount: 1, echoCancellation: true, noiseSuppression: true, autoGainControl: true };
    let stream: MediaStream;
    try {
      stream = await navigator.mediaDevices.getUserMedia({ audio: id ? { ...audio, deviceId: { exact: id } } : audio });
    } catch (e) {
      if (!id || (e instanceof DOMException && e.name === 'NotAllowedError')) throw e;
      stream = await navigator.mediaDevices.getUserMedia({ audio });   // 골라 둔 장치가 빠졌으면 기본 장치로
    }
    const ctx = new AudioContext();
    await ctx.audioWorklet.addModule(URL.createObjectURL(new Blob([WORKLET], { type: 'text/javascript' })));
    const node = new AudioWorkletNode(ctx, 'pcm16');
    node.port.onmessage = e => {
      const buf = e.data as ArrayBuffer, v = new Int16Array(buf);
      let s = 0; for (let i = 0; i < v.length; i += 4) s += (v[i] / 32768) ** 2;
      this.rms = Math.min(1, Math.sqrt(s / (v.length / 4)) * 6);
      this.onChunk(buf);
    };
    const source = ctx.createMediaStreamSource(stream);
    source.connect(node);                         // 출력에는 잇지 않는다(자기 목소리가 스피커로 나가지 않게)
    this.stream = stream; this.ctx = ctx; this.node = node; this.source = source;
  }

  stop() {
    this.source?.disconnect(); this.node?.disconnect();
    this.stream?.getTracks().forEach(t => t.stop());
    void this.ctx?.close();
    this.stream = this.ctx = this.node = this.source = undefined; this.rms = 0;
  }
}
