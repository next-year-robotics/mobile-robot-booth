import { settings } from '../settings';

// 자기소개 녹음은 다른 대사보다 14 dB쯤 작다(2026-11-15 측정: −31.2 LUFS, 녹음 대사는 −17 LUFS 안팎, 최고점 −5.6 dBFS).
// 그만큼 키우면 최고점이 넘치므로 키운 뒤에 리미터로 누른다. ffmpeg로 같은 처리를 해 보면 −17.5 LUFS가 나온다.
// 파일은 그대로 두고 재생할 때만 키운다. 입 모양은 키우기 전 소리로 재서 전과 같다.
const BOOST = 10 ** (13 / 20);
// 크롬의 컴프레서는 설정에 따라 보정 이득을 스스로 더한다(이 설정에서 약 +1.7 dB). 그만큼 뒤에서 뺀다.
const TRIM = 10 ** (-1.7 / 20);

export type PerformanceKind = 'intro' | 'keonroot-challenge';
// 자세가 대사의 강조점에 도달하도록 이동 시간만큼 미리 시작한다.
const INTRO_GESTURES = [
  { at: 0.12, name: 'hello' },     // “안녕하세요”에서 깊게 숙인다
  { at: 11.45, name: 'present' },  // “가족 곁에서”
  { at: 19.45, name: 'offer' },    // “가져다주는”
  { at: 31.35, name: 'look' },     // “주변도 살피고”
  { at: 39.95, name: 'nod' },      // “불러주세요”에 숙이고 끝인사에 일어난다
];

export class PerformanceAudio {
  private audio = new Audio();
  kind: PerformanceKind = 'intro';
  private boost?: GainNode;
  private request = 0;
  private ctx?: AudioContext;
  private analyser?: AnalyserNode;
  private data?: Uint8Array<ArrayBuffer>;
  private out?: GainNode;
  private nextGesture = 0;
  constructor(private status: (playing: boolean) => void, onIntroGesture: (name: string) => void) {
    this.audio.ontimeupdate = () => {
      if (!this.playing || this.kind !== 'intro') return;
      const t = this.currentTime;
      while (this.nextGesture < INTRO_GESTURES.length) {
        const cue = INTRO_GESTURES[this.nextGesture];
        if (t < cue.at) break;
        this.nextGesture++;
        if (t - cue.at < 0.5) onIntroGesture(cue.name);
      }
    };
    this.audio.preload = 'none';
    this.audio.onended = this.audio.onerror = () => this.status(false);
    // 컨트롤 패널의 음량과 스피커 선택을 자기소개에도 적용한다(전에는 대화·녹음 대사에만 먹었다)
    settings.subscribe(() => this.apply());
  }
  private apply() {
    const v = settings.value;
    if (this.out) this.out.gain.value = TRIM * Math.min(1, Math.max(0, v.volume));
    const ctx = this.ctx as (AudioContext & { setSinkId?: (id: string) => Promise<void> }) | undefined;
    if (ctx?.setSinkId) void ctx.setSinkId(v.sink).catch(() => {});
  }
  get currentTime() { return this.audio.currentTime; }
  /** 음악 0초가 스피커에서 나온 벽시계 시각[s]. 팔춤을 음악에 맞추는 데 쓴다(팔 데몬도 같은 노트북 시계를 쓴다). */
  startEpoch(): number | null {
    if (!this.playing || !this.ctx) return null;
    const out = (this.ctx.baseLatency || 0) + (this.ctx.outputLatency || 0);
    return Date.now() / 1000 - this.audio.currentTime + out;
  }
  /** 끝까지 재생해서 멈췄는가(중간에 끈 것과 구별) */
  get finished() { return this.audio.ended; }
  get playing() { return !this.audio.paused; }
  get duration() { return this.audio.duration; }
  async play(kind: PerformanceKind) {
    this.nextGesture = 0;
    this.kind = kind;
    const path = kind === 'intro' ? 'avatar/introduction.mp3?v=20261109' : 'motions/keonroot-challenge.mp3';
    this.audio.src = import.meta.env.BASE_URL + path;
    const request = ++this.request;
    if (!this.ctx) {
      this.ctx = new AudioContext();
      this.analyser = this.ctx.createAnalyser(); this.analyser.fftSize = 512;
      this.data = new Uint8Array(this.analyser.fftSize);
      this.ctx.createMediaElementSource(this.audio).connect(this.analyser);
      const boost = this.boost = this.ctx.createGain();
      const limiter = this.ctx.createDynamicsCompressor();
      limiter.threshold.value = -3; limiter.knee.value = 0; limiter.ratio.value = 20;
      limiter.attack.value = 0.002; limiter.release.value = 0.1;
      this.out = this.ctx.createGain();
      this.analyser.connect(boost); boost.connect(limiter); limiter.connect(this.out); this.out.connect(this.ctx.destination);
      this.apply();
    }
    this.boost!.gain.value = kind === 'intro' ? BOOST : 1;
    await this.ctx.resume();
    if (request !== this.request) return;
    this.audio.currentTime = 0;
    await this.audio.play();
    if (request === this.request) this.status(true);
  }
  stop() { this.request++; this.audio.pause(); this.audio.currentTime = 0; this.status(false); }
  level() {
    if (!this.playing || !this.analyser || !this.data) return 0;
    this.analyser.getByteTimeDomainData(this.data);
    let sum = 0; for (const v of this.data) sum += ((v - 128) / 128) ** 2;
    return Math.min(1, Math.sqrt(sum / this.data.length) * 5);
  }
}
