/** Local introduction audio never leaves this machine. */
export class Introduction {
  private audio = new Audio(`${import.meta.env.BASE_URL}avatar/introduction.mp3?v=20261109`);
  private ctx?: AudioContext;
  private analyser?: AnalyserNode;
  private data?: Uint8Array<ArrayBuffer>;
  constructor(private status: (playing: boolean) => void) {
    this.audio.preload = 'none';
    this.audio.onended = () => this.status(false);
  }
  get currentTime() { return this.audio.currentTime; }
  get playing() { return !this.audio.paused; }
  async play() {
    if (!this.ctx) {
      this.ctx = new AudioContext();
      this.analyser = this.ctx.createAnalyser(); this.analyser.fftSize = 512;
      this.data = new Uint8Array(this.analyser.fftSize);
      this.ctx.createMediaElementSource(this.audio).connect(this.analyser);
      this.analyser.connect(this.ctx.destination);
    }
    await this.ctx.resume(); this.audio.currentTime = 0;
    await this.audio.play(); this.status(true);
  }
  stop() { this.audio.pause(); this.audio.currentTime = 0; this.status(false); }
  level() {
    if (!this.playing || !this.analyser || !this.data) return 0;
    this.analyser.getByteTimeDomainData(this.data);
    let sum = 0; for (const v of this.data) sum += ((v - 128) / 128) ** 2;
    return Math.min(1, Math.sqrt(sum / this.data.length) * 5);
  }
}
