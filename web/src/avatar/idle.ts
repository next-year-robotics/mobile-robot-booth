/** Only advances while the avatar is visible. Conversation cancels and rearms it. */
export class IdleSchedule {
  private remaining = 10;
  private elapsed = 0;
  private kind: string | null = null;
  private previous = 'peace';
  constructor(private random = Math.random) {}
  reset() { this.kind = null; this.elapsed = 0; this.remaining = 10 + this.random() * 8; }
  update(dt: number, allowed: boolean, durations: Record<string, number>) {
    if (!allowed) { this.reset(); return null; }
    if (this.kind) {
      this.elapsed += dt;
      if (this.elapsed >= durations[this.kind]) { this.reset(); return null; }
    } else {
      this.remaining -= dt;
      if (this.remaining > 0) return null;
      const available = ['greeting', 'peace', 'dance', 'cheer', 'pose'].filter(k => durations[k] > 0);
      const alternatives = available.filter(k => k !== this.previous);
      const choices = alternatives.length ? alternatives : available;
      const kind = choices[Math.floor(this.random() * choices.length)];
      if (!kind) { this.reset(); return null; }
      this.kind = kind; this.previous = kind; this.elapsed = 0;
    }
    return { kind: this.kind, time: this.elapsed };
  }
}
