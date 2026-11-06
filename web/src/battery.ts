// 배터리. /base/status supply_voltage(MCU가 1 s로 거른 값)를 브리지가 20 s로 한 번 더 거르고, 역극성 보호
// 다이오드 강하를 더해 단자 전압으로 바꾼 뒤 잔량표로 판정한다(사용자 아티팩트 "배터리 잔량표" 기준).
import type { State } from './types';
import { C, drawChart, el, fmt, Series, surface } from './ui';

const SPAN = 300;
const ZONE_KO = { ok: '여유', charge: '충전 권장', stop: '즉시 충전', charging: '충전 직후' } as const;

export class BatteryPanel {
  private soc = el('bat-soc'); private big = this.soc.parentElement!;
  private v = el('bat-v'); private note = el('bat-note');
  private bar = el('bat-bar');
  private chart = el<HTMLCanvasElement>('bat-chart');
  private series = new Series(SPAN);
  private lastPush = 0; private lastText = 0;
  private segs: HTMLElement[] = [];

  constructor() {
    for (let i = 0; i < 10; i++) this.segs.push(this.bar.appendChild(document.createElement('i')));
  }

  onState(s: State) {
    const b = s.battery;
    const now = performance.now();
    if (now - this.lastPush > 1000) { this.series.push(now / 1000, b?.v ?? null); this.lastPush = now; }
    if (now - this.lastText < 500) return;
    this.lastText = now;
    // 평평한 구간이라 1 % 단위는 거짓 정밀도다. 큰 숫자는 5 % 단위, 아래에 ±0.05 V가 만드는 범위.
    this.soc.textContent = b?.soc != null ? fmt(Math.round(b.soc / 5) * 5, 0) : '-';
    this.v.textContent = b?.v != null ? fmt(b.v, 2) : '-';
    this.note.textContent = !b ? '추정 잔량'
      : b.zone === 'charging' ? '충전 직후'
      : `${ZONE_KO[b.zone]} · 추정 ${b.soc_lo === b.soc_hi ? b.soc_lo : `${b.soc_lo}~${b.soc_hi}`} %`;
    const low = !!b?.low;
    this.big.className = 'bat-big' + (low ? ' low' : '');
    this.bar.className = low ? 'low' : '';
    const n = b?.soc != null ? Math.round(b.soc / 10) : 0;
    this.segs.forEach((e, i) => e.classList.toggle('on', i < n));
  }

  draw() {
    const sf = surface(this.chart);
    if (!sf) return;
    const { ctx, w, h } = sf;
    const ys = this.series.y.filter((v): v is number => v !== null);
    if (ys.length < 2) return;
    const lo = Math.floor((Math.min(...ys) - 0.05) * 10) / 10, hi = Math.ceil((Math.max(...ys) + 0.05) * 10) / 10;
    drawChart(ctx, 30, 4, w - 30, h - 8, [{ s: this.series, color: C.body, width: 1.5 }], lo, hi,
      performance.now() / 1000, SPAN, [lo, hi], (x) => x.toFixed(1));
  }
}
