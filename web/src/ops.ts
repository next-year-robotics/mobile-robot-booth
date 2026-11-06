// 운영 버튼(시스템 패널 머리)과 단축키. 실행은 브리지가 한다(bridge/booth_bridge/ops.py).
//   출발 S · 멈춤 Space            바로 보낸다
//   재시작 · MCU                   1.5 s 눌러야 보낸다(잘못 눌러도 스택이 내려가지 않게)
import type { State } from './types';
import { el } from './ui';

const HOLD_MS = 1500;

export class Ops {
  private toast = el('toast');
  private toastTimer = 0;
  private lastSeq = -1;     // -1: 아직 상태를 못 받음
  private start = el('op-start') as HTMLButtonElement;

  constructor(private send: (cmd: string) => void) {
    this.start.onclick = () => this.op('start');
    (el('op-stop') as HTMLButtonElement).onclick = () => this.op('stop');
    this.hold(el('op-restart') as HTMLButtonElement, 'restart');
    this.hold(el('op-mcu') as HTMLButtonElement, 'mcu_reset');
    window.addEventListener('keydown', (e) => {
      if (e.repeat) return;
      if (e.key === 's' || e.key === 'S') this.op('start');
      else if (e.key === ' ') { e.preventDefault(); this.op('stop'); }
    });
  }

  private op(cmd: string) { this.send(`op:${cmd}`); }

  private hold(btn: HTMLButtonElement, cmd: string) {
    let t0 = 0, raf = 0;
    const reset = () => { cancelAnimationFrame(raf); t0 = 0; btn.style.setProperty('--p', '0'); };
    const tick = () => {
      const p = Math.min(1, (performance.now() - t0) / HOLD_MS);
      btn.style.setProperty('--p', String(p));
      if (p >= 1) { reset(); this.op(cmd); return; }
      raf = requestAnimationFrame(tick);
    };
    btn.addEventListener('pointerdown', (e) => {
      e.preventDefault(); btn.setPointerCapture(e.pointerId); t0 = performance.now(); raf = requestAnimationFrame(tick);
    });
    btn.addEventListener('pointerup', reset);
    btn.addEventListener('pointercancel', reset);
  }

  onState(s: State) {
    // 이미 주행 중이면 출발을 흐리게 둔다(눌러도 임무 노드가 무시한다)
    const busy = ['starting', 'active', 'cancelling'].includes(s.app?.mission ?? '');
    this.start.classList.toggle('dim', busy);
    const seq = s.ops?.seq ?? 0;
    // 화면을 처음 열었을 때는 지난 알림을 띄우지 않고 기준만 잡는다
    if (this.lastSeq >= 0 && seq !== this.lastSeq && s.ops) this.show(s.ops.text, s.ops.level);
    this.lastSeq = seq;
  }

  private show(text: string, level: string) {
    this.toast.textContent = text;
    this.toast.className = level;
    this.toast.hidden = false;
    clearTimeout(this.toastTimer);
    this.toastTimer = window.setTimeout(() => { this.toast.hidden = true; }, level === 'warn' ? 8000 : 4000);
  }
}
