// 디지털 트윈 오른쪽 위의 "보내기" 버튼과 진행 표시. 음성 도구(avatar/tools.ts)와 같은 요청을 브리지 순서기로 보낸다.
import type { Hello, State } from './types';
import { el } from './ui';
import type { Bridge } from './ws';

const NAV_OFF = '이동 노드 꺼짐';

export class NavBar {
  private root = el('nav-bar');
  private status = document.createElement('span');
  private buttons: HTMLButtonElement[] = [];
  private home?: HTMLButtonElement;

  constructor(private bridge: Bridge, private toast: (text: string) => void) {
    this.status.id = 'nav-status';
  }

  onHello(h: Hello) {
    const places = h.places ?? [];
    this.root.hidden = !places.length;
    const row = document.createElement('div'); row.className = 'nav-row';
    this.buttons = places.map(p => {
      const b = document.createElement('button');
      b.textContent = `${p.label}(으)로`; b.title = `마커 ${p.marker_id} 앞으로 간다`;
      b.onclick = () => this.ask({ cmd: 'goto', place: p.key });
      return b;
    });
    this.home = document.createElement('button');
    this.home.textContent = '돌아오기'; this.home.title = '떠나기 전 자리로 돌아온다';
    this.home.onclick = () => this.ask({ cmd: 'home' });
    row.append(...this.buttons, this.home);
    this.root.replaceChildren(row, this.status);
  }

  private async ask(msg: Record<string, unknown>) {
    const r = await this.bridge.task(msg);
    if (!r.ok) this.toast(r.error ?? '보낼 수 없어요.');
  }

  onState(s: State) {
    if (this.root.hidden) return;
    const t = s.tasks?.task ?? null, running = t?.state === 'running', nav = s.nav;
    // 가는 중에 다른 곳을 누르면 그쪽으로 바꾼다(브리지 순서기가 하던 이동을 그만둔다)
    for (const b of this.buttons) b.disabled = !nav;
    if (this.home) this.home.disabled = !nav || !nav.home;
    let text = NAV_OFF, state = 'off';
    if (nav) {
      const planTag = s.plan && s.plan.pts.length >= 2 ? ` · Nav2 A* ${s.plan.len_m.toFixed(2)} m` : '';
      if (running) { text = `${t!.title} ${t!.step}/${t!.steps}${t!.detail ? ` · ${t!.detail}` : ''}${planTag}`; state = 'running'; }
      else if (t?.state === 'failed') { text = `${t.title} 실패`; state = 'failed'; }
      else if (nav.backend === 'nav2') { text = '대기 · Nav2 A* + 라이다'; state = 'idle'; }
      else { text = nav.lidar ? '대기 · 라이다로 앞을 봄' : '대기 · 라이다 없음(장애물을 못 봄)'; state = 'idle'; }
    }
    if (this.status.textContent !== text) this.status.textContent = text;
    this.status.dataset.state = state;
  }
}
