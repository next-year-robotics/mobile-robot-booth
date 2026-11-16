// 배치 패널. P 키(또는 왼쪽 아래 지도 단추)로 연다. 부스마다 다른 시작 위치와 장소(책상 …) 자리를 지정한다.
//
//   로봇을 패드로 그 자리에 세우고 "지금 자리로" → 가장 쉽고 정확하다
//   장소를 고르고 지도를 누르기, 또는 숫자(시작 위치에서 앞·왼쪽 m, 방향 도)로 넣기
//
// 값은 브리지(bridge/booth_bridge/layout.py)가 config/layout.local.yaml에 저장한다.
import { MiniMap, type MapStore } from './minimap';
import type { State } from './types';
import type { Bridge } from './ws';

const deg = (rad: number) => Math.round((rad * 180) / Math.PI);

export class LayoutPanel {
  private root = document.createElement('aside');
  private button = document.createElement('button');
  private map: MiniMap;
  private open = false;
  private selected = '';
  private key = '';

  constructor(private bridge: Bridge, private store: MapStore, private toast: (text: string) => void) {
    this.button.id = 'layout-button'; this.button.textContent = '⌖'; this.button.title = '배치 패널 (P)';
    this.button.onclick = () => this.toggle();
    this.root.id = 'layout-panel'; this.root.hidden = true;
    this.root.innerHTML = `
      <header><h2>부스 배치</h2><span id="lp-robot"></span><button id="lp-close" title="닫기 (P)">닫기 <kbd>P</kbd></button></header>
      <div class="lp-body">
        <canvas id="lp-map"></canvas>
        <div class="lp-side">
          <p class="lp-help">로봇을 패드(LB + 스틱)로 원하는 자리에 세우고 <b>지금 자리로</b>를 누른다. 장소를 고른 뒤 지도를 눌러도 된다. 지도의 위쪽이 시작 위치에서 본 앞이다.</p>
          <section id="lp-start"></section>
          <div id="lp-places"></div>
          <div class="lp-row"><button id="lp-home">시작 위치로 돌아오기</button><button id="lp-stop" class="stop">멈춤</button></div>
          <p class="lp-foot">자리를 지정한 장소는 그 자리까지 바퀴 기록으로 간 다음 마커로 마무리한다. 자리가 없으면 마커만 찾아간다. 로봇을 다시 켜면 시작 위치는 켠 자리로 돌아간다.</p>
        </div>
      </div>`;
    document.body.append(this.button, this.root);
    this.map = new MiniMap(this.q<HTMLCanvasElement>('lp-map'), store);
    this.map.onPick = (x, y) => {
      if (!this.selected) { this.toast('먼저 오른쪽에서 장소를 고르세요.'); return; }
      const now = this.store.state?.tasks?.layout?.places?.[this.selected];
      // 방향은 그대로 두고, 처음 찍는 자리면 시작 위치에서 그쪽을 보는 방향으로 둔다
      const yaw_deg = now ? deg(now.rel[2]) : deg(Math.atan2(y, x));
      void this.send({ op: 'set_place', key: this.selected, x: +x.toFixed(2), y: +y.toFixed(2), yaw_deg });
    };
    this.q('lp-close').onclick = () => this.toggle(false);
    this.q('lp-home').onclick = async () => { const r = await bridge.task({ cmd: 'home' }); if (!r.ok) toast(r.error ?? '갈 수 없어요.'); };
    this.q('lp-stop').onclick = () => { bridge.send({ cmd: 'op:stop' }); void bridge.arm({ cmd: 'stop' }); };
    // 숫자 칸에서 누른 키가 화면 단축키로 새지 않게 한다
    this.root.addEventListener('keydown', e => { if (e.key !== 'Escape' && (e.target as HTMLElement).tagName === 'INPUT') e.stopPropagation(); });
    window.addEventListener('keydown', e => {
      if (e.repeat || e.ctrlKey || e.metaKey || e.altKey) return;
      const tag = (e.target as HTMLElement).tagName;
      if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT') return;
      if (e.code === 'KeyP') this.toggle(); else if (e.key === 'Escape' && this.open) this.toggle(false);
    });
  }

  private q<T extends HTMLElement = HTMLElement>(id: string) { return this.root.querySelector<T>(`#${id}`)!; }

  private async send(msg: Record<string, unknown>) {
    const r = await this.bridge.layout(msg);
    if (!r.ok) this.toast(r.error ?? '바꾸지 못했어요.');
  }

  toggle(open = !this.open) {
    this.open = open; this.root.hidden = !open; this.button.classList.toggle('on', open);
    this.map.setActive(open);
    if (open) { this.key = ''; if (this.store.state) this.onState(this.store.state); }
  }

  onState(s: State) {
    if (!this.open) return;
    const layout = s.tasks?.layout, pose = s.ekf, places = this.store.places;
    // 로봇이 시작 위치에서 어디에 있는지
    let where = '로봇 위치 모름';
    if (pose && layout) {
      const st = layout.start, c = Math.cos(st[2]), sn = Math.sin(st[2]), dx = pose[0] - st[0], dy = pose[1] - st[1];
      where = `로봇: 앞 ${(c * dx + sn * dy).toFixed(2)} m · 왼쪽 ${(-sn * dx + c * dy).toFixed(2)} m · ${deg(pose[2] - st[2])}°`;
    }
    this.q('lp-robot').textContent = where;
    this.q<HTMLButtonElement>('lp-home').disabled = !s.nav;
    // 목록은 내용이 바뀔 때만 다시 만든다(숫자를 치는 도중에 칸이 지워지지 않게)
    const key = JSON.stringify([layout, places.map(p => p.key), this.selected, !!pose, !!s.nav]);
    if (key === this.key) return;
    this.key = key;
    this.map.selected = this.selected;
    const start = this.q('lp-start');
    start.innerHTML = `<h3>시작 위치</h3><p>${layout?.start_set ? '지정함. "돌아와"는 언제나 여기로 온다.' : '로봇을 켠 자리(지정 안 함). "돌아와"는 떠나기 전 자리로 온다.'}</p>
      <div class="lp-row"><button data-op="set_start"${pose ? '' : ' disabled'}>지금 자리를 시작 위치로</button><button data-op="clear_start"${layout?.start_set ? '' : ' disabled'}>해제</button></div>`;
    start.querySelectorAll<HTMLButtonElement>('button').forEach(b => { b.onclick = () => void this.send({ op: b.dataset.op }); });
    const list = this.q('lp-places');
    list.replaceChildren(...places.map(p => {
      const spot = layout?.places?.[p.key], on = this.selected === p.key;
      const row = document.createElement('section');
      row.className = on ? 'on' : '';
      const [x, y, yaw] = spot?.rel ?? [0, 0, 0];
      row.innerHTML = `<h3><button class="lp-pick${on ? ' on' : ''}" title="고른 뒤 지도를 누르면 그 자리로 옮긴다">${on ? '● ' : '○ '}${p.label}</button><small>마커 ${p.marker_id}</small></h3>
        <p>${spot ? `시작에서 앞 ${x.toFixed(2)} m · 왼쪽 ${y.toFixed(2)} m · ${deg(yaw)}°를 보고 선다` : '자리 없음 — 마커만 찾아간다'}</p>
        <div class="lp-row"><button data-a="here"${pose ? '' : ' disabled'}>지금 자리로</button><button data-a="go"${s.nav ? '' : ' disabled'}>여기로 가 보기</button><button data-a="clear"${spot ? '' : ' disabled'}>지우기</button></div>
        <div class="lp-num"><label>앞<input data-n="x" type="number" step="0.1" value="${x.toFixed(2)}"></label><label>왼쪽<input data-n="y" type="number" step="0.1" value="${y.toFixed(2)}"></label><label>방향°<input data-n="yaw" type="number" step="5" value="${deg(yaw)}"></label><button data-a="apply">적용</button></div>
        <label class="lp-check"><input type="checkbox" data-a="marker"${spot?.marker !== false ? ' checked' : ''}${spot ? '' : ' disabled'}>도착해서 마커로 마무리</label>`;
      const num = (n: string) => Number(row.querySelector<HTMLInputElement>(`[data-n="${n}"]`)!.value);
      row.querySelector<HTMLButtonElement>('.lp-pick')!.onclick = () => { this.selected = on ? '' : p.key; this.onState(s); };
      row.querySelectorAll<HTMLElement>('[data-a]').forEach(b => {
        const act = b.dataset.a;
        if (act === 'marker') (b as HTMLInputElement).onchange = () => void this.send({ op: 'place_marker', key: p.key, on: (b as HTMLInputElement).checked });
        else b.onclick = async () => {
          if (act === 'here') await this.send({ op: 'set_place', key: p.key, here: true });
          else if (act === 'clear') await this.send({ op: 'clear_place', key: p.key });
          else if (act === 'apply') await this.send({ op: 'set_place', key: p.key, x: num('x'), y: num('y'), yaw_deg: num('yaw') });
          else if (act === 'go') { const r = await this.bridge.task({ cmd: 'goto', place: p.key }); if (!r.ok) this.toast(r.error ?? '갈 수 없어요.'); }
        };
      });
      return row;
    }));
    if (!places.length) list.textContent = 'config/booth.yaml의 nav.places에 장소가 없습니다.';
  }
}
