import { CLIP_KO } from './voice/clips';
// 심부름 화면(③). 막내가 보는 작업대(탑뷰)에 찾은 물건을 겹쳐 그리고, 운영자가 손으로도 시킬 수 있게 한다.
//
//   물건 이름을 누르면 집어 오고, "건네기"로 놓는다. 음성으로 시킨 것과 같은 상태머신(arm/maknae_arm/skills.py)을 탄다.
//   녹화 동작(재주) 재생·녹화, 고정 대사 사운드보드, 팔 상태가 오른쪽에 있다.
import type { Arm, ArmItem, State } from './types';
import type { Assistant } from './voice/assistant';
import type { Bridge } from './ws';

const STATE_KO: Record<string, string> = {
  idle: '대기', looking: '찾는 중', picking: '집는 중', holding: '들고 있음', releasing: '건네는 중',
  moving: '움직이는 중', teleop: '직접 조작', error: '멈춤',
};
const EVENT_KO: Record<string, string> = {
  shown: '보이지만 집을 부분이 안 보임', looking: '찾기 시작', picking: '집는 중', holding: '집었다', released: '건넸다', done: '끝', not_found: '못 찾음',
  unreachable: '닿지 않음', missed: '놓침', failed: '실패', stopped: '멈춤', motion: '동작 재생', motion_done: '동작 끝', dance: '장기자랑 팔춤', dance_done: '팔춤 끝',
};

export class ErrandView {
  private root = document.createElement('section');
  private img: HTMLImageElement;
  private wrist: HTMLImageElement;
  private canvas: HTMLCanvasElement;
  private arm: Arm | null = null;
  private active = false;
  private keys = { inventory: '', motions: '', clips: '', events: '' };
  private noteTimer = 0;

  constructor(private bridge: Bridge, private voice: Assistant, back: () => void) {
    this.root.id = 'errand-view'; this.root.hidden = true;
    this.root.innerHTML = `
      <header class="errand-head">
        <button id="errand-back">← 대시보드 <kbd>1</kbd></button>
        <div><p class="section">MAKNAE · 심부름</p><h1>막내가 보는 작업대</h1></div>
        <span class="chip" id="errand-state">연결 없음</span>
        <p id="errand-voice"></p>
      </header>
      <div class="errand-main">
        <div class="errand-stage">
          <div class="errand-top"><img id="errand-img" alt="탑뷰 카메라"><canvas id="errand-canvas" width="640" height="480"></canvas>
            <div id="errand-empty">탑뷰 영상 없음</div></div>
          <p id="errand-detail"></p>
        </div>
        <aside class="errand-side">
          <section><h2>작업대 위 물건 <span class="sub" id="errand-scan"></span></h2>
            <div class="errand-chips" id="errand-items"></div>
            <form id="errand-form"><input id="errand-name" placeholder="직접 입력: 예) 빨간 뚜껑 병" maxlength="40" autocomplete="off"><button>가져와</button></form></section>
          <section><h2>동작</h2><div class="errand-actions">
            <button id="errand-give" class="go">건네기</button><button id="errand-throw" title="팔을 뒤로 뺐다가 앞으로 휘두르며 놓는다">던지기</button><button id="errand-home">팔 접기</button>
            <button id="errand-rescan">다시 살피기</button><button id="errand-stop" class="stop">멈춤 <kbd>Space</kbd></button></div></section>
          <section><h2>재주 (녹화 동작)</h2><div class="errand-chips" id="errand-motions"></div>
            <div class="errand-rec"><input id="errand-rec-name" placeholder="새 동작 이름" maxlength="32" autocomplete="off">
              <button id="errand-rec-leader" title="리더 팔로 움직이며 녹화">리더로 녹화</button>
              <button id="errand-rec-hand" title="토크를 풀고 손으로 움직이며 녹화. 팔을 받치고 누르세요">손으로 녹화</button>
              <button id="errand-rec-stop" class="stop" hidden>녹화 끝</button></div></section>
          <section><h2>말하기 (녹음 대사)</h2><div class="errand-chips small" id="errand-clips"></div></section>
          <section class="errand-tele"><h2>팔 상태</h2><p class="mono" id="errand-tele"></p><img id="errand-wrist" alt="손목 카메라">
            <div id="errand-log" class="mono"></div></section>
        </aside>
      </div>
      <div id="errand-note" hidden></div>`;
    document.body.append(this.root);
    const get = <T extends HTMLElement>(id: string) => this.root.querySelector<T>(`#${id}`)!;
    this.img = get('errand-img'); this.wrist = get('errand-wrist'); this.canvas = get('errand-canvas');
    this.img.onerror = () => { this.img.removeAttribute('src'); };
    this.wrist.onerror = () => { this.wrist.hidden = true; };
    get('errand-back').onclick = back;
    get('errand-give').onclick = () => void this.cmd({ cmd: 'give' });
    get('errand-throw').onclick = () => void this.cmd({ cmd: 'throw' });
    get('errand-home').onclick = () => void this.cmd({ cmd: 'home' });
    get('errand-rescan').onclick = () => void this.cmd({ cmd: 'scan' }, '다시 살펴보는 중');
    get('errand-stop').onclick = () => void this.cmd({ cmd: 'stop' });
    get<HTMLFormElement>('errand-form').onsubmit = e => {
      e.preventDefault();
      const input = get<HTMLInputElement>('errand-name'); const name = input.value.trim();
      if (name) { void this.cmd({ cmd: 'fetch', name }); input.value = ''; input.blur(); }
    };
    const recName = get<HTMLInputElement>('errand-rec-name');
    const rec = (source: string) => {
      const name = recName.value.trim();
      if (!name) { this.note('동작 이름을 먼저 적어 주세요.', true); recName.focus(); return; }
      void this.cmd({ cmd: 'record_start', name, source }, `"${name}" 녹화 시작`);
    };
    get('errand-rec-leader').onclick = () => rec('leader');
    get('errand-rec-hand').onclick = () => rec('hand');
    get('errand-rec-stop').onclick = () => void this.cmd({ cmd: 'record_stop' }).then(r => {
      if (r.ok) { this.note(`"${r.saved}" 저장 (${r.seconds}초)`); recName.value = ''; if (this.arm?.errand.state === 'teleop') void this.bridge.arm({ cmd: 'teleop', on: false }); }
    });
    // 글자를 치는 동안에는 단축키(V, S, Space, 1~3)가 먹지 않게 한다
    this.root.querySelectorAll('input').forEach(i => i.addEventListener('keydown', e => e.stopPropagation()));
    this.root.querySelectorAll('input').forEach(i => i.addEventListener('keyup', e => e.stopPropagation()));
    // 탑뷰를 누르면 가장 가까운 물건을 집는다
    this.canvas.onclick = e => {
      const r = this.canvas.getBoundingClientRect();
      const u = (e.clientX - r.left) / r.width * 640, v = (e.clientY - r.top) / r.height * 480;
      const near = (this.arm?.errand.inventory ?? []).map(i => ({ i, d: Math.hypot(i.u - u, i.v - v) })).sort((a, b) => a.d - b.d)[0];
      if (near && near.d < 60) void this.cmd({ cmd: 'fetch', name: near.i.label });
    };
    voice.subscribe(ui => {
      get('errand-voice').textContent = ui.text ? `${ui.who === 'user' ? '관람객' : '막내'}: ${ui.text}` : `막내 · ${ui.state}${ui.detail ? ` · ${ui.detail}` : ''}`;
    });
    void voice.clips.ready.then(() => this.fillClips());
  }

  private async cmd(msg: Record<string, unknown>, okText = '') {
    const r = await this.bridge.arm(msg);
    if (!r.ok) this.note(r.error ?? '실패', true); else if (okText) this.note(okText);
    return r;
  }

  private note(text: string, warn = false) {
    const n = this.root.querySelector<HTMLElement>('#errand-note')!;
    n.textContent = text; n.className = warn ? 'warn' : ''; n.hidden = false;
    clearTimeout(this.noteTimer); this.noteTimer = window.setTimeout(() => { n.hidden = true; }, warn ? 5000 : 2500);
  }

  private fillClips() {
    const keys = this.voice.clips.keys();
    if (keys.join() === this.keys.clips) return;
    this.keys.clips = keys.join();
    const box = this.root.querySelector('#errand-clips')!;
    box.replaceChildren(...keys.map(k => {
      const b = document.createElement('button'); b.textContent = CLIP_KO[k] ?? k; b.title = k;
      b.onclick = () => { this.voice.touch(); this.voice.say(k, true); };
      return b;
    }));
  }

  setActive(active: boolean) {
    this.active = active; this.root.hidden = !active;
    const url = this.bridge.hello?.arm_url;
    // 영상 스트림은 화면이 보일 때만 받는다
    if (active && url) {
      if (!this.img.getAttribute('src')) this.img.src = `${url}/cam/top.mjpg`;
      if (!this.wrist.getAttribute('src')) { this.wrist.hidden = false; this.wrist.src = `${url}/cam/wrist.mjpg`; }
    } else { this.img.removeAttribute('src'); this.wrist.removeAttribute('src'); }
    if (active) this.fillClips();
  }

  onState(s: State) {
    const arm = s.arm ?? null;
    this.arm = arm;
    if (!this.active) return;
    const q = <T extends HTMLElement>(id: string) => this.root.querySelector<T>(`#${id}`)!;
    const chip = q('errand-state');
    q('errand-empty').hidden = !!arm && arm.cams.top?.age_s != null && arm.cams.top.age_s < 2;
    if (!arm) {
      chip.textContent = '팔 연결 없음'; chip.dataset.state = 'off';
      q('errand-detail').textContent = '팔 데몬이 꺼져 있습니다. scripts/arm.sh start 로 켭니다.';
      q('errand-tele').textContent = ''; this.draw(null);
      return;
    }
    const e = arm.errand;
    if (!this.img.getAttribute('src') && this.bridge.hello?.arm_url) this.setActive(true);   // 데몬이 나중에 떴을 때
    chip.textContent = `${STATE_KO[e.state] ?? e.state}${e.object ? ` · ${e.object}` : ''}${arm.fake ? ' (가짜 팔)' : ''}`;
    chip.dataset.state = e.state;
    q('errand-detail').textContent = arm.fault ? `팔 통신 오류: ${arm.fault}` : e.state === 'error' ? `${e.detail}` :
      e.state === 'holding' ? `${e.object}을(를) 들고 있습니다. "줘"라고 말하거나 건네기를 누르면 놓습니다.` :
      e.state === 'idle' ? '물건 이름을 누르거나 "막내야, ○○ 가져다줘"라고 말해 보세요.' :
      e.state === 'looking' ? `${e.object}을(를) 찾고 있습니다.` : e.state === 'picking' ? `${e.object}을(를) 집으러 갑니다.` :
      e.state === 'releasing' ? `${e.object}을(를) 건넵니다.` : e.detail || '';
    q('errand-scan').textContent = e.scanning ? '살펴보는 중…' : e.inventory_age_s != null ? `${Math.round(e.inventory_age_s)}초 전에 봄` : '아직 못 봄';
    q<HTMLButtonElement>('errand-give').disabled = e.state !== 'holding';
    q<HTMLButtonElement>('errand-throw').disabled = e.state !== 'holding';
    const invKey = e.inventory.map(i => `${i.label}:${i.reachable}`).join('|') + e.state + (e.inventory_age_s == null ? 'unscanned' : '');
    if (invKey !== this.keys.inventory) {
      this.keys.inventory = invKey;
      q('errand-items').replaceChildren(...(e.inventory.length ? e.inventory.map(i => {
        const b = document.createElement('button'); b.textContent = i.label; b.disabled = e.state !== 'idle';
        if (!i.reachable) { b.classList.add('far'); b.title = '팔이 닿지 않는 곳에 있다'; }
        b.onclick = () => void this.cmd({ cmd: 'fetch', name: i.label });
        return b;
      }) : [Object.assign(document.createElement('span'), { className: 'sub', textContent: e.inventory_age_s == null ? '물건 이름을 말하거나 입력해 주세요.' : '마지막으로 살폈을 때 보인 물건이 없습니다.' })]));
    }
    const motionKey = e.motions.join('|');
    if (motionKey !== this.keys.motions) {
      this.keys.motions = motionKey;
      q('errand-motions').replaceChildren(...(e.motions.length ? e.motions.map(m => {
        const b = document.createElement('button'); b.textContent = m; b.title = '누르면 재생. Shift+클릭하면 지운다';
        b.onclick = ev => {
          if (ev.shiftKey) { if (confirm(`"${m}" 동작을 지울까요?`)) void this.cmd({ cmd: 'motion_delete', name: m }, '지웠습니다'); }
          else void this.cmd({ cmd: 'motion', name: m });
        };
        return b;
      }) : [Object.assign(document.createElement('span'), { className: 'sub', textContent: '녹화한 동작이 없습니다.' })]));
    }
    const recording = !!e.recording;
    q('errand-rec-stop').hidden = !recording;
    q('errand-rec-leader').hidden = recording; q('errand-rec-hand').hidden = recording;
    if (recording) q('errand-rec-stop').textContent = `"${e.recording}" 녹화 끝`;
    const j = arm.joints;
    q('errand-tele').textContent =
      `${arm.torque ? '토크 켜짐' : '토크 꺼짐'} · ${arm.volt.toFixed(1)} V · ${arm.temp_c ?? '-'} °C · 집게 ${Math.round(j.gripper ?? 0)} (${arm.gripper_ma} mA)\n` +
      `관절 ${['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll'].map(m => (j[m] ?? 0).toFixed(0)).join(' ')}°\n` +
      `끝점 x ${arm.tip[0].toFixed(3)}  y ${arm.tip[1].toFixed(3)}  z ${arm.tip[2].toFixed(3)} m`;
    const evKey = String(e.events.at(-1)?.seq ?? 0);
    if (evKey !== this.keys.events) {
      this.keys.events = evKey;
      q('errand-log').textContent = e.events.slice(-5).reverse().map(v =>
        `${new Date(v.t * 1000).toTimeString().slice(0, 8)}  ${EVENT_KO[v.type] ?? v.type}${v.text ? `  ${v.text}` : ''}`).join('\n');
    }
    this.draw(arm);
  }

  private draw(arm: Arm | null) {
    const c = this.canvas.getContext('2d')!;
    c.clearRect(0, 0, 640, 480);
    if (!arm) return;
    const css = getComputedStyle(document.documentElement);
    const accent = css.getPropertyValue('--accent').trim(), warn = css.getPropertyValue('--warn').trim();
    if (arm.workspace) {
      c.beginPath(); arm.workspace.forEach(([u, v], i) => (i ? c.lineTo(u, v) : c.moveTo(u, v))); c.closePath();
      c.setLineDash([8, 6]); c.lineWidth = 1.5; c.strokeStyle = '#ffffffaa'; c.stroke(); c.setLineDash([]);
    }
    const e = arm.errand;
    const label = (i: ArmItem, color: string, strong: boolean) => {
      if (i.box) { c.lineWidth = strong ? 3 : 1.5; c.strokeStyle = color; c.strokeRect(i.box[0], i.box[1], i.box[2] - i.box[0], i.box[3] - i.box[1]); }
      c.beginPath(); c.arc(i.u, i.v, strong ? 7 : 4, 0, Math.PI * 2); c.fillStyle = color; c.fill();
      c.font = `600 ${strong ? 17 : 14}px Pretendard, sans-serif`;
      const text = i.label + (i.reachable ? '' : ' (멀다)'), w = c.measureText(text).width + 12;
      const x = Math.min(640 - w, Math.max(0, i.u - w / 2)), y = Math.max(2, (i.box ? i.box[1] : i.v - 12) - 24);
      c.fillStyle = '#0e1216d9'; c.fillRect(x, y, w, 21);
      c.fillStyle = color; c.textBaseline = 'middle'; c.fillText(text, x + 6, y + 11);
    };
    // 팔이 가리고 있을 때(집는 중)는 목록 위치가 옛것이라 대상만 그린다
    if (e.state === 'idle' || e.state === 'looking') for (const i of e.inventory) label(i, i.reachable ? '#ffffff' : warn, false);
    if (e.target) label(e.target, accent, true);
  }
}
