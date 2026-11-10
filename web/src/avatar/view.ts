import type { State } from '../types';
import type { Bridge } from '../ws';
import { Introduction } from './audio';
import { LiveVoice } from './live';
import { AvatarScene } from './scene';
import { NAME_WORDS, nameCard } from './performance';
import { EXPRESSIONS } from './motion';
import { expressionArgs, robotStatus } from './tools';

export class AvatarView {
  private root = document.createElement('section');
  private scene: AvatarScene;
  private intro: Introduction;
  private live: LiveVoice;
  private active = false;
  private state: State | null = null;
  private received = 0;
  private connected = false;
  private padDown = false;
  private inputSources = new Set<string>();
  private caption: HTMLElement;
  private badge: HTMLElement;
  private detail: HTMLElement;
  private statusLine: HTMLElement;
  private introButton: HTMLButtonElement;
  private ptt: HTMLButtonElement;
  constructor(bridge: Bridge, back: () => void) {
    this.root.id = 'avatar-view';
    this.root.innerHTML = `
      <div class="avatar-glow"></div><div id="avatar-stage"></div>
      <header class="avatar-head"><button id="avatar-back">← 대시보드 <kbd>1</kbd></button><span class="section">MAKNAE / COMPANION ROBOT</span><span id="avatar-badge" role="status">준비 중</span></header>
      <aside id="avatar-name-card" aria-label="Mobile Assistant for Kin’s Needs And Errands" aria-hidden="true">
        <div class="name-card-label">MAKNAE</div>
        ${NAME_WORDS.filter(w => w.initial).map(w => `<div class="name-line"><span class="name-word" data-at="${w.at}"><strong>${w.word[0]}</strong>${w.word.slice(1)}</span>${w.word === 'Assistant' ? `<span class="name-word name-for" data-at="${NAME_WORDS[2].at}">for</span>` : ''}</div>`).join('')}
      </aside>
      <aside id="avatar-robot"></aside>
      <aside class="avatar-dialogue"><p id="avatar-caption" aria-live="polite"></p></aside>
      <footer class="avatar-footer"><p id="avatar-detail">모델을 불러오고 있습니다.</p>
      <div class="avatar-actions"><button id="avatar-intro">자기소개 듣기</button><button id="avatar-ptt">누르고 말하기 <kbd>V</kbd></button><button id="avatar-end">대화 종료</button></div>
      <p id="avatar-stats" class="mono"></p><p class="avatar-credit">Animation credits to pixiv Inc.'s VRoid Project</p></footer>`;
    document.body.append(this.root);
    const get = <T extends HTMLElement>(id: string) => this.root.querySelector<T>(`#${id}`)!;
    this.caption = get('avatar-caption'); this.badge = get('avatar-badge'); this.detail = get('avatar-detail'); this.statusLine = get('avatar-robot');
    this.introButton = get('avatar-intro'); this.ptt = get('avatar-ptt');
    let modelError = '';
    const status = (state: string, detail = '') => {
      if (state === '대기' && modelError) { state = '모델 로드 실패'; detail = modelError; }
      this.badge.textContent = state; this.badge.dataset.state = state; this.detail.textContent = detail; };
    this.intro = new Introduction(playing => {
      this.introButton.textContent = playing ? '자기소개 멈추기' : '자기소개 듣기';
      status(playing ? '말하는 중' : '대기');
      this.caption.textContent = '';
      if (!playing) this.updateNameCard(null);
      if (this.scene?.motion) this.scene.motion.expression = 'neutral';
    });
    this.live = new LiveVoice(() => bridge.requestLiveToken(), status, text => { this.caption.textContent = text; }, {
      get_robot_status: () => JSON.stringify(robotStatus(this.state, this.connected && performance.now() - this.received < 3000)),
      set_expression: args => {
        const valid = expressionArgs(args);
        if (!valid || !this.scene.motion) return JSON.stringify({ ok: false });
        this.scene.motion.expression = valid.name; this.scene.motion.strength = valid.strength;
        return JSON.stringify({ ok: true });
      },
    }, undefined, (phase, duration) => bridge.send({ cmd: 'live_event', phase, duration_s: Math.round(duration) }));
    this.scene = new AvatarScene(get('avatar-stage'), get('avatar-stats'), () => Math.max(this.intro.level(), this.live.level()),
      () => this.intro.playing ? this.intro.currentTime : null, time => this.updateNameCard(time),
      () => this.badge.textContent === '대기' && !this.intro.playing);
    void this.scene.load().then(() => { if (this.badge.textContent === '준비 중') status('대기'); }).catch(() => {
      modelError = 'web/public/avatar/platinum.vrm을 확인한 뒤 새로고침해 주세요. 대시보드는 계속 사용할 수 있습니다.';
      status('모델 로드 실패', modelError);
    });
    if (import.meta.env.DEV) {
      const controls = document.createElement('div'); controls.className = 'avatar-dev';
      for (const name of ['aa', 'ih', 'ou', 'ee', 'oh', ...EXPRESSIONS, 'reset']) {
        const button = document.createElement('button'); button.textContent = name;
        button.onclick = () => {
          const motion = this.scene.motion; if (!motion) return;
          if (['aa', 'ih', 'ou', 'ee', 'oh'].includes(name)) motion.debugVowel = name;
          else if (name === 'reset') { motion.debugVowel = null; motion.expression = 'neutral'; }
          else motion.expression = name as typeof motion.expression;
        };
        controls.append(button);
      }
      this.root.append(controls);
    }
    get('avatar-back').onclick = back;
    this.introButton.onclick = async () => {
      if (this.intro.playing) { this.intro.stop(); return; }
      this.clearInput(); await this.live.stop();
      if (!this.active || document.hidden) return;
      try { await this.intro.play(); } catch { status('재생 불가', 'avatar/introduction.mp3 파일과 스피커를 확인해 주세요.'); }
    };
    get('avatar-end').onclick = () => { this.clearInput(); this.intro.stop(); void this.live.stop(); };
    this.ptt.onpointerdown = e => { e.preventDefault(); this.ptt.setPointerCapture(e.pointerId); this.input('pointer', true); };
    this.ptt.onpointerup = () => this.input('pointer', false);
    this.ptt.onpointercancel = () => this.input('pointer', false);
    this.ptt.onlostpointercapture = () => this.input('pointer', false);
    window.addEventListener('keydown', e => {
      if (this.active && e.code === 'KeyV' && !e.repeat && !e.ctrlKey && !e.metaKey && !e.altKey) { e.preventDefault(); this.input('keyboard', true); }
      if (import.meta.env.DEV && this.active && e.code === 'KeyH' && this.scene.motion) this.scene.motion.expression = 'happy';
    });
    window.addEventListener('keyup', e => { if (e.code === 'KeyV') this.input('keyboard', false); });
    window.addEventListener('blur', () => this.clearInput());
    document.addEventListener('visibilitychange', () => {
      this.scene.setActive(this.active && !document.hidden);
      if (document.hidden) { this.clearInput(); this.intro.stop(); void this.live.stop(); }
    });
    setInterval(() => {
      if (!this.connected || performance.now() - this.received > 3000) {
        this.statusLine.textContent = '로봇 상태 · 연결 없음'; this.input('pad', false);
      }
    }, 500);
  }
  private updateNameCard(time: number | null) {
    const card = this.root.querySelector<HTMLElement>('#avatar-name-card')!;
    const state = nameCard(time);
    card.style.opacity = String(state.opacity);
    card.setAttribute('aria-hidden', String(state.opacity === 0));
    card.querySelectorAll<HTMLElement>('.name-word').forEach(row => {
      row.classList.toggle('revealed', time !== null && time >= Number(row.dataset.at));
    });
  }
  private input(source: string, down: boolean) {
    if (down && !this.active) return;
    const was = this.inputSources.size > 0;
    if (down) this.inputSources.add(source); else this.inputSources.delete(source);
    const held = this.inputSources.size > 0; this.ptt.classList.toggle('held', held);
    if (held && !was) { this.intro.stop(); void this.live.press(); }
    else if (!held && was) this.live.release();
  }
  private clearInput() { this.inputSources.clear(); this.ptt.classList.remove('held'); this.live.release(); }
  setActive(active: boolean) {
    this.active = active; this.root.hidden = !active; this.scene.setActive(active && !document.hidden);
    if (!active) { this.clearInput(); this.intro.stop(); void this.live.stop(); }
  }
  onState(s: State) {
    this.state = s; this.received = performance.now();
    this.statusLine.textContent = `${s.sim ? '시뮬레이션 · ' : ''}로봇 상태\n배터리 ${s.battery?.soc != null ? `${s.battery.soc.toFixed(0)}%` : '알 수 없음'} · ${s.app?.mission ?? '임무 정보 없음'}`;
    if (s.ptt !== this.padDown) { this.padDown = s.ptt; this.input('pad', s.ptt); }
  }
  onLink(up: boolean) { this.connected = up; if (!up) { this.clearInput(); void this.live.stop(); } }
}
