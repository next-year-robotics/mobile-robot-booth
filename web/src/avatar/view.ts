import type { State } from '../types';
import type { Bridge } from '../ws';
import { PerformanceAudio, type PerformanceKind } from './audio';
import type { Assistant } from '../voice/assistant';
import { Mic } from '../voice/mic';
import { AvatarScene } from './scene';
import { NAME_WORDS, nameCard } from './performance';
import { EXPRESSIONS } from './motion';
import { MiniMap, type MapStore } from '../minimap';

export class AvatarView {
  private root = document.createElement('section');
  private scene: AvatarScene;
  private playback: PerformanceAudio;
  private modelReady: Promise<void>;
  private performanceRequest = 0;
  private armDancing = false;
  private active = false;
  private received = 0;
  private connected = false;
  private caption: HTMLElement;
  private badge: HTMLElement;
  private detail: HTMLElement;
  private statusLine: HTMLElement;
  private introButton: HTMLButtonElement;
  private ptt: HTMLButtonElement;
  private map?: MiniMap;
  private mapShown = false;
  private mapUntil = 0;
  constructor(private bridge: Bridge, private voice: Assistant, back: () => void, store?: MapStore) {
    this.root.id = 'avatar-view';
    this.root.innerHTML = `
      <div class="avatar-glow"></div><div id="avatar-stage"></div>
      <header class="avatar-head"><button id="avatar-back">← 대시보드 <kbd>1</kbd></button><span id="avatar-title" class="section">MAKNAE / COMPANION ROBOT</span><span id="avatar-badge" role="status">준비 중</span></header>
      <aside id="avatar-name-card" aria-label="Mobile Assistant for Kin’s Needs And Errands" aria-hidden="true">
        <div class="name-card-label">MAKNAE</div>
        ${NAME_WORDS.filter(w => w.initial).map(w => `<div class="name-line"><span class="name-word" data-at="${w.at}"><strong>${w.word[0]}</strong>${w.word.slice(1)}</span>${w.word === 'Assistant' ? `<span class="name-word name-for" data-at="${NAME_WORDS[2].at}">for</span>` : ''}</div>`).join('')}
      </aside>
      <aside id="avatar-robot"></aside>
      <aside id="avatar-eye" hidden><img alt="탑뷰 카메라"><p id="avatar-eye-text"></p></aside>
      <aside id="avatar-map" hidden><canvas></canvas><p id="avatar-map-text"></p></aside>
      <aside class="avatar-dialogue"><p id="avatar-caption" aria-live="polite"></p></aside>
      <footer class="avatar-footer"><p id="avatar-detail">모델을 불러오고 있습니다.</p>
      <div class="avatar-actions"><button id="avatar-intro">자기소개 듣기</button><button id="avatar-talent">장기자랑 보기</button><button id="avatar-ptt">누르고 말하기 <kbd>V</kbd> · 패드 <kbd>Y</kbd></button><button id="avatar-end">대화 종료</button><select id="avatar-mic" title="마이크 고르기" aria-label="마이크"></select></div>
      <p id="avatar-stats" class="mono"></p><p class="avatar-credit">Animation credits to pixiv Inc.'s VRoid Project</p></footer>`;
    document.body.append(this.root);
    const get = <T extends HTMLElement>(id: string) => this.root.querySelector<T>(`#${id}`)!;
    this.caption = get('avatar-caption'); this.badge = get('avatar-badge'); this.detail = get('avatar-detail'); this.statusLine = get('avatar-robot');
    this.introButton = get('avatar-intro'); this.ptt = get('avatar-ptt');
    let modelError = '';
    const status = (state: string, detail = '') => {
      if (state === '대기' && modelError) { state = '모델 로드 실패'; detail = modelError; }
      this.badge.textContent = state; this.badge.dataset.state = state; this.detail.textContent = detail; };
    this.playback = new PerformanceAudio(playing => {
      // 장기자랑을 중간에 끄면 팔춤도 끝낸다. 끝까지 들었으면 팔이 V를 잠깐 유지한 뒤 스스로 접는다
      if (!playing && this.armDancing) {
        this.armDancing = false;
        if (!this.playback.finished) void this.bridge.arm({ cmd: 'dance_stop' });
      }
      this.voice.performanceActive = playing;
      this.voice.touch();
      this.introButton.textContent = playing && this.playback.kind === 'intro' ? '자기소개 멈추기' : '자기소개 듣기';
      const dancing = playing && this.playback.kind === 'keonroot-challenge';
      get('avatar-title').hidden = dancing;
      get('avatar-talent').textContent = dancing ? '장기자랑 멈추기' : '장기자랑 보기';
      status(dancing ? '장기자랑 중' : playing ? '말하는 중' : '대기');
      this.caption.textContent = '';
      if (!playing) this.updateNameCard(null);
      if (this.scene?.motion) this.scene.motion.expression = 'neutral';
    }, name => {
      if (this.active && !document.hidden && this.bridge.hello?.arm_url) {
        void this.bridge.arm({ cmd: 'intro_gesture', name });
      }
    });
    voice.stopPerformance = () => this.stopPerformance();
    voice.subscribe(ui => {
      if (this.playback.playing) {
        if (ui.state !== '말하는 중' || ui.who !== 'maknae') return;
        this.playback.stop();
      }
      status(ui.state, ui.detail);
      this.caption.textContent = ui.text ? `${ui.who === 'user' ? '관람객' : '막내'} · ${ui.text}` : '';
      this.caption.dataset.who = ui.who;
    });
    voice.expression = (name, strength) => {
      if (!this.scene?.motion) return false;
      this.scene.motion.expression = name; this.scene.motion.strength = strength;
      window.setTimeout(() => { if (this.scene.motion?.expression === name) this.scene.motion.expression = 'neutral'; }, 6000);
      return true;
    };
    this.scene = new AvatarScene(get('avatar-stage'), get('avatar-stats'), () => this.playback.playing && this.playback.kind === 'keonroot-challenge'
      ? { level: 0, vowel: 'aa', key: 'keonroot-challenge', time: this.playback.currentTime, duration: this.playback.duration }
      : this.playback.playing ? { level: this.playback.level(), vowel: 'aa' } : this.voice.frame(),
      () => this.playback.playing && this.playback.kind === 'intro' ? this.playback.currentTime : null, time => this.updateNameCard(time),
      () => this.badge.textContent === '대기' && !this.playback.playing);
    this.modelReady = this.scene.load().then(() => { if (this.badge.textContent === '준비 중') status('대기'); }).catch(() => {
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
          else { motion.expression = name as typeof motion.expression; motion.strength = 1; }
        };
        controls.append(button);
      }
      this.root.append(controls);
    }
    if (store) this.map = new MiniMap(this.root.querySelector<HTMLCanvasElement>('#avatar-map canvas')!, store);
    get('avatar-back').onclick = back;
    get('avatar-talent').onclick = async () => {
      if (this.playback.playing && this.playback.kind === 'keonroot-challenge') { this.stopPerformance(); return; }
      if (!await this.playPerformance('keonroot-challenge') && this.active) status('재생 불가', '장기자랑 모션과 음원을 확인해 주세요.');
    };
    this.introButton.onclick = async () => {
      if (this.playback.playing && this.playback.kind === 'intro') { this.stopPerformance(); return; }
      this.voice.stop();
      if (!await this.playPerformance('intro')) status('재생 불가', 'avatar/introduction.mp3 파일과 스피커를 확인해 주세요.');
    };
    get('avatar-end').onclick = () => { this.stopPerformance(); this.voice.stop(); if (this.active) this.voice.live.warm(); };
    void this.fillMics(get<HTMLSelectElement>('avatar-mic'));
    this.ptt.onpointerdown = e => { e.preventDefault(); this.ptt.setPointerCapture(e.pointerId); this.input('pointer', true); };
    this.ptt.onpointerup = () => this.input('pointer', false);
    this.ptt.onpointercancel = () => this.input('pointer', false);
    this.ptt.onlostpointercapture = () => this.input('pointer', false);
    document.addEventListener('visibilitychange', () => {
      this.scene.setActive(this.active && !document.hidden);
      if (document.hidden) this.stopPerformance();
    });
    setInterval(() => {
      if (!this.connected || performance.now() - this.received > 3000) {
        this.statusLine.textContent = '로봇 상태 · 연결 없음';
      }
    }, 500);
  }
  private stopPerformance() { this.performanceRequest++; this.playback.stop(); }
  async playPerformance(kind: PerformanceKind) {
    if (!this.active || document.hidden) return false;
    if (this.playback.playing && this.playback.kind === kind) return true;
    this.stopPerformance();
    const request = this.performanceRequest;
    this.voice.preparePerformance();
    try {
      if (kind === 'keonroot-challenge') {
        await this.modelReady;
        if (request !== this.performanceRequest) return false;
        if (!this.scene.motion?.greeting.talentReady) throw new Error('Missing dance motion');
      }
      await this.playback.play(kind);
      const ok = request === this.performanceRequest && this.playback.playing;
      if (ok && kind === 'keonroot-challenge') this.armDance();
      return ok;
    } catch {
      if (request === this.performanceRequest) this.stopPerformance();
      return false;
    }
  }

  /** 팔도 음악에 맞춰 춘다(집게를 벌린 L자). 팔이 없거나 물건을 들고 있어 거절하면 화면만 춘다. */
  private armDance() {
    const at = this.playback.startEpoch();
    if (at === null || !this.bridge.hello?.arm_url) return;
    this.armDancing = true;
    void this.bridge.arm({ cmd: 'dance', start_epoch: at }).then(r => {
      if (!r.ok) { this.armDancing = false; console.info('팔춤 생략:', r.error); }
    });
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
    this.voice.input(source, down); this.ptt.classList.toggle('held', this.voice.held);
  }
  private async fillMics(select: HTMLSelectElement) {
    const fill = async () => {
      const list = await Mic.devices().catch(() => []);
      select.replaceChildren(new Option('기본 마이크', ''), ...list.filter(d => d.deviceId && d.deviceId !== 'default')
        .map(d => new Option(d.label || `마이크 ${d.deviceId.slice(0, 4)}`, d.deviceId)));
      select.value = [...select.options].some(o => o.value === Mic.deviceId) ? Mic.deviceId : '';
    };
    select.onchange = () => { Mic.deviceId = select.value; this.voice.live.stop(); if (this.active) this.voice.live.warm(); select.blur(); };
    navigator.mediaDevices.addEventListener('devicechange', () => void fill());
    await fill();
  }
  setActive(active: boolean) {
    this.active = active; this.root.hidden = !active; this.scene.setActive(active && !document.hidden);
    if (!active) { this.ptt.classList.remove('held'); this.stopPerformance(); }
    this.map?.setActive(active && this.mapShown);
    const eye = this.root.querySelector<HTMLImageElement>('#avatar-eye img')!;
    const url = this.bridge.hello?.arm_url;
    // 탑뷰 스트림은 화면이 보일 때만 받는다
    if (active && url) { if (!eye.src) eye.src = `${url}/cam/top.mjpg`; } else eye.removeAttribute('src');
  }
  onState(s: State) {
    this.received = performance.now();
    this.statusLine.textContent = `${s.sim ? '시뮬레이션 · ' : ''}로봇 상태\n배터리 ${s.battery?.soc != null ? `${s.battery.soc.toFixed(0)}%` : '알 수 없음'} · ${s.app?.mission ?? '임무 정보 없음'}`;
    this.ptt.classList.toggle('held', this.voice.held);
    // 심부름 중에는 막내가 보는 화면(탑뷰)을 같이 보여 준다
    // 말로 시킨 이동 중에는 위에서 본 지도와 가는 길을 왼쪽에 보여 준다. 끝나고도 잠깐 둔다
    const t = s.tasks?.task, going = t?.state === 'running';
    if (going) this.mapUntil = performance.now() + 6000;
    const showMap = !!this.map && (going || performance.now() < this.mapUntil);
    if (showMap !== this.mapShown) {
      this.mapShown = showMap;
      this.root.querySelector<HTMLElement>('#avatar-map')!.hidden = !showMap;
      this.map?.setActive(showMap && this.active);
    }
    if (showMap && t) this.root.querySelector('#avatar-map-text')!.textContent = going
      ? `${t.title}${t.detail ? ` · ${t.detail}` : ''}` : t.state === 'done' ? `${t.title} · 끝` : t.state === 'failed' ? `${t.title} · 못 감` : `${t.title} · 멈춤`;
    const e = s.arm?.errand, eye = this.root.querySelector<HTMLElement>('#avatar-eye')!;
    const busy = !!e && e.state !== 'idle' && e.state !== 'teleop' && !(e.state === 'moving' && e.detail === '자기소개');
    eye.hidden = !busy || !this.bridge.hello?.arm_url;
    if (busy && e) this.root.querySelector('#avatar-eye-text')!.textContent =
      ({ looking: `${e.object} 찾는 중`, picking: `${e.object} 집는 중`, holding: `${e.object} 들고 있음`, releasing: '건네는 중', moving: '움직이는 중', error: '멈춤' } as Record<string, string>)[e.state] ?? '';
  }
  onLink(up: boolean) { this.connected = up; }
}
