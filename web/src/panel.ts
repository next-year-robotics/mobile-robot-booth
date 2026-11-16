// 컨트롤 패널. 평소에는 숨겨 두고 C 키(또는 왼쪽 아래 톱니)로 연다. 어느 화면에서든 뜬다.
// 여기서 바꾼 값은 settings.ts를 거쳐 바로 적용되고 이 브라우저에 남는다.
import { settings, type Settings } from './settings';
import type { Assistant } from './voice/assistant';
import { CLIP_KO } from './voice/clips';
import { Mic } from './voice/mic';
import { VoicePlayer } from './voice/player';
import type { Bridge } from './ws';

const GEMINI_VOICES = ['Leda', 'Aoede', 'Zephyr', 'Autonoe', 'Laomedeia', 'Kore', 'Callirrhoe', 'Despina', 'Puck', 'Charon'];

export class ControlPanel {
  private root = document.createElement('aside');
  private gear = document.createElement('button');
  private open = false;

  constructor(private bridge: Bridge, private voice: Assistant) {
    this.gear.id = 'panel-gear'; this.gear.textContent = '⚙'; this.gear.title = '컨트롤 패널 (C)';
    this.gear.onclick = () => this.toggle();
    this.root.id = 'control-panel'; this.root.hidden = true;
    this.root.innerHTML = `
      <header><h2>컨트롤 패널</h2><button id="cp-close" title="닫기 (C)">닫기 <kbd>C</kbd></button></header>
      <section><h3>목소리</h3>
        <label>대화 목소리<span class="seg" id="cp-chat">
          <button data-v="gemini" title="Gemini Live가 바로 말한다. 가장 빠르다">Gemini (빠름)</button>
          <button data-v="hina" title="Gemini가 한 말을 Hina 목소리로 다시 읽힌다. 고정 대사와 같은 목소리, 반 박자 늦다">Hina (한 목소리)</button></span></label>
        <label>Gemini 목소리<select id="cp-gvoice">${GEMINI_VOICES.map(v => `<option>${v}</option>`).join('')}</select></label>
        <label>로봇 동작 대사<span class="seg" id="cp-clips"><button data-v="1">켜기</button><button data-v="0">끄기</button></span></label>
        <label>호객 대사<select id="cp-attract"><option value="0">끄기</option><option value="30">30초마다</option><option value="60">1분마다</option><option value="120">2분마다</option><option value="300">5분마다</option></select></label>
      </section>
      <section><h3>듣기</h3>
        <label>말하는 방식<span class="seg" id="cp-hands">
          <button data-v="0" title="V 키, 패드 Y, 화면 버튼을 누르고 있는 동안만 듣는다. 시끄러운 부스용">누르고 말하기</button>
          <button data-v="1" title="버튼 없이 말소리를 알아서 잡는다. 조용한 곳에서만">그냥 말하기</button></span></label>
        <label>마이크<select id="cp-mic"></select></label>
        <div class="cp-meter" title="마이크 입력"><i id="cp-level"></i></div>
      </section>
      <section><h3>소리</h3>
        <label>스피커<select id="cp-sink"></select></label>
        <label>음량<input id="cp-volume" type="range" min="0" max="1" step="0.05"></label>
        <p id="cp-warn" hidden></p>
        <div class="cp-row"><button id="cp-test">소리 시험</button><button id="cp-reconnect">대화 다시 연결</button><button id="cp-reset">기본값으로</button></div>
      </section>
      <aside id="cp-board" aria-labelledby="cp-board-title">
        <h2 id="cp-board-title">녹음 대사</h2>
        <div id="cp-recordings">대사를 불러오는 중…</div>
        <p id="cp-board-status" role="status"></p>
      </aside>
      <p class="cp-foot">V · 패드 Y 누르고 말하기 &nbsp; Space · 패드 B 멈춤 &nbsp; 1 대시보드 &nbsp; 2 아바타 &nbsp; 3 심부름</p>`;
    document.body.append(this.gear, this.root);
    const q = <T extends HTMLElement>(id: string) => this.root.querySelector<T>(`#${id}`)!;
    const board = q('cp-board');
    const boardStatus = q('cp-board-status');
    const list = q('cp-recordings');
    void voice.clips.ready.then(() => {
      list.replaceChildren();
      for (const key of voice.clips.keys()) {
        const group = document.createElement('section');
        const heading = document.createElement('h3');
        heading.textContent = CLIP_KO[key] ?? key;
        group.append(heading);
        for (const clip of voice.clips.recordings(key)) {
          const button = document.createElement('button');
          button.textContent = clip.text;
          button.onclick = () => {
            if (voice.held) { boardStatus.textContent = '말하기 버튼을 놓으면 재생할 수 있어요.'; return; }
            voice.touch();
            voice.interruptForRecording();
            voice.playRecording(key, clip);
          };
          group.append(button);
        }
        list.append(group);
      }
      if (!list.childElementCount) list.textContent = '재생할 녹음 대사가 없습니다.';
    });
    voice.subscribe(ui => {
      const playing = ui.state === '말하는 중' && ui.who === 'maknae' ? ui.text : '';
      for (const button of list.querySelectorAll('button')) {
        button.classList.toggle('playing', Boolean(playing) && button.textContent === playing);
      }
      boardStatus.textContent = playing ? `재생 중 · ${playing}` : ui.state === '재생 불가' ? ui.detail : '';
    });
    board.addEventListener('keydown', e => {
      if (e.code === 'Space' && e.target instanceof HTMLButtonElement) e.stopPropagation();
    });
    q('cp-close').onclick = () => this.toggle(false);
    const seg = (id: string, key: keyof Settings, parse: (v: string) => Settings[keyof Settings]) => {
      q(id).querySelectorAll<HTMLButtonElement>('button').forEach(b => { b.onclick = () => settings.set({ [key]: parse(b.dataset.v!) } as Partial<Settings>); });
    };
    seg('cp-chat', 'chatVoice', v => v as Settings['chatVoice']);
    seg('cp-clips', 'clips', v => v === '1');
    seg('cp-hands', 'handsFree', v => v === '1');
    q<HTMLSelectElement>('cp-gvoice').onchange = e => settings.set({ geminiVoice: (e.target as HTMLSelectElement).value });
    q<HTMLSelectElement>('cp-attract').onchange = e => settings.set({ attract: Number((e.target as HTMLSelectElement).value) });
    q<HTMLInputElement>('cp-volume').oninput = e => settings.set({ volume: Number((e.target as HTMLInputElement).value) });
    q<HTMLSelectElement>('cp-sink').onchange = e => settings.set({ sink: (e.target as HTMLSelectElement).value });
    q<HTMLSelectElement>('cp-mic').onchange = e => { Mic.deviceId = (e.target as HTMLSelectElement).value; this.voice.reconnect(); };
    q('cp-test').onclick = () => { this.voice.touch(); if (!this.voice.say('greet', true)) this.warn('고정 대사가 꺼져 있거나 파일이 없습니다.'); };
    q('cp-reconnect').onclick = () => this.voice.reconnect();
    q('cp-reset').onclick = () => settings.reset();
    // 패널 안에서 누른 키가 화면 단축키(V, S, Space, 숫자)로 새지 않게 한다
    this.root.addEventListener('keydown', e => { if (e.key !== 'Escape' && (e.target as HTMLElement).tagName !== 'BUTTON') e.stopPropagation(); });
    window.addEventListener('keydown', e => {
      if (e.repeat || e.ctrlKey || e.metaKey || e.altKey) return;
      const tag = (e.target as HTMLElement).tagName;
      if (tag === 'INPUT' || tag === 'TEXTAREA') return;
      if (e.code === 'KeyC') this.toggle(); else if (e.key === 'Escape' && this.open) this.toggle(false);
    });
    settings.subscribe(() => this.render());
    navigator.mediaDevices?.addEventListener('devicechange', () => { if (this.open) void this.devices(); });
    setInterval(() => { if (this.open) q('cp-level').style.width = `${Math.round(this.voice.live.micLevel * 100)}%`; }, 100);
    this.render();
  }

  toggle(open = !this.open) {
    this.open = open; this.root.hidden = !open; this.gear.classList.toggle('on', open);
    if (open) { this.render(); void this.devices(); }
  }

  private warn(text: string) { const w = this.root.querySelector<HTMLElement>('#cp-warn')!; w.textContent = text; w.hidden = !text; }

  private render() {
    const v = settings.value;
    const q = <T extends HTMLElement>(id: string) => this.root.querySelector<T>(`#${id}`)!;
    const mark = (id: string, value: string) => q(id).querySelectorAll<HTMLButtonElement>('button').forEach(b => b.classList.toggle('on', b.dataset.v === value));
    mark('cp-chat', v.chatVoice); mark('cp-clips', v.clips ? '1' : '0'); mark('cp-hands', v.handsFree ? '1' : '0');
    const hina = q<HTMLButtonElement>('cp-chat').querySelector<HTMLButtonElement>('[data-v="hina"]')!;
    hina.disabled = !this.bridge.hello?.live?.tts;
    const gv = q<HTMLSelectElement>('cp-gvoice');
    if (![...gv.options].some(o => o.value === v.geminiVoice)) gv.add(new Option(v.geminiVoice));
    gv.value = v.geminiVoice; gv.disabled = v.chatVoice === 'hina';
    const at = q<HTMLSelectElement>('cp-attract');
    if (![...at.options].some(o => Number(o.value) === v.attract)) at.add(new Option(`${v.attract}초마다`, String(v.attract)));
    at.value = String(v.attract);
    q<HTMLInputElement>('cp-volume').value = String(v.volume);
  }

  private async devices() {
    const q = <T extends HTMLElement>(id: string) => this.root.querySelector<T>(`#${id}`)!;
    const fill = (sel: HTMLSelectElement, list: MediaDeviceInfo[], first: string, current: string) => {
      sel.replaceChildren(new Option(first, ''), ...list.filter(d => d.deviceId && d.deviceId !== 'default')
        .map(d => new Option(d.label || `${d.kind === 'audioinput' ? '마이크' : '스피커'} ${d.deviceId.slice(0, 4)}`, d.deviceId)));
      sel.value = [...sel.options].some(o => o.value === current) ? current : '';
    };
    const [mics, sinks] = await Promise.all([Mic.devices().catch(() => []), VoicePlayer.sinks().catch(() => [])]);
    fill(q('cp-mic'), mics, '시스템 기본 마이크', Mic.deviceId);
    fill(q('cp-sink'), sinks, '시스템 기본 스피커', settings.value.sink);
    // 스피커 드라이버가 안 잡힌 노트북은 출력이 'Dummy Output' 하나뿐이다. 소리가 안 나는 이유를 바로 알 수 있게 알린다
    const real = sinks.filter(d => d.label && !/dummy|더미/i.test(d.label));
    this.warn(sinks.some(d => d.label) && !real.length ? '이 컴퓨터에 쓸 수 있는 스피커가 없습니다(더미 출력만 보임). 소리가 나지 않습니다.' : '');
  }
}
