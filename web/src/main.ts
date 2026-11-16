import './style.css';
import type { AvatarView } from './avatar/view';
import { BatteryPanel } from './battery';
import { CameraPanel } from './camera';
import { ErrandView } from './errand';
import { Gauge } from './gauge';
import { ImuPanel } from './imu';
import { LayoutPanel } from './layout';
import { MapStore } from './minimap';
import { NavBar } from './navbar';
import { Ops } from './ops';
import { StatusView } from './status';
import { Twin } from './twin';
import { Assistant } from './voice/assistant';
import type { State } from './types';
import { ControlPanel } from './panel';
import { settings } from './settings';
import { el, fmt, hms } from './ui';
import { Bridge } from './ws';

async function boot() {
  // 캔버스 글자는 글꼴이 늦게 오면 대체 글꼴로 그려진다. 먼저 받아 둔다.
  await Promise.all(['400 16px Pretendard', '600 16px Pretendard', '900 16px Pretendard',
    '400 16px "JetBrains Mono"', '700 16px "JetBrains Mono"'].map((f) => document.fonts.load(f, '0가A')));

  const cam = new CameraPanel();
  const gl = new Gauge('gauge-l', '왼쪽 바퀴');
  const gr = new Gauge('gauge-r', '오른쪽 바퀴');
  const imu = new ImuPanel();
  const bat = new BatteryPanel();
  const status = new StatusView();
  let bridge: Bridge | null = null;
  let navbar: NavBar | undefined;
  let layout: LayoutPanel | undefined;
  const mapStore = new MapStore();
  let avatar: AvatarView | undefined;
  let voice: Assistant | undefined;
  let avatarLoading: Promise<void> | undefined;
  let avatarActive = location.hash === '#avatar';
  let errand: ErrandView | undefined;
  let errandActive = location.hash === '#errand';
  let lastState: State | null = null;
  let linkUp = false;
  let padDown = false;
  type View = 'dashboard' | 'avatar' | 'errand';
  const show = (view: View, gesture = false) => {
    errandActive = view === 'errand';
    document.body.classList.toggle('view-errand', errandActive);
    errand?.setActive(errandActive);
    switchView(view === 'avatar', gesture, view);
  };
  const switchView = (active: boolean, gesture = false, view: View = active ? 'avatar' : 'dashboard') => {
    avatarActive = active;
    if (view !== 'errand' && errandActive) { errandActive = false; document.body.classList.remove('view-errand'); errand?.setActive(false); }
    document.body.classList.toggle('view-avatar', active);
    location.hash = view;
    if (gesture && active && !document.fullscreenElement) void document.documentElement.requestFullscreen().catch(() => {});
    if (active && !avatar && !avatarLoading) {
      avatarLoading = import('./avatar/view').then(({ AvatarView }) => {
        avatar = new AvatarView(bridge!, voice!, () => switchView(false), mapStore);
        avatar.onLink(linkUp); if (lastState) avatar.onState(lastState);
        avatar.setActive(avatarActive);
      }).catch(() => {
        document.getElementById('avatar-view')?.remove();
        avatarLoading = undefined; switchView(false);
        const toast = el('toast'); toast.textContent = '아바타를 불러오지 못했습니다. 다시 시도해 주세요.'; toast.hidden = false;
      });
    }
    avatar?.setActive(active);
  };
  const tab = document.createElement('button'); tab.textContent = '아바타 ②'; tab.className = 'avatar-tab';
  tab.onclick = () => switchView(true, true); document.querySelector('.head-right')!.prepend(tab);
  const tab3 = document.createElement('button'); tab3.textContent = '심부름 ③'; tab3.className = 'avatar-tab';
  tab3.onclick = () => show('errand'); tab.after(tab3);
  window.addEventListener('hashchange', () => {
    const want: View = location.hash === '#avatar' ? 'avatar' : location.hash === '#errand' ? 'errand' : 'dashboard';
    const now: View = avatarActive ? 'avatar' : errandActive ? 'errand' : 'dashboard';
    if (want !== now) show(want);
  });
  window.addEventListener('keydown', e => {
    if (e.repeat || e.ctrlKey || e.metaKey || e.altKey) return;
    if (e.key === '1') switchView(false);
    else if (e.key === '2') switchView(true, true);
    else if (e.key === '3') show('errand');
    else if (e.key === ' ') void bridge?.arm({ cmd: 'stop' });      // 멈춤은 바퀴(ops.ts)와 팔을 같이 세운다
  });
  const twin = new Twin(() => bridge?.send({ cmd: 'reset_fused' }));
  const ops = new Ops((cmd) => bridge?.send({ cmd }));
  const gaugeMeta = el('gauge-meta');
  const sim = el('sim');
  const offline = el('offline');
  const clock = el('clock');

  bridge = new Bridge({
    dpad: direction => {
      if (direction === 'down') {
        const expand = avatarActive || errandActive || !document.body.classList.contains('twin-wide');
        show('dashboard');
        twin.toggleWide(expand);
      } else {
        twin.toggleWide(false);
        show(({ left: 'dashboard', up: 'avatar', right: 'errand' } as const)[direction]);
      }
    },
    hello: (h) => { mapStore.onHello(h); twin.onHello(h); navbar?.onHello(h); sim.hidden = !h.sim; settings.defaults(h.live); if (voice && h.live?.enabled) voice.live.warm(); },
    state: (s: State) => {
      lastState = s; mapStore.onState(s); layout?.onState(s); voice?.onState(s); avatar?.onState(s); errand?.onState(s);
      if (s.ptt !== padDown) { padDown = s.ptt; voice?.input('pad', s.ptt); }
      sim.hidden = !s.sim;
      cam.onState(s); imu.onState(s); bat.onState(s); status.onState(s); twin.onState(s); ops.onState(s); navbar?.onState(s);
      gl.set(s.wheels.l); gr.set(s.wheels.r);
      gaugeMeta.textContent = s.odom
        ? `차체 ${fmt(s.odom.v, 2, true)} m/s  ${fmt((s.odom.w * 180) / Math.PI, 0, true)}°/s  ${fmt(s.rates.joint_states, 0)} Hz`
        : `${fmt(s.rates.joint_states, 0)} Hz`;
    },
    frame: (h, blob) => { if (!avatarActive && !errandActive) cam.onFrame(h, blob); },
    scan: (h, mm) => { mapStore.onScan(h, mm); if (!avatarActive && !errandActive) twin.onScan(h, mm); },
    link: (up) => { linkUp = up; offline.hidden = up; voice?.onLink(up); avatar?.onLink(up); },
  });
  const toast = el('toast');
  navbar = new NavBar(bridge, text => {
    toast.textContent = text; toast.className = 'warn'; toast.hidden = false;
    window.setTimeout(() => { if (toast.textContent === text) toast.hidden = true; }, 5000);
  });
  if (bridge.hello) { navbar.onHello(bridge.hello); mapStore.onHello(bridge.hello); }
  layout = new LayoutPanel(bridge, mapStore, text => {
    toast.textContent = text; toast.className = 'warn'; toast.hidden = false;
    window.setTimeout(() => { if (toast.textContent === text) toast.hidden = true; }, 5000);
  });
  voice = new Assistant(bridge, async kind => {
    if (document.hidden) return false;
    show('avatar');
    await avatarLoading;
    return avatar?.playPerformance(kind) ?? false;
  });
  errand = new ErrandView(bridge, voice, () => show('dashboard'));
  new ControlPanel(bridge, voice);
  // 누르고 말하기: V. 어느 화면에서든 된다.
  window.addEventListener('keydown', e => {
    if (e.code === 'KeyV' && !e.repeat && !e.ctrlKey && !e.metaKey && !e.altKey) { e.preventDefault(); voice!.input('keyboard', true); }
  });
  window.addEventListener('keyup', e => { if (e.code === 'KeyV') voice!.input('keyboard', false); });
  window.addEventListener('blur', () => voice!.clearInput());
  const strip = document.createElement('div'); strip.id = 'voice-strip';
  strip.innerHTML = '<span class="chip" id="voice-chip">대기</span><span id="voice-text"></span>';
  document.getElementById('head')!.insertBefore(strip, document.getElementById('pills'));
  voice.subscribe(ui => {
    const chip = strip.querySelector<HTMLElement>('#voice-chip')!;
    chip.textContent = `막내 · ${ui.state}`; chip.dataset.state = ui.state;
    strip.querySelector('#voice-text')!.textContent = ui.text ? `${ui.who === 'user' ? '관람객' : '막내'}: ${ui.text}` : ui.detail || 'V 또는 패드 Y를 누르고 말하기';
  });

  show(errandActive ? 'errand' : avatarActive ? 'avatar' : 'dashboard');
  let last = performance.now();
  const loop = (t: number) => {
    const dt = Math.min(0.1, (t - last) / 1000);
    last = t;
    if (!avatarActive && !errandActive) { cam.draw(); gl.draw(dt); gr.draw(dt); imu.draw(); bat.draw(); twin.draw(dt); }
    requestAnimationFrame(loop);
  };
  requestAnimationFrame(loop);
  setInterval(() => { clock.textContent = hms(Date.now()); }, 500);
}

boot();
