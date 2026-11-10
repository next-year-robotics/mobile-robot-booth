import './style.css';
import type { AvatarView } from './avatar/view';
import { BatteryPanel } from './battery';
import { CameraPanel } from './camera';
import { Gauge } from './gauge';
import { ImuPanel } from './imu';
import { Ops } from './ops';
import { StatusView } from './status';
import { Twin } from './twin';
import type { State } from './types';
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
  let avatar: AvatarView | undefined;
  let avatarLoading: Promise<void> | undefined;
  let avatarActive = location.hash === '#avatar';
  let lastState: State | null = null;
  let linkUp = false;
  const switchView = (active: boolean, gesture = false) => {
    avatarActive = active;
    document.body.classList.toggle('view-avatar', active);
    location.hash = active ? 'avatar' : 'dashboard';
    if (gesture && active && !document.fullscreenElement) void document.documentElement.requestFullscreen().catch(() => {});
    if (active && !avatar && !avatarLoading) {
      avatarLoading = import('./avatar/view').then(({ AvatarView }) => {
        avatar = new AvatarView(bridge!, () => switchView(false));
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
  window.addEventListener('hashchange', () => { if ((location.hash === '#avatar') !== avatarActive) switchView(location.hash === '#avatar'); });
  window.addEventListener('keydown', e => {
    if (e.repeat || e.ctrlKey || e.metaKey || e.altKey) return;
    if (e.key === '1') switchView(false);
    else if (e.key === '2') switchView(true, true);
  });
  const twin = new Twin(() => bridge?.send({ cmd: 'reset_fused' }));
  const ops = new Ops((cmd) => bridge?.send({ cmd }));
  const gaugeMeta = el('gauge-meta');
  const sim = el('sim');
  const offline = el('offline');
  const clock = el('clock');

  bridge = new Bridge({
    hello: (h) => { twin.onHello(h); sim.hidden = !h.sim; },
    state: (s: State) => {
      lastState = s; avatar?.onState(s);
      sim.hidden = !s.sim;
      cam.onState(s); imu.onState(s); bat.onState(s); status.onState(s); twin.onState(s); ops.onState(s);
      gl.set(s.wheels.l); gr.set(s.wheels.r);
      gaugeMeta.textContent = s.odom
        ? `차체 ${fmt(s.odom.v, 2, true)} m/s  ${fmt((s.odom.w * 180) / Math.PI, 0, true)}°/s  ${fmt(s.rates.joint_states, 0)} Hz`
        : `${fmt(s.rates.joint_states, 0)} Hz`;
    },
    frame: (h, blob) => { if (!avatarActive) cam.onFrame(h, blob); },
    link: (up) => { linkUp = up; offline.hidden = up; avatar?.onLink(up); },
  });

  switchView(avatarActive);
  let last = performance.now();
  const loop = (t: number) => {
    const dt = Math.min(0.1, (t - last) / 1000);
    last = t;
    if (!avatarActive) { cam.draw(); gl.draw(dt); gr.draw(dt); imu.draw(); bat.draw(); twin.draw(dt); }
    requestAnimationFrame(loop);
  };
  requestAnimationFrame(loop);
  setInterval(() => { clock.textContent = hms(Date.now()); }, 500);
}

boot();
