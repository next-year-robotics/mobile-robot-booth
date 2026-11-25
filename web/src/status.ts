// 머리의 연결 표시, 시스템 패널, 아래 데이터 흐름 띠.
import type { State } from './types';
import { ago, el, fmt, stamp } from './ui';

type Level = 'ok' | 'warn' | 'off';

const MISSION_KO: Record<string, string> = {
  idle: '대기', starting: '출발 중', active: '주행 중', arrived: '도착', cancelling: '취소 중',
  cancelled: '취소됨', error: '오류', unknown: '알 수 없음',
};
const TRACKING_KO: Record<string, string> = {
  unavailable: '추적 불가', searching: '찾는 중', following: '따라가는 중', lost: '놓침', error: '오류',
  idle: '추적 안 함', unknown: '알 수 없음',
};
const MUX_KO: Record<string, string> = { teleop: '수동 패드', align: '장소 이동', nav: 'Nav2', follow: '추종' };
// Pi systemd 유닛 mr-booth@이름 (config/booth.yaml stack.units)
const STACK_KO: Record<string, string> = {
  follow: '주행', phone_bridge: '폰 브리지', mission: '임무', telemetry: '텔레메트리', pad: '패드 버튼', nav: '장소 이동',
};
const EVENT_KO: Record<string, string> = {
  'agent.online': '텔레메트리 시작', 'agent.offline': '텔레메트리 종료', 'link.connected': '앱 연결',
  'link.disconnected': '앱 끊김', 'trip.start': '주행 시작', 'trip.arrived': '도착', 'trip.cancelled': '주행 취소',
  'trip.error': '주행 오류', 'sos.request': 'SOS 요청', 'sos.ack': 'SOS 전달', 'sos.clear': 'SOS 해제', error: '오류',
};

interface Pill { root: HTMLElement; val: HTMLElement }

function hz(s: State, k: string): number { return s.rates[k] ?? 0; }

export class StatusView {
  private pills = new Map<string, Pill>();
  private rows = el('sys-rows');
  private events = el('events');
  private flow = el('flow');
  private sysMeta = el('sys-meta');
  private cells = new Map<string, HTMLElement>();
  private nodes = new Map<string, HTMLElement>();
  private edges = new Map<string, HTMLElement>();
  private lastSlow = 0;

  constructor() {
    for (const [k, label] of [['mcu', 'MCU'], ['camera', '카메라'], ['aruco', 'ArUco'], ['imu', 'IMU'],
      ['odom', '오도메트리'], ['app', '폰 앱'], ['cloud', '보호자 대시보드']] as const) {
      const root = document.createElement('span');
      root.className = 'pill off';
      root.innerHTML = `<span class="lbl">${label}</span><span class="val"></span>`;
      el('pills').appendChild(root);
      this.pills.set(k, { root, val: root.querySelector('.val')! });
    }
    for (const [k, label] of [['mcu', 'MCU 제어'], ['mux', '명령 출처'], ['cmd', '속도 지령'], ['mission', '임무'],
      ['pi', 'Pi 5'], ['phone', '폰 앱 링크'], ['cloud', '클라우드']] as const) {
      const kd = document.createElement('span'); kd.className = 'sys-k'; kd.textContent = label;
      const vd = document.createElement('span'); vd.className = 'sys-v';
      this.rows.append(kd, vd);
      this.cells.set(k, vd);
    }
    this.buildFlow();
  }

  private buildFlow() {
    const groups: [string, [string, string][]][] = [
      ['인지·제어', [['cam', 'C920 카메라'], ['aruco', 'ArUco 검출'], ['track', '추적 필터'], ['ctrl', '추종 제어'],
        ['mux', 'twist_mux'], ['mcu', 'STM32 PI'], ['wheel', '바퀴']]],
      ['시니어케어', [['phone', '폰 앱'], ['bridge', '폰 브리지'], ['mission', '임무'], ['cloud', 'Firebase'],
        ['pad', '보호자 패드']]],
    ];
    for (const [title, nodes] of groups) {
      const g = document.createElement('div'); g.className = 'flow-group';
      const t = document.createElement('span'); t.className = 'flow-title'; t.textContent = title;
      g.appendChild(t);
      nodes.forEach(([k, name], i) => {
        if (i > 0) {
          const e = document.createElement('i'); e.className = 'fedge';
          g.appendChild(e); this.edges.set(`${nodes[i - 1][0]}>${k}`, e);
        }
        const n = document.createElement('div'); n.className = 'fnode';
        n.innerHTML = `<span class="n">${name}</span><span class="r">-</span>`;
        g.appendChild(n); this.nodes.set(k, n);
      });
      this.flow.appendChild(g);
    }
  }

  private pill(k: string, level: Level, val: string) {
    const p = this.pills.get(k)!;
    p.root.className = `pill ${level}`;
    p.val.textContent = val;
  }

  private node(k: string, level: 'on' | 'hot' | 'bad' | 'off', rate: string) {
    const n = this.nodes.get(k)!;
    n.className = 'fnode' + (level === 'off' ? '' : ' ' + level);
    (n.querySelector('.r') as HTMLElement).textContent = rate;
  }

  private edge(k: string, on: boolean) { this.edges.get(k)!.classList.toggle('on', on); }

  private cell(k: string, html: string, warn = false) {
    const c = this.cells.get(k)!;
    c.innerHTML = html;
    c.classList.toggle('warn', warn);
  }

  onState(s: State) {
    const now = performance.now();
    if (now - this.lastSlow < 250) return;
    this.lastSlow = now;

    // ---- 머리 표시
    const b = s.base;
    // 지령이 나가는데 MCU가 안 받는다(cmd_age가 안 줄어든다). Pi 패드 노드가 2초 뒤 MCU를 리셋한다.
    const rejecting = s.mux.v != null && !!b?.fresh && (b.cmd_age_s ?? 0) > 1;
    const mcuOk = !!b?.fresh && b.agent === 'connected' && !rejecting;
    this.pill('mcu', mcuOk ? 'ok' : b ? 'warn' : 'off',
      rejecting ? '지령 거부' : b?.loop_hz ? `${fmt(b.loop_hz, 0)} Hz` : b?.fresh ? b.agent : '');
    const camHz = hz(s, 'image');
    this.pill('camera', camHz > 5 ? 'ok' : camHz > 0 ? 'warn' : 'off', camHz ? `${fmt(camHz, 0)} Hz` : '');
    const detHz = hz(s, 'detections');
    this.pill('aruco', detHz > 5 ? 'ok' : detHz > 0 ? 'warn' : 'off', detHz ? `${fmt(detHz, 0)} Hz` : '');
    const imuHz = hz(s, 'imu');
    this.pill('imu', imuHz > 50 ? 'ok' : imuHz > 0 ? 'warn' : 'off', imuHz ? `${fmt(imuHz, 0)} Hz` : '');
    const odomHz = hz(s, 'odom');
    this.pill('odom', odomHz > 10 ? 'ok' : odomHz > 0 ? 'warn' : 'off', odomHz ? `${fmt(odomHz, 0)} Hz` : '');
    const phone = s.phone;
    this.pill('app', phone?.connected ? 'ok' : phone ? 'warn' : 'off', phone ? (phone.connected ? '연결' : '끊김') : '');
    const cl = s.cloud;
    this.pill('cloud', cl?.ok && cl.online ? 'ok' : cl ? 'warn' : 'off',
      cl?.ok ? (cl.updated_age_s != null ? ago(cl.updated_age_s) : '기록 없음') : cl ? '확인 불가' : '');

    // ---- 시스템 패널
    if (b) {
      const ovr = b.overruns_10s ?? 0;
      const state = b.fresh ? (b.agent === 'connected' ? '연결' : b.agent) : '응답 없음';
      // 지령이 흐를 때만 마지막 지령 나이를 붙인다. 부팅 뒤 한 번도 없으면 수십만 ms라 의미가 없다.
      const age = b.cmd_age_s != null && b.cmd_age_s < 2 ? `  ${fmt(b.cmd_age_s * 1000, 0)} ms` : '';
      if (rejecting) this.cell('mcu', '<b>지령 거부</b> · 자동 리셋 대기', true);
      else this.cell('mcu', `<b>${state}</b> · 워치독 ${b.watchdog_ok ? '정상' : '정지'}`
        + `<span class="mono">${fmt(b.loop_hz, 0)} Hz${age}</span>`
        + (ovr > 0 ? ` · 루프 초과 ${ovr}` : ''), !b.fresh || ovr > 0);
    } else this.cell('mcu', '데이터 없음');
    const src = ['teleop', 'align', 'nav', 'follow'].map((k) =>
      `<span class="mux-src${s.mux.active === k ? ' on' : ''}">${MUX_KO[k]}</span>`).join('');
    // 패드가 붙어 있으면 joy_linux가 30 Hz로 다시 낸다
    const pad = hz(s, 'joy') > 5;
    this.cell('mux', src + `<span class="mono">${pad ? '패드 연결' : '패드 없음'}</span>`);
    this.cell('cmd', s.mux.v != null
      ? `<span class="mono" style="margin:0;color:var(--title)">v ${fmt(s.mux.v, 2, true)} m/s   w ${fmt(s.mux.w, 2, true)} rad/s</span>`
      : '지령 없음');
    const app = s.app;
    this.cell('mission', app ? `<b>${MISSION_KO[app.mission] ?? app.mission}</b> · ${TRACKING_KO[app.tracking] ?? app.tracking}`
      + (app.last_error ? `<span class="mono">${app.last_error}</span>` : '') : '신호 없음',
      app?.mission === 'error');
    const pi = s.pi;
    if (pi?.ok) {
      const uv = pi.undervoltage_now ? '저전압' : pi.throttled_now ? '스로틀' : pi.undervoltage_seen ? '저전압 이력' : '전원 정상';
      // 스택 5개가 다 active면 숫자만, 아니면 꺼진 것 이름을 앞에 둔다. 텔레메트리는 서비스 계정이 없으면 일부러 끝난다.
      const st = Object.entries(pi.stacks ?? {});
      const down = st.filter(([, v]) => v !== 'active').map(([k]) => STACK_KO[k] ?? k);
      const stack = st.length ? (down.length ? `스택 ${down.join('·')} 꺼짐` : `스택 ${st.length}/${st.length}`) : '';
      this.cell('pi', `<b>${stack}</b> ${fmt(pi.temp_c, 0)} °C · ${uv}`,
        down.length > 0 || !!(pi.undervoltage_now || pi.throttled_now) || (pi.temp_c ?? 0) > 75);
    } else this.cell('pi', pi ? '응답 없음' : s.sim ? '시뮬레이션' : '확인 중', !!pi);
    this.cell('phone', phone ? (phone.connected ? `<b>연결됨</b><span class="mono">${ago(phone.since_s)}</span>` : '<b>끊김</b>')
      : '신호 없음', !!phone && !phone.connected);
    if (cl?.ok) {
      this.cell('cloud', `<b>${cl.online ? '패드 갱신 중' : '갱신 멈춤'}</b><span class="mono">${ago(cl.updated_age_s)}</span>`, !cl.online);
    } else this.cell('cloud', cl ? '확인 불가' : s.sim ? '시뮬레이션' : '확인 중', !!cl);
    this.sysMeta.textContent = '';

    const evs = cl?.events ?? [];
    this.events.innerHTML = evs.length ? evs.slice(0, 3).map((e) => {
      const t = e.ts_ms ? stamp(e.ts_ms) : '';
      const warn = e.severity === 'warning' || e.severity === 'critical';
      return `<div class="ev${warn ? ' warn' : ''}"><span class="mono">${t}</span><span>${EVENT_KO[e.kind ?? ''] ?? e.kind ?? ''}</span><span>${e.message ?? ''}</span></div>`;
    }).join('') : '<div class="ev-empty">최근 이벤트 없음</div>';

    // ---- 흐름 띠
    const track = s.marker;
    const tracking = !!track && track.status !== 'lost';
    const followAge = s.mux.inputs.follow;
    const followOn = followAge != null && followAge < 0.3;
    const muxHz = hz(s, 'cmd_vel');
    this.node('cam', camHz > 0 ? 'on' : 'off', camHz ? `${fmt(camHz, 0)} Hz` : '-');
    this.node('aruco', detHz > 0 ? 'on' : 'off', detHz ? `${fmt(detHz, 0)} Hz` : '-');
    this.node('track', tracking ? 'hot' : hz(s, 'marker_track') > 0 ? 'on' : 'off',
      track ? (tracking ? (track.locked ? '고정' : '추적') : '찾는 중') : '-');
    this.node('ctrl', followOn ? 'on' : 'off', followOn ? '출력 중' : '대기');
    this.node('mux', muxHz > 0 ? 'on' : 'off', s.mux.active ? (MUX_KO[s.mux.active] ?? s.mux.active) : muxHz ? `${fmt(muxHz, 0)} Hz` : '-');
    this.node('mcu', mcuOk ? 'on' : b ? 'bad' : 'off', b?.loop_hz ? `${fmt(b.loop_hz, 0)} Hz` : '-');
    const moving = Math.abs(s.wheels.l.v ?? 0) + Math.abs(s.wheels.r.v ?? 0) > 0.01;
    this.node('wheel', hz(s, 'joint_states') > 0 ? 'on' : 'off', moving ? '회전' : hz(s, 'joint_states') ? '정지' : '-');
    this.edge('cam>aruco', camHz > 0 && detHz > 0);
    this.edge('aruco>track', detHz > 0);
    this.edge('track>ctrl', tracking && followOn);
    this.edge('ctrl>mux', followOn);
    this.edge('mux>mcu', muxHz > 0);
    this.edge('mcu>wheel', moving);

    const bridgeUp = !!app && (app.age_s ?? 99) < 6;
    this.node('phone', phone?.connected ? 'on' : phone ? 'bad' : 'off', phone?.connected ? '연결' : phone ? '끊김' : '-');
    this.node('bridge', phone ? 'on' : 'off', phone ? 'WebSocket' : '-');
    this.node('mission', bridgeUp ? 'on' : 'off', app ? (MISSION_KO[app.mission] ?? app.mission) : '-');
    this.node('cloud', cl?.ok && cl.online ? 'on' : cl?.ok ? 'bad' : 'off', cl?.ok ? (cl.online ? ago(cl.updated_age_s) : '멈춤') : '-');
    // 패드가 실제로 열려 있는지는 알 수 없다. 패드가 읽는 경로가 최신인지까지만 본다.
    this.node('pad', cl?.ok && cl.online ? 'on' : 'off', cl?.ok && cl.online ? '데이터 최신' : '-');
    this.edge('phone>bridge', !!phone?.connected);
    this.edge('bridge>mission', bridgeUp);
    this.edge('mission>cloud', !!cl?.ok && !!cl.online);
    this.edge('cloud>pad', !!cl?.ok && !!cl.online);
  }
}
