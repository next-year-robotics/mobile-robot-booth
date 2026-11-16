// 미리 녹음한 고정 대사(tools/make_clips.py가 만든 web/public/avatar/clips). 로봇 상태가 바뀌는 순간 바로 튼다.
import { player } from './player';

export const CLIP_KO: Record<string, string> = {
  greet: '인사', attract: '호객', follow_start: '출발', follow_stop: '도착', passing: '지나갈게요', wait_close: '가까이 옴',
  marker_lost: '놓침', marker_found: '다시 찾음', looking: '찾아볼게요', picking: '집어 올게요', holding: '가져왔어요',
  released: '여기요', not_found: '안 보여요', unreachable: '안 닿아요', missed: '놓쳤어요', failed: '문제 발생',
  stopped: '멈췄어요', motion: '재주', sos_sent: 'SOS 보냄', battery_low: '배고파요', thanks: '감사', bye: '작별',
  nav_go: '다녀올게요', nav_arrived: '도착', nav_return: '돌아갈게요', nav_back: '다녀왔어요', nav_back_empty: '못 가져옴',
  nav_not_found: '장소를 못 찾음', nav_blocked: '길이 막힘', nav_blocked_wait: '비켜 주세요', nav_failed: '이동 실패',
  guard_close: '너무 가까워요', throwing: '던질게요',
};

export interface Clip { file: string; text: string }
const BASE = `${import.meta.env.BASE_URL}avatar/clips/`;

export class Clips {
  private manifest: Record<string, Clip[]> = {};
  private last: Record<string, number> = {};
  ready: Promise<void>;
  constructor() {
    this.ready = fetch(`${BASE}manifest.json`, { cache: 'no-cache' }).then(r => r.ok ? r.json() : {}).then(m => {
      this.manifest = m;
      // 자주 쓰는 것만 먼저 받아 둔다. 나머지는 처음 틀 때 받는다
      for (const k of ['follow_start', 'passing', 'marker_lost', 'looking', 'holding', 'stopped', 'nav_go', 'nav_arrived']) this.manifest[k]?.forEach(c => player.preload(BASE + c.file));
    }).catch(() => { this.manifest = {}; });
  }
  recordings(key: string): readonly Clip[] { return this.manifest[key] ?? []; }
  keys() { return Object.keys(this.manifest).filter(k => this.manifest[k].length); }
  has(key: string) { return !!this.manifest[key]?.length; }
  /** 같은 키에 여러 개가 있으면 방금 튼 것을 피해 고른다. 틀었으면 자막 문장을, 없으면 null. */
  pick(key: string): Clip | null {
    const list = this.manifest[key];
    if (!list?.length) return null;
    let i = Math.floor(Math.random() * list.length);
    if (list.length > 1 && i === this.last[key]) i = (i + 1) % list.length;
    this.last[key] = i;
    return list[i];
  }
  url(c: Clip) { return BASE + c.file; }
}
