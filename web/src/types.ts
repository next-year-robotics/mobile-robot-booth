// 브리지(bridge/booth_bridge/state.py snapshot)가 보내는 모양. null은 "모름·끊김".

export interface Wheel {
  v: number | null; rpm: number | null; pos: number | null;
  target: number | null; duty: number | null; sat: boolean;
}

export interface Marker3D { id: number; p: [number, number, number]; q: [number, number, number, number]; }

export interface CloudEvent { ts_ms: number | null; kind: string | null; severity: string | null; message: string | null; }

export interface State {
  type: 'state';
  ptt: boolean;
  sim: boolean;
  wall_ms: number;
  rates: Record<string, number>;
  ages: Record<string, number | null>;
  wheels: { l: Wheel; r: Wheel };
  base: null | {
    fresh: boolean; agent: string; watchdog_ok: boolean; cmd_age_s: number | null;
    loop_count: number; loop_overruns: number; overruns_10s: number | null; loop_hz: number | null;
  };
  battery: null | { v: number | null; v_topic: number | null; v_filt: number | null; soc: number | null;
    soc_lo: number; soc_hi: number; zone: 'ok' | 'charge' | 'stop' | 'charging'; low: boolean };
  odom: null | { x: number; y: number; yaw: number; v: number; w: number };
  fused: null | { x: number; y: number; yaw: number };
  imu: null | {
    gyro_z: number | null; odom_w: number | null; mismatch_pct: number | null;
    ax: number | null; ay: number | null; roll: number | null; pitch: number | null;
    heading_gyro: number | null; v_imu: number | null; v_wheel: number | null; still: boolean;
  };
  marker: null | {
    status: 'lost' | 'tracking' | 'holding'; locked: boolean; id: number;
    x: number | null; y: number | null; range: number | null; bearing: number | null;
    yaw: number | null; yaw_valid: boolean; hold_s: number | null; rejects_10s: number; age_s: number | null;
  };
  markers3d: Marker3D[];
  markers3d_age: number | null;
  mux: { active: string | null; inputs: Record<string, number | null>; v: number | null; w: number | null };
  phone: null | { connected: boolean; since_s: number | null };
  app: null | { mission: string; tracking: string; trip_id: string | null; last_error: string | null; age_s: number | null };
  cloud: null | {
    ok: boolean; error?: string; checked_ms?: number; updated_age_s?: number | null; online?: boolean;
    phone_link?: boolean | null; mission?: string | null; tracking?: string | null; events?: CloudEvent[];
  };
  pi: null | {
    ok: boolean; error?: string; temp_c?: number | null; throttled?: string; undervoltage_now?: boolean;
    throttled_now?: boolean; undervoltage_seen?: boolean; load1?: number | null; ncpu?: number | null;
    stacks?: Record<string, string> | null;   // mr-booth@이름 → systemctl is-active
  };
  ops: null | { seq: number; text: string; level: 'info' | 'ok' | 'warn'; busy: string | null };
  arm?: null | Arm;
  lidar?: null | { n: number; hz: number; nearest_m: number | null; nearest_deg: number | null };
  nav?: null | Nav;
  /** Nav2 전역 경로(/plan, EKF/odom 좌표로 변환된 2D 점열) */
  plan?: null | NavPlan;
  /** 이동 노드가 쓰는 좌표(EKF)의 로봇 자세 [x, y, yaw] */
  ekf?: null | [number, number, number];
  tasks?: null | { enabled: boolean; task: TaskInfo | null; events: TaskEvent[]; layout?: null | Layout };
}

export interface NavPlan {
  pts: [number, number][];
  len_m: number;
  frame: string;
  age_s: number | null;
}

/** Pi 이동 노드(robot/errand_nav.py)의 /nav/status. goal·home은 지금 로봇에서 본 위치 [앞, 왼쪽] m */
export interface Nav {
  state: 'idle' | 'searching' | 'approaching' | 'aligning' | 'driving' | 'turning' | 'following' | 'arrived' | 'failed' | 'cancelled';
  detail: string; id: string; kind: string; marker_id: number | null;
  backend?: string;
  blocked: boolean; passing?: boolean; paused?: boolean; lidar: boolean; ahead_m: number | null;
  goal: [number, number] | null; home: [number, number] | null; dist_m: number | null; visible: number[];
  /** 접근 감지: 따라가는 중에 누가 바짝 붙어 서 있다. near는 로봇에서 본 그 점 [앞, 왼쪽] m */
  guard?: { hold: boolean; near: [number, number] | null; enabled?: boolean };
}
/** 부스 배치(bridge/booth_bridge/layout.py). pose는 EKF 좌표, rel은 시작 위치 기준 [앞, 왼쪽, 방향] */
export interface Layout {
  start: [number, number, number]; start_set: boolean;
  places: Record<string, { pose: [number, number, number]; rel: [number, number, number]; marker: boolean }>;
}
/** 브리지 작업 순서기(bridge/booth_bridge/tasks.py) */
export interface TaskInfo {
  id: string; kind: 'goto' | 'fetch_from' | 'home' | 'move' | 'follow_marker' | 'fetch' | 'give'; title: string; state: 'running' | 'done' | 'failed' | 'cancelled';
  detail: string; place: string | null; label: string | null; object: string; got: string | null; problem: string;
  step: number; steps: number;
  /** 아직 더 가야 한다(집은 물건을 들고 돌아가는 중). "가져왔어요"는 도착해서 말한다 */
  carry?: boolean;
}
export interface TaskEvent {
  seq: number; type: 'go_start' | 'arrived' | 'fetching' | 'return_start' | 'returned' | 'failed' | 'cancelled' | 'move_start' | 'moved' | 'follow_start';
  task: string; label: string | null; object: string; got: string | null; problem: string; reason?: string; say?: string;
  /** 이 뒤에 건네기가 이어진다 */
  give?: boolean;
}
export interface Place { key: string; label: string; marker_id: number; aliases: string[] }

export interface ArmItem { label: string; u: number; v: number; x: number; y: number; height_m: number; reachable: boolean; box?: number[] }
export interface ArmEvent { seq: number; type: string; text: string; t: number; reason?: string }
/** 팔 데몬(arm/maknae_arm __main__.py state)이 보내는 모양 */
export interface Arm {
  ok: boolean; fake: boolean; fault: string;
  joints: Record<string, number>; tip: [number, number, number];
  torque: boolean; moving: boolean; volt: number; temp_c: number | null; gripper_ma: number;
  table_z: number | null; cams: Record<string, { age_s: number | null }>; poses: string[];
  workspace: [number, number][] | null;
  errand: {
    state: 'idle' | 'looking' | 'picking' | 'holding' | 'releasing' | 'moving' | 'teleop' | 'error';
    since_s: number; object: string; detail: string; target: ArmItem | null;
    inventory: ArmItem[]; inventory_age_s: number | null; scanning: boolean;
    motions: string[]; recording: string; events: ArmEvent[];
    /** 작업대 높이 [m, 팔 받침 기준]. marker는 마커 실측, vision은 현재 영상 추정, unknown은 미측정 */
    table?: { z: number | null; source: 'marker' | 'vision' | 'unknown'; spread_m: number | null; marker_id: number | null };
  };
}

export interface Hello {
  type: 'hello'; sim: boolean;
  camera: { origin: [number, number, number]; width: number; height: number; K: number[] };
  marker_size: number; battery_charge_at: number;
  arm_url?: string | null;
  /** 앞 보기(전방 카메라 한 장을 Gemini에게 묻기)가 켜져 있는가 */
  look?: boolean;
  places?: Place[];
  nav?: { enabled: boolean; follow_marker_id?: number; nudge_m?: number; turn_deg?: number };
  lidar?: { z: number; xy: [number, number] };
  live?: { enabled: boolean; attract_s: number; voice?: string; chat_voice?: string; tts?: boolean };
}

export interface OverlayMarker { id: number; corners: [number, number][]; axes: [number, number][]; dist: number; }

export interface FrameHeader {
  type: 'frame'; stamp_ns: number; mime: string; w: number; h: number; overlay: OverlayMarker[] | null;
}
