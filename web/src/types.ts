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
}

export interface Hello {
  type: 'hello'; sim: boolean;
  camera: { origin: [number, number, number]; width: number; height: number; K: number[] };
  marker_size: number; battery_charge_at: number;
}

export interface OverlayMarker { id: number; corners: [number, number][]; axes: [number, number][]; dist: number; }

export interface FrameHeader {
  type: 'frame'; stamp_ns: number; mime: string; w: number; h: number; overlay: OverlayMarker[] | null;
}
