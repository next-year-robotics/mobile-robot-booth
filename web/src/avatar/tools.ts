import type { State } from '../types';
import { EXPRESSIONS, type Expression } from './motion';
export function robotStatus(s: State | null, fresh: boolean) {
  if (!fresh || !s) return { available: false, mission: null, tracking: null, battery: null, marker: null, input: null, simulation: null };
  const appFresh = s.app?.age_s != null && s.app.age_s < 3;
  return { available: true, simulation: s.sim,
    mission: appFresh ? s.app?.mission : null, tracking: appFresh ? s.app?.tracking : null,
    battery: s.battery ? { percent: s.battery.soc, zone: s.battery.zone, voltage: s.battery.v } : null,
    marker: s.marker && s.marker.age_s != null && s.marker.age_s < 3 ? s.marker.status : null,
    input: s.mux.active };
}
export function expressionArgs(args: unknown): { name: Expression; strength: number } | null {
  if (!args || typeof args !== 'object') return null;
  const a = args as Record<string, unknown>;
  if (!EXPRESSIONS.includes(a.name as Expression) || typeof a.strength !== 'number' || !Number.isFinite(a.strength) || a.strength < 0 || a.strength > 1) return null;
  return { name: a.name as Expression, strength: a.strength };
}
