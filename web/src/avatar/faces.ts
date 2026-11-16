import { VRMExpression, VRMExpressionMorphTargetBind, type VRM } from '@pixiv/three-vrm';
import profiles from './face-poses.json';

// Give the subtler human brow and lid poses enough travel on Platinum's stylized face.
function amplify(shapes: Record<string, number>, gain: number) {
  return Object.fromEntries(Object.entries(shapes).map(([shape, weight]) => [shape, Math.min(1, weight * gain)]));
}
const poses = {
  ...profiles,
  laughing: { ...profiles.laughing, face: { ...amplify(profiles.laughing.face, 1.9), 'キラキラ': 0.7 } },
  startled: { ...profiles.startled, face: amplify(profiles.startled.face, 2.2) },
  determined: { ...profiles.determined, face: amplify(profiles.determined.face, 2.4) },
  worried: { ...profiles.worried, face: { ...amplify(profiles.worried.face, 1.6), '汗': 0.85 } },
  wink: { face: { 'ウインク': 1, '喜び.R': 0.4, 'ピンクほっぺ': 0.45 }, mouth: { 'にこ口': 0.65 } },
  bashful: { face: { '頬染め': 0.95, '困惑.L': 0.55, '困惑.R': 0.45, '下を見る': 0.3 }, mouth: { 'にこ口': 0.45 } },
};
export type Face = keyof typeof poses;
export const faceNames = Object.keys(poses) as Face[];

export function bindFaces(vrm: VRM) {
  const manager = vrm.expressionManager;
  const blink = manager?.getExpression('blink');
  if (!manager || !blink) return;
  const meshes = new Set(blink.binds.flatMap(b => b instanceof VRMExpressionMorphTargetBind ? b.primitives : []));
  for (const name of faceNames) {
    for (const part of ['face', 'mouth'] as const) {
      const expression = new VRMExpression(part === 'face' ? name : name + 'Mouth');
      for (const [shape, weight] of Object.entries(poses[name][part])) {
        for (const mesh of meshes) {
          const index = mesh.morphTargetDictionary?.[shape];
          if (index !== undefined) expression.addBind(new VRMExpressionMorphTargetBind({ primitives: [mesh], index, weight }));
        }
      }
      if (expression.binds.length) manager.registerExpression(expression);
    }
  }
}

const recordedFaces: Record<string, Face> = {
  greet: 'wink', attract: 'laughing', thanks: 'bashful', bye: 'wink', marker_found: 'laughing',
  looking: 'thinking', picking: 'determined', follow_start: 'determined', motion: 'determined',
  missed: 'startled', wait_close: 'startled', marker_lost: 'worried', not_found: 'worried',
  failed: 'worried', unreachable: 'worried', battery_low: 'sleepy',
};

export function recordedFace(key: string, time: number, duration: number): Face | undefined {
  const face = recordedFaces[key];
  if (face === 'wink' && (time < 0.28 || time > Math.min(1.05, duration * 0.65))) return;
  if (face === 'startled' && time > 1.1) return 'worried';
  return face;
}

const introFaces: ReadonlyArray<[number, number, Face]> = [
  [1.05, 1.95, 'wink'], [8.05, 9.3, 'laughing'], [10.2, 12.1, 'bashful'],
  [17.7, 21.5, 'determined'], [27.6, 29.2, 'laughing'], [31.9, 33.6, 'thinking'],
  [34.2, 35.2, 'startled'], [39.0, 40.8, 'bashful'],
];
export function introductionFace(time: number) {
  return introFaces.find(([start, end]) => time >= start && time < end)?.[2];
}
