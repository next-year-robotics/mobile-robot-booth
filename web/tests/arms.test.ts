import { test } from 'node:test';
import assert from 'node:assert/strict';
import { Object3D, Vector3 } from 'three';
import type { VRM } from '@pixiv/three-vrm';
import { Arms } from '../src/avatar/arms';
import { acting } from '../src/avatar/performance';

function rig() {
  const root = new Object3D(), chest = new Object3D();
  chest.position.y = 1; root.add(chest);
  const nodes: Record<string, Object3D> = { chest };
  for (const [side, sign] of [['left', 1], ['right', -1]] as const) {
    const upper = new Object3D(), lower = new Object3D(), hand = new Object3D();
    upper.position.set(sign * 0.12, 0.2, 0); lower.position.x = sign * 0.2; hand.position.x = sign * 0.2;
    chest.add(upper); upper.add(lower); lower.add(hand);
    nodes[`${side}UpperArm`] = upper; nodes[`${side}LowerArm`] = lower; nodes[`${side}Hand`] = hand;
  }
  const vrm = { humanoid: { getNormalizedBoneNode: (name: string) => nodes[name] } } as unknown as VRM;
  return { root, nodes, arms: new Arms(vrm) };
}

test('forward gestures keep hands in front, preserve bone lengths and avoid snapping', () => {
  const { root, nodes, arms } = rig();
  for (let frame = 0; frame < 600; frame++) {
    const time = 7 + frame / 30;
    const before = nodes.leftUpperArm.quaternion.clone();
    arms.update(1 / 30, acting(time), time); root.updateMatrixWorld(true);
    assert.ok(before.angleTo(nodes.leftUpperArm.quaternion) < 0.3);
    for (const side of ['left', 'right']) {
      const points = ['UpperArm', 'LowerArm', 'Hand'].map(n => nodes[side + n].getWorldPosition(new Vector3()));
      assert.ok(Math.abs(points[0].distanceTo(points[1]) - 0.2) < 1e-6);
      assert.ok(Math.abs(points[1].distanceTo(points[2]) - 0.2) < 1e-6);
      assert.ok(points[2].z > -0.001);
    }
  }
});

test('아니거든요 crosses wrists in front with separate depths, then releases', () => {
  const { root, nodes, arms } = rig();
  for (let frame = 0; frame < 90; frame++) arms.update(1 / 30, acting(31.1), 31.1);
  root.updateMatrixWorld(true);
  const left = nodes.leftHand.getWorldPosition(new Vector3());
  const right = nodes.rightHand.getWorldPosition(new Vector3());
  assert.ok(left.x < -0.05 && right.x > 0.05);
  assert.ok(left.z > 0.15 && right.z > 0.15);
  assert.ok(Math.abs(left.z - right.z) > 0.035);
  assert.equal(acting(31.1).cross, 1);
  assert.equal(acting(33).cross, 0);
  assert.equal(acting(null).cross, 0);
});
