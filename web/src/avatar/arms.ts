import { Euler, Quaternion, Vector3, type Object3D } from 'three';
import type { VRM } from '@pixiv/three-vrm';

export interface ArmActing { present: number; shrug: number; proud: number; cross: number }
const mix = (a: Vector3, b: Vector3, weight: number) => a.lerp(b, Math.max(0, Math.min(1, weight)));

/** Two-bone reach: wrists move through depth, elbows stay outside the torso.
 * Targets are relative to measured arm length, rather than hard-coded joint angles.
 */
export class Arms {
  constructor(private vrm: VRM) {}
  private aim(bone: Object3D, child: Object3D, target: Vector3) {
    const p = bone.getWorldPosition(new Vector3());
    const from = child.getWorldPosition(new Vector3()).sub(p).normalize();
    const to = target.clone().sub(p).normalize();
    const world = bone.getWorldQuaternion(new Quaternion());
    const parent = bone.parent!.getWorldQuaternion(new Quaternion()).invert();
    bone.quaternion.copy(parent.multiply(new Quaternion().setFromUnitVectors(from, to)).multiply(world));
    bone.updateWorldMatrix(false, true);
  }
  update(dt: number, a: ArmActing, time: number) {
    const chest = this.vrm.humanoid.getNormalizedBoneNode('chest')!;
    chest.updateWorldMatrix(true, true);
    const center = chest.getWorldPosition(new Vector3());
    for (const side of ['left', 'right'] as const) {
      const upper = this.vrm.humanoid.getNormalizedBoneNode(`${side}UpperArm`)!;
      const lower = this.vrm.humanoid.getNormalizedBoneNode(`${side}LowerArm`)!;
      const hand = this.vrm.humanoid.getNormalizedBoneNode(`${side}Hand`)!;
      const previous = [upper, lower, hand].map(n => n.quaternion.clone());
      // Solve from the same anatomical rest frame on every update, avoiding twist accumulation.
      upper.quaternion.identity(); lower.quaternion.identity(); hand.quaternion.identity();
      upper.updateWorldMatrix(true, true);
      const shoulder = upper.getWorldPosition(new Vector3());
      const elbow = lower.getWorldPosition(new Vector3());
      const wrist = hand.getWorldPosition(new Vector3());
      const l1 = shoulder.distanceTo(elbow), l2 = elbow.distanceTo(wrist), length = l1 + l2;
      const sign = shoulder.x >= center.x ? 1 : -1;
      const point = (x: number, y: number, z: number) => shoulder.clone().add(new Vector3(sign * x, y, z).multiplyScalar(length));
      const target = point(0.06, -0.985, 0.045);
      // Forward 30–45 degree invitation, with a small conversational arc, not a sideward right angle.
      const present = side === 'right' ? a.present : a.present * 0.12;
      mix(target, point(0.20, -0.36 + 0.025 * Math.sin(time * 1.7), 0.73), present);
      mix(target, point(0.24, side === 'left' ? -0.43 : -0.36, side === 'left' ? 0.64 : 0.70), a.shrug * 0.8);
      mix(target, point(0.05, -0.45, 0.62), a.proud * (side === 'right' ? 0.65 : 0.2));
      // Wrists cross in front of the chest, with depth separation for the wide sleeves.
      const crossed = new Vector3(center.x - sign * length * 0.22,
        shoulder.y - length * (side === 'left' ? 0.10 : 0.14), center.z + length * (side === 'left' ? 0.67 : 0.52));
      mix(target, crossed, a.cross);
      const direction = target.clone().sub(shoulder);
      const distance = Math.min(length * (0.99 - 0.03 * Math.max(a.present, a.shrug, a.proud, a.cross)), Math.max(Math.abs(l1 - l2) + 0.001, direction.length()));
      direction.normalize(); target.copy(shoulder).addScaledVector(direction, distance);
      const along = (l1 * l1 - l2 * l2 + distance * distance) / (2 * distance);
      const bend = Math.sqrt(Math.max(0, l1 * l1 - along * along));
      const pole = new Vector3(sign * 0.7, -0.9, 0.16);
      pole.addScaledVector(direction, -pole.dot(direction)).normalize();
      const wantedElbow = shoulder.clone().addScaledVector(direction, along).addScaledVector(pole, bend);
      this.aim(upper, lower, wantedElbow);
      this.aim(lower, hand, target);
      // Share palm-up rotation along forearm and wrist; avoid a twisted wrist alone.
      const roll = 1.2 * Math.max(present, a.shrug * 0.8) * (1 - a.cross);
      lower.rotateX(roll * 0.75);
      hand.quaternion.setFromEuler(new Euler(sign * (0.18 + 0.15 * a.cross) + roll * 0.25, 0, sign * 0.06));
      const blend = 1 - Math.exp(-dt / 0.22);
      [upper, lower, hand].forEach((node, i) => node.quaternion.copy(previous[i].slerp(node.quaternion, blend)));
      upper.updateWorldMatrix(false, true);
    }
  }
}
