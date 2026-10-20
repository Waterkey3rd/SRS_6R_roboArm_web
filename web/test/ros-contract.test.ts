import { describe, expect, it } from 'vitest';
import { mapJointState } from '../src/ros/arm-web-client';
import { jointTarget, importSequence } from '../src/sequence';

describe('single-arm ROS and sequence contract', () => {
  it('maps six named joints in arbitrary order', () => {
    const result = mapJointState({ name: ['joint_6','joint_1','joint_2','joint_3','joint_4','joint_5'], position: [6,1,2,3,4,5] });
    expect(result?.right).toEqual([1,2,3,4,5,6]);
  });
  it('accepts ordered unnamed feedback and rejects old/invalid states', () => {
    expect(mapJointState({ position: [1,2,3,4,5,6] })?.right).toEqual([1,2,3,4,5,6]);
    expect(mapJointState({ position: Array(14).fill(0) })).toBeNull();
    expect(mapJointState({ position: Array(7).fill(0) })).toBeNull();
    expect(mapJointState({ position: [0,0,0,0,0,NaN] })).toBeNull();
    expect(mapJointState({ name: Array(6).fill('joint_1'), position: Array(6).fill(0) })).toBeNull();
  });
  it('requires explicit regeneration of legacy seven-joint and dual-arm files', () => {
    expect(() => jointTarget(Array(7).fill(0))).toThrow('6');
    expect(() => importSequence({ format: 'performance-action-sequence-v2', sequences: { left: {frames: [{}]} } }, {} as any)).toThrow('单臂');
  });
});
