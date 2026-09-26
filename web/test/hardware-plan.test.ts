import { describe, expect, it } from 'vitest';
import { buildHardwareFrame, buildHardwarePlan } from '../src/runtime/hardware-plan';
import { jointTarget } from '../src/sequence';
import type { HistoryPoint } from '../src/types';
import { zeroState } from '../src/types';

const planner = (_start: any, _target: any, requested: number) => requested / 2;

function history(): HistoryPoint[] {
  return [0.1, 0.2].map((value, index) => {
    const q = Array(7).fill(value);
    return {
      label: `frame-${index}`,
      start: zeroState(),
      target: { left: q, right: q },
      frameTargets: { left: jointTarget(q), right: jointTarget(q) },
      durationMs: 1000 * (index + 1),
      timeoutMs: 0,
    };
  });
}

describe('hardware plan', () => {
  it('rebases the first duration on the real feedback state', () => {
    const actual = zeroState();
    actual.left[0] = 0.5;
    actual.right[0] = 0.5;
    const plan = buildHardwarePlan(history(), actual, planner);
    expect(plan).toHaveLength(2);
    expect(plan[0].durationMs).toBe(500);
    expect(plan[1].durationMs).toBe(1000);
    expect(plan[1].target.left[0]).toBe(0.2);
  });

  it('builds a single frame from actual feedback', () => {
    const actual = zeroState();
    const frame = buildHardwareFrame(history(), 1, actual, planner);
    expect(frame.label).toBe('frame-1');
    expect(frame.durationMs).toBe(1000);
    expect(frame.target.right[0]).toBe(0.2);
  });
});