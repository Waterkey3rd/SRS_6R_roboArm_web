import { readFile } from 'node:fs/promises';
import { describe, expect, it } from 'vitest';
import { MotionRecorder } from '../src/runtime/motion-recorder';
import { RoboArmController } from '../src/controller';
import { DeployIK } from '../src/wasm';
import { zeroState } from '../src/types';

describe('fixed interval motion recording', () => {
  it('samples once per slot, clones feedback, and does not backfill delayed or stale data', () => {
    const state = zeroState();
    const recorder = new MotionRecorder(100, state, 1000);
    expect(recorder.capture(state, 1099)).toBeNull();
    expect(recorder.samples).toHaveLength(1);
    state.right[0] = 0.1;
    expect(recorder.capture(state, 1107)).toBeNull();
    state.right[0] = 0.2;
    expect(recorder.capture(state, 1199)).toBeNull();
    expect(recorder.samples[1].right[0]).toBe(0.1);
    expect(recorder.samples[0].right[0]).toBe(0);
    expect(recorder.capture(state, 1200)).toBeNull();
    expect(recorder.durationMs).toBe(200);
    expect(recorder.capture(state, 1400)).toContain('定时中断');
    expect(recorder.samples).toHaveLength(3);
    expect(recorder.capture(null, 1300)).toContain('过期');
    expect(() => new MotionRecorder(0, state, 0)).toThrow();
    expect(() => new MotionRecorder(100, { ...state, right: [NaN] }, 0)).toThrow();
  });

  it('appends a take atomically, preserves the live pose, and exports a deterministic initial frame', async () => {
    const wasmBase = new URL('../public/wasm/', import.meta.url).href;
    const ik = await DeployIK.load(wasmBase, { wasmBinary: await readFile(new URL('../public/wasm/deploy_ik.wasm', import.meta.url)) });
    const controller = new (RoboArmController as any)(ik, wasmBase) as RoboArmController;
    const samples = [0.1, 0.2, 0.3].map(q => ({ ...zeroState(), right: [q, 0, 0, 0, 0, 0] }));
    expect(controller.appendRecording(samples, 100, 'test')).toBe(2);
    expect(controller.current).toEqual(zeroState());
    expect(controller.base).toEqual(samples[0]);
    expect(controller.history[0].start).toEqual(samples[0]);
    expect(controller.history[1].target).toEqual(samples[2]);
    const exported = controller.export();
    expect((exported.sequences.right.frames[0] as any).target.jointAngles).toEqual(samples[0].right);
    const history = structuredClone(controller.history);
    expect(() => controller.appendRecording([samples[0], { ...zeroState(), right: [100, 0, 0, 0, 0, 0] }], 100, 'bad')).toThrow();
    expect(controller.history).toEqual(history);
    expect(controller.appendRecording(samples, 200, 'second')).toBe(3);
    expect(controller.history[2].start).toEqual(samples[2]);
    expect(controller.history[2].target).toEqual(samples[0]);
    expect(controller.history.slice(2).map(p => p.durationMs)).toEqual([200, 200, 200]);
  });
});
