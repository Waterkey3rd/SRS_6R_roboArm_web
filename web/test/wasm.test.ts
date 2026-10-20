import { readFile } from 'node:fs/promises';
import { describe, expect, it } from 'vitest';
import { DeployIK, resolveWasmBase } from '../src/wasm';

async function model() {
  return DeployIK.load(new URL('../public/wasm/', import.meta.url).href, {
    wasmBinary: await readFile(new URL('../public/wasm/deploy_ik.wasm', import.meta.url)),
  });
}
describe('Engineer2026 six-axis deployment', () => {
  it('resolves static WASM relative to the page', () => {
    expect(resolveWasmBase('./wasm/', 'https://host.example/robot/index.html')).toBe('https://host.example/robot/wasm/');
  });
  it('uses six firmware joint limits', async () => {
    const ik = await model();
    const limits = ik.limits('right');
    expect(limits.lower).toHaveLength(6);
    expect(limits.upper).toHaveLength(6);
    expect(limits.lower[0] * 180 / Math.PI).toBeCloseTo(-240, 3);
    expect(limits.upper[0] * 180 / Math.PI).toBeCloseTo(240, 3);
    expect(() => ik.limits('left')).toThrow();
  });
  it('round-trips current 6R FK targets through smart IK', async () => {
    const ik = await model();
    for (const q of [[0, -.4, .2, .7, -.15, .25], [.1, -.7, .5, -.2, .4, -.3]]) {
      const fk = ik.fk('right', q);
      const result = ik.solve('right', fk.rotation, fk.position, q);
      expect(result.q).toHaveLength(6);
      expect(Math.max(...result.q.map((v, i) => Math.abs(v-q[i])))).toBeLessThan(.01);
      expect(result.positionErrorMm).toBeLessThan(.3);
      expect(result.orientationErrorRad).toBeLessThan(.001);
      const chain = ik.chain('right', q);
      expect(chain.positions).toHaveLength(7);
      expect(chain.axes).toHaveLength(6);
      chain.positions[6].forEach((v, i) => expect(v).toBeCloseTo(fk.position[i], 3));
    }
  });
  it('samples a six-joint quintic preview within the firmware speed caps', async () => {
    const ik = await model();
    const start = Array(6).fill(0), target = [0, .8, 0, 0, 0, 0];
    const duration = ik.trajectoryDuration(start, target, 500);
    expect(duration).toBeGreaterThanOrEqual(1.875 * .8 / .8 * 1000);
    const half = ik.trajectorySample(start, target, duration, duration/2);
    expect(half.position).toHaveLength(6);
    expect(half.position[1]).toBeCloseTo(.4, 6);
    expect(half.velocity[1]).toBeLessThanOrEqual(.800001);
    const end = ik.trajectorySample(start, target, duration, duration);
    expect(end.position[1]).toBeCloseTo(.8, 6);
    expect(end.velocity[1]).toBe(0);
    expect(() => ik.trajectoryDuration(Array(7).fill(0), target, 500)).toThrow();
  });
});
