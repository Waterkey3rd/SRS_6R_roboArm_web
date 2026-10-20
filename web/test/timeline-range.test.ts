import { readFile } from 'node:fs/promises';
import { beforeEach, describe, expect, it } from 'vitest';
import { RoboArmController } from '../src/controller';
import { DeployIK } from '../src/wasm';
import { cartesianTarget } from '../src/sequence';
import { cloneState } from '../src/types';
import { TimelineSelection } from '../src/ui/components/timeline-selection';

describe('timeline range operations', () => {
  let controller: RoboArmController;
  beforeEach(async () => {
    const wasmBase = new URL('../public/wasm/', import.meta.url).href;
    const ik = await DeployIK.load(wasmBase, { wasmBinary: await readFile(new URL('../public/wasm/deploy_ik.wasm', import.meta.url)) });
    controller = new (RoboArmController as any)(ik, wasmBase);
    [0, 5, 10, 15, 20].forEach((q, index) => controller.commandJoints({ right: [q, 0, 0, 0, 0, 0] } as any,
      { label: `point-${index}`, durationMs: 100 + index * 50, timeoutMs: index * 10 }));
  });

  it('selects both inclusive endpoints with Ctrl clicks, supports reverse ranges, and clears the anchor', () => {
    const selection = new TimelineSelection();
    expect(selection.range).toBeNull();
    selection.selectEndpoint(3);
    expect(selection.range).toEqual({ from: 3, to: 3 });
    selection.selectEndpoint(1);
    expect(selection.range).toEqual({ from: 1, to: 3 });
    selection.selectEndpoint(4);
    expect(selection.range).toEqual({ from: 3, to: 4 });
    selection.clear();
    selection.selectEndpoint(0);
    expect(selection.range).toEqual({ from: 0, to: 0 });
  });

  it('copies and inserts complete independent points in order, preserving timing and formula metadata', () => {
    controller.history[1].function = { group: 1, pointIndex: 1, pointCount: 1, t: 0.5, tStart: 0, tEnd: 1,
      space: 'JointAngleSpace', sources: { right: ['5', '0', '0', '0', '0', '0'] } as any };
    const original = structuredClone(controller.history);
    const clipboard = controller.copyHistoryRange(1, 3);
    expect(controller.pasteHistory(clipboard, 0)).toEqual({ from: 1, to: 3 });
    expect(controller.history.map(p => p.label)).toEqual(['point-0', 'point-1', 'point-2', 'point-3', 'point-1', 'point-2', 'point-3', 'point-4']);
    expect(controller.history.slice(1, 4).map(p => [p.durationMs, p.timeoutMs]))
      .toEqual(original.slice(1, 4).map(p => [p.durationMs, p.timeoutMs]));
    expect(controller.history[1].function).toEqual(original[1].function);
    controller.history.forEach((point, i) => expect(point.start).toEqual(i ? controller.history[i - 1].target : controller.base));
    clipboard[0].function!.sources.right[0] = 'changed';
    expect(controller.history[1].function!.sources.right[0]).toBe('5');
    expect(controller.history[4].function!.sources.right[0]).toBe('5');
    expect(controller.current).toEqual(controller.history[3].target);
  });

  it('deletes the whole inclusive range and supports deleting all then pasting the saved block', () => {
    const clipboard = controller.copyHistoryRange(1, 3);
    controller.deleteHistoryRange(1, 3);
    expect(controller.history.map(p => p.label)).toEqual(['point-0', 'point-4']);
    expect(controller.history[1].start).toEqual(controller.history[0].target);
    const base = cloneState(controller.base);
    controller.deleteHistoryRange(0, 1);
    expect(controller.history).toHaveLength(0);
    expect(controller.current).toEqual(base);
    expect(controller.pasteHistory(clipboard, -1)).toEqual({ from: 0, to: 2 });
    expect(controller.history[0].start).toEqual(base);
    expect(controller.history.map(p => p.durationMs)).toEqual([150, 200, 250]);
  });

  it('keeps the original history and live state when a downstream Cartesian solve or pasted target fails', () => {
    controller.history[4].frameTargets.right = cartesianTarget([1e9, 1e9, 1e9, 0, 0, 0]);
    const history = structuredClone(controller.history), current = cloneState(controller.current);
    expect(() => controller.deleteHistoryRange(1, 3)).toThrow();
    expect(controller.history).toEqual(history);
    expect(controller.current).toEqual(current);
    const clipboard = controller.copyHistoryRange(0, 1);
    expect(() => controller.pasteHistory(clipboard, 0)).toThrow();
    expect(controller.history).toEqual(history);
    expect(controller.current).toEqual(current);
    expect(() => controller.deleteHistoryRange(-1, 2)).toThrow();
    expect(() => controller.pasteHistory(clipboard, 10)).toThrow();
  });
});
