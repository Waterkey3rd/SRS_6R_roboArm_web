import type { HistoryPoint, ArmState } from '../types';
import { cloneState } from '../types';

export interface HardwareFrame {
  label: string;
  target: ArmState;
  /** Planned or requested duration for this frame. */
  durationMs: number;
  /** Feedback settle timeout after the planned duration. 0 disables the wait. */
  timeoutMs: number;
}

export interface HardwarePlanOptions {
  /** Extra conservative slowdown applied to the web-planned duration. */
  durationScale?: number;
  minDurationMs?: number;
  maxDurationMs?: number;
}

export type DurationPlanner = (start: ArmState, target: ArmState, requestedDurationMs: number) => number;

const DEFAULT_MIN_DURATION_MS = 50;
const DEFAULT_MAX_DURATION_MS = 60000;

export function buildHardwarePlan(
  history: HistoryPoint[],
  actual: ArmState,
  planner: DurationPlanner,
  options: HardwarePlanOptions = {},
): HardwareFrame[] {
  const durationScale = options.durationScale ?? 1;
  const minDurationMs = options.minDurationMs ?? DEFAULT_MIN_DURATION_MS;
  const maxDurationMs = options.maxDurationMs ?? DEFAULT_MAX_DURATION_MS;

  let previous = cloneState(actual);
  return history.map((point) => {
    const target = cloneState(point.target);
    const planned = planner(previous, target, point.durationMs);
    const durationMs = clampDuration(planned * durationScale, minDurationMs, maxDurationMs);
    const timeoutMs = point.timeoutMs > 0
      ? point.timeoutMs
      : Math.max(500, Math.round(durationMs * 0.75));
    previous = target;
    return { label: point.label, target, durationMs, timeoutMs };
  });
}

export function buildHardwareFrame(
  history: HistoryPoint[],
  index: number,
  actual: ArmState,
  planner: DurationPlanner,
  options: HardwarePlanOptions = {},
): HardwareFrame {
  const point = history[index];
  if (!point) throw new Error(`动作关键点 ${index + 1} 不存在`);
  const target = cloneState(point.target);
  const planned = planner(actual, target, point.durationMs);
  const durationMs = clampDuration(
    planned * (options.durationScale ?? 1),
    options.minDurationMs ?? DEFAULT_MIN_DURATION_MS,
    options.maxDurationMs ?? DEFAULT_MAX_DURATION_MS,
  );
  return {
    label: point.label,
    target,
    durationMs,
    timeoutMs: point.timeoutMs > 0 ? point.timeoutMs : Math.max(500, Math.round(durationMs * 0.75)),
  };
}

function clampDuration(value: number, minDurationMs: number, maxDurationMs: number): number {
  if (!Number.isFinite(value)) throw new Error('实机运动时长计算失败');
  return Math.max(minDurationMs, Math.min(maxDurationMs, Math.round(value)));
}