import { cloneState, type ArmState } from '../types';

/** Fixed-grid snapshots. A missed slot ends the take instead of inventing motion. */
export class MotionRecorder {
  readonly samples: ArmState[] = [];
  private nextAt: number;

  constructor(readonly intervalMs: number, initial: ArmState, startedAt: number) {
    if (!Number.isInteger(intervalMs) || intervalMs < 50 || intervalMs > 2000) {
      throw new Error('采样间隔必须为 50–2000 ms 的整数');
    }
    this.nextAt = startedAt + intervalMs;
    this.samples.push(this.snapshot(initial));
  }

  get durationMs(): number { return (this.samples.length - 1) * this.intervalMs; }

  capture(state: ArmState | null, now: number): string | null {
    if (now < this.nextAt) return null;
    if (now - this.nextAt >= this.intervalMs) return '采样定时中断，请保持页面在前台';
    if (!state) return '实机反馈已断开或过期';
    if (this.samples.length >= 10000) return '已达到 10000 个采样点上限';
    try {
      this.samples.push(this.snapshot(state));
    } catch {
      return '收到无效关节反馈';
    }
    this.nextAt += this.intervalMs;
    return null;
  }

  private snapshot(state: ArmState): ArmState {
    if (state.right.length !== 6 || state.right.some(value => !Number.isFinite(value))) {
      throw new Error('录包需要 6 个有效关节角');
    }
    return cloneState(state);
  }
}
