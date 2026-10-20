import type { ArmState } from '../types';
import { ARM_MASK_SINGLE, OUTCOME_ACK, OUTCOME_SUPERSEDED, type CommandResultJson } from '../ros/types';
import { ArmWebClient, commandResultText } from '../ros/arm-web-client';
import type { HardwareFrame } from './hardware-plan';

export interface HardwareExecutionOptions {
  onProgress?: (completed: number, total: number, label: string) => void;
}

interface AbortDeferred {
  promise: Promise<never>;
  reject: (error: Error) => void;
}

export class HardwareExecutor {
  private running = false;
  private aborted = false;
  private abortReason = '';
  private abortDeferred: AbortDeferred | null = null;

  constructor(private readonly client: ArmWebClient) {}

  get isRunning(): boolean {
    return this.running;
  }

  get wasAborted(): boolean {
    return this.aborted;
  }

  assertReady(): string | null {
    const check = this.client.readyCheck();
    return check.ok ? null : check.reason;
  }

  async executeSingle(target: ArmState, requestedDurationMs: number): Promise<void> {
    if (this.running) throw new Error('已有实机动作正在执行');
    this.beginRun();
    try {
      await this.sendFrame(target, requestedDurationMs);
    } finally {
      this.finishRun();
    }
  }

  async executePlan(frames: HardwareFrame[], options: HardwareExecutionOptions = {}): Promise<void> {
    if (this.running) throw new Error('已有实机动作正在执行');
    if (frames.length === 0) throw new Error('实机动作序列为空');

    this.beginRun();
    try {
      for (let index = 0; index < frames.length; index += 1) {
        this.ensureNotAborted();
        const frame = frames[index];
        options.onProgress?.(index, frames.length, frame.label);
        await this.sendFrame(frame.target, frame.durationMs);
        await this.wait(frame.durationMs);
        options.onProgress?.(index + 1, frames.length, frame.label);
      }
    } finally {
      this.finishRun();
    }
  }

  abort(reason = '用户取消'): void {
    if (!this.running) {
      this.abortReason = reason;
      return;
    }
    this.aborted = true;
    this.abortReason = reason;
    const deferred = this.abortDeferred;
    if (deferred) {
      deferred.reject(new Error(reason));
    }
  }

  private async sendFrame(target: ArmState, requestedDurationMs: number): Promise<void> {
    this.ensureNotAborted();
    const reason = this.assertReady();
    if (reason) throw new Error(`实机未就绪：${reason}`);

    const durationMs = clampDuration(requestedDurationMs);
    const clientId = this.client.publishServo(target, ARM_MASK_SINGLE, durationMs);
    const result = await this.raceWithAbort(
      this.client.waitForCommandResult(clientId, Math.max(1500, durationMs + 800)),
    );
    assertCommandResult(result);
  }

  private async wait(durationMs: number): Promise<void> {
    this.ensureNotAborted();
    await this.raceWithAbort(new Promise<void>((resolve) => {
      window.setTimeout(resolve, Math.max(0, durationMs));
    }));
    this.ensureNotAborted();
  }

  private beginRun(): void {
    this.running = true;
    this.aborted = false;
    this.abortReason = '';
    let reject!: (error: Error) => void;
    const promise = new Promise<never>((_, rejectPromise) => {
      reject = rejectPromise;
    });
    promise.catch(() => undefined);
    this.abortDeferred = { promise, reject };
  }

  private finishRun(): void {
    this.running = false;
    this.abortDeferred = null;
  }

  private ensureNotAborted(): void {
    if (this.aborted) throw new Error(this.abortReason || '实机动作已取消');
  }

  private raceWithAbort<T>(promise: Promise<T>): Promise<T> {
    const deferred = this.abortDeferred;
    return deferred ? Promise.race([promise, deferred.promise]) : promise;
  }
}

function clampDuration(value: number): number {
  if (!Number.isFinite(value) || value <= 0) throw new Error('运动时长必须大于 0');
  return Math.max(20, Math.min(60000, Math.round(value)));
}

function assertCommandResult(result: CommandResultJson): void {
  const outcome = Number(result.outcome);
  const mcuResult = Number(result.mcu_result);

  if (outcome === OUTCOME_SUPERSEDED) {
    throw new Error(`实机目标被更新值替代：${result.message ?? ''}`);
  }
  if (outcome !== OUTCOME_ACK || (Number.isFinite(mcuResult) && mcuResult !== 0)) {
    throw new Error(`实机拒绝：${commandResultText(result)}`);
  }
}