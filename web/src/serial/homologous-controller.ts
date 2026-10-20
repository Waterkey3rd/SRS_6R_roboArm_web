import {
  CONTROLLER_BAUD_RATE, CONTROLLER_HEARTBEAT_MS, CONTROLLER_PERIOD_MS,
  ControllerFrameParser, decodeControllerTarget, encodeControllerHeartbeat,
  encodeControllerJoints, encodeControllerPose, type ControllerDataType, type ControllerMode,
} from './controller-protocol';

// Minimal Web Serial surface; no browser extension or additional ROS node.
export interface ControllerSerialPort {
  readable: ReadableStream<Uint8Array> | null;
  writable: WritableStream<Uint8Array> | null;
  open(options: { baudRate: number; dataBits: 8; stopBits: 1; parity: 'none'; flowControl: 'none' }): Promise<void>;
  close(): Promise<void>;
}

export interface ControllerSerialApi {
  requestPort(): Promise<ControllerSerialPort>;
}

export type ControllerConnection = 'disconnected' | 'connecting' | 'connected' | 'disconnecting';

export interface ControllerCallbacks {
  // Return current simulation or fresh ROS feedback (rad or m/rad).
  getFollowTarget: (type: ControllerDataType) => readonly number[] | null;
  onActiveJoints: (joints: number[]) => void;
  onActivePose: (pose: number[]) => void;
  onState: () => void;
  onError: (error: Error) => void;
}

export class HomologousController {
  connection: ControllerConnection = 'disconnected';
  mode: ControllerMode = 'follow';
  dataType: ControllerDataType | null = null;
  identificationTimedOut = false;
  private identificationAt: number | null = null;
  private followType: ControllerDataType | null = null;
  private port: ControllerSerialPort | null = null;
  private reader: ReadableStreamDefaultReader<Uint8Array> | null = null;
  private writer: WritableStreamDefaultWriter<Uint8Array> | null = null;
  private readTask: Promise<void> | null = null;
  private writeTask: Promise<void> | null = null;
  private timer: ReturnType<typeof setInterval> | null = null;
  private parser = new ControllerFrameParser();
  private announcedMode: ControllerMode | null = null;
  private lastHeartbeatAt = -Infinity;
  private lastForwardAt = -Infinity;

  constructor(private readonly callbacks: ControllerCallbacks) {}

  get identifying(): boolean { return this.identificationAt !== null; }
  get feedbackType(): ControllerDataType | null { return this.followType; }

  identifyDataType(): void {
    if (this.connection !== 'connected') return;
    this.dataType = null;
    this.identificationTimedOut = false;
    this.identificationAt = performance.now();
    this.callbacks.onState();
    // Listen for the next valid data packet. No type/mode request is sent.
  }

  async connect(serial?: ControllerSerialApi): Promise<void> {
    if (this.connection !== 'disconnected') return;
    const api = serial ?? (navigator as Navigator & { serial?: ControllerSerialApi }).serial;
    if (!api) throw new Error('USB 串口需要桌面 Chrome/Edge，并通过 localhost 或 HTTPS 打开网页');
    this.connection = 'connecting';
    this.mode = 'follow';
    this.resetIdentification();
    this.callbacks.onState();
    let opened = false;
    try {
      // Keep requestPort directly in the button's user activation.
      const port = await api.requestPort();
      this.port = port;
      await port.open({ baudRate: CONTROLLER_BAUD_RATE, dataBits: 8, stopBits: 1, parity: 'none', flowControl: 'none' });
      opened = true;
      if (!port.readable || !port.writable) throw new Error('USB 串口缺少可读/可写通道');
      this.reader = port.readable.getReader();
      this.writer = port.writable.getWriter();
      this.parser.reset();
      this.announcedMode = null;
      this.lastHeartbeatAt = this.lastForwardAt = -Infinity;
      this.connection = 'connected';
      this.readTask = this.readLoop();
      this.timer = setInterval(() => this.pump(), CONTROLLER_PERIOD_MS);
      this.callbacks.onState();
      this.pump();
    } catch (error) {
      this.reader?.releaseLock();
      this.writer?.releaseLock();
      if (opened) await this.port?.close().catch(() => undefined);
      this.port = null;
      this.reader = this.writer = null;
      this.connection = 'disconnected';
      this.callbacks.onState();
      throw error;
    }
  }

  setMode(mode: ControllerMode): void {
    if (this.connection !== 'connected' || this.mode === mode) return;
    this.mode = mode;
    this.parser.reset();
    this.announcedMode = null;
    this.lastForwardAt = -Infinity;
    this.callbacks.onState();
    this.pump();
  }

  async disconnect(): Promise<void> {
    if (this.connection !== 'connected') return;
    this.connection = 'disconnecting';
    this.mode = 'follow';
    if (this.timer !== null) clearInterval(this.timer);
    this.timer = null;
    this.callbacks.onState();
    try {
      await this.reader?.cancel().catch(() => undefined);
      await this.readTask;
      // Abort a stalled write so unplug/disconnect cannot retain stream locks.
      await this.writer?.abort().catch(() => undefined);
      await this.writeTask;
      this.writer?.releaseLock();
      await this.port?.close();
    } catch (error) {
      this.callbacks.onError(asError(error));
    } finally {
      this.reader = this.writer = null;
      this.port = null;
      this.readTask = this.writeTask = null;
      this.parser.reset();
      this.announcedMode = null;
      this.resetIdentification();
      this.connection = 'disconnected';
      this.callbacks.onState();
    }
  }

  private pump(): void {
    if (this.identificationAt !== null && performance.now() - this.identificationAt >= 2000) {
      this.identificationAt = null;
      this.identificationTimedOut = true;
      this.callbacks.onState();
    }
    if (this.connection !== 'connected' || this.writeTask) return;
    // One writer and one periodic task; old joint targets are never queued.
    this.writeTask = this.writeTick().catch((error) => this.fail(error)).finally(() => {
      this.writeTask = null;
    });
  }

  private async writeTick(): Promise<void> {
    const now = performance.now();
    const mode = this.mode;
    if (this.announcedMode !== mode || now - this.lastHeartbeatAt >= CONTROLLER_HEARTBEAT_MS) {
      await this.writer!.write(encodeControllerHeartbeat(mode, now));
      this.announcedMode = mode;
      this.lastHeartbeatAt = now;
    }
    if (this.connection !== 'connected' || this.mode !== mode || mode !== 'follow') return;
    const type = this.followType;
    // Wait for the controller to announce its type before sending feedback.
    if (!type) return;
    const target = this.callbacks.getFollowTarget(type);
    if (target) await this.writer!.write(type === 'pose' ? encodeControllerPose(target) : encodeControllerJoints(target));
  }

  private async readLoop(): Promise<void> {
    const reader = this.reader!;
    try {
      while (this.connection === 'connected') {
        const { value, done } = await reader.read();
        if (done) break;
        if (!value) continue;
        for (const frame of this.parser.feed(value, performance.now())) {
          const target = decodeControllerTarget(frame);
          if (!target) continue;
          this.followType = target.type;
          if (this.dataType !== target.type || this.identifying || this.identificationTimedOut) {
            this.dataType = target.type;
            this.identificationAt = null;
            this.identificationTimedOut = false;
            this.callbacks.onState();
          }
          if (this.mode !== 'active' || this.announcedMode !== 'active') continue;
          const now = performance.now();
          if (now - this.lastForwardAt < CONTROLLER_PERIOD_MS) continue;
          this.lastForwardAt = now;
          if (target.type === 'joints') this.callbacks.onActiveJoints(target.values);
          else this.callbacks.onActivePose(target.values);
        }
      }
    } catch (error) {
      if (this.connection === 'connected') this.fail(error);
    } finally {
      reader.releaseLock();
      if (this.connection === 'connected') this.fail(new Error('USB 串口已断开'));
    }
  }

  private fail(error: unknown): void {
    if (this.connection !== 'connected') return;
    this.callbacks.onError(asError(error));
    void this.disconnect();
  }

  private resetIdentification(): void {
    this.dataType = null;
    this.followType = null;
    this.identificationAt = null;
    this.identificationTimedOut = false;
  }
}

function asError(error: unknown): Error {
  return error instanceof Error ? error : new Error(String(error));
}
