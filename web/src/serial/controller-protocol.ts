// Same envelope as HostComputer. Target type is carried by the command ID.
export type ControllerMode = 'follow' | 'active';
export type ControllerDataType = 'joints' | 'pose';
export type ControllerTarget = { type: ControllerDataType; values: number[] };
// Web Serial requires a baudRate. Native STM32 USB CDC uses this as line-coding
// metadata; it does not set USB transfer speed unless firmware applies it.
export const CONTROLLER_BAUD_RATE = 921600;
export const CONTROLLER_PERIOD_MS = 50;
export const CONTROLLER_HEARTBEAT_MS = 100;
export const CONTROLLER_JOINTS = 0x02;
export const CONTROLLER_POSE = 0x03;
export const CONTROLLER_HEARTBEAT = 0x01;

export interface ControllerFrame {
  command: number;
  payload: Uint8Array;
}

function encode(command: number, payload: Uint8Array): Uint8Array {
  const frame = new Uint8Array(payload.length + 7);
  const view = new DataView(frame.buffer);
  frame[0] = 0xa5;
  view.setUint16(1, frame.length, true);
  view.setUint16(3, command, true);
  frame.set(payload, 5);
  frame[frame.length - 2] = payload.reduce((sum, byte) => (sum + byte) & 0xff, 0);
  frame[frame.length - 1] = 0x5a;
  return frame;
}

export function encodeControllerHeartbeat(mode: ControllerMode, uptimeMs: number): Uint8Array {
  const payload = new Uint8Array(5);
  new DataView(payload.buffer).setUint32(0, uptimeMs >>> 0, true);
  payload[4] = mode === 'active' ? 1 : 0;
  return encode(CONTROLLER_HEARTBEAT, payload);
}

export function encodeControllerJoints(joints: readonly number[]): Uint8Array {
  return encodeVector(CONTROLLER_JOINTS, joints);
}

// Base-frame absolute TCP pose: [x, y, z, yaw, pitch, roll], m/rad.
export function encodeControllerPose(pose: readonly number[]): Uint8Array {
  return encodeVector(CONTROLLER_POSE, pose);
}

function encodeVector(command: number, values: readonly number[]): Uint8Array {
  if (values.length !== 6 || values.some(q => !Number.isFinite(Math.fround(q)))) {
    throw new Error('USB 目标必须是 6 个有限 float32；关节角 rad，位姿 m/rad');
  }
  const payload = new Uint8Array(24);
  const view = new DataView(payload.buffer);
  values.forEach((q, index) => view.setFloat32(index * 4, q, true));
  return encode(command, payload);
}

export function decodeControllerJoints(frame: ControllerFrame): number[] | null {
  const target = decodeControllerTarget(frame);
  return target?.type === 'joints' ? target.values : null;
}

export function decodeControllerTarget(frame: ControllerFrame): ControllerTarget | null {
  if ((frame.command !== CONTROLLER_JOINTS && frame.command !== CONTROLLER_POSE) || frame.payload.length !== 24) return null;
  const view = new DataView(frame.payload.buffer, frame.payload.byteOffset, frame.payload.byteLength);
  const values = Array.from({ length: 6 }, (_, index) => view.getFloat32(index * 4, true));
  return values.every(Number.isFinite) ? { type: frame.command === CONTROLLER_JOINTS ? 'joints' : 'pose', values } : null;
}

export class ControllerFrameParser {
  private buffer: number[] = [];
  private lastByteAt = 0;

  reset(): void {
    this.buffer = [];
    this.lastByteAt = 0;
  }

  feed(bytes: Uint8Array, nowMs: number): ControllerFrame[] {
    if (nowMs - this.lastByteAt >= 100) this.reset();
    this.lastByteAt = nowMs;
    const frames: ControllerFrame[] = [];
    // Parse as bytes arrive so even a large chunk cannot grow the buffer.
    for (const byte of bytes) {
      this.buffer.push(byte);
      while (this.buffer.length) {
        if (this.buffer[0] !== 0xa5) { this.buffer.shift(); continue; }
        if (this.buffer.length < 5) break;
        const length = this.buffer[1] | (this.buffer[2] << 8);
        const command = this.buffer[3] | (this.buffer[4] << 8);
        const expected = command === CONTROLLER_HEARTBEAT ? 12
          : command === CONTROLLER_JOINTS || command === CONTROLLER_POSE ? 31 : 0;
        if (length !== expected || expected === 0) { this.buffer.shift(); continue; }
        if (this.buffer.length < length) break;
        const payload = new Uint8Array(this.buffer.slice(5, length - 2));
        const checksum = payload.reduce((sum, value) => (sum + value) & 0xff, 0);
        if (this.buffer[length - 1] !== 0x5a || this.buffer[length - 2] !== checksum) {
          this.buffer.shift();
          continue;
        }
        frames.push({ command, payload });
        this.buffer.splice(0, length);
      }
    }
    return frames;
  }
}
