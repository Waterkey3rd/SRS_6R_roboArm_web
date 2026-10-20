import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import {
  CONTROLLER_HEARTBEAT, CONTROLLER_JOINTS, CONTROLLER_POSE, ControllerFrameParser,
  decodeControllerJoints, decodeControllerTarget, encodeControllerHeartbeat, encodeControllerJoints, encodeControllerPose,
} from '../src/serial/controller-protocol';
import { HomologousController, type ControllerSerialPort } from '../src/serial/homologous-controller';
import { ArmWebClient } from '../src/ros/arm-web-client';
import { rpyMatrix } from '../src/math';

const q = [0, 0.25, -0.5, 0.75, -1, 1.25];
const pose = [0.25, 0, 0.5, 0, 0, 0];

describe('controller USB wire protocol', () => {
  it('matches documented little-endian heartbeat and six-float joint vectors', () => {
    expect([...encodeControllerHeartbeat('active', 0x12345678)])
      .toEqual([0xa5, 12, 0, 1, 0, 0x78, 0x56, 0x34, 0x12, 1, 0x15, 0x5a]);
    const frame = encodeControllerJoints(q);
    expect(frame.length).toBe(31);
    expect([...frame.slice(0, 5)]).toEqual([0xa5, 31, 0, 2, 0]);
    expect([...frame.slice(9, 13)]).toEqual([0, 0, 0x80, 0x3e]); // 0.25f
    const decoded = new ControllerFrameParser().feed(frame, 0);
    expect(decodeControllerJoints(decoded[0])).toEqual(q);
  });

  it('handles fragmentation, coalescing, junk, invalid lengths and corrupt checksums', () => {
    const parser = new ControllerFrameParser();
    const bytes = encodeControllerJoints(q);
    const corrupt = bytes.slice();
    corrupt[29] ^= 1;
    expect(parser.feed(new Uint8Array([9, 8, 0xa5, 255, 255, 2, 0, ...corrupt, ...bytes.slice(0, 11)]), 0)).toEqual([]);
    const frames = parser.feed(new Uint8Array([...bytes.slice(11), ...encodeControllerHeartbeat('follow', 100), ...bytes]), 20);
    expect(frames.map(f => f.command)).toEqual([CONTROLLER_JOINTS, CONTROLLER_HEARTBEAT, CONTROLLER_JOINTS]);
    expect(decodeControllerJoints(frames[2])).toEqual(q);
  });

  it('discards expired partial frames and non-finite or seven-axis data', () => {
    const parser = new ControllerFrameParser();
    const frame = encodeControllerJoints(q);
    parser.feed(frame.slice(0, 10), 0);
    expect(parser.feed(frame.slice(10), 100)).toEqual([]);
    expect(parser.feed(frame, 101)).toHaveLength(1);
    expect(() => encodeControllerJoints(Array(7).fill(0))).toThrow();
    expect(() => encodeControllerJoints([0, 0, 0, 0, 0, Infinity])).toThrow();
    expect(() => encodeControllerJoints([0, 0, 0, 0, 0, 1e40])).toThrow();
    const payload = new Uint8Array(24);
    new DataView(payload.buffer).setFloat32(0, NaN, true);
    expect(decodeControllerJoints({ command: 2, payload })).toBeNull();
    expect(decodeControllerTarget({ command: 3, payload })).toBeNull();
  });

  it('distinguishes poses by command, preserves m/rad values and accepts mixed fragmented frames', () => {
    const frame = encodeControllerPose(pose);
    expect([...frame]).toEqual([
      0xa5, 31, 0, 3, 0, 0, 0, 0x80, 0x3e, 0, 0, 0, 0,
      0, 0, 0, 0x3f, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xfd, 0x5a,
    ]);
    const parser = new ControllerFrameParser();
    expect(parser.feed(frame.slice(0, 13), 0)).toEqual([]);
    const frames = parser.feed(new Uint8Array([...frame.slice(13), ...encodeControllerJoints(q)]), 1);
    expect(frames.map(f => f.command)).toEqual([CONTROLLER_POSE, CONTROLLER_JOINTS]);
    expect(decodeControllerTarget(frames[0])).toEqual({ type: 'pose', values: pose });
    expect(decodeControllerJoints(frames[0])).toBeNull();
    expect(decodeControllerTarget(frames[1])).toEqual({ type: 'joints', values: q });
  });
});

class FakePort implements ControllerSerialPort {
  input!: ReadableStreamDefaultController<Uint8Array>;
  writes: Uint8Array[] = [];
  readable = new ReadableStream<Uint8Array>({ start: c => { this.input = c; } });
  writable = new WritableStream<Uint8Array>({ write: bytes => { this.writes.push(bytes.slice()); } });
  open = vi.fn(async () => undefined);
  close = vi.fn(async () => undefined);
  frames() {
    const parser = new ControllerFrameParser();
    return this.writes.flatMap(bytes => parser.feed(bytes, 0));
  }
}

describe('controller USB forwarding', () => {
  let port: FakePort;
  let usb: HomologousController;
  let follow: number[] | null;
  let followPose: number[] | null;
  let active: ReturnType<typeof vi.fn>;
  let activePose: ReturnType<typeof vi.fn>;
  let error: ReturnType<typeof vi.fn>;

  beforeEach(() => {
    vi.useFakeTimers({ toFake: ['setInterval', 'clearInterval', 'performance'] });
    port = new FakePort();
    follow = q;
    followPose = [0.5, 0.25, 0.75, 0.5, -0.25, 0.75];
    active = vi.fn();
    activePose = vi.fn();
    error = vi.fn();
    usb = new HomologousController({ getFollowTarget: type => type === 'pose' ? followPose : follow, onActiveJoints: active, onActivePose: activePose, onState: vi.fn(), onError: error });
  });

  afterEach(async () => {
    await usb.disconnect();
    vi.useRealTimers();
  });

  async function connect(announce = true) {
    await usb.connect({ requestPort: async () => port });
    await vi.advanceTimersByTimeAsync(0);
    if (announce) {
      port.input.enqueue(encodeControllerJoints(q));
      await vi.advanceTimersByTimeAsync(0);
      await vi.advanceTimersByTimeAsync(50);
    }
  }

  it('opens the selected port in default follow, sends mode heartbeats and only fresh feedback', async () => {
    await connect();
    expect(port.open).toHaveBeenCalledWith({ baudRate: 921600, dataBits: 8, stopBits: 1, parity: 'none', flowControl: 'none' });
    expect(usb.mode).toBe('follow');
    expect(port.frames().map(f => f.command)).toEqual([1, 2]);
    expect(port.frames()[0].payload[4]).toBe(0);
    expect(decodeControllerJoints(port.frames()[1])).toEqual(q);
    port.input.enqueue(encodeControllerJoints(q));
    await vi.advanceTimersByTimeAsync(0);
    expect(active).not.toHaveBeenCalled();
    port.writes = [];
    follow = null;
    await vi.advanceTimersByTimeAsync(200);
    expect(port.frames().map(f => f.command)).toEqual([1, 1]);
    follow = q.map(v => v + 0.5);
    await vi.advanceTimersByTimeAsync(50);
    expect(decodeControllerJoints(port.frames().at(-1)!)).toEqual(follow);
  });

  it('announces active mode before forwarding uploads, limits to 20Hz and stops echoing feedback', async () => {
    await connect();
    port.writes = [];
    usb.setMode('active');
    await vi.advanceTimersByTimeAsync(0);
    expect(port.frames()[0].payload[4]).toBe(1);
    port.input.enqueue(encodeControllerJoints(q));
    port.input.enqueue(encodeControllerJoints(q));
    await vi.advanceTimersByTimeAsync(0);
    expect(active).toHaveBeenCalledExactlyOnceWith(q);
    await vi.advanceTimersByTimeAsync(100);
    port.input.enqueue(encodeControllerJoints(q));
    await vi.advanceTimersByTimeAsync(0);
    expect(active).toHaveBeenCalledTimes(2);
    expect(port.frames().every(f => f.command === 1 && f.payload[4] === 1)).toBe(true);
    usb.setMode('follow');
    await vi.advanceTimersByTimeAsync(0);
    expect(port.frames().at(-2)?.payload[4]).toBe(0);
    expect(port.frames().at(-1)?.command).toBe(2);
    port.input.enqueue(encodeControllerJoints(q));
    await vi.advanceTimersByTimeAsync(0);
    expect(active).toHaveBeenCalledTimes(2);
  });

  it('closes unlocked streams and reconnects in follow mode', async () => {
    await connect();
    usb.setMode('active');
    await vi.advanceTimersByTimeAsync(0);
    await usb.disconnect();
    expect(usb.connection).toBe('disconnected');
    expect(usb.mode).toBe('follow');
    expect(port.readable.locked).toBe(false);
    expect(port.writable.locked).toBe(false);
    expect(port.close).toHaveBeenCalledOnce();
    port = new FakePort();
    await connect(false);
    expect(port.frames()[0].payload[4]).toBe(0);
    expect(usb.dataType).toBeNull();
  });

  it('waits for a type declaration, then returns actual feedback in the matching space', async () => {
    await connect(false);
    await vi.advanceTimersByTimeAsync(100);
    expect(port.frames().every(f => f.command === CONTROLLER_HEARTBEAT)).toBe(true);
    expect(usb.feedbackType).toBeNull();
    port.writes = [];
    port.input.enqueue(encodeControllerPose(pose));
    await vi.advanceTimersByTimeAsync(50);
    const outgoingPose = port.frames().find(f => f.command === CONTROLLER_POSE)!;
    expect(decodeControllerTarget(outgoingPose)).toEqual({ type: 'pose', values: followPose });
    expect(decodeControllerTarget(outgoingPose)?.values).not.toEqual(pose);
    expect(usb.feedbackType).toBe('pose');
    expect(port.frames().some(f => f.command === CONTROLLER_JOINTS)).toBe(false);
    expect(activePose).not.toHaveBeenCalled();
    port.writes = [];
    port.input.enqueue(encodeControllerJoints(q.map(v => v + 0.5)));
    await vi.advanceTimersByTimeAsync(50);
    expect(port.frames().some(f => f.command === CONTROLLER_POSE)).toBe(false);
    expect(decodeControllerJoints(port.frames().find(f => f.command === CONTROLLER_JOINTS)!)).toEqual(follow);
    expect(usb.feedbackType).toBe('joints');
  });

  it('keeps the pose feedback type across mode switches and re-identification, but clears it on reconnect', async () => {
    await connect();
    usb.setMode('active');
    await vi.advanceTimersByTimeAsync(0);
    port.input.enqueue(encodeControllerPose(pose));
    await vi.advanceTimersByTimeAsync(0);
    port.writes = [];
    usb.setMode('follow');
    await vi.advanceTimersByTimeAsync(0);
    expect(port.frames().at(-1)?.command).toBe(CONTROLLER_POSE);
    usb.identifyDataType();
    expect(usb.dataType).toBeNull();
    expect(usb.feedbackType).toBe('pose');
    port.writes = [];
    await vi.advanceTimersByTimeAsync(2000);
    expect(usb.identificationTimedOut).toBe(true);
    expect(port.frames().some(f => f.command === CONTROLLER_JOINTS)).toBe(false);
    expect(port.frames().some(f => f.command === CONTROLLER_POSE)).toBe(true);
    followPose = null;
    port.writes = [];
    await vi.advanceTimersByTimeAsync(200);
    expect(port.frames().every(f => f.command === CONTROLLER_HEARTBEAT)).toBe(true);
    await usb.disconnect();
    port = new FakePort();
    await connect(false);
    expect(usb.feedbackType).toBeNull();
    expect(port.frames().every(f => f.command === CONTROLLER_HEARTBEAT)).toBe(true);
  });

  it('identifies the next valid upload locally without sending a request or changing mode', async () => {
    await connect();
    port.writes = [];
    usb.identifyDataType();
    expect(usb.identifying).toBe(true);
    expect(usb.dataType).toBeNull();
    expect(usb.mode).toBe('follow');
    await vi.advanceTimersByTimeAsync(0);
    expect(port.writes).toEqual([]);
    port.input.enqueue(encodeControllerPose(pose));
    await vi.advanceTimersByTimeAsync(0);
    expect(usb.dataType).toBe('pose');
    expect(usb.identifying).toBe(false);
    expect(activePose).not.toHaveBeenCalled(); // identification in follow cannot move the robot
    expect(port.writes).toEqual([]);
    port.input.enqueue(encodeControllerJoints(q));
    await vi.advanceTimersByTimeAsync(0);
    expect(usb.dataType).toBe('joints'); // controller can change type at any time
    expect(port.writes).toEqual([]);
  });

  it('times out identification on silence or invalid frames, then accepts a later valid upload', async () => {
    await connect();
    usb.identifyDataType();
    const bad = encodeControllerPose(pose);
    bad[29] ^= 1;
    port.input.enqueue(bad);
    await vi.advanceTimersByTimeAsync(2000);
    expect(usb.identifying).toBe(false);
    expect(usb.identificationTimedOut).toBe(true);
    expect(usb.dataType).toBeNull();
    port.input.enqueue(encodeControllerPose(pose));
    await vi.advanceTimersByTimeAsync(0);
    expect(usb.dataType).toBe('pose');
    expect(usb.identificationTimedOut).toBe(false);
    expect(usb.mode).toBe('follow');
  });

  it('routes active pose and joint uploads by each packet type while maintaining the shared rate limit', async () => {
    await connect();
    usb.setMode('active');
    await vi.advanceTimersByTimeAsync(0);
    port.input.enqueue(encodeControllerPose(pose));
    port.input.enqueue(encodeControllerJoints(q));
    await vi.advanceTimersByTimeAsync(0);
    expect(activePose).toHaveBeenCalledExactlyOnceWith(pose);
    expect(active).not.toHaveBeenCalled();
    expect(usb.dataType).toBe('joints'); // detection is independent from rate limiting
    await vi.advanceTimersByTimeAsync(50);
    port.input.enqueue(encodeControllerJoints(q));
    await vi.advanceTimersByTimeAsync(0);
    expect(active).toHaveBeenCalledExactlyOnceWith(q);
    expect(usb.mode).toBe('active');
  });

  it('handles an unplug and a cancelled port picker without leaving timers or locks', async () => {
    await expect(usb.connect({ requestPort: async () => { throw new Error('cancel'); } })).rejects.toThrow('cancel');
    expect(usb.connection).toBe('disconnected');
    await connect();
    port.input.error(new Error('unplugged'));
    await vi.advanceTimersByTimeAsync(0);
    expect(error).toHaveBeenCalled();
    expect(usb.connection).toBe('disconnected');
    expect(port.close).toHaveBeenCalledOnce();
    const count = port.writes.length;
    await vi.advanceTimersByTimeAsync(500);
    expect(port.writes.length).toBe(count);
  });
});

describe('controller angles reuse existing ROS compatibility node', () => {
  it('builds m/rad TCP feedback from fresh measured ROS joints and rejects stale/disconnected feedback', () => {
    const client = new ArmWebClient('ws://localhost:9090');
    const connected = vi.spyOn(client, 'isConnected', 'get').mockReturnValue(true);
    client.latestArmState = { left: Array(6).fill(0), right: q };
    client.jointStateAtMs = Date.now();
    const fk = vi.fn(() => ({ position: [250, -500, 750], rotation: rpyMatrix(0.75, -0.25, 0.5) }));
    expect(client.getControllerFeedback('joints', { fk })).toEqual(q);
    expect(fk).not.toHaveBeenCalled();
    const result = client.getControllerFeedback('pose', { fk })!;
    expect(result.slice(0, 3)).toEqual([0.25, -0.5, 0.75]);
    [0.5, -0.25, 0.75].forEach((value, index) => expect(result[index + 3]).toBeCloseTo(value));
    expect(fk).toHaveBeenCalledExactlyOnceWith('right', q);
    client.jointStateAtMs = Date.now() - 301;
    expect(client.getControllerFeedback('pose', { fk })).toBeNull();
    expect(client.getControllerFeedback('joints', { fk })).toBeNull();
    client.jointStateAtMs = Date.now();
    connected.mockReturnValue(false);
    expect(client.getControllerFeedback('pose', { fk })).toBeNull();
    expect(fk).toHaveBeenCalledTimes(1);
  });
  it.each(['web_bridge', 'ros2_real'] as const)('publishes through the existing %s topic and validates readiness', (profile) => {
    const client = new ArmWebClient('ws://localhost:9090', profile);
    const ready = vi.spyOn(client, 'readyCheck').mockReturnValue({ ok: true, reason: 'ready' });
    const publish = vi.spyOn(client.bridge, 'publish').mockReturnValue(true);
    expect(client.publishControllerJoints(q)).toBe(true);
    if (profile === 'web_bridge') {
      expect(publish.mock.calls[0][0]).toBe('/arm_web/servo_target');
      expect(JSON.parse((publish.mock.calls[0][1] as { data: string }).data))
        .toMatchObject({ right: q, arm_mask: 1, duration_ms: 100, source: 'controller_usb' });
    } else {
      expect(publish).toHaveBeenCalledWith('/joint_command', {
        name: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'], position: q,
      }, 'sensor_msgs/msg/JointState');
    }
    ready.mockReturnValue({ ok: false, reason: 'stale' });
    expect(client.publishControllerJoints(q)).toBe(false);
    expect(publish).toHaveBeenCalledTimes(1);
  });

  it('forwards poses unchanged in m/rad through the same compatibility topic', () => {
    const client = new ArmWebClient('ws://localhost:9090');
    const ready = vi.spyOn(client, 'readyCheck').mockReturnValue({ ok: true, reason: 'ready' });
    const publish = vi.spyOn(client.bridge, 'publish').mockReturnValue(true);
    const solver = vi.fn();
    expect(client.publishControllerPose(pose, solver)).toBe(true);
    expect(publish.mock.calls[0][0]).toBe('/arm_web/servo_target');
    expect(JSON.parse((publish.mock.calls[0][1] as { data: string }).data)).toMatchObject({
      right: pose, space: 'CartesianSpace', reference_frame: 0, duration_ms: 100, arm_mask: 1, source: 'controller_usb',
    });
    expect(solver).not.toHaveBeenCalled();
    ready.mockReturnValue({ ok: false, reason: 'stale' });
    expect(client.publishControllerPose(pose)).toBe(false);
    expect(publish).toHaveBeenCalledTimes(1);
    expect(() => client.publishControllerPose([0, 0, 0, 0, 0, NaN])).toThrow();
  });

  it('uses the supplied current-model IK only for the legacy JointState-only profile', () => {
    const client = new ArmWebClient('ws://localhost:9090', 'ros2_real');
    vi.spyOn(client, 'readyCheck').mockReturnValue({ ok: true, reason: 'ready' });
    const publish = vi.spyOn(client.bridge, 'publish').mockReturnValue(true);
    const solver = vi.fn(() => q);
    expect(client.publishControllerPose(pose, solver)).toBe(true);
    expect(solver).toHaveBeenCalledExactlyOnceWith(pose);
    expect(publish).toHaveBeenCalledWith('/joint_command', {
      name: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'], position: q,
    }, 'sensor_msgs/msg/JointState');
  });
});
