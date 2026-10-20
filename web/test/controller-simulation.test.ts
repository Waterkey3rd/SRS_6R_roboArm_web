import { readFile } from 'node:fs/promises';
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import { RoboArmController } from '../src/controller';
import { DeployIK } from '../src/wasm';
import { App } from '../src/ui/app';
import { cloneState } from '../src/types';
import { EVENT_LINK_LOST } from '../src/ros/types';
import { ControllerFrameParser, decodeControllerTarget, encodeControllerJoints, encodeControllerPose } from '../src/serial/controller-protocol';
import type { ControllerSerialPort, HomologousController } from '../src/serial/homologous-controller';

vi.mock('../src/ui/components/toast', () => ({ showToast: vi.fn() }));

class FakePort implements ControllerSerialPort {
  input!: ReadableStreamDefaultController<Uint8Array>;
  writes: Uint8Array[] = [];
  readable = new ReadableStream<Uint8Array>({ start: c => { this.input = c; } });
  writable = new WritableStream<Uint8Array>({ write: bytes => { this.writes.push(bytes.slice()); } });
  async open(): Promise<void> {}
  async close(): Promise<void> {}
  frames() {
    const parser = new ControllerFrameParser();
    return this.writes.flatMap(bytes => parser.feed(bytes, 0));
  }
}

describe('USB simulation without ROS', () => {
  let controller: RoboArmController;
  let app: any;
  let usb: HomologousController;
  let port: FakePort;
  const q = [0.2, -0.3, 0.4, 0.25, -0.2, 0.15];

  beforeEach(async () => {
    vi.useFakeTimers({ toFake: ['setInterval', 'clearInterval', 'performance'] });
    vi.stubGlobal('window', { setInterval, location: { search: '', protocol: 'http:', hostname: 'localhost' }, localStorage: { getItem: () => null } });
    vi.stubGlobal('localStorage', { getItem: () => null });
    vi.stubGlobal('cancelAnimationFrame', vi.fn());
    const wasmBase = new URL('../public/wasm/', import.meta.url).href;
    const wasmBinary = await readFile(new URL('../public/wasm/deploy_ik.wasm', import.meta.url));
    const ik = await DeployIK.load(wasmBase, { wasmBinary });
    controller = new (RoboArmController as any)(ik, wasmBase);
    app = new (App as any)({}, controller);
    app.topBar = { setRosStatus: vi.fn(), setControllerStatus: vi.fn(), setControllerDataType: vi.fn(), setMode: vi.fn(), setHardwareRunning: vi.fn(), setRecordingStatus: vi.fn() };
    app.controlPanel = { setHardwareStatus: vi.fn(), syncState: vi.fn(), isFollowEnabled: () => true };
    app.viewportHud = { setSource: vi.fn(), update: vi.fn(), showMotionProgress: vi.fn() };
    app.timeline = { setPlayState: vi.fn(), updateTimeline: vi.fn() };
    app.renderer = { update: vi.fn() };
    app.showControllerError = vi.fn();
    app.setupRos();
    controller.subscribe(() => app.syncAllViews());
    usb = app.controllerUsb;
    port = new FakePort();
    vi.spyOn(app.rosClient, 'publishControllerJoints');
    vi.spyOn(app.rosClient, 'publishControllerPose');
    await usb.connect({ requestPort: async () => port });
    await vi.advanceTimersByTimeAsync(0);
  });

  afterEach(async () => {
    app?.stopRecording();
    await usb?.disconnect();
    vi.clearAllTimers();
    vi.useRealTimers();
    vi.unstubAllGlobals();
  });

  async function upload(frame: Uint8Array) {
    port.input.enqueue(frame);
    await vi.advanceTimersByTimeAsync(50);
  }

  it('defaults to follow and sends the displayed simulation, including in-flight joint/pose states', async () => {
    expect(app.rosClient.isConnected).toBe(false);
    expect(usb.mode).toBe('follow');
    expect(port.frames()).toHaveLength(1); // Only heartbeat until the board declares its type.
    expect(port.frames()[0].payload[4]).toBe(0);
    controller.commandJoints({ right: [20, -30, 40, 25, -20, 15] } as any);
    // The rendered animation is still between its endpoints; feedback must match it.
    app.renderState({ ...cloneState(controller.current), right: q });
    await upload(encodeControllerJoints(Array(6).fill(0)));
    const jointFeedback = decodeControllerTarget(port.frames().at(-1)!);
    expect(jointFeedback?.type).toBe('joints');
    jointFeedback!.values.forEach((value, i) => expect(value).toBeCloseTo(q[i], 6));
    expect(controller.current.right[0]).toBeCloseTo(20 * Math.PI / 180);

    await upload(encodeControllerPose([0, 0, 0, 0, 0, 0]));
    const feedback = decodeControllerTarget(port.frames().at(-1)!)!;
    expect(feedback.type).toBe('pose');
    const fk = controller.ik.fk('right', q);
    feedback.values.slice(0, 3).forEach((value, i) => expect(value).toBeCloseTo(fk.position[i] / 1000, 6));
    expect(app.rosClient.publishControllerJoints).not.toHaveBeenCalled();
    expect(app.rosClient.publishControllerPose).not.toHaveBeenCalled();
  });

  it('accepts joint uploads as slave simulation, keeps history, and survives ROS offline/events', async () => {
    controller.commandJoints({ right: Array(6).fill(0) } as any);
    const history = structuredClone(controller.history);
    app.isPlaying = true;
    app.isAnimating = true;
    app.toggleControllerMode();
    await vi.advanceTimersByTimeAsync(0);
    expect(usb.mode).toBe('active');
    expect(app.isPlaying).toBe(false);
    expect(app.isAnimating).toBe(false);
    expect(port.frames().at(-1)!.payload[4]).toBe(1);
    await upload(encodeControllerJoints(q));
    controller.current.right.forEach((value, i) => expect(value).toBeCloseTo(q[i], 6));
    expect(app.renderer.update).toHaveBeenLastCalledWith(controller.current);
    expect(controller.history).toEqual(history);
    app.executeJoint({ right: Array(6).fill(0) }, 100);
    app.startPlayback();
    expect(controller.history).toEqual(history);
    expect(app.isPlaying).toBe(false);
    await vi.advanceTimersByTimeAsync(600); // Periodic ROS readiness update must not reset simulation.
    app.handleRosEvent({ type: EVENT_LINK_LOST });
    expect(usb.mode).toBe('active');
    expect(app.viewportHud.setSource).toHaveBeenLastCalledWith('USB 从动仿真', false);
    // A background ROS connection also must not turn local simulation into hardware control.
    vi.spyOn(app.rosClient, 'isConnected', 'get').mockReturnValue(true);
    vi.spyOn(app.rosClient, 'readyCheck').mockReturnValue({ ok: true, reason: '' });
    await upload(encodeControllerJoints(q));
    expect(app.rosClient.publishControllerJoints).not.toHaveBeenCalled();
    expect(app.rosClient.publishControllerPose).not.toHaveBeenCalled();

    app.toggleControllerMode();
    await upload(encodeControllerJoints(Array(6).fill(0)));
    controller.current.right.forEach((value, i) => expect(value).toBeCloseTo(q[i], 6));
    expect(usb.mode).toBe('follow');
    const feedback = decodeControllerTarget(port.frames().at(-1)!)!;
    feedback.values.forEach((value, i) => expect(value).toBeCloseTo(q[i], 6));
  });

  it('solves uploaded m/rad poses using local IK and preserves the last valid pose on bad targets', async () => {
    app.toggleControllerMode();
    await vi.advanceTimersByTimeAsync(0);
    await upload(encodeControllerJoints(q));
    const target = [0.22, -0.32, 0.42, 0.27, -0.18, 0.17];
    const pose = controller.simulationFeedback('pose', { ...controller.current, right: target });
    await upload(encodeControllerPose(pose));
    const expectedFk = controller.ik.fk('right', target);
    const actualFk = controller.ik.fk('right', controller.current.right);
    actualFk.position.forEach((value, i) => expect(Math.abs(value - expectedFk.position[i])).toBeLessThan(0.5));
    actualFk.rotation.forEach((value, i) => expect(Math.abs(value - expectedFk.rotation[i])).toBeLessThan(0.002));
    const lastValid = cloneState(controller.current);
    await upload(encodeControllerPose([100, 100, 100, 0, 0, 0]));
    await upload(encodeControllerJoints([100, 0, 0, 0, 0, 0]));
    expect(controller.current).toEqual(lastValid);
    expect(app.showControllerError).toHaveBeenCalledTimes(2);
    expect(controller.history).toHaveLength(0);
    expect(app.rosClient.publishControllerPose).not.toHaveBeenCalled();
    await usb.disconnect();
    expect(usb.mode).toBe('follow');
    expect(controller.current).toEqual(lastValid);
  });

  it('resets USB authorization on runtime changes and retains hardware ROS gating', async () => {
    app.toggleControllerMode();
    await vi.advanceTimersByTimeAsync(0);
    vi.spyOn(app.rosClient, 'connect').mockImplementation(() => undefined);
    app.setOperationMode('hardware');
    expect(usb.mode).toBe('follow');
    app.toggleControllerMode();
    expect(usb.mode).toBe('follow'); // Offline hardware cannot authorize controller uploads.
    app.setOperationMode('simulation');
    app.toggleControllerMode();
    expect(usb.mode).toBe('active');
    expect(app.rosClient.publishControllerJoints).not.toHaveBeenCalled();
  });

  it('stops slave simulation locally without requesting ROS STOP', async () => {
    const stop = vi.spyOn(app.rosClient, 'requestStop');
    app.toggleControllerMode();
    await vi.advanceTimersByTimeAsync(0);
    await upload(encodeControllerJoints(q));
    app.emergencyStop();
    await upload(encodeControllerJoints(Array(6).fill(0)));
    expect(usb.mode).toBe('follow');
    controller.current.right.forEach((value, i) => expect(value).toBeCloseTo(q[i], 6));
    expect(stop).not.toHaveBeenCalled();
  });

  it('records USB slave joints and poses at fixed intervals without forwarding to ROS', async () => {
    app.toggleControllerMode();
    await vi.advanceTimersByTimeAsync(0);
    app.toggleRecording(100);
    await upload(encodeControllerJoints(q)); // t=50: no scheduled sample yet
    expect(app.recording.samples).toHaveLength(1);
    await vi.advanceTimersByTimeAsync(50); // t=100
    const second = app.recording.samples[1].right;
    second.forEach((value: number, i: number) => expect(value).toBeCloseTo(q[i], 6));
    const target = [0.22, -0.32, 0.42, 0.27, -0.18, 0.17];
    await upload(encodeControllerPose(controller.simulationFeedback('pose', { ...controller.current, right: target })));
    await vi.advanceTimersByTimeAsync(50); // t=200
    const displayed = cloneState(controller.current);
    app.toggleRecording(100);
    expect(controller.history).toHaveLength(2);
    expect(controller.history.map(point => point.durationMs)).toEqual([100, 100]);
    expect(controller.current).toEqual(displayed);
    expect(controller.history[1].target).toEqual(displayed);
    expect(app.rosClient.publishControllerJoints).not.toHaveBeenCalled();
    expect(app.rosClient.publishControllerPose).not.toHaveBeenCalled();
    const replay = new (RoboArmController as any)(controller.ik, controller.wasmBase) as RoboArmController;
    replay.import(controller.export());
    expect(replay.history).toHaveLength(2);
    expect(replay.history[1].target.right).toEqual(displayed.right);
  });

  it('records actual hardware feedback rather than USB targets and saves before stale feedback', async () => {
    app.operationMode = 'hardware';
    vi.spyOn(app.rosClient, 'isConnected', 'get').mockReturnValue(true);
    vi.spyOn(app.rosClient, 'readyCheck').mockReturnValue({ ok: true, reason: '' });
    vi.spyOn(app.rosClient, 'publishControllerJoints').mockReturnValue(true);
    vi.spyOn(app.rosClient, 'publishControllerPose').mockReturnValue(true);
    app.hardwareState = { ...cloneState(controller.current), right: [...q] };
    app.hardwareStateAtMs = Date.now();
    app.toggleControllerMode();
    await vi.advanceTimersByTimeAsync(0);
    app.toggleRecording(100);
    await upload(encodeControllerJoints(q.map(value => value * 0.5)));
    await vi.advanceTimersByTimeAsync(50);
    expect(app.recording.samples[1].right).toEqual(q);
    expect(app.rosClient.publishControllerJoints).toHaveBeenCalled();
    app.hardwareState = { ...cloneState(controller.current), right: q.map(value => value + 0.01) };
    app.hardwareStateAtMs = Date.now();
    await upload(encodeControllerPose([0.25, 0, 0.3, 0, 0, 0]));
    await vi.advanceTimersByTimeAsync(50);
    expect(app.recording.samples[2].right).toEqual(app.hardwareState.right);
    expect(app.rosClient.publishControllerPose).toHaveBeenCalled();
    app.hardwareStateAtMs = Date.now() - 1000;
    await vi.advanceTimersByTimeAsync(100);
    expect(app.recording).toBeNull();
    expect(controller.history).toHaveLength(2);
    expect(controller.history[0].start.right).toEqual(q);
    expect(controller.history[1].target.right).toEqual(app.hardwareState.right);
  });

  it('saves a take on USB mode exit and on operation mode changes', async () => {
    app.toggleControllerMode();
    await vi.advanceTimersByTimeAsync(0);
    app.toggleRecording(100);
    await upload(encodeControllerJoints(q));
    await vi.advanceTimersByTimeAsync(50);
    app.toggleControllerMode();
    expect(app.recording).toBeNull();
    expect(controller.history).toHaveLength(1);
    app.toggleRecording(100);
    await vi.advanceTimersByTimeAsync(100);
    vi.spyOn(app.rosClient, 'connect').mockImplementation(() => undefined);
    app.setOperationMode('hardware');
    expect(app.recording).toBeNull();
    expect(controller.history.length).toBeGreaterThan(1);
  });
});
