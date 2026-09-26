import { SIDES, type ArmState, type Side } from '../types';
import { RosBridgeClient } from './rosbridge-client';
import {
  ARM_MASK_BOTH,
  MODE_PC_DEBUG,
  MODE_VLA,
  OUTCOME_ACK,
  MCU_OK,
  outcomeLabel,
  type CommandResultJson,
  type DriverStateJson,
  type HardwareReadyCheck,
  type JointStateJson,
  type RobotEventJson,
  type RosConnectionKind,
  type ServoTargetJson,
  type TriggerResponse,
} from './types';

export type ArmWebClientEvent =
  | 'connection'
  | 'driverState'
  | 'jointState'
  | 'commandResult'
  | 'robotEvent'
  | 'error';

type EventListener = (...args: any[]) => void;
export type ArmRosProfile = 'web_bridge' | 'ros2_real';

const STATE_STALE_MS = 500;
const JOINT_STALE_MS = 300;

/**
 * High-level WebView contract against `eng26_web_bridge`.
 *
 * The browser never builds a custom ROS message by itself: it sends JSON on
 * std_msgs/String and receives JSON back. The ROS bridge owns session/epoch,
 * timestamp and command gating.
 */
export class ArmWebClient {
  readonly bridge: RosBridgeClient;
  driverState: DriverStateJson | null = null;
  driverStateAtMs = 0;
  jointState: JointStateJson | null = null;
  jointStateAtMs = 0;
  latestArmState: ArmState | null = null;

  private unsubscribers: Array<() => void> = [];
  private listeners = new Map<ArmWebClientEvent, Set<EventListener>>();
  private nextClientId = 1;
  private results = new Map<string, CommandResultJson>();
  private resultWaiters = new Map<string, Array<{
    resolve: (result: CommandResultJson) => void;
    reject: (error: Error) => void;
    timer: number;
  }>>();
  private readonly profile: ArmRosProfile;

  constructor(url: string, profile: ArmRosProfile = 'ros2_real') {
    this.profile = profile;
    this.bridge = new RosBridgeClient(url);
    this.bridge.onState((state) => this.handleConnectionState(state));
    this.bridge.onError((error) => this.emit('error', error));
    const jointTopic = profile === 'ros2_real' ? '/joint_states' : '/arm_web/joint_state';
    this.unsubscribers.push(this.bridge.subscribe(jointTopic, 'sensor_msgs/msg/JointState', (message) => this.handleJointStateMessage(message)));
    if (profile === 'web_bridge') {
      this.unsubscribers.push(
        this.bridge.subscribe('/arm_web/driver_state', 'std_msgs/msg/String', (message) => this.handleDriverStateMessage(message)),
        this.bridge.subscribe('/arm_web/command_result', 'std_msgs/msg/String', (message) => this.handleCommandResultMessage(message)),
        this.bridge.subscribe('/arm_web/event', 'std_msgs/msg/String', (message) => this.handleRobotEventMessage(message)),
      );
    }
  }

  get url(): string {
    return this.bridge.url;
  }

  get connectionState(): RosConnectionKind {
    return this.bridge.connectionState;
  }

  get isConnected(): boolean {
    return this.bridge.isConnected;
  }

  on(event: ArmWebClientEvent, listener: EventListener): () => void {
    let listeners = this.listeners.get(event);
    if (!listeners) {
      listeners = new Set<EventListener>();
      this.listeners.set(event, listeners);
    }
    listeners.add(listener);
    return () => listeners?.delete(listener);
  }

  connect(): void {
    this.bridge.connect();
  }

  disconnect(): void {
    this.bridge.disconnect();
  }

  setUrl(url: string): void {
    this.bridge.setUrl(url);
  }

  publishServo(target: ArmState, armMask = ARM_MASK_BOTH, durationMs = 800, requestedClientId?: string): string {
    const clientId = requestedClientId ?? String(this.nextClientId++);
    if (this.profile === 'ros2_real') {
      const positions = [...target.right, ...target.left];
      this.bridge.publish('/joint_command', {
        name: ['right_joint_1', 'right_joint_2', 'right_joint_3', 'right_joint_4', 'right_joint_5', 'right_joint_6', 'right_joint_7',
          'left_joint_1', 'left_joint_2', 'left_joint_3', 'left_joint_4', 'left_joint_5', 'left_joint_6', 'left_joint_7'],
        position: positions,
      }, 'sensor_msgs/msg/JointState');
      this.results.set(clientId, {
        client_id: clientId,
        outcome: OUTCOME_ACK,
        mcu_result: MCU_OK,
        message: 'ros2_real /joint_command 已发布',
      });
      return clientId;
    }
    const payload: ServoTargetJson = {
      client_id: clientId,
      arm_mask: armMask,
      duration_ms: Math.round(durationMs),
      right: [...target.right],
      left: [...target.left],
      source: 'webview',
    };
    this.bridge.publish('/arm_web/servo_target', { data: JSON.stringify(payload) }, 'std_msgs/msg/String');
    return clientId;
  }

  async waitForCommandResult(clientId: string, timeoutMs = 2000): Promise<CommandResultJson> {
    const existing = this.results.get(clientId);
    if (existing) {
      this.results.delete(clientId);
      return existing;
    }

    return new Promise<CommandResultJson>((resolve, reject) => {
      const timer = window.setTimeout(() => {
        const waiters = this.resultWaiters.get(clientId);
        if (waiters) {
          const index = waiters.findIndex((waiter) => waiter.timer === timer);
          if (index >= 0) waiters.splice(index, 1);
          if (waiters.length === 0) this.resultWaiters.delete(clientId);
        }
        reject(new Error(`等待实机指令结果超时 (client_id=${clientId})`));
      }, Math.max(200, timeoutMs));

      const waiters = this.resultWaiters.get(clientId) ?? [];
      waiters.push({ resolve, reject, timer });
      this.resultWaiters.set(clientId, waiters);
    });
  }

  async requestStop(): Promise<TriggerResponse> {
    if (this.profile === 'ros2_real') return { success: false, message: 'ros2_real 未提供 STOP 服务' };
    const response = await this.bridge.callService('/arm_web/stop', 'std_srvs/srv/Trigger', {});
    return normalizeTriggerResponse(response);
  }

  async requestHold(): Promise<TriggerResponse> {
    if (this.profile === 'ros2_real') return { success: false, message: 'ros2_real 未提供 HOLD 服务' };
    const response = await this.bridge.callService('/arm_web/hold', 'std_srvs/srv/Trigger', {});
    return normalizeTriggerResponse(response);
  }

  readyCheck(): HardwareReadyCheck {
    if (!this.isConnected) return { ok: false, reason: 'rosbridge 未连接' };
    if (this.profile === 'ros2_real') {
      const jointAge = Date.now() - this.jointStateAtMs;
      if (!this.latestArmState || jointAge > JOINT_STALE_MS) return { ok: false, reason: `ros2_real 关节反馈已过期 ${Math.round(jointAge)} ms` };
      return { ok: true, reason: 'ros2_real 关节反馈正常；模式由串口桥内部校验' };
    }
    if (!this.driverState) return { ok: false, reason: '尚未收到 driver/state' };

    const stateAge = Date.now() - this.driverStateAtMs;
    if (stateAge > STATE_STALE_MS) return { ok: false, reason: `driver/state 已过期 ${Math.round(stateAge)} ms` };

    const jointAge = Date.now() - this.jointStateAtMs;
    if (!this.latestArmState || jointAge > JOINT_STALE_MS) {
      return { ok: false, reason: `关节反馈已过期 ${Math.round(jointAge)} ms` };
    }

    const state = this.driverState;
    if (!state.connected) return { ok: false, reason: '下位机串口未连接' };
    if (state.rearm_required) return { ok: false, reason: 'MCU 需要重新授权/rearm' };
    if (state.control_blocked) return { ok: false, reason: 'MCU control_blocked' };
    if (!state.snapshot_valid) return { ok: false, reason: 'MCU 无有效快照' };
    if (!state.motion_ready) return { ok: false, reason: 'motion_ready=false' };
    if (state.mode !== MODE_PC_DEBUG && state.mode !== MODE_VLA) {
      return { ok: false, reason: `当前模式 ${state.mode_name || state.mode} 不允许 servo_target（需 PC_DEBUG/VLA）` };
    }
    if (state.trajectory_reserved) return { ok: false, reason: 'driver 正在处理轨迹上传' };

    return { ok: true, reason: '实机就绪' };
  }

  private handleConnectionState(state: RosConnectionKind): void {
    this.emit('connection', state);
  }

  private handleDriverStateMessage(message: unknown): void {
    const parsed = parseStringJson<DriverStateJson>(message);
    if (!parsed || typeof parsed.motion_ready !== 'boolean') return;
    this.driverState = parsed;
    this.driverStateAtMs = Date.now();
    this.emit('driverState', parsed);
  }

  private handleJointStateMessage(message: unknown): void {
    const raw = message as JointStateJson;
    if (!raw || !Array.isArray(raw.name) || !Array.isArray(raw.position)) return;
    const mapped = mapJointState(raw);
    if (!mapped) return;
    this.jointState = raw;
    this.jointStateAtMs = Date.now();
    this.latestArmState = mapped;
    this.emit('jointState', raw, mapped);
  }

  private handleCommandResultMessage(message: unknown): void {
    const parsed = parseStringJson<CommandResultJson>(message);
    if (!parsed) return;
    const clientId = String(parsed.client_id ?? '');
    if (!clientId) return;
    parsed.client_id = clientId;
    this.emit('commandResult', parsed);

    const waiters = this.resultWaiters.get(clientId);
    if (!waiters || waiters.length === 0) {
      this.results.set(clientId, parsed);
      while (this.results.size > 64) {
        const first = this.results.keys().next();
        if (first.done) break;
        this.results.delete(first.value);
      }
      return;
    }

    const waiter = waiters.shift();
    if (!waiter) return;
    window.clearTimeout(waiter.timer);
    if (waiters.length === 0) this.resultWaiters.delete(clientId);
    waiter.resolve(parsed);
  }

  private handleRobotEventMessage(message: unknown): void {
    const parsed = parseStringJson<RobotEventJson>(message);
    if (!parsed) return;
    this.emit('robotEvent', parsed);
  }

  private emit(event: ArmWebClientEvent, ...args: any[]): void {
    const listeners = this.listeners.get(event);
    if (!listeners) return;
    for (const listener of listeners) listener(...args);
  }
}

function parseStringJson<T>(message: unknown): T | null {
  const data = (message as { data?: unknown } | null)?.data;
  if (typeof data !== 'string') return null;
  try {
    return JSON.parse(data) as T;
  } catch {
    return null;
  }
}

function mapJointState(raw: JointStateJson): ArmState | null {
  const names = raw.name ?? [];
  const positions = raw.position ?? [];
  const velocities = raw.velocity ?? [];
  if (positions.length < 14) return null;

  const nameIndex = new Map<string, number>();
  names.forEach((name, index) => {
    if (typeof name === 'string') nameIndex.set(name, index);
  });

  const read = (side: Side, index: number, values: number[]): number => {
    const expected = `${side}_joint_${index + 1}`;
    const direct = nameIndex.get(expected);
    if (direct !== undefined && direct < values.length) return Number(values[direct]);
    if (names.length !== 14) return NaN;
    const offset = side === 'right' ? 0 : 7;
    return Number(values[offset + index] ?? NaN);
  };

  const state = {} as ArmState;
  const velocity = {} as ArmState;
  for (const side of SIDES) {
    state[side] = Array.from({ length: 7 }, (_, index) => read(side, index, positions));
    velocity[side] = Array.from({ length: 7 }, (_, index) => read(side, index, velocities));
    if (state[side].some((value) => !Number.isFinite(value))) return null;
  }

  // Some driver configurations omit velocity. Keep an explicit zero fallback
  // so the UI does not display NaN while the primary position feedback is valid.
  for (const side of SIDES) {
    if (velocity[side].some((value) => !Number.isFinite(value))) {
      velocity[side] = Array(7).fill(0);
    }
  }

  return state;
}

function normalizeTriggerResponse(value: unknown): TriggerResponse {
  if (!value || typeof value !== 'object') return { success: false, message: '空服务响应' };
  const record = value as Record<string, unknown>;
  return {
    success: record.success === true,
    message: String(record.message ?? ''),
  };
}

export function commandResultOk(result: CommandResultJson): boolean {
  return Number(result.outcome) === OUTCOME_ACK && Number(result.mcu_result) === MCU_OK;
}

export function commandResultText(result: CommandResultJson): string {
  const outcome = Number(result.outcome);
  return `${outcomeLabel(outcome)}${result.mcu_result !== undefined ? ` / MCU=${result.mcu_result}` : ''}: ${result.message ?? ''}`;
}
