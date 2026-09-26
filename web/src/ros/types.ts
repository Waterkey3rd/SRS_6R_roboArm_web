import type { ArmState } from '../types';

export type RosConnectionKind = 'disconnected' | 'connecting' | 'connected' | 'error';
export type OperationMode = 'simulation' | 'hardware';

export const ARM_MASK_RIGHT = 1;
export const ARM_MASK_LEFT = 2;
export const ARM_MASK_BOTH = 3;

export const MODE_IDLE = 0;
export const MODE_REMOTE = 1;
export const MODE_PC_DEBUG = 2;
export const MODE_ROS_TRAJECTORY = 3;
export const MODE_VLA = 4;
export const MODE_HOMING = 5;
export const MODE_FAULT = 6;
export const MODE_ESTOP = 7;

export const OUTCOME_ACK = 0;
export const OUTCOME_REJECTED = 1;
export const OUTCOME_TIMEOUT = 2;
export const OUTCOME_INVALIDATED = 3;
export const OUTCOME_SUPERSEDED = 4;
export const OUTCOME_WRITTEN_UNCONFIRMED = 5;
export const MCU_OK = 0;

export const EVENT_MODE_CHANGED = 0;
export const EVENT_FAULT = 1;
export const EVENT_TRAJECTORY_READY = 2;
export const EVENT_TRAJECTORY_FINISHED = 3;
export const EVENT_VLA_TIMEOUT = 4;
export const EVENT_LINK_LOST = 5;
export type DriverStateJson = {
  stamp_ns: string;
  connected: boolean;
  mode: number;
  mode_name: string;
  session_id: string;
  mode_epoch: number;
  heartbeat_fresh: boolean;
  mode_fresh: boolean;
  peer_online: boolean;
  control_blocked: boolean;
  snapshot_valid: boolean;
  joint_state_fresh: boolean;
  motion_ready: boolean;
  rearm_required: boolean;
  safety_flags: number;
  fault_code: number;
  heartbeat_age_ms: number;
  mode_age_ms: number;
  joint_state_age_ms: number;
  left_gripper_state: number;
  right_gripper_state: number;
  trajectory_reserved: boolean;
  trajectory_id: number;
  last_error: string;
};

export type JointStateJson = {
  header?: { stamp?: { sec?: number; nanosec?: number }; frame_id?: string };
  name?: string[];
  position?: number[];
  velocity?: number[];
  effort?: number[];
};

export type CommandResultJson = {
  stamp_ns?: string;
  ticket?: string;
  client_id?: string | number;
  session_id?: string;
  request_command?: number;
  sequence?: number;
  mode_epoch?: number;
  outcome?: number | string;
  mcu_result?: number | string;
  detail?: number | string;
  message?: string;
};

export type RobotEventJson = {
  stamp_ns?: string;
  session_id?: string;
  type?: number;
  mcu_timestamp_ms?: number;
  mode?: number;
  mode_epoch?: number;
  detail?: number;
  matches_current_epoch?: boolean;
};

export type ServoTargetJson = {
  client_id: string;
  arm_mask: number;
  duration_ms: number;
  right: number[];
  left: number[];
  source?: string;
};

export type HardwareReadyCheck = {
  ok: boolean;
  reason: string;
};

export type TriggerResponse = {
  success: boolean;
  message: string;
};

export type ArmStateFeedback = {
  state: ArmState;
  velocity: ArmState;
  receivedAtMs: number;
  source: 'ros';
};

export const MODE_LABELS: Record<number, string> = {
  [MODE_IDLE]: 'IDLE',
  [MODE_REMOTE]: 'REMOTE',
  [MODE_PC_DEBUG]: 'PC_DEBUG',
  [MODE_ROS_TRAJECTORY]: 'ROS_TRAJECTORY',
  [MODE_VLA]: 'VLA',
  [MODE_HOMING]: 'HOMING',
  [MODE_FAULT]: 'FAULT',
  [MODE_ESTOP]: 'ESTOP',
};

export function modeLabel(mode: number | undefined): string {
  return mode === undefined ? 'UNKNOWN' : (MODE_LABELS[mode] ?? `MODE_${mode}`);
}

export function outcomeLabel(outcome: number): string {
  switch (outcome) {
    case OUTCOME_ACK: return 'ACK';
    case OUTCOME_REJECTED: return 'REJECTED';
    case OUTCOME_TIMEOUT: return 'TIMEOUT';
    case OUTCOME_INVALIDATED: return 'INVALIDATED';
    case OUTCOME_SUPERSEDED: return 'SUPERSEDED';
    case OUTCOME_WRITTEN_UNCONFIRMED: return 'WRITTEN_UNCONFIRMED';
    default: return `OUTCOME_${outcome}`;
  }
}