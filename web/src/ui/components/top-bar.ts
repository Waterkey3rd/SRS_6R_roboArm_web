import type { OperationMode } from '../../ros/types';
import type { ControllerDataType, ControllerMode } from '../../serial/controller-protocol';
import type { ControllerConnection } from '../../serial/homologous-controller';
import { ICONS } from '../icons';

export type RosStatusKind = 'offline' | 'connecting' | 'connected' | 'ready' | 'error';

export interface TopBarCallbacks {
  onLoadDemo: () => void;
  onOpenTrajectory: () => void;
  onOpenHistory: () => void;
  onOpenIO: () => void;
  onToggleTheme: () => void;
  onOpenHelp: () => void;
  onSetMode: (mode: OperationMode) => void;
  onToggleRosConnect: () => void;
  onEditRosUrl: () => void;
  onEmergencyStop: () => void;
  onToggleControllerConnect: () => void;
  onToggleControllerMode: () => void;
  onIdentifyControllerType: () => void;
  onToggleRecording: (intervalMs: number) => void;
}

export class TopBar {
  readonly element: HTMLElement;
  private currentTheme: 'light' | 'dark' = 'light';
  private currentMode: OperationMode = 'simulation';
  private themeBtn: HTMLButtonElement;
  private modeButtons: HTMLButtonElement[] = [];
  private rosStatusEl: HTMLElement;
  private rosStatusText: HTMLElement;
  private rosConnectBtn: HTMLButtonElement;
  private emergencyBtn: HTMLButtonElement;

  constructor(private readonly callbacks: TopBarCallbacks) {
    this.element = document.createElement('header');
    this.element.className = 'top-bar';
    this.element.innerHTML = `
      <div class="top-bar-left">
        <div class="brand-badge">
          <div class="brand-icon">6R</div>
          <span>Engineer2026 6R 机械臂机器人控制台</span>
        </div>

        <div class="mode-switch" id="mode-switch" role="group" aria-label="运行模式">
          <button class="mode-btn active" data-mode="simulation" title="仅使用浏览器本地仿真，不向实机发送指令">
            <span class="mode-dot simulation"></span>
            <span>纯仿真</span>
          </button>
          <button class="mode-btn" data-mode="hardware" title="连接 ROS2 实机链路，显示实机反馈并可下发 servo_target">
            <span class="mode-dot hardware"></span>
            <span>实机接入</span>
          </button>
        </div>

        <div class="status-indicator ros-status is-offline" id="ros-status" title="rosbridge 未连接">
          <span class="status-dot"></span>
          <span id="ros-status-text">ROS 未连接</span>
        </div>
      </div>

      <div class="top-bar-right">
        <button class="btn btn-secondary btn-sm" id="btn-ros-connect" title="连接/断开 rosbridge WebSocket">
          <span id="ros-connect-text">连接 ROS</span>
        </button>
        <button class="btn btn-ghost btn-icon btn-sm" id="btn-ros-config" title="设置 rosbridge WebSocket 地址"></button>
          <button class="btn btn-danger btn-sm" id="btn-ros-stop" title="请求 ROS STOP；ros2_real 未提供该服务" style="display: none;">
          STOP
        </button>

        <button class="btn btn-secondary btn-sm" id="btn-top-demo" title="载入预置实机动作序列">
          <span class="btn-icon-slot">${ICONS.DEMO}</span>
          <span>载入演示动作</span>
        </button>
        <button class="btn btn-primary btn-sm" id="btn-top-trajectory" title="使用 LaTeX 公式生成关节或笛卡尔函数轨迹">
          <span class="btn-icon-slot">${ICONS.FORMULA}</span>
          <span>函数轨迹</span>
        </button>
        <button class="btn btn-secondary btn-sm" id="btn-top-history" title="查看全部历史操作、关键点和函数来源">
          <span class="btn-icon-slot">${ICONS.HISTORY}</span>
          <span>历史记录</span>
        </button>
        <button class="btn btn-secondary btn-sm" id="btn-top-io" title="导入/导出 performance-action-sequence-v2 JSON">
          <span class="btn-icon-slot">${ICONS.IO}</span>
          <span>动作序列 IO</span>
        </button>
        <button class="btn btn-ghost btn-icon btn-sm" id="btn-top-theme" title="切换深浅色彩模式">
          <span id="theme-icon">${ICONS.SUN}</span>
        </button>
        <button class="btn btn-ghost btn-icon btn-sm" id="btn-top-help" title="快捷键与使用指南">
          <span>${ICONS.HELP}</span>
        </button>
      </div>
      <div class="controller-link" role="group" aria-label="同构控制器 USB">
        <span class="controller-label">同构控制器</span>
        <span id="controller-status" role="status">USB 未连接</span>
        <button class="btn btn-secondary btn-sm" id="btn-controller-connect">选择 USB 串口</button>
          <button class="btn btn-secondary btn-sm" id="btn-controller-mode" aria-pressed="false" disabled>跟随 · 点击从动仿真</button>
        <button class="btn btn-secondary btn-sm" id="btn-controller-identify" title="从控制器下一份有效数据识别类型，仅监听，不发送类型切换命令" disabled>识别数据类型</button>
        <span id="controller-data-type" role="status">数据：待识别</span>
        <span class="controller-hint" id="controller-direction">等待控制器声明数据类型</span>
      </div>
      <div class="controller-link" role="group" aria-label="固定间隔录包">
        <span class="controller-label">离散点录包</span>
        <label for="recording-interval">采样间隔</label>
        <select class="form-select" id="recording-interval" style="width: auto; padding: 3px 8px;">
          <option value="50">50 ms</option>
          <option value="100" selected>100 ms</option>
          <option value="200">200 ms</option>
          <option value="500">500 ms</option>
          <option value="1000">1000 ms</option>
        </select>
        <button class="btn btn-secondary btn-sm" id="btn-recording" aria-pressed="false">开始录包</button>
        <span id="recording-status" role="status">仿真姿态 · 未录制</span>
        <span class="controller-hint">停止后追加到时间轴，可在动作序列 IO 导出</span>
      </div>
    `;

    this.themeBtn = this.element.querySelector('#btn-top-theme') as HTMLButtonElement;
    this.rosStatusEl = this.element.querySelector('#ros-status') as HTMLElement;
    this.rosStatusText = this.element.querySelector('#ros-status-text') as HTMLElement;
    this.rosConnectBtn = this.element.querySelector('#btn-ros-connect') as HTMLButtonElement;
    this.emergencyBtn = this.element.querySelector('#btn-ros-stop') as HTMLButtonElement;
    this.modeButtons = Array.from(this.element.querySelectorAll<HTMLButtonElement>('.mode-btn'));

    for (const button of this.modeButtons) {
      button.addEventListener('click', () => {
        const mode = button.dataset.mode as OperationMode | undefined;
        if (mode) this.callbacks.onSetMode(mode);
      });
    }

    this.rosConnectBtn.addEventListener('click', () => this.callbacks.onToggleRosConnect());
    this.element.querySelector('#btn-ros-config')?.addEventListener('click', () => this.callbacks.onEditRosUrl());
    this.emergencyBtn.addEventListener('click', () => this.callbacks.onEmergencyStop());
    this.element.querySelector('#btn-controller-connect')?.addEventListener('click', () => this.callbacks.onToggleControllerConnect());
    this.element.querySelector('#btn-controller-mode')?.addEventListener('click', () => this.callbacks.onToggleControllerMode());
    this.element.querySelector('#btn-controller-identify')?.addEventListener('click', () => this.callbacks.onIdentifyControllerType());
    this.element.querySelector('#btn-recording')?.addEventListener('click', () => {
      const interval = this.element.querySelector('#recording-interval') as HTMLSelectElement;
      this.callbacks.onToggleRecording(Number(interval.value));
    });

    this.element.querySelector('#btn-top-demo')?.addEventListener('click', () => this.callbacks.onLoadDemo());
    this.element.querySelector('#btn-top-trajectory')?.addEventListener('click', () => this.callbacks.onOpenTrajectory());
    this.element.querySelector('#btn-top-history')?.addEventListener('click', () => this.callbacks.onOpenHistory());
    this.element.querySelector('#btn-top-io')?.addEventListener('click', () => this.callbacks.onOpenIO());
    this.themeBtn.addEventListener('click', () => this.callbacks.onToggleTheme());
    this.element.querySelector('#btn-top-help')?.addEventListener('click', () => this.callbacks.onOpenHelp());

    this.setMode('simulation');
  }

  setTheme(theme: 'light' | 'dark'): void {
    this.currentTheme = theme;
    const icon = this.element.querySelector('#theme-icon');
    if (icon) {
      icon.innerHTML = theme === 'light' ? ICONS.SUN : ICONS.MOON;
    }
  }

  setMode(mode: OperationMode): void {
    this.currentMode = mode;
    for (const button of this.modeButtons) {
      button.classList.toggle('active', button.dataset.mode === mode);
    }
    this.emergencyBtn.style.display = mode === 'hardware' ? 'inline-flex' : 'none';
  }

  get mode(): OperationMode {
    return this.currentMode;
  }

  setRosStatus(kind: RosStatusKind, text: string, title?: string): void {
    const classes = ['status-indicator', 'ros-status', `is-${kind}`];
    this.rosStatusEl.className = classes.join(' ');
    this.rosStatusEl.title = title ?? text;
    this.rosStatusText.textContent = text;
    if (kind === 'connected' || kind === 'ready') {
      this.rosConnectBtn.textContent = '断开 ROS';
    } else if (kind === 'connecting') {
      this.rosConnectBtn.textContent = '连接中';
    } else {
      this.rosConnectBtn.textContent = '连接 ROS';
    }
  }

  setHardwareRunning(running: boolean): void {
    this.emergencyBtn.textContent = 'STOP';
    this.emergencyBtn.classList.toggle('is-running', running);
  }

  setControllerStatus(connection: ControllerConnection, mode: ControllerMode, detail: string, feedbackType: ControllerDataType | null): void {
    const status = this.element.querySelector('#controller-status') as HTMLElement;
    const connect = this.element.querySelector('#btn-controller-connect') as HTMLButtonElement;
    const toggle = this.element.querySelector('#btn-controller-mode') as HTMLButtonElement;
    status.textContent = detail;
    connect.disabled = connection === 'connecting' || connection === 'disconnecting';
    connect.textContent = connection === 'connected' ? '断开 USB' : connection === 'connecting' ? '选择 / 连接中…' : connection === 'disconnecting' ? '断开中…' : '选择 USB 串口';
    toggle.disabled = connection !== 'connected';
    const simulation = this.currentMode === 'simulation';
    toggle.textContent = mode === 'active' ? simulation ? '从动仿真 · 点击跟随' : '主动 · 点击跟随'
      : simulation ? '跟随 · 点击从动仿真' : '跟随 · 点击主动';
    toggle.setAttribute('aria-pressed', String(mode === 'active'));
    toggle.classList.toggle('btn-primary', mode === 'active');
    toggle.classList.toggle('btn-secondary', mode !== 'active');
    this.emergencyBtn.style.display = !simulation || connection === 'connected' ? 'inline-flex' : 'none';
    this.emergencyBtn.title = simulation ? '停止仿真并恢复 USB 跟随' : '向实机发送 STOP';
    const direction = this.element.querySelector('#controller-direction') as HTMLElement;
    const source = simulation ? '虚拟臂' : '实机';
    direction.textContent = mode === 'active' ? simulation ? '控制器关节角 / 位姿 → 虚拟臂' : '控制器关节角 / 位姿 → ROS → 实机'
      : feedbackType === 'pose' ? `${source} TCP 位姿（m / rad） → 控制器`
        : feedbackType === 'joints' ? `${source}关节角（rad） → 控制器` : '等待控制器声明数据类型';
  }

  setControllerDataType(connection: ControllerConnection, type: ControllerDataType | null, identifying: boolean, timedOut: boolean): void {
    const button = this.element.querySelector('#btn-controller-identify') as HTMLButtonElement;
    const status = this.element.querySelector('#controller-data-type') as HTMLElement;
    button.disabled = connection !== 'connected' || identifying;
    button.textContent = identifying ? '识别中…' : '识别数据类型';
    status.textContent = identifying ? '等待下一份有效上传…' : timedOut ? '未收到数据，可重试'
      : type === 'joints' ? '数据：关节角（rad）' : type === 'pose' ? '数据：位姿（m / rad）' : '数据：待识别';
  }

  setRecordingStatus(recording: boolean, ready: boolean, source: string, count = 0, durationMs = 0): void {
    const button = this.element.querySelector('#btn-recording') as HTMLButtonElement;
    const interval = this.element.querySelector('#recording-interval') as HTMLSelectElement;
    button.disabled = !recording && !ready;
    button.textContent = recording ? '停止录包并保存' : '开始录包';
    button.setAttribute('aria-pressed', String(recording));
    button.classList.toggle('btn-danger', recording);
    button.classList.toggle('btn-secondary', !recording);
    interval.disabled = recording;
    const status = this.element.querySelector('#recording-status') as HTMLElement;
    status.textContent = recording ? `${source} · 录制中 ${count} 点 / ${(durationMs / 1000).toFixed(1)} s`
      : `${source} · ${ready ? '未录制' : '等待新鲜实机反馈'}`;
  }
}
