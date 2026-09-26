import type { OperationMode } from '../../ros/types';
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
          <div class="brand-icon">7R</div>
          <span>SRS 7R 双臂机器人控制台</span>
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
}
