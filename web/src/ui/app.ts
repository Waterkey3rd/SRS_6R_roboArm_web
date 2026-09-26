import { RoboArmController } from '../controller';
import type { PlaybackFrame } from '../controller';
import { DEG, matrixToYpr, multiply3, rpyMatrix } from '../math';
import { ArmRenderer } from '../renderer';
import { ArmWebClient, type ArmRosProfile } from '../ros/arm-web-client';
import type { OperationMode, RobotEventJson } from '../ros/types';
import { EVENT_FAULT, EVENT_LINK_LOST, EVENT_MODE_CHANGED, EVENT_VLA_TIMEOUT } from '../ros/types';
import { HardwareExecutor } from '../runtime/hardware-executor';
import { buildHardwareFrame, buildHardwarePlan, type HardwareFrame } from '../runtime/hardware-plan';
import { cartesianTarget, solveTarget } from '../sequence';
import type { ArmState, Side } from '../types';
import { cloneState, SIDES } from '../types';
import { ControlPanel } from './components/control-panel';
import { HelpDialog } from './components/help-dialog';
import { HistoryDialog } from './components/history-dialog';
import { IoDialog } from './components/io-dialog';
import { OperationLogDialog } from './components/operation-log-dialog';
import { TimelineDock } from './components/timeline';
import { showToast } from './components/toast';
import { TopBar } from './components/top-bar';
import { TrajectoryDialog } from './components/trajectory-dialog';
import { ViewportHud } from './components/viewport-hud';

export class App {
  private renderer!: ArmRenderer;
  private topBar!: TopBar;
  private viewportHud!: ViewportHud;
  private controlPanel!: ControlPanel;
  private timeline!: TimelineDock;

  private trajectoryDialog!: TrajectoryDialog;
  private historyDialog!: HistoryDialog;
  private ioDialog!: IoDialog;
  private operationLogDialog!: OperationLogDialog;
  private helpDialog!: HelpDialog;

  private currentTheme: 'light' | 'dark' = 'light';
  private activeKeypointIndex = -1;

  // Animation & Playback state
  private isAnimating = false;
  private isPlaying = false;
  private isLooping = false;
  private playbackSpeed = 1.0;
  private currentPlaybackIndex = 0;
  private playbackStartTime = 0;
  private animationRafId = 0;
  private playbackFrames: PlaybackFrame[] = [];

  // Simulation / hardware runtime
  private operationMode: OperationMode = 'simulation';
  private rosClient!: ArmWebClient;
  private hardwareExecutor!: HardwareExecutor;
  private hardwareState: ArmState | null = null;
  private hardwareStateAtMs = 0;
  private lastControlSyncAtMs = 0;
  private isHardwareRunning = false;
  private lastRosErrorAtMs = 0;
  private rosStatusTimer = 0;

  private constructor(
    private readonly host: HTMLElement,
    private readonly controller: RoboArmController,
  ) {}

  static async init(host: HTMLElement, wasmBase = './wasm/'): Promise<App> {
    const controller = await RoboArmController.create(wasmBase);
    const app = new App(host, controller);
    app.setup();
    return app;
  }

  private setup(): void {
    // Root container
    this.host.innerHTML = '';
    const root = document.createElement('div');
    root.className = 'app-root';
    this.host.appendChild(root);

    // Setup TopBar
    this.topBar = new TopBar({
      onLoadDemo: () => this.loadDemoSequence(),
      onOpenTrajectory: () => this.trajectoryDialog.open(),
      onOpenHistory: () => this.operationLogDialog.open(),
      onOpenIO: () => this.ioDialog.open(),
      onToggleTheme: () => this.toggleTheme(),
      onOpenHelp: () => this.helpDialog.open(),
      onSetMode: (mode) => this.setOperationMode(mode),
      onToggleRosConnect: () => this.toggleRosConnection(),
      onEditRosUrl: () => this.editRosUrl(),
      onEmergencyStop: () => this.emergencyStop(),
    });
    root.appendChild(this.topBar.element);

    // Workspace Body
    const workspace = document.createElement('div');
    workspace.className = 'workspace-body';
    root.appendChild(workspace);

    // 3D Viewport
    const viewportContainer = document.createElement('div');
    viewportContainer.className = 'viewport-container';
    const canvasHost = document.createElement('div');
    canvasHost.className = 'viewport-canvas-host';
    viewportContainer.appendChild(canvasHost);
    workspace.appendChild(viewportContainer);

    this.renderer = new ArmRenderer(canvasHost, this.controller.ik, {
      theme: this.currentTheme,
      showGrid: true,
      showAxes: true,
      showTool: true,
    });

    // Viewport HUD
    this.viewportHud = new ViewportHud(viewportContainer, this.controller.ik, {
      onSelectCamera: (preset) => this.renderer.setCameraPreset(preset, true),
      onResetCamera: () => this.renderer.resetCamera(),
      onToggleGrid: () => this.renderer.toggleGrid(),
      onToggleAxes: () => this.renderer.toggleAxes(),
      onToggleTool: () => this.renderer.toggleTool(),
    });

    // Right Control Dock
    this.controlPanel = new ControlPanel(workspace, this.controller, {
      onExecuteJoint: (values, duration) => this.executeJoint(values, duration),
      onExecuteCartesian: (values, duration) => this.executeCartesian(values, duration),
      onExecuteDelta: (values, duration) => this.executeDelta(values, duration),
      onUndo: () => this.undo(),
      onReset: () => this.reset(),
    });

    // Bottom Timeline Dock
    this.timeline = new TimelineDock(root, this.controller, {
      onPlayPause: () => this.togglePlayPause(),
      onStepFrame: (dir) => this.stepFrame(dir),
      onGoToStart: () => this.goToKeypoint(0),
      onGoToEnd: () => this.goToKeypoint(this.controller.history.length - 1),
      onToggleLoop: (loop) => { this.isLooping = loop; },
      onChangeSpeed: (speed) => { this.playbackSpeed = speed; },
      onScrub: (progress) => this.scrubTo(progress),
      onSelectKeypoint: (index) => this.goToKeypoint(index),
      onEditKeypoint: (index) => this.historyDialog.open(index),
      onDeleteKeypoint: (index) => this.deleteKeypoint(index),
    });

    // Dialogs
    this.trajectoryDialog = new TrajectoryDialog(this.controller, (generated) => {
      const firstGeneratedIndex = this.controller.history.length - generated.length;
      this.stopPlayback();
      this.isAnimating = false;
      this.activeKeypointIndex = this.controller.history.length - 1;
      this.timeline.updateTimeline(this.activeKeypointIndex);
      if (generated.length > 0) {
        this.renderer.update(generated[0].start);
        this.viewportHud.update(generated[0].start);
        this.controlPanel.syncState(generated[0].start);
        this.startPlayback(firstGeneratedIndex);
      }
    });

    this.historyDialog = new HistoryDialog(this.controller, () => {
      this.timeline.updateTimeline(this.activeKeypointIndex);
      this.syncAllViews();
    });

    this.ioDialog = new IoDialog(this.controller, () => {
      this.activeKeypointIndex = this.controller.history.length ? 0 : -1;
      this.timeline.updateTimeline(this.activeKeypointIndex);
      this.syncAllViews();
    });

    this.operationLogDialog = new OperationLogDialog(this.controller, {
      onLocate: (index) => this.goToKeypoint(index),
      onEdit: (index) => this.historyDialog.open(index),
    });

    this.helpDialog = new HelpDialog();

    // ROS bridge / hardware runtime
    this.setupRos();

    // Subscribe to controller state changes
    this.controller.subscribe(() => {
      this.syncAllViews();
    });

    // Initial render sync
    this.syncAllViews();

    // Bind Keyboard shortcuts
    this.bindKeyboardShortcuts();

    // Automatically pop up operation guide for first-time visitors
    const hasSeenGuide = localStorage.getItem('srs_7r_guide_viewed');
    if (!hasSeenGuide) {
      setTimeout(() => {
        this.helpDialog.open();
      }, 300);
    }
  }

  private syncAllViews(): void {
    this.updateViewportSource();
    const cur = this.getDisplayState();
    this.renderer.update(cur);
    this.viewportHud.update(cur);
    this.controlPanel.syncState(cur);
  }

  private syncDisplayOnly(): void {
    this.updateViewportSource();
    const cur = this.getDisplayState();
    this.renderer.update(cur);
    this.viewportHud.update(cur);
  }

  private updateViewportSource(): void {
    const hardware = this.operationMode === 'hardware';
    const stale = hardware && (!this.rosClient || !this.rosClient.readyCheck().ok);
    this.viewportHud.setSource(hardware ? '实机反馈' : '仿真姿态', stale);
  }

  private getDisplayState(): ArmState {
    if (this.operationMode === 'hardware' && this.hardwareState) return this.hardwareState;
    return this.controller.current;
  }

  private getHardwareCurrentState(): ArmState {
    if (!this.hardwareState) throw new Error('尚未收到实机关节反馈');
    return this.hardwareState;
  }

  private setupRos(): void {
    this.rosClient = new ArmWebClient(loadRosBridgeUrl(), loadRosProfile());
    this.hardwareExecutor = new HardwareExecutor(this.rosClient);

    this.rosClient.on('connection', () => {
      this.updateRosStatus();
      this.syncDisplayOnly();
    });

    this.rosClient.on('driverState', () => {
      this.updateRosStatus();
    });

    this.rosClient.on('jointState', (_message: unknown, state: ArmState) => {
      this.hardwareState = state;
      this.hardwareStateAtMs = Date.now();
      if (this.operationMode === 'hardware') {
        this.syncDisplayOnly();
        const now = performance.now();
        if (now - this.lastControlSyncAtMs > 250) {
          this.lastControlSyncAtMs = now;
          this.controlPanel.syncState(state, { respectFocus: true });
        }
      }
    });

    this.rosClient.on('commandResult', (result: any) => {
      if (Number(result?.outcome) !== 0 || Number(result?.mcu_result) !== 0) {
        const message = String(result?.message ?? '');
        if (message) showToast({ title: '实机指令结果', message, type: 'warning' });
      }
    });

    this.rosClient.on('robotEvent', (event: RobotEventJson) => this.handleRosEvent(event));

    this.rosClient.on('error', (error: Error) => {
      const now = Date.now();
      if (now - this.lastRosErrorAtMs < 1500) return;
      this.lastRosErrorAtMs = now;
      showToast({ title: 'ROS 连接', message: error.message || String(error), type: 'warning' });
    });

    this.topBar.setRosStatus('offline', 'ROS 未连接', '点击右侧连接 ROS按钮建立 rosbridge WebSocket 连接');
    this.rosStatusTimer = window.setInterval(() => {
      this.updateRosStatus();
      if (this.operationMode === 'hardware' && !this.rosClient.readyCheck().ok) this.syncDisplayOnly();
    }, 500);
  }

  private updateRosStatus(): void {
    if (!this.rosClient || !this.controlPanel) return;
    const connection = this.rosClient.connectionState;
    const check = this.rosClient.readyCheck();
    const enabled = this.operationMode === 'hardware';

    if (connection !== 'connected') {
      const kind = connection === 'connecting' ? 'connecting' : (connection === 'error' ? 'error' : 'offline');
      const text = connection === 'connecting' ? 'ROS 连接中' : (connection === 'error' ? 'ROS 错误' : 'ROS 未连接');
      this.topBar.setRosStatus(kind, text, this.rosClient.url);
    } else if (check.ok) {
      const ds = this.rosClient.driverState;
      this.topBar.setRosStatus(
        'ready',
        '实机就绪',
        `session=${ds?.session_id ?? '-'} epoch=${ds?.mode_epoch ?? '-'} mode=${ds?.mode_name ?? '-'}`,
      );
    } else {
      this.topBar.setRosStatus('connected', 'ROS 已连接', check.reason);
    }

    this.controlPanel.setHardwareStatus(enabled, check.ok, check.reason);
    this.updateViewportSource();
  }

  private handleRosEvent(event: RobotEventJson): void {
    const type = Number(event.type ?? -1);
    const dangerous = [EVENT_MODE_CHANGED, EVENT_FAULT, EVENT_VLA_TIMEOUT, EVENT_LINK_LOST].includes(type);
    if (!dangerous) return;

    const labels: Record<number, string> = {
      [EVENT_MODE_CHANGED]: 'MCU 模式变化',
      [EVENT_FAULT]: 'MCU 故障',
      [EVENT_VLA_TIMEOUT]: 'VLA 超时',
      [EVENT_LINK_LOST]: '串口链路丢失',
    };
    const label = labels[type] ?? `ROS 事件 ${type}`;
    if (this.hardwareExecutor?.isRunning) {
      this.hardwareExecutor.abort(label);
      this.setHardwareRunning(false);
      this.timeline.setPlayState(false);
      this.viewportHud.showMotionProgress(false);
    }
    showToast({ title: label, message: `事件 type=${type} mode_epoch=${event.mode_epoch ?? '-'}`, type: 'danger' });
  }

  private setHardwareRunning(running: boolean): void {
    this.isHardwareRunning = running;
    this.topBar.setHardwareRunning(running);
  }

  private setOperationMode(mode: OperationMode): void {
    if (mode === this.operationMode) return;
    this.operationMode = mode;
    this.topBar.setMode(mode);

    if (mode === 'hardware') {
      this.stopPlayback();
      if (!this.rosClient.isConnected && this.rosClient.connectionState !== 'connecting') {
        this.rosClient.connect();
      }
      showToast({
        message: `实机模式：显示 ROS 关节反馈，执行按钮将下发 ${loadRosProfile() === 'ros2_real' ? '/joint_command' : '/arm_web/servo_target'}；仿真时间轴仍会保留`,
        type: 'info',
      });
    } else {
      if (this.hardwareExecutor.isRunning) this.hardwareExecutor.abort('切换到纯仿真模式');
      this.setHardwareRunning(false);
      this.timeline.setPlayState(false);
      this.viewportHud.showMotionProgress(false);
      showToast({ message: '已切换到纯仿真模式，仿真状态与时间轴不受实机反馈影响', type: 'info' });
    }

    this.updateRosStatus();
    this.syncAllViews();
  }

  private toggleRosConnection(): void {
    if (this.rosClient.isConnected || this.rosClient.connectionState === 'connecting') {
      this.rosClient.disconnect();
      showToast({ message: '已断开 rosbridge 连接', type: 'info' });
    } else {
      this.rosClient.connect();
      showToast({ message: `正在连接 ${this.rosClient.url}`, type: 'info' });
    }
    this.updateRosStatus();
  }

  private editRosUrl(): void {
    const next = window.prompt('rosbridge WebSocket 地址', this.rosClient.url);
    if (next === null) return;
    const url = next.trim();
    if (!/^wss?:\/\/.+/.test(url)) {
      showToast({ title: '地址无效', message: '请输入 ws://host:port 或 wss://host:port', type: 'danger' });
      return;
    }
    localStorage.setItem('roboarm.rosbridge.url', url);
    this.rosClient.setUrl(url);
    this.rosClient.connect();
    showToast({ message: `已更新 rosbridge 地址并开始连接：${url}`, type: 'info' });
    this.updateRosStatus();
  }

  private emergencyStop(): void {
    if (this.hardwareExecutor?.isRunning) {
      this.hardwareExecutor.abort('用户点击 STOP');
    }
    this.setHardwareRunning(false);
    this.timeline.setPlayState(false);
    this.viewportHud.showMotionProgress(true, 'STOP 发送中...');
    this.rosClient.requestStop()
      .then((response) => {
        this.viewportHud.showMotionProgress(false);
        showToast({
          title: response.success ? 'STOP 已排队' : 'STOP 未发送',
          message: response.message || (response.success ? 'MCU 停止与重新授权仍需观察 driver/events' : '请检查 ROS 连接'),
          type: response.success ? 'warning' : 'danger',
        });
      })
      .catch((err: any) => {
        this.viewportHud.showMotionProgress(false);
        showToast({ title: 'STOP 调用失败', message: err.message || String(err), type: 'danger' });
      });
  }

  private toggleTheme(): void {
    this.currentTheme = this.currentTheme === 'light' ? 'dark' : 'light';
    document.documentElement.setAttribute('data-theme', this.currentTheme);
    this.renderer.setTheme(this.currentTheme);
    this.topBar.setTheme(this.currentTheme);
    showToast({ message: `已切换为${this.currentTheme === 'light' ? '浅色明亮实验室' : '深色座舱'}主题`, type: 'info' });
  }

  // =========================================================
  // Motion Execution with Smooth Interpolation
  // =========================================================
  private executeJoint(values: Record<Side, number[]>, durationMs: number): void {
    if (this.operationMode === 'hardware') {
      void this.executeHardwareJoint(values, durationMs);
      return;
    }
    if (this.isAnimating) return;
    try {
      const prev = cloneState(this.controller.current);
      const point = this.controller.commandJoints(values, { durationMs });
      this.animateTransition(prev, point.target, durationMs, () => {
        this.activeKeypointIndex = this.controller.history.length - 1;
        this.timeline.updateTimeline(this.activeKeypointIndex);
      });
      showToast({ message: '关节运动执行完成', type: 'success' });
    } catch (err: any) {
      showToast({ title: '执行失败', message: err.message || '指令超出关节限位', type: 'danger' });
    }
  }

  private executeCartesian(values: Record<Side, number[]>, durationMs: number): void {
    if (this.operationMode === 'hardware') {
      void this.executeHardwareCartesian(values, durationMs);
      return;
    }
    if (this.isAnimating) return;
    try {
      const prev = cloneState(this.controller.current);
      const point = this.controller.commandCartesian(values, { durationMs });
      this.animateTransition(prev, point.target, durationMs, () => {
        this.activeKeypointIndex = this.controller.history.length - 1;
        this.timeline.updateTimeline(this.activeKeypointIndex);
      });
      showToast({ message: '笛卡尔空间运动执行完成', type: 'success' });
    } catch (err: any) {
      showToast({ title: '逆解失败', message: err.message || '目标超出工作空间', type: 'danger' });
    }
  }

  private executeDelta(values: Record<Side, number[]>, durationMs: number): void {
    if (this.operationMode === 'hardware') {
      void this.executeHardwareDelta(values, durationMs);
      return;
    }
    if (this.isAnimating) return;
    try {
      const prev = cloneState(this.controller.current);
      const point = this.controller.commandDelta(values, { durationMs });
      this.animateTransition(prev, point.target, durationMs, () => {
        this.activeKeypointIndex = this.controller.history.length - 1;
        this.timeline.updateTimeline(this.activeKeypointIndex);
      });
    } catch (err: any) {
      showToast({ title: '微调失败', message: err.message || '微调增量超出机械臂极限', type: 'danger' });
    }
  }

  private validateHardwareTarget(target: ArmState): void {
    for (const side of SIDES) {
      if (!Array.isArray(target[side]) || target[side].length !== 7) {
        throw new Error(`${side === 'left' ? '左' : '右'}臂目标必须包含 7 个关节角`);
      }
      const limits = this.controller.ik.limits(side);
      target[side].forEach((value, index) => {
        if (!Number.isFinite(value)) throw new Error(`${side === 'left' ? '左' : '右'}臂 J${index + 1} 不是有限数字`);
        if (value < limits.lower[index] - 2e-6 || value > limits.upper[index] + 2e-6) {
          throw new Error(`${side === 'left' ? '左' : '右'}臂 J${index + 1} 超出实机限位`);
        }
      });
    }
  }

  private plannedHardwareDuration(start: ArmState, target: ArmState, requestedDurationMs: number): number {
    return this.controller.planPlaybackFrame(start, target, requestedDurationMs).durationMs;
  }

  private async executeHardwareTarget(target: ArmState, durationMs: number, label: string): Promise<void> {
    this.validateHardwareTarget(target);
    try {
      const start = this.getHardwareCurrentState();
      const effectiveDurationMs = this.plannedHardwareDuration(start, target, durationMs);
      this.viewportHud.showMotionProgress(true, `${label}下发中...`);
      await this.hardwareExecutor.executeSingle(target, effectiveDurationMs);
      showToast({ title: '实机下发成功', message: `${label}已发送至 ${loadRosProfile() === 'ros2_real' ? '/joint_command' : '/arm_web/servo_target'}`, type: 'success' });
    } catch (err: any) {
      showToast({ title: '实机下发失败', message: err.message || String(err), type: 'danger' });
    } finally {
      this.viewportHud.showMotionProgress(false);
      this.updateRosStatus();
    }
  }

  private async executeHardwareJoint(values: Record<Side, number[]>, durationMs: number): Promise<void> {
    try {
      const target: ArmState = {
        left: values.left.map((value) => value * DEG),
        right: values.right.map((value) => value * DEG),
      };
      await this.executeHardwareTarget(target, durationMs, '关节目标');
    } catch (err: any) {
      showToast({ title: '实机关节目标无效', message: err.message || String(err), type: 'danger' });
    }
  }

  private async executeHardwareCartesian(values: Record<Side, number[]>, durationMs: number): Promise<void> {
    try {
      const current = this.getHardwareCurrentState();
      const target = {} as ArmState;
      for (const side of SIDES) {
        const value = [...values[side]];
        if (value.length !== 6) throw new Error('位姿需要 X/Y/Z/Yaw/Pitch/Roll 六个分量');
        for (let i = 3; i < 6; i += 1) value[i] *= DEG;
        target[side] = solveTarget(this.controller.ik, side, cartesianTarget(value), current[side]);
      }
      await this.executeHardwareTarget(target, durationMs, '笛卡尔目标');
    } catch (err: any) {
      showToast({ title: '实机笛卡尔目标无效', message: err.message || String(err), type: 'danger' });
    }
  }

  private async executeHardwareDelta(values: Record<Side, number[]>, durationMs: number): Promise<void> {
    try {
      const current = this.getHardwareCurrentState();
      const target = {} as ArmState;
      for (const side of SIDES) {
        const delta = values[side];
        if (delta.length !== 6) throw new Error('Delta 需要 ΔX/ΔY/ΔZ/ΔYaw/ΔPitch/ΔRoll 六个分量');
        const fk = this.controller.ik.fk(side, current[side]);
        const rotation = multiply3(fk.rotation, rpyMatrix(delta[5] * DEG, delta[4] * DEG, delta[3] * DEG));
        const [yaw, pitch, roll] = matrixToYpr(rotation);
        target[side] = solveTarget(this.controller.ik, side, cartesianTarget([
          fk.position[0] + delta[0],
          fk.position[1] + delta[1],
          fk.position[2] + delta[2],
          yaw,
          pitch,
          roll,
        ]), current[side]);
      }
      await this.executeHardwareTarget(target, durationMs, '微调目标');
    } catch (err: any) {
      showToast({ title: '实机微调目标无效', message: err.message || String(err), type: 'danger' });
    }
  }

  private animateTransition(
    from: ArmState,
    to: ArmState,
    durationMs: number,
    onComplete?: () => void,
  ): void {
    this.isAnimating = true;
    const start = performance.now();

    const frame = this.controller.planPlaybackFrame(from, to, durationMs);
    const effectiveDurationMs = frame.durationMs;

    const tick = (now: number) => {
      const elapsed = (now - start) * this.playbackSpeed;
      const progress = Math.min(1, elapsed / effectiveDurationMs);

      const interpolated = this.controller.interpolate(frame, elapsed);
      this.renderer.update(interpolated);
      this.viewportHud.update(interpolated);
      this.viewportHud.showMotionProgress(true, `运动过渡中... ${(progress * 100).toFixed(0)}%`);

      if (progress < 1) {
        this.animationRafId = requestAnimationFrame(tick);
      } else {
        this.isAnimating = false;
        this.viewportHud.showMotionProgress(false);
        this.syncAllViews();
        if (onComplete) onComplete();
      }
    };

    this.animationRafId = requestAnimationFrame(tick);
  }

  private undo(): void {
    if (this.operationMode === 'hardware') {
      showToast({ message: '实机模式不会修改仿真历史；请切回纯仿真模式再回退', type: 'warning' });
      return;
    }
    if (this.isAnimating) return;
    if (this.controller.history.length === 0) {
      showToast({ message: '当前已在初始位姿，无法继续回退', type: 'info' });
      return;
    }
    this.controller.undo();
    this.activeKeypointIndex = this.controller.history.length - 1;
    this.timeline.updateTimeline(this.activeKeypointIndex);
    this.syncAllViews();
    showToast({ message: '已撤销回退至上一状态', type: 'info' });
  }

  private reset(): void {
    if (this.operationMode === 'hardware') {
      showToast({ message: '实机模式不会重置仿真场景；请切回纯仿真模式执行重置', type: 'warning' });
      return;
    }
    if (this.isAnimating) return;
    this.stopPlayback();
    this.controller.reset();
    this.activeKeypointIndex = -1;
    this.timeline.updateTimeline(-1);
    this.syncAllViews();
    showToast({ message: '机械臂已重置为初始零位姿态', type: 'info' });
  }

  // =========================================================
  // Timeline Playback Engine
  // =========================================================
  private togglePlayPause(): void {
    if (this.operationMode === 'hardware') {
      void this.toggleHardwarePlayback();
      return;
    }
    if (this.isPlaying) {
      this.stopPlayback();
    } else {
      this.startPlayback();
    }
  }

  private async toggleHardwarePlayback(): Promise<void> {
    if (this.hardwareExecutor.isRunning) {
      this.hardwareExecutor.abort('用户取消实机序列');
      this.setHardwareRunning(false);
      showToast({ message: '已请求取消实机序列', type: 'warning' });
      return;
    }

    if (this.controller.history.length === 0) {
      showToast({ message: '动作序列为空，无法下发', type: 'warning' });
      return;
    }

    try {
      const actual = this.getHardwareCurrentState();
      const plan = buildHardwarePlan(
        this.controller.history,
        actual,
        (start, target, requested) => this.plannedHardwareDuration(start, target, requested),
      );
      this.setHardwareRunning(true);
      this.timeline.setPlayState(true);
      await this.hardwareExecutor.executePlan(plan, {
        onProgress: (completed, total, label) => {
          this.activeKeypointIndex = Math.max(0, completed - 1);
          this.timeline.updateTimeline(this.activeKeypointIndex);
          let elapsedMs = 0;
          let totalMs = 0;
          for (let i = 0; i < plan.length; i += 1) {
            if (i < completed - 1) elapsedMs += plan[i].durationMs;
            if (i === completed - 1) elapsedMs += plan[i].durationMs;
            totalMs += plan[i].durationMs;
          }
          const progress = total > 0 ? completed / total : 0;
          this.timeline.setProgress(progress, Math.min(elapsedMs, totalMs) / 1000, totalMs / 1000);
          this.viewportHud.showMotionProgress(true, `实机下发中 #${completed}/${total}: ${label}`);
        },
      });
      showToast({ title: '实机序列完成', message: `已按顺序下发 ${plan.length} 个关键点`, type: 'success' });
    } catch (err: any) {
      showToast({ title: '实机序列中止', message: err.message || String(err), type: 'danger' });
    } finally {
      this.setHardwareRunning(false);
      this.timeline.setPlayState(false);
      this.viewportHud.showMotionProgress(false);
      this.updateRosStatus();
    }
  }

  private startPlayback(fromIndex = 0): void {
    if (this.controller.history.length === 0) {
      showToast({ message: '动作序列为空，无法播放', type: 'warning' });
      return;
    }

    this.isPlaying = true;
    this.timeline.setPlayState(true);
    this.currentPlaybackIndex = Math.max(0, Math.min(this.controller.history.length - 1, fromIndex));
    this.playbackFrames = this.controller.playback();
    this.playbackStartTime = performance.now();
    this.playNextFrame();
  }

  private stopPlayback(): void {
    this.isPlaying = false;
    this.timeline.setPlayState(false);
    this.viewportHud.showMotionProgress(false);
    cancelAnimationFrame(this.animationRafId);
  }

  private playNextFrame(): void {
    if (!this.isPlaying) return;

    const history = this.controller.history;
    if (this.currentPlaybackIndex >= history.length) {
      if (this.isLooping && history.length > 0) {
        this.currentPlaybackIndex = 0;
      } else {
        this.stopPlayback();
        showToast({ message: '动作序列播放完毕', type: 'success' });
        return;
      }
    }

    const point = history[this.currentPlaybackIndex];
    this.activeKeypointIndex = this.currentPlaybackIndex;
    this.timeline.updateTimeline(this.currentPlaybackIndex);

    const frame = this.playbackFrames[this.currentPlaybackIndex]
      ?? this.controller.planPlaybackFrame(point.start, point.target, point.durationMs, this.currentPlaybackIndex);

    const start = performance.now();
    const duration = frame.durationMs;

    const tick = (now: number) => {
      if (!this.isPlaying) return;
      const elapsed = (now - start) * this.playbackSpeed;
      const progress = Math.min(1, elapsed / duration);

      const interpolated = this.controller.interpolate(frame, elapsed);
      this.renderer.update(interpolated);
      this.viewportHud.update(interpolated);

      // Calculate total sequence progress
      let elapsedTotalMs = 0;
      let sequenceTotalMs = 0;
      for (let i = 0; i < history.length; i++) {
        const plannedDuration = this.playbackFrames[i]?.durationMs ?? history[i].durationMs;
        if (i < this.currentPlaybackIndex) elapsedTotalMs += plannedDuration;
        sequenceTotalMs += plannedDuration;
      }
      elapsedTotalMs += progress * duration;
      const totalRatio = sequenceTotalMs > 0 ? elapsedTotalMs / sequenceTotalMs : 0;
      this.timeline.setProgress(totalRatio, elapsedTotalMs / 1000, sequenceTotalMs / 1000);

      this.viewportHud.showMotionProgress(
        true,
        `播放关键点 #${this.currentPlaybackIndex + 1}/${history.length}: ${point.label} (${(progress * 100).toFixed(0)}%)`,
      );

      if (progress < 1) {
        this.animationRafId = requestAnimationFrame(tick);
      } else {
        this.currentPlaybackIndex++;
        this.playNextFrame();
      }
    };

    this.animationRafId = requestAnimationFrame(tick);
  }

  private stepFrame(direction: -1 | 1): void {
    if (this.operationMode === 'hardware') {
      const history = this.controller.history;
      if (history.length === 0) return;
      let next = this.activeKeypointIndex + direction;
      next = Math.max(0, Math.min(history.length - 1, next));
      this.activeKeypointIndex = next;
      this.timeline.updateTimeline(next);
      void this.sendHardwareFrameAt(next);
      return;
    }
    this.stopPlayback();
    const history = this.controller.history;
    if (history.length === 0) return;

    let next = this.activeKeypointIndex + direction;
    next = Math.max(0, Math.min(history.length - 1, next));
    this.goToKeypoint(next);
  }

  private async sendHardwareFrameAt(index: number): Promise<void> {
    const history = this.controller.history;
    if (!history[index]) return;
    try {
      const actual = this.getHardwareCurrentState();
      const frame = buildHardwareFrame(
        history,
        index,
        actual,
        (start, target, requested) => this.plannedHardwareDuration(start, target, requested),
      );
      await this.executeHardwareTarget(frame.target, frame.durationMs, `关键点 #${index + 1}`);
    } catch (err: any) {
      showToast({ title: '实机单步下发失败', message: err.message || String(err), type: 'danger' });
    }
  }

  private goToKeypoint(index: number): void {
    this.stopPlayback();
    const history = this.controller.history;
    if (index < 0 || index >= history.length) return;

    if (this.operationMode === 'hardware') {
      this.activeKeypointIndex = index;
      this.timeline.updateTimeline(index);
      return;
    }

    this.activeKeypointIndex = index;
    this.controller.restore(index);
    this.timeline.updateTimeline(index);
    this.syncAllViews();
  }

  private scrubTo(progress: number): void {
    if (this.operationMode === 'hardware') {
      showToast({ message: '实机模式不支持拖动时间轴，请使用关键点单步或序列下发', type: 'warning' });
      return;
    }
    this.stopPlayback();
    const history = this.controller.history;
    if (history.length === 0) return;

    const frames = this.controller.playback();
    let totalDurationMs = 0;
    frames.forEach(frame => totalDurationMs += frame.durationMs);
    const targetMs = progress * totalDurationMs;

    let accumulated = 0;
    for (let i = 0; i < history.length; i++) {
      const frame = frames[i];
      const dur = frame.durationMs;
      if (accumulated + dur >= targetMs || i === history.length - 1) {
        const frameProgress = Math.max(0, Math.min(1, (targetMs - accumulated) / dur));
        const state = this.controller.interpolate(frame, frameProgress * dur);
        this.renderer.update(state);
        this.viewportHud.update(state);
        this.controlPanel.syncState(state);
        this.timeline.setProgress(progress, targetMs / 1000, totalDurationMs / 1000);
        this.activeKeypointIndex = i;
        break;
      }
      accumulated += dur;
    }
  }

  // =========================================================
  private deleteKeypoint(index: number): void {
    if (this.operationMode === 'hardware') {
      showToast({ message: '实机模式不会修改仿真动作序列；请切回纯仿真模式再编辑', type: 'warning' });
      return;
    }
    if (this.isAnimating) return;
    this.stopPlayback();
    if (this.controller.history.length <= 1) {
      this.reset();
      return;
    }

    try {
      this.controller.deleteHistory(index);
      this.activeKeypointIndex = Math.min(this.activeKeypointIndex, this.controller.history.length - 1);
      this.timeline.updateTimeline(this.activeKeypointIndex);
      this.syncAllViews();
      showToast({ message: `已删除关键点 #${index + 1}`, type: 'info' });
    } catch (err: any) {
      showToast({ title: '删除失败', message: err.message || '删除该点后下游关键点无有效解', type: 'danger' });
    }
  }
  // Demo Sequence Loader
  // =========================================================
  private async loadDemoSequence(): Promise<void> {
    try {
      const res = await fetch('./demo_dance.json');
      if (!res.ok) throw new Error(`HTTP ${res.status}`);
      const payload = await res.json();
      this.controller.import(payload);
      this.activeKeypointIndex = 0;
      this.timeline.updateTimeline(0);
      this.syncAllViews();
      showToast({
        title: '演示动作载入成功',
        message: `成功载入内置实机舞蹈动作（共 ${this.controller.history.length} 个关键点），点击播放即可演示`,
        type: 'success',
      });
    } catch (err: any) {
      showToast({
        title: '载入失败',
        message: '无法读取 demo_dance.json: ' + (err.message || String(err)),
        type: 'danger',
      });
    }
  }

  // =========================================================
  // Keyboard Shortcuts
  // =========================================================
  private bindKeyboardShortcuts(): void {
    window.addEventListener('keydown', (e: KeyboardEvent) => {
      // If typing in input or textarea, skip global shortcuts
      if (['INPUT', 'TEXTAREA', 'SELECT'].includes((e.target as HTMLElement).tagName)) {
        if (e.key === 'Escape') {
          (e.target as HTMLElement).blur();
        }
        return;
      }

      if (e.key === ' ' || e.code === 'Space') {
        e.preventDefault();
        this.togglePlayPause();
      } else if (e.key === 'Enter' && (e.ctrlKey || e.metaKey)) {
        e.preventDefault();
        this.controlPanel.handleExecute();
      } else if ((e.key === 'z' || e.key === 'Z') && (e.ctrlKey || e.metaKey)) {
        e.preventDefault();
        this.undo();
      } else if (e.key === 'F2') {
        e.preventDefault();
        if (this.activeKeypointIndex >= 0) {
          this.historyDialog.open(this.activeKeypointIndex);
        } else if (this.controller.history.length > 0) {
          this.historyDialog.open(0);
        }
      } else if (e.key === '1') {
        this.renderer.setCameraPreset('iso', true);
      } else if (e.key === '2') {
        this.renderer.setCameraPreset('top', true);
      } else if (e.key === '3') {
        this.renderer.setCameraPreset('front', true);
      } else if (e.key === '4') {
        this.renderer.setCameraPreset('side', true);
      } else if (e.key === 'Escape') {
        this.trajectoryDialog.close();
        this.historyDialog.close();
        this.ioDialog.close();
        this.helpDialog.close();
      }
    });
  }
}

function loadRosBridgeUrl(): string {
  const params = new URLSearchParams(window.location.search);
  const fromQuery = params.get('rosbridge');
  if (fromQuery && /^wss?:\/\/.+/.test(fromQuery)) return fromQuery;

  const fromStorage = window.localStorage.getItem('roboarm.rosbridge.url');
  if (fromStorage && /^wss?:\/\/.+/.test(fromStorage)) return fromStorage;

  const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  const host = window.location.hostname || '127.0.0.1';
  return `${protocol}//${host}:9090`;
}

function loadRosProfile(): ArmRosProfile {
  const params = new URLSearchParams(window.location.search);
  return params.get('rosProfile') === 'web_bridge' ? 'web_bridge' : 'ros2_real';
}
