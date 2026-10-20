import type { ViewportToolMode } from '../../renderer';
import type { Side } from '../../types';
import { ICONS } from '../icons';

export interface ViewportToolbarCallbacks {
  onSelectTool: (mode: ViewportToolMode) => void;
  onSelectArm: (side: Side) => void;
  onResetRefFrame: () => void;
  onAlignArmToRefFrame: () => void;
}

export class ViewportToolbar {
  readonly element: HTMLElement;
  private currentMode: ViewportToolMode = 'none';
  private currentArm: Side = 'right';
  private btnArmLeft: HTMLButtonElement;
  private btnArmRight: HTMLButtonElement;
  private toolButtons: Map<ViewportToolMode, HTMLButtonElement> = new Map();
  private extraActionsGroup: HTMLElement;

  constructor(
    private readonly container: HTMLElement,
    private readonly callbacks: ViewportToolbarCallbacks,
  ) {
    this.element = document.createElement('div');
    this.element.className = 'viewport-overlay-left-toolbar';

    this.element.innerHTML = `
      <!-- Top Arm Selector (L / R) -->
      <div class="vtool-arm-switcher" title="切换当前操作机械臂 (快捷键 Tab)">
        <button hidden class="vtool-arm-btn" data-arm="left" title="当前操作: 左臂 (Tab 切换)">L</button>
        <button class="vtool-arm-btn" data-arm="right" title="6 轴机械臂">6R</button>
      </div>

      <div class="vtool-divider"></div>

      <!-- 3 Blender-style Viewport Tools -->
      <div class="vtool-buttons-group">
        <button class="vtool-btn" data-tool="frame" title="动坐标系 (快捷键 1 / C)&#10;在场景中平移/标定参考坐标系，可点击地面吸附放置">
          <span class="vtool-icon">${ICONS.TOOL_FRAME}</span>
          <span class="vtool-badge">1</span>
        </button>

        <button class="vtool-btn" data-tool="translate" title="移动末端位置 (快捷键 2 / G)&#10;鼠标拖拽平移活动臂末端，高帧率实时 IK 逆解跟随">
          <span class="vtool-icon">${ICONS.TOOL_TRANSLATE}</span>
          <span class="vtool-badge">2</span>
        </button>

        <button class="vtool-btn" data-tool="rotate" title="移动末端位姿 (快捷键 3 / R)&#10;鼠标拖拽三轴旋转环调整末端朝向，实时 IK 解算姿态">
          <span class="vtool-icon">${ICONS.TOOL_ROTATE}</span>
          <span class="vtool-badge">3</span>
        </button>
      </div>

      <!-- Secondary Actions (Quick Frame Reset & Align) -->
      <div class="vtool-extra-group" id="vtool-extra-group" style="display: none;">
        <div class="vtool-divider"></div>
        <button class="vtool-btn vtool-btn-sub" id="btn-vtool-reset-origin" title="参考坐标系复位到世界原点 (0, 0, 0)">
          <span class="vtool-icon">${ICONS.ORIGIN}</span>
        </button>
        <button class="vtool-btn vtool-btn-sub" id="btn-vtool-align-ee" title="将当前机械臂末端对齐至该参考坐标系">
          <span class="vtool-icon">${ICONS.ALIGN_EE}</span>
        </button>
      </div>
    `;

    container.appendChild(this.element);

    this.btnArmLeft = this.element.querySelector('[data-arm="left"]') as HTMLButtonElement;
    this.btnArmRight = this.element.querySelector('[data-arm="right"]') as HTMLButtonElement;
    this.extraActionsGroup = this.element.querySelector('#vtool-extra-group') as HTMLElement;

    // Map tool buttons
    const toolModes: ViewportToolMode[] = ['frame', 'translate', 'rotate'];
    for (const mode of toolModes) {
      const btn = this.element.querySelector(`[data-tool="${mode}"]`) as HTMLButtonElement;
      if (btn) {
        this.toolButtons.set(mode, btn);
        btn.addEventListener('click', () => {
          const next = this.currentMode === mode ? 'none' : mode;
          this.setTool(next);
          this.callbacks.onSelectTool(next);
        });
      }
    }

    // Bind Arm Switcher
    this.btnArmLeft.addEventListener('click', () => {
      this.setArm('left');
      this.callbacks.onSelectArm('left');
    });

    this.btnArmRight.addEventListener('click', () => {
      this.setArm('right');
      this.callbacks.onSelectArm('right');
    });

    // Bind Extra Actions
    this.element.querySelector('#btn-vtool-reset-origin')?.addEventListener('click', () => {
      this.callbacks.onResetRefFrame();
    });

    this.element.querySelector('#btn-vtool-align-ee')?.addEventListener('click', () => {
      this.callbacks.onAlignArmToRefFrame();
    });
  }

  setTool(mode: ViewportToolMode): void {
    this.currentMode = mode;
    this.toolButtons.forEach((btn, m) => {
      btn.classList.toggle('active', m === mode);
    });

    // Show extra actions when reference frame tool is active
    this.extraActionsGroup.style.display = (mode === 'frame') ? 'flex' : 'none';
  }

  getTool(): ViewportToolMode {
    return this.currentMode;
  }

  setArm(side: Side): void {
    side = 'right';
    this.currentArm = side;
    this.btnArmLeft.classList.remove('active');
    this.btnArmRight.classList.toggle('active', side === 'right');
  }

  getArm(): Side {
    return this.currentArm;
  }
}
