import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { TransformControls } from 'three/examples/jsm/controls/TransformControls.js';
import { DEG, matrixToYpr } from './math';
import type { ArmState, Side } from './types';
import { cloneState, SIDES, zeroState } from './types';
import type { DeployIK } from './wasm';

interface ArmMeshes {
  group: THREE.Group;
  joints: THREE.Mesh[];
  links: THREE.Mesh[];
  axes: THREE.ArrowHelper[];
  tool: THREE.AxesHelper;
}

export type ThemeMode = 'light' | 'dark';
export type CameraPreset = 'iso' | 'top' | 'front' | 'side';
export type ViewportToolMode = 'none' | 'frame' | 'translate' | 'rotate';

export interface RendererCallbacks {
  onToolChange?: (mode: ViewportToolMode) => void;
  onActiveSideChange?: (side: Side) => void;
  onInteractiveMove?: (side: Side, q: number[], cartesian: number[]) => void;
  onInteractiveCommit?: (side: Side, q: number[], cartesian: number[], label: string) => void;
  onRefFrameChange?: (pos: THREE.Vector3, rot: THREE.Euler) => void;
}

export interface RendererOptions {
  theme?: ThemeMode;
  showGrid?: boolean;
  showAxes?: boolean;
  showTool?: boolean;
  callbacks?: RendererCallbacks;
}

export class ArmRenderer {
  private readonly scene = new THREE.Scene();
  private readonly camera = new THREE.PerspectiveCamera(42, 1, 1, 10000);
  private readonly renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  private readonly controls: OrbitControls;
  private readonly transformControls: TransformControls;
  private readonly arms = {} as Record<Side, ArmMeshes>;
  private readonly resizeObserver: ResizeObserver;

  private currentTheme: ThemeMode = 'light';
  private hemiLight: THREE.HemisphereLight;
  private keyLight: THREE.DirectionalLight;
  private fillLight: THREE.DirectionalLight;
  private grid: THREE.GridHelper;
  private baseMesh: THREE.Mesh;
  private disposed = false;

  // Viewport 3-Tool Interactive Dragging & Coordinate Frame State
  private currentToolMode: ViewportToolMode = 'none';
  private activeSide: Side = 'right';
  private readonly refFrame: THREE.Group;
  private readonly ikTargetProxy = new THREE.Object3D();
  private isDraggingGizmo = false;
  private currentState: ArmState = zeroState();
  private callbacks: RendererCallbacks;
  private readonly raycaster = new THREE.Raycaster();
  private readonly floorPlane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);
  private pointerDownPos = new THREE.Vector2();
  private pointerDownTime = 0;

  private targetCamPos = new THREE.Vector3();
  private targetCamTarget = new THREE.Vector3();
  private isTransitioningCam = false;

  constructor(private readonly host: HTMLElement, private readonly ik: DeployIK, options: RendererOptions = {}) {
    this.callbacks = options.callbacks || {};
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    host.appendChild(this.renderer.domElement);

    this.camera.position.set(720, -760, 520);
    this.camera.up.set(0, 0, 1);
    this.camera.lookAt(160, 0, 160);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(160, 0, 170);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.maxDistance = 2500;
    this.controls.minDistance = 150;
    this.controls.addEventListener('change', () => this.render());

    // Interactive Gizmo & Helpers
    this.transformControls = new TransformControls(this.camera, this.renderer.domElement);
    this.transformControls.size = 0.85;
    this.scene.add(this.transformControls.getHelper());

    this.scene.add(this.ikTargetProxy);
    this.refFrame = this.createRefFrame();

    this.transformControls.addEventListener('dragging-changed', (e: any) => {
      this.isDraggingGizmo = !!e.value;
      this.controls.enabled = !e.value;
      if (!e.value) {
        this.handleDragEnd();
      }
    });

    this.transformControls.addEventListener('change', () => {
      if (this.isDraggingGizmo) {
        this.handleGizmoChange();
      }
      this.render();
    });

    // Blender-style Navigation Controls:
    // MMB: Rotate (Orbit) | Shift + MMB: Pan | Ctrl + MMB / Wheel: Zoom (Dolly)
    // Alt + LMB: Trackpad Emulate 3-Button Mouse (Rotate) | Alt + Shift + LMB: Pan | Alt + Ctrl + LMB: Zoom
    // LMB: Disabled for camera rotation (reserved for interactions)
    // RMB: Pan (fallback convenient pan)
    this.controls.mouseButtons = {
      LEFT: -1 as any,
      MIDDLE: THREE.MOUSE.ROTATE,
      RIGHT: THREE.MOUSE.PAN,
    };

    // Prevent default browser context menu when right-clicking on canvas
    this.renderer.domElement.addEventListener('contextmenu', (e) => {
      e.preventDefault();
    });

    // Intercept pointerdown in capture phase to dynamically handle Blender modifier combinations
    this.renderer.domElement.addEventListener(
      'pointerdown',
      (e: PointerEvent) => {
        this.pointerDownPos.set(e.clientX, e.clientY);
        this.pointerDownTime = performance.now();

        // Trackpad Emulate 3 Button Mouse with Alt key
        if (e.button === 0) {
          if (e.altKey) {
            if (e.ctrlKey || e.metaKey) {
              this.controls.mouseButtons.LEFT = THREE.MOUSE.DOLLY;
            } else if (e.shiftKey) {
              this.controls.mouseButtons.LEFT = THREE.MOUSE.PAN;
            } else {
              this.controls.mouseButtons.LEFT = THREE.MOUSE.ROTATE;
            }
          } else {
            this.controls.mouseButtons.LEFT = -1 as any;
          }
        }

        // Middle Mouse Button: Rotate / Pan (Shift) / Zoom (Ctrl)
        if (e.button === 1) {
          if (e.ctrlKey || e.metaKey) {
            this.controls.mouseButtons.MIDDLE = THREE.MOUSE.DOLLY;
          } else {
            // OrbitControls natively checks event.shiftKey and switches to PAN when mouseButtons.MIDDLE is ROTATE
            this.controls.mouseButtons.MIDDLE = THREE.MOUSE.ROTATE;
          }
        }
      },
      { capture: true },
    );

    // Click interaction (e.g. raycasting ground plane for Tool 1 or selecting arm)
    this.renderer.domElement.addEventListener('pointerup', (e: PointerEvent) => {
      if (e.button === 0 && !this.isDraggingGizmo && !e.altKey) {
        const dist = Math.hypot(e.clientX - this.pointerDownPos.x, e.clientY - this.pointerDownPos.y);
        const elapsed = performance.now() - this.pointerDownTime;
        if (dist < 6 && elapsed < 400) {
          this.handleClickInteraction(e);
        }
      }
    });

    // Lights
    this.hemiLight = new THREE.HemisphereLight(0xffffff, 0xd0d7de, 2.4);
    this.scene.add(this.hemiLight);

    this.keyLight = new THREE.DirectionalLight(0xffffff, 2.6);
    this.keyLight.position.set(380, -320, 750);
    this.keyLight.castShadow = true;
    this.keyLight.shadow.mapSize.width = 2048;
    this.keyLight.shadow.mapSize.height = 2048;
    this.keyLight.shadow.bias = -0.0005;
    this.scene.add(this.keyLight);

    this.fillLight = new THREE.DirectionalLight(0xe0f2fe, 1.2);
    this.fillLight.position.set(-300, 400, 500);
    this.scene.add(this.fillLight);

    // Ground Grid
    this.grid = new THREE.GridHelper(1200, 24, 0x94a3b8, 0xd5dde5);
    this.grid.rotateX(Math.PI / 2);
    this.scene.add(this.grid);

    // Waist Base
    this.baseMesh = new THREE.Mesh(
      new THREE.BoxGeometry(90, 300, 45),
      new THREE.MeshStandardMaterial({ color: 0x94a3b8, metalness: 0.35, roughness: 0.45 }),
    );
    this.baseMesh.position.set(0, 0, -25);
    this.baseMesh.castShadow = true;
    this.baseMesh.receiveShadow = true;
    this.scene.add(this.baseMesh);

    // Arms
    for (const side of SIDES) {
      this.arms[side] = this.makeArm(side);
    }

    // Apply theme
    this.setTheme(options.theme || 'light');

    if (options.showGrid !== undefined) this.grid.visible = options.showGrid;
    if (options.showAxes !== undefined) this.toggleAxes(options.showAxes);
    if (options.showTool !== undefined) this.toggleTool(options.showTool);

    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(host);

    const loop = () => {
      if (this.disposed) return;
      if (this.isTransitioningCam) {
        this.camera.position.lerp(this.targetCamPos, 0.12);
        this.controls.target.lerp(this.targetCamTarget, 0.12);
        if (
          this.camera.position.distanceTo(this.targetCamPos) < 1 &&
          this.controls.target.distanceTo(this.targetCamTarget) < 1
        ) {
          this.camera.position.copy(this.targetCamPos);
          this.controls.target.copy(this.targetCamTarget);
          this.isTransitioningCam = false;
        }
      }
      this.controls.update();
      this.renderer.render(this.scene, this.camera);
      requestAnimationFrame(loop);
    };
    loop();
  }

  private makeArm(side: Side): ArmMeshes {
    const group = new THREE.Group();
    this.scene.add(group);

    const colors = this.getArmColors(side, this.currentTheme);
    const jointMaterial = new THREE.MeshStandardMaterial({
      color: colors.joint,
      metalness: 0.45,
      roughness: 0.3,
    });
    const linkMaterial = new THREE.MeshStandardMaterial({
      color: colors.link,
      metalness: 0.3,
      roughness: 0.4,
    });

    const joints = Array.from({ length: 6 }, () => {
      const mesh = new THREE.Mesh(new THREE.SphereGeometry(15, 24, 18), jointMaterial);
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      group.add(mesh);
      return mesh;
    });

    const links = Array.from({ length: 6 }, () => {
      const mesh = new THREE.Mesh(new THREE.CylinderGeometry(9, 9, 1, 20), linkMaterial);
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      group.add(mesh);
      return mesh;
    });

    const axes = Array.from({ length: 6 }, () => {
      const arrow = new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), undefined, 42, 0xd97706, 10, 5);
      group.add(arrow);
      return arrow;
    });

    const tool = new THREE.AxesHelper(55);
    (tool.material as THREE.Material).depthTest = false;
    tool.renderOrder = 10;
    group.add(tool);

    return { group, joints, links, axes, tool };
  }

  private getArmColors(side: Side, theme: ThemeMode) {
    if (theme === 'light') {
      return side === 'left'
        ? { joint: 0x0284c7, link: 0x38bdf8 } // Cerulean Blue / Sky
        : { joint: 0xea580c, link: 0xfb923c }; // Deep Tangerine / Orange
    }
    return side === 'left'
      ? { joint: 0x38bdf8, link: 0x93c5fd }
      : { joint: 0xf97316, link: 0xfdba74 };
  }

  setTheme(theme: ThemeMode): void {
    this.currentTheme = theme;
    this.scene.fog = null;
    if (theme === 'light') {
      this.scene.background = new THREE.Color(0xeef2f7);
      this.hemiLight.color.setHex(0xffffff);
      this.hemiLight.groundColor.setHex(0xd0d7de);
      this.hemiLight.intensity = 2.4;
      this.keyLight.intensity = 2.6;
      this.fillLight.intensity = 1.2;
      this.scene.remove(this.grid);
      this.grid = new THREE.GridHelper(1200, 24, 0x94a3b8, 0xd5dde5);
      this.grid.rotateX(Math.PI / 2);
      this.scene.add(this.grid);
      (this.baseMesh.material as THREE.MeshStandardMaterial).color.setHex(0x94a3b8);
    } else {
      this.scene.background = new THREE.Color(0x07101d);
      this.hemiLight.color.setHex(0xb9dcff);
      this.hemiLight.groundColor.setHex(0x18202e);
      this.hemiLight.intensity = 2.0;
      this.keyLight.intensity = 2.8;
      this.fillLight.intensity = 0.8;
      this.scene.remove(this.grid);
      this.grid = new THREE.GridHelper(1200, 24, 0x29415f, 0x172940);
      this.grid.rotateX(Math.PI / 2);
      this.scene.add(this.grid);
      (this.baseMesh.material as THREE.MeshStandardMaterial).color.setHex(0x1d3048);
    }

    for (const side of SIDES) {
      const colors = this.getArmColors(side, theme);
      for (const joint of this.arms[side].joints) {
        (joint.material as THREE.MeshStandardMaterial).color.setHex(colors.joint);
      }
      for (const link of this.arms[side].links) {
        (link.material as THREE.MeshStandardMaterial).color.setHex(colors.link);
      }
    }
    this.render();
  }

  setCameraPreset(preset: CameraPreset, animate = true): void {
    const target = new THREE.Vector3(160, 0, 170);
    const pos = new THREE.Vector3();

    switch (preset) {
      case 'iso':
        pos.set(720, -760, 520);
        break;
      case 'top':
        pos.set(160, 0, 1250);
        break;
      case 'front':
        pos.set(160, -1150, 200);
        break;
      case 'side':
        pos.set(1250, 0, 200);
        break;
    }

    if (animate) {
      this.targetCamPos.copy(pos);
      this.targetCamTarget.copy(target);
      this.isTransitioningCam = true;
    } else {
      this.camera.position.copy(pos);
      this.controls.target.copy(target);
      this.controls.update();
      this.render();
    }
  }

  resetCamera(): void {
    this.setCameraPreset('iso', true);
  }

  toggleGrid(visible?: boolean): boolean {
    this.grid.visible = visible ?? !this.grid.visible;
    this.render();
    return this.grid.visible;
  }

  toggleAxes(visible?: boolean): boolean {
    let nowVisible = true;
    for (const side of SIDES) {
      for (const axis of this.arms[side].axes) {
        axis.visible = visible ?? !axis.visible;
        nowVisible = axis.visible;
      }
    }
    this.render();
    return nowVisible;
  }

  toggleTool(visible?: boolean): boolean {
    let nowVisible = true;
    for (const side of SIDES) {
      this.arms[side].tool.visible = visible ?? !this.arms[side].tool.visible;
      nowVisible = this.arms[side].tool.visible;
    }
    this.render();
    return nowVisible;
  }

  update(state: ArmState): void {
    this.currentState = cloneState(state);
    for (const side of SIDES) {
      this.updateSingleArmMesh(side, state[side]);
    }
    if (!this.isDraggingGizmo && (this.currentToolMode === 'translate' || this.currentToolMode === 'rotate')) {
      this.syncProxyToActiveArm();
    }
  }

  private updateSingleArmMesh(side: Side, q: number[]): void {
    const chain = this.ik.chain(side, q);
    const meshes = this.arms[side];
    for (let i = 0; i < 6; i++) {
      const a = new THREE.Vector3(...(chain.positions[i] as [number, number, number]));
      const b = new THREE.Vector3(...(chain.positions[i + 1] as [number, number, number]));
      meshes.joints[i].position.copy(a);
      meshes.axes[i].position.copy(a);
      meshes.axes[i].setDirection(new THREE.Vector3(...(chain.axes[i] as [number, number, number])).normalize());
      this.placeCylinder(meshes.links[i], a, b);
    }
    const fk = this.ik.fk(side, q);
    meshes.tool.position.set(...(fk.position as [number, number, number]));
    const m = new THREE.Matrix4().set(
      fk.rotation[0], fk.rotation[1], fk.rotation[2], 0,
      fk.rotation[3], fk.rotation[4], fk.rotation[5], 0,
      fk.rotation[6], fk.rotation[7], fk.rotation[8], 0,
      0, 0, 0, 1,
    );
    meshes.tool.quaternion.setFromRotationMatrix(m);
  }

  private syncProxyToActiveArm(): void {
    const fk = this.ik.fk(this.activeSide, this.currentState[this.activeSide]);
    this.ikTargetProxy.position.set(...(fk.position as [number, number, number]));
    const m = new THREE.Matrix4().set(
      fk.rotation[0], fk.rotation[1], fk.rotation[2], 0,
      fk.rotation[3], fk.rotation[4], fk.rotation[5], 0,
      fk.rotation[6], fk.rotation[7], fk.rotation[8], 0,
      0, 0, 0, 1,
    );
    this.ikTargetProxy.quaternion.setFromRotationMatrix(m);
  }

  private createRefFrame(): THREE.Group {
    const group = new THREE.Group();
    group.name = 'reference-frame';

    // 3D Crosshair Axes (X=Red, Y=Green, Z=Blue)
    const axes = new THREE.AxesHelper(75);
    (axes.material as THREE.Material).depthTest = false;
    axes.renderOrder = 20;
    group.add(axes);

    // Center 3D Cursor Marker (Double ring + center sphere, Blender 3D cursor aesthetic)
    const centerSphere = new THREE.Mesh(
      new THREE.SphereGeometry(3.5, 16, 16),
      new THREE.MeshStandardMaterial({ color: 0xffffff, roughness: 0.2, metalness: 0.8 }),
    );
    group.add(centerSphere);

    const ringGeo = new THREE.TorusGeometry(12, 0.8, 12, 36);
    const ringMat = new THREE.MeshBasicMaterial({ color: 0xd97706, depthTest: false });
    const ring1 = new THREE.Mesh(ringGeo, ringMat);
    ring1.renderOrder = 21;
    group.add(ring1);

    const ring2 = new THREE.Mesh(ringGeo, ringMat);
    ring2.rotation.x = Math.PI / 2;
    ring2.renderOrder = 21;
    group.add(ring2);

    group.position.set(220, 0, 100);
    group.visible = false;
    this.scene.add(group);
    return group;
  }

  private handleClickInteraction(e: PointerEvent): void {
    const rect = this.renderer.domElement.getBoundingClientRect();
    const mouse = new THREE.Vector2(
      ((e.clientX - rect.left) / rect.width) * 2 - 1,
      -((e.clientY - rect.top) / rect.height) * 2 + 1,
    );
    this.raycaster.setFromCamera(mouse, this.camera);

    // If in reference frame mode, click on ground grid to place reference frame
    if (this.currentToolMode === 'frame') {
      const hit = new THREE.Vector3();
      if (this.raycaster.ray.intersectPlane(this.floorPlane, hit)) {
        if (Math.abs(hit.x) < 800 && Math.abs(hit.y) < 800) {
          this.refFrame.position.set(Math.round(hit.x), Math.round(hit.y), 0);
          this.callbacks.onRefFrameChange?.(this.refFrame.position.clone(), this.refFrame.rotation.clone());
          this.render();
          return;
        }
      }
    }

    // Check if user clicked an arm's joints/tool to select that arm
    for (const side of SIDES) {
      const meshes = this.arms[side];
      const hits = this.raycaster.intersectObjects([...meshes.joints, ...meshes.links, meshes.tool], true);
      if (hits.length > 0) {
        this.setActiveSide(side);
        return;
      }
    }
  }

  private handleGizmoChange(): void {
    if (this.currentToolMode === 'frame') {
      this.callbacks.onRefFrameChange?.(
        this.refFrame.position.clone(),
        this.refFrame.rotation.clone(),
      );
      return;
    }

    if (this.currentToolMode === 'translate') {
      const targetPos = [
        this.ikTargetProxy.position.x,
        this.ikTargetProxy.position.y,
        this.ikTargetProxy.position.z,
      ];
      const curQ = this.currentState[this.activeSide];
      const fk = this.ik.fk(this.activeSide, curQ);
      try {
        const solved = this.ik.solve(this.activeSide, fk.rotation, targetPos, curQ);
        this.currentState[this.activeSide] = [...solved.q];
        this.updateSingleArmMesh(this.activeSide, solved.q);
        const [yaw, pitch, roll] = matrixToYpr(fk.rotation);
        const cartesian = [...targetPos, yaw / DEG, pitch / DEG, roll / DEG];
        this.callbacks.onInteractiveMove?.(this.activeSide, solved.q, cartesian);
      } catch {
        // Target is outside workspace reach; keep last valid arm pose
      }
      return;
    }

    if (this.currentToolMode === 'rotate') {
      const m = new THREE.Matrix4().makeRotationFromQuaternion(this.ikTargetProxy.quaternion);
      const e = m.elements;
      const targetRot = [
        e[0], e[4], e[8],
        e[1], e[5], e[9],
        e[2], e[6], e[10],
      ];
      const curQ = this.currentState[this.activeSide];
      const fk = this.ik.fk(this.activeSide, curQ);
      try {
        const solved = this.ik.solve(this.activeSide, targetRot, fk.position, curQ);
        this.currentState[this.activeSide] = [...solved.q];
        this.updateSingleArmMesh(this.activeSide, solved.q);
        const [yaw, pitch, roll] = matrixToYpr(targetRot);
        const cartesian = [...fk.position, yaw / DEG, pitch / DEG, roll / DEG];
        this.callbacks.onInteractiveMove?.(this.activeSide, solved.q, cartesian);
      } catch {
        // Target rotation outside reachable pose
      }
      return;
    }
  }

  private handleDragEnd(): void {
    if (this.currentToolMode === 'translate' || this.currentToolMode === 'rotate') {
      const q = this.currentState[this.activeSide];
      const fk = this.ik.fk(this.activeSide, q);
      const [yaw, pitch, roll] = matrixToYpr(fk.rotation);
      const cartesian = [...fk.position, yaw / DEG, pitch / DEG, roll / DEG];
      const label = this.currentToolMode === 'translate'
        ? `${this.activeSide === 'left' ? '左' : '右'}臂鼠标位置拖拽`
        : `${this.activeSide === 'left' ? '左' : '右'}臂鼠标位姿旋转`;
      this.callbacks.onInteractiveCommit?.(this.activeSide, q, cartesian, label);
      this.syncProxyToActiveArm();
    }
  }

  setInteractiveTool(mode: ViewportToolMode): void {
    this.currentToolMode = mode;
    this.refFrame.visible = (mode === 'frame');

    if (mode === 'none') {
      this.transformControls.detach();
    } else if (mode === 'frame') {
      this.transformControls.attach(this.refFrame);
      this.transformControls.setMode('translate');
      this.transformControls.setSpace('world');
    } else if (mode === 'translate') {
      this.syncProxyToActiveArm();
      this.transformControls.attach(this.ikTargetProxy);
      this.transformControls.setMode('translate');
      this.transformControls.setSpace('world');
    } else if (mode === 'rotate') {
      this.syncProxyToActiveArm();
      this.transformControls.attach(this.ikTargetProxy);
      this.transformControls.setMode('rotate');
      this.transformControls.setSpace('local');
    }
    this.callbacks.onToolChange?.(mode);
    this.render();
  }

  getCurrentToolMode(): ViewportToolMode {
    return this.currentToolMode;
  }

  setActiveSide(side: Side): void {
    side = 'right';
    if (this.activeSide === side) return;
    this.activeSide = 'right';
    if (this.currentToolMode === 'translate' || this.currentToolMode === 'rotate') {
      this.syncProxyToActiveArm();
    }
    this.callbacks.onActiveSideChange?.(side);
    this.render();
  }

  getActiveSide(): Side {
    return this.activeSide;
  }

  resetRefFrameToOrigin(): void {
    this.refFrame.position.set(0, 0, 0);
    this.refFrame.rotation.set(0, 0, 0);
    if (this.currentToolMode === 'frame') {
      this.transformControls.attach(this.refFrame);
    }
    this.callbacks.onRefFrameChange?.(this.refFrame.position.clone(), this.refFrame.rotation.clone());
    this.render();
  }

  getRefFramePose(): { position: THREE.Vector3; rotation: THREE.Euler } {
    return {
      position: this.refFrame.position.clone(),
      rotation: this.refFrame.rotation.clone(),
    };
  }

  setCallbacks(callbacks: RendererCallbacks): void {
    this.callbacks = { ...this.callbacks, ...callbacks };
  }

  private placeCylinder(mesh: THREE.Mesh, a: THREE.Vector3, b: THREE.Vector3): void {
    const delta = b.clone().sub(a);
    const length = delta.length();
    mesh.visible = length > 0.01;
    if (!mesh.visible) return;
    mesh.position.copy(a).add(b).multiplyScalar(0.5);
    mesh.scale.set(1, length, 1);
    mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), delta.normalize());
  }

  private resize(): void {
    const width = this.host.clientWidth;
    const height = this.host.clientHeight;
    if (width === 0 || height === 0) return;
    this.renderer.setSize(width, height, false);
    this.camera.aspect = width / Math.max(height, 1);
    this.camera.updateProjectionMatrix();
    this.render();
  }

  render(): void {
    this.renderer.render(this.scene, this.camera);
  }

  dispose(): void {
    this.disposed = true;
    this.resizeObserver.disconnect();
    this.transformControls.dispose();
    this.renderer.dispose();
    this.controls.dispose();
    if (this.renderer.domElement.parentElement) {
      this.renderer.domElement.parentElement.removeChild(this.renderer.domElement);
    }
  }
}
