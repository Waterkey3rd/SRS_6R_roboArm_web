import type { RosConnectionKind } from './types';

interface TopicSubscription {
  id: string;
  topic: string;
  type: string;
  queueLength: number;
  handler: (message: unknown) => void;
}

interface PendingService {
  resolve: (value: unknown) => void;
  reject: (error: Error) => void;
  timer: number;
}

type StateListener = (state: RosConnectionKind) => void;
type ErrorListener = (error: Error) => void;

let nextRequestId = 1;

function makeId(prefix: string): string {
  return `${prefix}-${nextRequestId++}-${Date.now().toString(36)}`;
}

/**
 * Minimal ROS2 rosbridge websocket client.
 *
 * The bridge side (`eng26_web_bridge`) intentionally exposes only standard
 * ROS message types, so this client can live without roslib or message IDL
 * generation. It implements the `subscribe`, `unsubscribe`, `publish` and
 * `call_service` operations used by the WebView.
 */
export class RosBridgeClient {
  url: string;
  connectionState: RosConnectionKind = 'disconnected';

  private socket: WebSocket | null = null;
  private subscriptions = new Map<string, TopicSubscription>();
  private advertisedTopics = new Map<string, string>();
  private pendingServices = new Map<string, PendingService>();
  private stateListeners = new Set<StateListener>();
  private errorListeners = new Set<ErrorListener>();
  private manualClose = false;
  private reconnectAttempt = 0;
  private reconnectTimer = 0;

  constructor(url: string) {
    this.url = url;
  }

  onState(listener: StateListener): () => void {
    this.stateListeners.add(listener);
    return () => this.stateListeners.delete(listener);
  }

  onError(listener: ErrorListener): () => void {
    this.errorListeners.add(listener);
    return () => this.errorListeners.delete(listener);
  }

  get isConnected(): boolean {
    return this.socket?.readyState === WebSocket.OPEN;
  }

  connect(): void {
    this.manualClose = false;
    this.openSocket();
  }

  disconnect(): void {
    this.manualClose = true;
    this.clearReconnectTimer();
    this.rejectPendingServices(new Error('rosbridge connection closed'));
    if (this.socket) {
      try {
        this.socket.close();
      } catch {
        // WebSocket may already be closing.
      }
      this.socket = null;
    }
    this.setState('disconnected');
  }

  setUrl(url: string): void {
    const wasConnected = this.isConnected;
    this.disconnect();
    this.url = url;
    if (wasConnected) this.connect();
  }

  publish(topic: string, message: unknown, type?: string): boolean {
    if (!this.isConnected) return false;
    if (type && !this.advertisedTopics.has(topic)) this.advertise(topic, type);
    const payload: Record<string, unknown> = {
      op: 'publish',
      topic,
      msg: message,
      queue_length: 1,
      latch: false,
    };
    return this.send(payload);
  }

  advertise(topic: string, type: string, queueLength = 1, latch = false): boolean {
    this.advertisedTopics.set(topic, type);
    if (!this.isConnected) return false;
    return this.send({ op: 'advertise', topic, type, queue_length: queueLength, queue_size: queueLength, latch });
  }

  subscribe(topic: string, type: string, handler: (message: unknown) => void, queueLength = 1): () => void {
    const id = makeId('sub');
    const subscription: TopicSubscription = { id, topic, type, queueLength, handler };
    this.subscriptions.set(id, subscription);
    if (this.isConnected) this.sendSubscribe(subscription);

    return () => {
      if (!this.subscriptions.delete(id)) return;
      if (this.isConnected) {
        this.send({ op: 'unsubscribe', id, topic });
      }
    };
  }

  callService(service: string, type: string, args: Record<string, unknown> = {}, timeoutMs = 5000): Promise<unknown> {
    if (!this.isConnected) {
      return Promise.reject(new Error('rosbridge 未连接'));
    }

    const id = makeId('srv');
    return new Promise<unknown>((resolve, reject) => {
      const timer = window.setTimeout(() => {
        this.pendingServices.delete(id);
        reject(new Error(`服务调用超时: ${service}`));
      }, timeoutMs);

      this.pendingServices.set(id, { resolve, reject, timer });
      const sent = this.send({ op: 'call_service', id, service, type, args });
      if (!sent) {
        window.clearTimeout(timer);
        this.pendingServices.delete(id);
        reject(new Error('rosbridge 未连接'));
      }
    });
  }

  private openSocket(): void {
    this.clearReconnectTimer();
    if (this.socket && (this.socket.readyState === WebSocket.OPEN || this.socket.readyState === WebSocket.CONNECTING)) {
      return;
    }

    this.setState('connecting');
    let socket: WebSocket;
    try {
      socket = new WebSocket(this.url);
    } catch (error: any) {
      this.emitError(new Error(`无法创建 WebSocket: ${error?.message || String(error)}`));
      this.setState('error');
      this.scheduleReconnect();
      return;
    }

    this.socket = socket;

    socket.onopen = () => {
      if (this.socket !== socket) return;
      this.reconnectAttempt = 0;
      this.setState('connected');
      for (const [topic, type] of this.advertisedTopics) {
        this.send({ op: 'advertise', topic, type, queue_length: 1, queue_size: 1, latch: false });
      }
      for (const subscription of this.subscriptions.values()) this.sendSubscribe(subscription);
    };

    socket.onmessage = (event) => {
      if (typeof event.data !== 'string') return;
      this.handleMessage(event.data);
    };

    socket.onerror = () => {
      this.emitError(new Error(`rosbridge 连接错误: ${this.url}`));
    };

    socket.onclose = () => {
      if (this.socket === socket) this.socket = null;
      this.rejectPendingServices(new Error('rosbridge 连接已断开'));
      this.setState('disconnected');
      if (!this.manualClose) this.scheduleReconnect();
    };
  }

  private handleMessage(raw: string): void {
    let payload: Record<string, unknown>;
    try {
      payload = JSON.parse(raw) as Record<string, unknown>;
    } catch {
      this.emitError(new Error('rosbridge 返回了无效 JSON'));
      return;
    }

    const op = String(payload.op ?? '');
    if (op === 'publish') {
      const topic = String(payload.topic ?? '');
      for (const subscription of this.subscriptions.values()) {
        if (subscription.topic !== topic) continue;
        subscription.handler(payload.msg);
      }
      return;
    }

    if (op === 'service_response') {
      const id = String(payload.id ?? '');
      const pending = this.pendingServices.get(id);
      if (!pending) return;
      this.pendingServices.delete(id);
      window.clearTimeout(pending.timer);

      const result = payload.result;
      const serviceOk = result === true || (result === undefined && payload.values !== undefined);
      if (serviceOk) {
        pending.resolve(payload.values ?? result);
        return;
      }

      const values = payload.values as { message?: unknown } | undefined;
      pending.reject(new Error(String(values?.message ?? payload.message ?? '服务调用失败')));
      return;
    }

    if (op === 'status') {
      const level = String(payload.level ?? '');
      if (level === 'error') {
        this.emitError(new Error(String(payload.msg ?? 'rosbridge 错误状态')));
      }
    }
  }

  private sendSubscribe(subscription: TopicSubscription): boolean {
    return this.send({
      op: 'subscribe',
      id: subscription.id,
      topic: subscription.topic,
      type: subscription.type,
      queue_length: subscription.queueLength,
      throttle_rate: 0,
      compression: 'none',
    });
  }

  private send(payload: Record<string, unknown>): boolean {
    if (!this.socket || this.socket.readyState !== WebSocket.OPEN) return false;
    this.socket.send(JSON.stringify(payload));
    return true;
  }

  private setState(state: RosConnectionKind): void {
    if (this.connectionState === state) return;
    this.connectionState = state;
    for (const listener of this.stateListeners) listener(state);
  }

  private emitError(error: Error): void {
    for (const listener of this.errorListeners) listener(error);
  }

  private rejectPendingServices(error: Error): void {
    for (const pending of this.pendingServices.values()) {
      window.clearTimeout(pending.timer);
      pending.reject(error);
    }
    this.pendingServices.clear();
  }

  private scheduleReconnect(): void {
    if (this.manualClose || this.reconnectTimer) return;
    const delayMs = Math.min(5000, 500 * 2 ** this.reconnectAttempt) + Math.floor(Math.random() * 250);
    this.reconnectAttempt += 1;
    this.reconnectTimer = window.setTimeout(() => {
      this.reconnectTimer = 0;
      this.openSocket();
    }, delayMs);
  }

  private clearReconnectTimer(): void {
    if (!this.reconnectTimer) return;
    window.clearTimeout(this.reconnectTimer);
    this.reconnectTimer = 0;
  }
}