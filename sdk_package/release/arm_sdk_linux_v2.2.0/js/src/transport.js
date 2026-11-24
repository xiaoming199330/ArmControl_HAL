/**
 * transport.js — 四种承载（UDP / WebSocket 桥 / Web Serial / 离线模拟器）
 *
 * 上层（client.js）只认识 Transport 接口，换承载业务代码一行不改 ——
 * 和 Python 版 `arm_sdk/transport.py` 同构。
 *
 * 承载选型
 * --------
 *   UdpTransport        Node.js / Electron 主进程        ★ 首选，p50 1.61 ms
 *   WsTransport         浏览器 / Electron 渲染进程        走边车（bridge.js）
 *   WebSerialTransport  浏览器 / Electron 渲染进程        需 Web Serial，p50 4.78 ms
 *   SimTransport        任何环境                          离线开发 / CI
 *
 * ⚠ 浏览器**做不到**原始 UDP（沙箱限制）。想在网页里用网口，必须跑 bridge.js
 *   边车（Node 侧收 WebSocket → 发 UDP），然后在页面里用 WsTransport。
 *
 * ⚠ Node 里没有 `navigator.serial`，所以 WebSerialTransport 在 Node 下不可用；
 *   要串口请用 console.js（文本控制台）或在 Node 侧自己接 serialport 包。
 */

import { packLogical, unpackLogical, ProtocolError } from './protocol.js';

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

/** 收到的事件。kind: 'packet' | 'bad' | 'text' | 'timeout' */
export class RxEvent {
  constructor(kind, fields = {}) {
    this.kind = kind;
    Object.assign(this, fields);
  }
  toString() {
    if (this.kind === 'packet') return `packet seq=${this.seq} cmd=0x${this.cmd.toString(16)}`;
    if (this.kind === 'bad') return `bad ${this.error ? this.error.message : ''}`;
    if (this.kind === 'text') return `text ${JSON.stringify(this.line || '')}`;
    return 'timeout';
  }
}

/* ==========================================================================
 * 基类
 * ========================================================================== */

export class Transport {
  /** @returns {Promise<void>} */
  async open() { throw new Error('未实现'); }
  async close() { throw new Error('未实现'); }
  /** 发一条逻辑包。@param {number} seq @param {number} cmd @param {Uint8Array} payload */
  send(seq, cmd, payload = new Uint8Array(0)) { throw new Error('未实现'); }
  /** 排空收到的字节 → RxEvent[]（非阻塞）。@returns {RxEvent[]} */
  drain() { throw new Error('未实现'); }
  /** 文本通道（只有串口有）。 */
  sendText(line) { throw new Error('本承载无文本通道'); }
  get supportsText() { return false; }
  /** 线路往返耗时（ms），用于统计。 */
  get link() { return this.constructor.name; }
  /** 供 client 判断可否复用。 */
  reset() {}
}

/* ==========================================================================
 * UDP（Node.js / Electron 主进程）★
 * ========================================================================== */

/**
 * UDP 承载：**没有帧头**，一个数据报 = 一个逻辑包。
 *
 * ⚠ 目的地必须是 `ip:5000`；数据报长度必须**恰好** 5+LEN，否则固件静默丢弃。
 * ⚠ 固件把回包发往**请求的源 IP:源端口**，所以本端不需要绑固定端口。
 * ⚠ Node 里必须 `await open()` 之后才能 send —— socket 是异步绑定的。
 */
export class UdpTransport extends Transport {
  /**
   * @param {string} host
   * @param {number} port
   * @param {{timeoutMs?:number, onEvent?:(e:string)=>void}} [opts]
   */
  constructor(host, port = 5000, opts = {}) {
    super();
    this.host = host;
    this.port = port;
    this.timeoutMs = opts.timeoutMs ?? 2000;
    this.onEvent = opts.onEvent || null;
    this._queue = [];
    this._sock = null;
    this._dgram = null;
    this._txPackets = 0;
    this._rxPackets = 0;
    this._rxBytes = 0;
    this._lastRxAt = 0;
  }

  get link() { return `udp://${this.host}:${this.port}`; }
  get stats() {
    return {
      txPackets: this._txPackets, rxPackets: this._rxPackets, rxBytes: this._rxBytes,
      lastRxAt: this._lastRxAt,
    };
  }

  async open() {
    if (this._sock) return;
    // 动态 import：浏览器打包器不会把 node:dgram 打进产物
    const { default: dgram } = await import('node:dgram');
    this._dgram = dgram;
    await new Promise((resolve, reject) => {
      const sock = dgram.createSocket('udp4');
      this._sock = sock;
      sock.on('error', (err) => {
        this._queue.push(new RxEvent('bad', { error: err }));
        if (this.onEvent) this.onEvent('UDP 错误: ' + err.message);
        reject(err);
      });
      sock.on('message', (msg, rinfo) => {
        this._rxPackets++;
        this._rxBytes += msg.length;
        this._lastRxAt = Date.now();
        this._ingest(new Uint8Array(msg), rinfo);
      });
      sock.bind(0, () => {
        try { sock.setRecvBufferSize?.(1 << 20); } catch { /* 非致命 */ }
        resolve();
      });
    });
  }

  _ingest(buf, rinfo) {
    try {
      const { seq, cmd, payload } = unpackLogical(buf);
      this._queue.push(new RxEvent('packet', { seq, cmd, payload, from: rinfo }));
    } catch (e) {
      // 长度不符 / CRC 错：固件不会这样回，多半是网络噪声或别的程序在往本端口发
      this._queue.push(new RxEvent('bad', { error: e, raw: buf }));
      if (this.onEvent) {
        this.onEvent(`收到非法包（${buf.length} B）：${e.message}`);
      }
    }
  }

  send(seq, cmd, payload = new Uint8Array(0)) {
    if (!this._sock) throw new Error('UdpTransport 未 open()');
    const pkt = packLogical(seq, cmd, payload);
    this._sock.send(Buffer.from(pkt), this.port, this.host);
    this._txPackets++;
    return pkt;
  }

  drain() {
    const out = this._queue;
    this._queue = [];
    return out;
  }

  async close() {
    if (!this._sock) return;
    const s = this._sock;
    this._sock = null;
    await new Promise((r) => s.close(r));
  }
}

/* ==========================================================================
 * WebSocket 桥（浏览器 / Electron 渲染进程）
 * ========================================================================== */

/**
 * 浏览器里没有 UDP，所以走边车：
 *
 *     页面(WsTransport) ──ws://127.0.0.1:8765──> bridge.js(Node)
 *                                              └──UDP──> 机械臂
 *
 * 二进制帧 = 逻辑包原样透传（不带 AA55 前缀）。文本帧 = 控制台文本 / 事件。
 */
export class WsTransport extends Transport {
  constructor(url = 'ws://127.0.0.1:8765', opts = {}) {
    super();
    this.url = url;
    this.timeoutMs = opts.timeoutMs ?? 2000;
    this.onEvent = opts.onEvent || null;
    this._queue = [];
    this._ws = null;
  }

  get link() { return this.url; }
  get supportsText() { return true; }

  async open() {
    if (this._ws) return;
    if (typeof WebSocket === 'undefined') {
      throw new Error('当前环境没有 WebSocket；Node 里请直接用 UdpTransport');
    }
    await new Promise((resolve, reject) => {
      const ws = new WebSocket(this.url);
      ws.binaryType = 'arraybuffer';
      this._ws = ws;
      const to = setTimeout(() => reject(new Error('连接桥超时: ' + this.url)), 5000);
      ws.onopen = () => { clearTimeout(to); resolve(); };
      ws.onerror = () => {
        clearTimeout(to);
        this._queue.push(new RxEvent('bad', { error: new Error('WebSocket 错误') }));
        reject(new Error('WebSocket 连接失败: ' + this.url));
      };
      ws.onclose = () => {
        this._queue.push(new RxEvent('bad', { error: new Error('桥断开') }));
        if (this.onEvent) this.onEvent('桥连接已断开');
      };
      ws.onmessage = (ev) => {
        if (ev.data instanceof ArrayBuffer) {
          const buf = new Uint8Array(ev.data);
          try {
            const { seq, cmd, payload } = unpackLogical(buf);
            this._queue.push(new RxEvent('packet', { seq, cmd, payload }));
          } catch (e) {
            // 桥也可能推事件包（JSON 文本），二进制解析失败按事件处理
            this._queue.push(new RxEvent('bad', { error: e, raw: buf }));
          }
        } else {
          this._queue.push(new RxEvent('text', { line: String(ev.data) }));
        }
      };
    });
  }

  send(seq, cmd, payload = new Uint8Array(0)) {
    if (!this._ws) throw new Error('WsTransport 未 open()');
    const pkt = packLogical(seq, cmd, payload);
    this._ws.send(pkt);
    return pkt;
  }

  sendText(line) {
    if (!this._ws) throw new Error('WsTransport 未 open()');
    this._ws.send(String(line));
  }

  drain() {
    const out = this._queue;
    this._queue = [];
    return out;
  }

  async close() {
    if (!this._ws) return;
    const ws = this._ws;
    this._ws = null;
    try { ws.close(); } catch { /* 忽略 */ }
  }
}

/* ==========================================================================
 * Web Serial（浏览器 / Electron 渲染进程，串口）
 * ========================================================================== */

/**
 * 串口承载。**与固件控制台文本共口**：二进制帧以 `AA 55` 起头，其余字节归文本。
 *
 * ⚠ 浏览器里必须在**用户手势**（点击）里调 open()，否则权限弹窗会被拦。
 * ⚠ 别发以 0xAA 开头的文本命令，会被固件协议状态机吃掉。
 */
export class WebSerialTransport extends Transport {
  /**
   * @param {{baudRate?:number, port?:any}} [opts]
   */
  constructor(opts = {}) {
    super();
    this.baudRate = opts.baudRate ?? 115200;
    this._port = opts.port || null;
    this._reader = null;
    this._writer = null;
    this._queue = [];
    this._textBuf = [];
    this._reading = false;
    this.onEvent = opts.onEvent || null;
    // 二进制帧缓冲（AA55 状态机，和 FrameParser 同逻辑，这里按块处理）
    this._buf = [];
    this._lastByteAt = 0;
    this.timeoutMs = opts.timeoutMs ?? 2000;
  }

  get link() { return `webserial ${this.baudRate}`; }
  get supportsText() { return true; }

  /** 已授权的串口列表（Electron 里可用来做下拉框）。 */
  static async listPorts() {
    if (typeof navigator === 'undefined' || !navigator.serial) return [];
    return navigator.serial.getPorts();
  }

  /** 弹权限框选串口。 */
  static async requestPort() {
    if (typeof navigator === 'undefined' || !navigator.serial) {
      throw new Error('当前环境没有 Web Serial（需 Chrome/Edge 89+ 或 Electron，且 https/localhost）');
    }
    return navigator.serial.requestPort();
  }

  async open() {
    if (this._port) { await this._openPort(this._port); return; }
    await this._openPort(await WebSerialTransport.requestPort());
  }

  async _openPort(port) {
    if (typeof navigator === 'undefined' || !navigator.serial) {
      throw new Error('当前环境没有 Web Serial');
    }
    this._port = port;
    await port.open({ baudRate: this.baudRate, dataBits: 8, stopBits: 1, parity: 'none' });
    this._writer = port.writable.getWriter();
    this._reading = true;
    this._readLoop();
  }

  async _readLoop() {
    const dec = new TextDecoder();
    while (this._reading && this._port && this._port.readable) {
      this._reader = this._port.readable.getReader();
      try {
        for (;;) {
          const { value, done } = await this._reader.read();
          if (done) break;
          if (value) this._feedChunk(value);
        }
      } catch (e) {
        this._queue.push(new RxEvent('bad', { error: e }));
      } finally {
        try { this._reader.releaseLock(); } catch { /* 忽略 */ }
      }
      if (!this._reading) break;
    }
    dec.decode();   // 冲掉解码器
  }

  /** 把一块字节按 AA55 状态机切开：二进制帧 vs 文本。 */
  _feedChunk(chunk) {
    const now = Date.now();
    for (const byte of chunk) {
      if (this._buf.length && now - this._lastByteAt > 200) this._buf = [];
      this._lastByteAt = now;
      this._buf.push(byte);
      const b = this._buf, n = b.length;

      if (n === 1) {
        if (b[0] !== 0xAA) { this._buf = []; this._textBuf.push(byte); this._flushText(); }
        continue;
      }
      if (n === 2) {
        if (b[1] === 0x55) continue;
        if (b[1] === 0xAA) { this._buf.shift(); continue; }
        // 不是帧头：把首字节当文本吐回
        this._buf = [];
        this._textBuf.push(0xAA, b[1]);
        this._flushText();
        continue;
      }
      if (n < 5) continue;
      if (n === 5 && b[4] > 64) { this._buf = []; continue; }
      const need = 5 + b[4] + 2;
      if (n < need) continue;

      const seq = b[2], cmd = b[3], len = b[4];
      const payload = Uint8Array.from(b.slice(5, 5 + len));
      const got = b[5 + len] | (b[5 + len + 1] << 8);
      let want = 0xFFFF;
      for (let i = 2; i < 5 + len; i++) {
        want ^= (b[i] & 0xFF) << 8;
        for (let k = 0; k < 8; k++) {
          want = (want & 0x8000) ? ((want << 1) ^ 0x1021) & 0xFFFF : (want << 1) & 0xFFFF;
        }
      }
      this._buf = [];
      if (got !== want) this._queue.push(new RxEvent('bad', { error: new ProtocolError('CRC 错'), seq, cmd }));
      else this._queue.push(new RxEvent('packet', { seq, cmd, payload }));
    }
  }

  _flushText() {
    // 文本按 \n 切行
    let s = '';
    for (const c of this._textBuf) s += String.fromCharCode(c);
    const lines = s.split(/\r?\n/);
    const tail = lines.pop();
    this._textBuf = tail ? Array.from(tail, (c) => c.charCodeAt(0)) : [];
    for (const line of lines) if (line) this._queue.push(new RxEvent('text', { line }));
  }

  send(seq, cmd, payload = new Uint8Array(0)) {
    if (!this._writer) throw new Error('WebSerialTransport 未 open()');
    const pkt = packLogical(seq, cmd, payload);
    const out = new Uint8Array(pkt.length + 2);
    out[0] = 0xAA; out[1] = 0x55;
    out.set(pkt, 2);
    this._writer.write(out);
    return out;
  }

  sendText(line) {
    if (!this._writer) throw new Error('WebSerialTransport 未 open()');
    this._writer.write(new TextEncoder().encode(String(line) + '\r\n'));
  }

  drain() {
    const out = this._queue;
    this._queue = [];
    return out;
  }

  async close() {
    this._reading = false;
    try { await this._reader?.cancel(); } catch { /* 忽略 */ }
    try { this._writer?.releaseLock(); } catch { /* 忽略 */ }
    try { await this._port?.close(); } catch { /* 忽略 */ }
    this._writer = null;
  }
}

/* ==========================================================================
 * 工厂
 * ========================================================================== */

/**
 * 按 URI 造承载。
 *
 *   net://192.168.7.120:5000      Node UDP（首选）
 *   udp://192.168.7.120:5000      同上（别名）
 *   ws://127.0.0.1:8765           浏览器走桥
 *   webserial://                  浏览器串口（弹权限框）
 *   sim://                        离线模拟器（惰性 import 避免循环依赖）
 *
 * @param {string} uri
 * @param {object} [opts]
 * @returns {Promise<Transport>}
 */
export async function makeTransport(uri, opts = {}) {
  const s = String(uri || '').trim();
  const m = /^([a-zA-Z]+):\/\/(.*)$/.exec(s);
  const scheme = m ? m[1].toLowerCase() : 'net';
  const rest = m ? m[2] : '';

  switch (scheme) {
    case 'net':
    case 'udp': {
      let host = rest, port = 5000;
      if (rest.includes(':')) {
        const i = rest.lastIndexOf(':');
        host = rest.slice(0, i) || '127.0.0.1';
        port = parseInt(rest.slice(i + 1), 10) || 5000;
      } else if (!rest) {
        host = '192.168.7.120';
      }
      if (scheme === 'udp' && /^[a-z]+$/i.test(rest) && !rest.includes(':')) {
        // 允许 "udp://sim" 这种写法，不常见但别误判成主机名
      }
      return new UdpTransport(host, port, opts);
    }
    case 'ws':
    case 'wss':
      return new WsTransport(s, opts);
    case 'webserial':
    case 'serial':
      return new WebSerialTransport(opts);
    case 'sim': {
      const { SimTransport } = await import('./sim.js');
      return new SimTransport(opts);
    }
    default:
      throw new Error('不认识的承载: ' + uri +
        '\n  可选：net://<ip>:<port> | ws://<host>:<port> | webserial:// | sim://');
  }
}

export { sleep };
