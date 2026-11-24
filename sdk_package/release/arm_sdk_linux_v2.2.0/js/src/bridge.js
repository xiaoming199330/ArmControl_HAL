#!/usr/bin/env node
/**
 * bridge.js — WebSocket ↔ UDP 边车（浏览器用网口的**唯一**通路）
 *
 * 为什么必须有它
 * --------------
 * 浏览器的 JS 沙箱**做不到**原始 UDP（没有 socket API）。所以网页想在
 * p50 1.61ms 的网口上跟机械臂说话，只能让一个 Node 进程当"翻译"：
 *
 *     页面(WsTransport)  ──ws://127.0.0.1:8765──>  bridge.js(Node)  ──UDP──>  机械臂
 *                        <──二进制帧（逻辑包）──                   <──回包────
 *
 * 逻辑包**原样透传**（不带 AA55 帧头），所以上下行逐字节等同 Node 直连 UDP ——
 * 这也是为什么跨链路字节级一致性测试能覆盖到浏览器这条路。
 *
 * 零依赖
 * ------
 * 不引 `ws` 包：本文件自带 RFC 6455 握手 + 帧编解码（服务端帧不掩码，
 * 客户端帧必须掩码）。整个 SDK 因此 `npm install` 一个包都不用装。
 *
 * 每个连接**独占一个 UDP socket**（绑 0 号端口）
 * ------------------------------------------------
 * 固件把回包发往"请求的源 IP:源端口"。给每个浏览器连接单独开一个 socket，
 * 就把不同页面的回包天然隔离开 —— 于是**同一个边车能同时服务两路**：
 * 主臂一条连接、从臂另一条连接，互不串包（遥操就靠这个）。
 *
 * 目标地址怎么给
 * --------------
 *   1) 启动默认：   node src/bridge.js --target 192.168.7.120:5000
 *   2) 连接级覆盖： ws://127.0.0.1:8765/?target=192.168.7.121:5000
 *   3) 运行期切换： 文本帧 {"op":"setTarget","target":"192.168.7.121:5000"}
 *
 * 安全
 * ----
 * 默认只允许 loopback 连入（`--allow 127.0.0.1`）。要放开用 `--allow 0.0.0.0/0`
 * 或逗号列表。别在公网上裸跑这个边车：它没有任何鉴权。
 *
 * 用法
 * ----
 *    node src/bridge.js                                  # 默认目标 192.168.7.120:5000
 *    node src/bridge.js --target 192.168.7.120:5000 --port 8765 -v
 *    node src/bridge.js --target 192.168.7.120:5000 --allow 127.0.0.1,192.168.1.5
 *    node src/bridge.js --selftest                        # 起临时端口自检后退出
 */

import http from 'node:http';
import dgram from 'node:dgram';
import crypto from 'node:crypto';
import { fileURLToPath } from 'node:url';

/* ==========================================================================
 * 常量 / 小工具
 * ========================================================================== */

/** RFC 6455 握手指定的魔术字符串。 */
const WS_GUID = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11';

const OP = {
  CONT: 0x0, TEXT: 0x1, BIN: 0x2, CLOSE: 0x8, PING: 0x9, PONG: 0xA,
};

/** WASM/Rust 那套用来算 Sec-WebSocket-Accept。 */
export function wsAccept(key) {
  return crypto.createHash('sha1').update(String(key) + WS_GUID).digest('base64');
}

/** "ip:port" / "ip" → {host, port}；port 缺省 5000（固件网口固定 5000）。 */
export function parseTarget(s, defaultPort = 5000) {
  const t = String(s || '').trim();
  if (!t) return null;
  const i = t.lastIndexOf(':');
  if (i < 0) return { host: t, port: defaultPort };
  const host = t.slice(0, i) || '127.0.0.1';
  const port = parseInt(t.slice(i + 1), 10) || defaultPort;
  return { host, port };
}

/* ==========================================================================
 * WebSocket 帧编解码（RFC 6455 子集）
 * ========================================================================== */

/** 服务端 → 客户端：不掩码。opcode 0x1 文本 / 0x2 二进制。 */
export function encodeWsFrame(opcode, payload) {
  const body = Buffer.isBuffer(payload) ? payload : Buffer.from(payload);
  const len = body.length;
  let header;
  if (len < 126) {
    header = Buffer.alloc(2);
    header[1] = len;
  } else if (len < 65536) {
    header = Buffer.alloc(4);
    header[1] = 126;
    header.writeUInt16BE(len, 2);
  } else {
    header = Buffer.alloc(10);
    header[1] = 127;
    header.writeBigUInt64BE(BigInt(len), 2);
  }
  header[0] = 0x80 | (opcode & 0x0f);          // FIN=1
  return Buffer.concat([header, body]);
}

/**
 * 增量解析客户端帧。客户端帧**必须**掩码，否则按协议错误断开。
 *
 * @param {(opcode:number, payload:Buffer)=>void} onMessage
 * @param {(code:number, reason:string)=>void} onClose
 * @param {(msg:string)=>void} [onFatal]
 */
export class WsFrameParser {
  constructor(onMessage, onClose, onFatal) {
    this.onMessage = onMessage;
    this.onClose = onClose;
    this.onFatal = onFatal || (() => {});
    this._buf = Buffer.alloc(0);
    this._frag = [];
    this._fragOp = 0;
  }

  push(chunk) {
    this._buf = this._buf.length ? Buffer.concat([this._buf, chunk]) : Buffer.from(chunk);
    for (;;) {
      if (this._buf.length < 2) return;
      const b0 = this._buf[0];
      const b1 = this._buf[1];
      const fin = (b0 & 0x80) !== 0;
      const opcode = b0 & 0x0f;
      const rsv = b0 & 0x70;
      const masked = (b1 & 0x80) !== 0;

      if (rsv) { this._fail('RSV 位非零（未协商扩展）'); return; }

      let len = b1 & 0x7f;
      let off = 2;
      if (len === 126) {
        if (this._buf.length < 4) return;
        len = this._buf.readUInt16BE(2);
        off = 4;
      } else if (len === 127) {
        if (this._buf.length < 10) return;
        const big = this._buf.readBigUInt64BE(2);
        if (big > 8n * 1024n * 1024n) { this._fail('帧过大'); return; }
        len = Number(big);
        off = 10;
      }

      let maskKey = null;
      if (masked) {
        if (this._buf.length < off + 4) return;
        maskKey = this._buf.subarray(off, off + 4);
        off += 4;
      }
      if (this._buf.length < off + len) return;

      const payload = Buffer.from(this._buf.subarray(off, off + len));
      if (maskKey) for (let i = 0; i < payload.length; i++) payload[i] ^= maskKey[i & 3];
      this._buf = this._buf.subarray(off + len);

      // —— 控制帧（可以插在分片中间）——
      if (opcode === OP.CLOSE) {
        const code = payload.length >= 2 ? payload.readUInt16BE(0) : 1005;
        this.onClose(code, payload.length > 2 ? payload.subarray(2).toString('utf8') : '');
        return;
      }
      if (opcode === OP.PING) { this.onMessage(OP.PING, payload); continue; }
      if (opcode === OP.PONG) { this.onMessage(OP.PONG, payload); continue; }

      // —— 数据帧 ——
      if (opcode === OP.CONT) {
        if (!this._fragOp) { this._fail('收到孤立的继续帧'); return; }
        this._frag.push(payload);
        if (fin) { const whole = Buffer.concat(this._frag); const op = this._fragOp;
                   this._frag = []; this._fragOp = 0; this.onMessage(op, whole); }
        continue;
      }
      if (opcode === OP.TEXT || opcode === OP.BIN) {
        if (!fin) { this._fragOp = opcode; this._frag = [payload]; continue; }
        this.onMessage(opcode, payload);
        continue;
      }
      this._fail('未知 opcode 0x' + opcode.toString(16));
      return;
    }
  }

  _fail(msg) {
    if (this._fatal) return;
    this._fatal = true;
    try { this.onFatal(msg); } catch { /* 忽略 */ }
    try { this.onClose(1002, msg); } catch { /* 忽略 */ }
  }
}

/* ==========================================================================
 * 源地址白名单
 * ========================================================================== */

/** 支持 `127.0.0.1` / `192.168.*.*` / `10.0.0.0/8` / `0.0.0.0/0`。 */
export function makeAllowFilter(spec) {
  const rules = String(spec || '127.0.0.1').split(',').map((s) => s.trim()).filter(Boolean);
  const compiled = rules.map((r) => {
    if (r.includes('/')) {
      const [ip, bitsStr] = r.split('/');
      const bits = parseInt(bitsStr, 10);
      const v4 = ip.split('.').map(Number);
      const mask = bits === 0 ? 0 : (0xFFFFFFFF << (32 - bits)) >>> 0;
      const net = (((v4[0] << 24) | (v4[1] << 16) | (v4[2] << 8) | v4[3]) >>> 0) & mask;
      return (addr) => {
        const a = addr.split('.').map(Number);
        if (a.length !== 4 || a.some((n) => !Number.isFinite(n))) return false;
        const v = (((a[0] << 24) | (a[1] << 16) | (a[2] << 8) | a[3]) >>> 0) & mask;
        return v === net;
      };
    }
    if (r.includes('*')) {
      const re = new RegExp('^' + r.split('.').map((p) => (p === '*' ? '\\d+' : p)).join('\\.') + '$');
      return (addr) => re.test(addr);
    }
    if (r === 'localhost') return (addr) => addr === '127.0.0.1' || addr === '::1' || addr === '::ffff:127.0.0.1';
    return (addr) => addr === r || addr === '::ffff:' + r;
  });
  return (addr) => compiled.some((fn) => fn(addr));
}

/* ==========================================================================
 * 单个浏览器连接
 * ========================================================================== */

let CLIENT_ID = 0;

/**
 * 一条 WS 连接 = 页面。它独占一个 UDP socket，所以两路（主/从臂）互不串包。
 */
export class BridgeClient {
  /**
   * @param {import('node:net').Socket} sock 已升级的 TCP socket
   * @param {import('node:http').IncomingMessage} req
   * @param {{target:{host:string,port:number}, defaultPort?:number,
   *          log?:(s:string)=>void, verbose?:boolean}} opts
   */
  constructor(sock, req, opts) {
    this.id = ++CLIENT_ID;
    this.sock = sock;
    this.req = req;
    this.log = opts.log || (() => {});
    this.verbose = !!opts.verbose;
    this.defaultPort = opts.defaultPort ?? 5000;

    this.closed = false;
    this.target = opts.target ? { ...opts.target } : null;
    this.stats = { tx: 0, rx: 0, txBytes: 0, rxBytes: 0, bad: 0, openedAt: Date.now() };

    this._udp = dgram.createSocket('udp4');
    this._pending = [];                         // bind 完成前先攒着

    this._parser = new WsFrameParser(
      (op, payload) => this._onWsMessage(op, payload),
      (code, reason) => this._close(code, reason),
      (msg) => this._sendText({ type: 'error', msg }),
    );

    this._udp.on('message', (msg, rinfo) => this._onUdpMessage(msg, rinfo));
    this._udp.on('error', (err) => {
      this.log(`#${this.id} UDP 错误: ${err.message}`);
      this._sendText({ type: 'error', msg: 'UDP 错误: ' + err.message });
    });
    this._udp.bind(0, () => {
      for (const p of this._pending) this._udp.send(p);
      this._pending = [];
    });

    sock.on('data', (buf) => this._parser.push(buf));
    sock.on('error', () => this._close(1006, ''));
    sock.on('close', () => this._close(1006, ''));

    // 连接级 target 覆盖：?target=192.168.7.121:5000
    try {
      const u = new URL(req.url || '/', 'http://localhost');
      const t = u.searchParams.get('target');
      if (t) {
        const parsed = parseTarget(t, this.defaultPort);
        if (parsed) this.target = parsed;
      }
    } catch { /* URL 不合法就算了 */ }

    const who = `${req.socket.remoteAddress}:${req.socket.remotePort}`;
    this.log(`#${this.id} 连接来自 ${who} → 目标 ${this.targetStr}`);
    this._sendText({ type: 'hello', clientId: this.id, target: this.targetStr });
  }

  get targetStr() {
    return this.target ? `${this.target.host}:${this.target.port}` : '(未设置)';
  }

  /* ---- WS → UDP ---- */

  _onWsMessage(opcode, payload) {
    if (opcode === OP.PING) { this._sendRaw(encodeWsFrame(OP.PONG, payload)); return; }
    if (opcode === OP.PONG) return;

    if (opcode === OP.BIN) {
      this._toUdp(payload, 'bin');
      return;
    }
    if (opcode === OP.TEXT) {
      const s = payload.toString('utf8');
      // 疑似控制帧：`{` 开头且能解析出 op
      const trimmed = s.trim();
      if (trimmed.startsWith('{')) {
        let obj = null;
        try { obj = JSON.parse(trimmed); } catch { obj = null; }
        if (obj && typeof obj === 'object' && obj.op) { this._onControl(obj); return; }
        if (obj && typeof obj === 'object' && obj.target) { this._setTarget(obj.target); return; }
      }
      // 其余当"裸文本"原样打出去（固件网口不认文本，但保持透明，便于调试/抓包）
      this._toUdp(payload, 'text');
    }
  }

  _onControl(obj) {
    switch (obj.op) {
      case 'setTarget':
        this._setTarget(obj.target);
        break;
      case 'stats':
        this._sendText({ type: 'stats', clientId: this.id, ...this._statsOut() });
        break;
      case 'ping':
        this._sendText({ type: 'pong', clientId: this.id, t: Date.now() });
        break;
      case 'close':
        this._close(1000, '');
        break;
      default:
        this._sendText({ type: 'error', msg: '未知 op: ' + obj.op });
    }
  }

  _setTarget(t) {
    const parsed = parseTarget(t, this.defaultPort);
    if (!parsed) { this._sendText({ type: 'error', msg: 'target 不合法: ' + t }); return; }
    this.target = parsed;
    this.log(`#${this.id} 目标切到 ${this.targetStr}`);
    this._sendText({ type: 'target', target: this.targetStr });
  }

  _toUdp(buf, kind) {
    if (!this.target) {
      this.stats.bad++;
      this._sendText({ type: 'error', msg: '未设置目标；用 --target 或 ?target=ip:port' });
      return;
    }
    const b = Buffer.isBuffer(buf) ? buf : Buffer.from(buf);
    this.stats.tx++;
    this.stats.txBytes += b.length;
    try {
      this._udp.send(b, this.target.port, this.target.host);
    } catch (e) {
      this.stats.bad++;
      this._sendText({ type: 'error', msg: 'UDP 发送失败: ' + e.message });
    }
    if (this.verbose) this.log(`#${this.id} → ${this.targetStr} ${b.length}B (${kind})`);
  }

  /* ---- UDP → WS ---- */

  _onUdpMessage(msg, rinfo) {
    this.stats.rx++;
    this.stats.rxBytes += msg.length;
    // 只回推"目标端口发来的"：同一台机器上别的服务也可能往本 socket 发东西。
    // 源**地址**不强校验（NAT / 多网卡场景下可能与 --target 写的不同），只看端口。
    if (this.target && rinfo.port !== this.target.port) {
      this.stats.bad++;
      if (this.verbose) this.log(`#${this.id} 丢弃来路不明的包 ${rinfo.address}:${rinfo.port}`);
      return;
    }
    this._sendRaw(encodeWsFrame(OP.BIN, msg));
    if (this.verbose) this.log(`#${this.id} ← ${rinfo.address}:${rinfo.port} ${msg.length}B`);
  }

  /* ---- 下行 ---- */

  _sendRaw(buf) {
    if (this.closed) return;
    try { this.sock.write(buf); } catch { /* 对端已断 */ }
  }

  _sendText(objOrStr) {
    const s = typeof objOrStr === 'string' ? objOrStr : JSON.stringify(objOrStr);
    this._sendRaw(encodeWsFrame(OP.TEXT, Buffer.from(s, 'utf8')));
  }

  _statsOut() {
    return {
      target: this.targetStr,
      tx: this.stats.tx, rx: this.stats.rx,
      txBytes: this.stats.txBytes, rxBytes: this.stats.rxBytes,
      bad: this.stats.bad,
      uptimeS: (Date.now() - this.stats.openedAt) / 1000,
    };
  }

  close() { this._close(1000, ''); }

  _close(code, reason) {
    if (this.closed) return;
    this.closed = true;
    try { this.sock.write(encodeWsFrame(OP.CLOSE, Buffer.from([code >> 8, code & 0xff]))); } catch { /* 忽略 */ }
    try { this.sock.end(); } catch { /* 忽略 */ }
    try { this.sock.destroy(); } catch { /* 忽略 */ }
    try { this._udp.close(); } catch { /* 忽略 */ }
    this.log(`#${this.id} 断开（${code}${reason ? ' ' + reason : ''}）` +
             ` tx=${this.stats.tx} rx=${this.stats.rx}`);
  }
}

/* ==========================================================================
 * 边车主体
 * ========================================================================== */

/**
 * 起一个 WebSocket↔UDP 边车。
 *
 * @param {{port?:number, host?:string, target?:string, defaultPort?:number,
 *          allow?:string, verbose?:boolean, log?:(s:string)=>void}} [opts]
 * @returns {Promise<{server:import('node:http').Server, port:number, clients:Set<any>,
 *                    stats:()=>object, close:()=>Promise<void>}>}
 */
export async function createBridge(opts = {}) {
  const host = opts.host || '127.0.0.1';
  const port = opts.port ?? 8765;
  const defaultPort = opts.defaultPort ?? 5000;
  const target = opts.target ? parseTarget(opts.target, defaultPort) : null;
  const allow = makeAllowFilter(opts.allow ?? '127.0.0.1');
  const verbose = !!opts.verbose;
  const log = opts.log || ((s) => console.log('[bridge] ' + s));

  const clients = new Set();
  const totals = { conns: 0, denied: 0 };

  const server = http.createServer((req, res) => {
    // 普通 HTTP 请求：给个探活/自述页，方便浏览器里点开看
    res.writeHead(200, { 'content-type': 'text/plain; charset=utf-8' });
    res.end('arm-sdk-js bridge\n' +
            `target=${target ? target.host + ':' + target.port : '(未设置)'}\n` +
            `clients=${clients.size}\n\n` +
            '浏览器请用 WsTransport 连 ws://' + host + ':' + port + '/\n');
  });

  server.on('upgrade', (req, sock, head) => {
    const key = req.headers['sec-websocket-key'];
    const up = String(req.headers.upgrade || '').toLowerCase();
    if (up !== 'websocket' || !key) { sock.destroy(); return; }

    const ip = req.socket.remoteAddress || '';
    if (!allow(ip)) {
      totals.denied++;
      log(`拒绝 ${ip}（不在 --allow 白名单）`);
      sock.write('HTTP/1.1 403 Forbidden\r\n\r\n');
      sock.destroy();
      return;
    }

    sock.write(
      'HTTP/1.1 101 Switching Protocols\r\n' +
      'Upgrade: websocket\r\n' +
      'Connection: Upgrade\r\n' +
      'Sec-WebSocket-Accept: ' + wsAccept(key) + '\r\n' +
      '\r\n');

    totals.conns++;
    const client = new BridgeClient(sock, req, { target, defaultPort, log, verbose });
    clients.add(client);
    sock.on('close', () => clients.delete(client));

    if (head && head.length) client._parser.push(head);
  });

  await new Promise((resolve, reject) => {
    server.once('error', reject);
    server.listen(port, host, () => {
      server.removeListener('error', reject);
      resolve();
    });
  });

  const actualPort = server.address().port;
  log(`监听 ws://${host}:${actualPort}  默认目标 ${target ? target.host + ':' + target.port : '(每连接用 ?target= 指定)'}`);
  log(`白名单 ${opts.allow ?? '127.0.0.1'}；浏览器里 openArm('ws://${host}:${actualPort}/')`);

  return {
    server,
    host,
    port: actualPort,
    clients,
    stats: () => ({
      conns: totals.conns,
      denied: totals.denied,
      active: clients.size,
      clients: [...clients].map((c) => c._statsOut()),
    }),
    async close() {
      for (const c of [...clients]) c.close();
      clients.clear();
      await new Promise((r) => server.close(r));
    },
  };
}

/* ==========================================================================
 * 自检
 * ========================================================================== */

/** 边车离线自检：起临时端口，跑一遍握手/帧/白名单/目标解析。 */
export async function selftest() {
  let pass = 0; const fail = [];
  const ok = (cond, msg) => { if (cond) pass++; else fail.push(msg); };

  // 1) 帧编解码
  const f1 = encodeWsFrame(OP.TEXT, Buffer.from('hi'));
  ok(f1[0] === 0x81 && f1[1] === 2 && f1.toString('utf8', 2) === 'hi', 'encode TEXT 短帧');
  const f2 = encodeWsFrame(OP.BIN, Buffer.alloc(200, 7));
  ok(f2[1] === 126 && f2.readUInt16BE(2) === 200, 'encode BIN 126 扩展长度');
  const f3 = encodeWsFrame(OP.BIN, Buffer.alloc(70000, 1));
  ok(f3[1] === 127 && f3.readBigUInt64BE(2) === 70000n, 'encode BIN 127 超长');

  // 2) 客户端掩码帧解析（构造一个掩码文本帧）
  const mkMasked = (op, text) => {
    const p = Buffer.from(text, 'utf8');
    const key = Buffer.from([1, 2, 3, 4]);
    const masked = Buffer.from(p);
    for (let i = 0; i < masked.length; i++) masked[i] ^= key[i & 3];
    const h = Buffer.alloc(2);
    h[0] = 0x80 | op; h[1] = 0x80 | p.length;
    return Buffer.concat([h, key, masked]);
  };
  const got = [];
  const p = new WsFrameParser((op, pl) => got.push([op, pl.toString('utf8')]), () => {});
  p.push(mkMasked(OP.TEXT, 'hello'));
  ok(got.length === 1 && got[0][0] === OP.TEXT && got[0][1] === 'hello', '解码掩码文本帧');

  // 3) 分片重组
  const frag = [];
  const p2 = new WsFrameParser((op, pl) => frag.push([op, pl.toString('utf8')]), () => {});
  const partA = mkMasked(OP.TEXT, 'ab'); partA[0] &= 0x7f;      // FIN=0
  const partB = mkMasked(OP.CONT, 'cd');                        // CONT + FIN=1
  p2.push(Buffer.concat([partA, partB]));
  ok(frag.length === 1 && frag[0][1] === 'abcd', '分片重组 TEXT');

  // 4) 目标解析
  ok(JSON.stringify(parseTarget('192.168.7.120:5000')) === '{"host":"192.168.7.120","port":5000}', 'parseTarget ip:port');
  ok(JSON.stringify(parseTarget('192.168.7.9')) === '{"host":"192.168.7.9","port":5000}', 'parseTarget 缺省端口');

  // 5) 白名单
  const a = makeAllowFilter('127.0.0.1');
  ok(a('127.0.0.1') && a('::ffff:127.0.0.1') && !a('192.168.1.9'), '白名单 loopback');
  const b = makeAllowFilter('192.168.7.0/24');
  ok(b('192.168.7.55') && !b('192.168.8.55'), '白名单 CIDR');
  const c = makeAllowFilter('192.168.*.*');
  ok(c('192.168.1.1') && !c('10.0.0.1'), '白名单通配');
  const d = makeAllowFilter('0.0.0.0/0');
  ok(d('8.8.8.8'), '白名单放行全部');

  // 6) 真实起服务 + 真 WS 握手 + 真 UDP 回包（起一个假固件 UDP 服务）
  //    ⚠ 需要全局 WebSocket（Node 21+ 自带；Node 18 没有）→ 没有就跳过这 3 项。
  if (typeof WebSocket === 'undefined') {
    return { pass, fail, ok: fail.length === 0, skipped: '当前 Node 无全局 WebSocket，跳过在线透传测试' };
  }
  const fake = dgram.createSocket('udp4');
  await new Promise((r) => fake.bind(0, '127.0.0.1', r));
  const fakePort = fake.address().port;
  fake.on('message', (msg, rinfo) => {
    // 原样回一个"响应包"：把 CMD 置高位置 1（逻辑包 SEQ|CMD|LEN|... → CMD 在 index 1）
    const echo = Buffer.from(msg);
    if (echo.length >= 2) echo[1] = echo[1] | 0x80;
    fake.send(echo, rinfo.port, rinfo.address);
  });

  const br = await createBridge({ port: 0, host: '127.0.0.1', target: `127.0.0.1:${fakePort}`, log: () => {} });
  const ws = new WebSocket(`ws://127.0.0.1:${br.port}/`);
  ws.binaryType = 'arraybuffer';
  const rxCb = [];
  await new Promise((res, rej) => {
    const to = setTimeout(() => rej(new Error('连接边车超时')), 3000);
    ws.onopen = () => { clearTimeout(to); res(); };
    ws.onerror = () => { clearTimeout(to); rej(new Error('连接边车失败')); };
  });
  ws.onmessage = (ev) => { if (ev.data instanceof ArrayBuffer) rxCb.push(new Uint8Array(ev.data)); };

  const probe = Buffer.from([0x11, 0x01, 0x00, 0x00]);
  ws.send(probe);
  await new Promise((r) => setTimeout(r, 300));
  ok(rxCb.length >= 1, '真 WS 握手 + 二进制透传（收到回包）');
  if (rxCb.length) {
    ok(rxCb[0].length === probe.length && rxCb[0][1] === 0x81, '回包逐字节透传 + CMD 置高位');
  }

  // 7) 控制帧 stats
  const txt = [];
  ws.onmessage = (ev) => { if (typeof ev.data === 'string') txt.push(JSON.parse(ev.data)); };
  ws.send(JSON.stringify({ op: 'stats' }));
  await new Promise((r) => setTimeout(r, 200));
  ok(txt.some((m) => m.type === 'stats' && m.tx >= 1), '控制帧 stats');

  ws.close();
  await br.close();
  fake.close();

  return { pass, fail, ok: fail.length === 0 };
}

/* ==========================================================================
 * CLI
 * ========================================================================== */

function parseArgs(argv) {
  const a = { port: 8765, host: '127.0.0.1', target: null, allow: '127.0.0.1',
              verbose: false, selftest: false };
  for (let i = 0; i < argv.length; i++) {
    const s = argv[i];
    const nx = () => argv[++i];
    if (s === '--port' || s === '-p') a.port = parseInt(nx(), 10);
    else if (s === '--host') a.host = nx();
    else if (s === '--target' || s === '-t') a.target = nx();
    else if (s === '--allow') a.allow = nx();
    else if (s === '--verbose' || s === '-v') a.verbose = true;
    else if (s === '--selftest') a.selftest = true;
    else if (s === '--help' || s === '-h') a.help = true;
  }
  return a;
}

const HELP = `bridge.js —— WebSocket ↔ UDP 边车（浏览器用网口）

用法:
  node src/bridge.js [选项]

选项:
  -t, --target <ip:port>   默认目标机械臂（缺省 192.168.7.120:5000）
  -p, --port <n>           监听端口（缺省 8765）
      --host <ip>          监听地址（缺省 127.0.0.1）
      --allow <列表>       允许连入的来源（缺省 127.0.0.1；支持 CIDR / 通配 / 逗号列表）
  -v, --verbose            打印每包
      --selftest           跑离线自检后退出
  -h, --help               本帮助

浏览器侧:
  import { openArm } from 'arm-sdk-js';
  const arm = await openArm('ws://127.0.0.1:8765/');                     // 用默认目标
  const arm2 = await openArm('ws://127.0.0.1:8765/?target=192.168.7.121:5000'); // 指定目标
`;

async function main() {
  const a = parseArgs(process.argv.slice(2));
  if (a.help) { console.log(HELP); return 0; }
  if (a.selftest) {
    const r = await selftest();
    console.log(`bridge 自检：${r.pass} 项通过` + (r.fail.length ? `，${r.fail.length} 项失败` : '，全部通过'));
    for (const f of r.fail) console.log('  ✗ ' + f);
    return r.ok ? 0 : 1;
  }
  const br = await createBridge({
    port: a.port, host: a.host, target: a.target, allow: a.allow, verbose: a.verbose,
  });

  const shutdown = async () => {
    console.log('\n[bridge] 收工…');
    await br.close();
    process.exit(0);
  };
  process.on('SIGINT', shutdown);
  process.on('SIGTERM', shutdown);

  // 心跳日志（只在 -v 下），顺便让 event loop 有事做
  if (a.verbose) {
    setInterval(() => {
      const s = br.stats();
      console.log(`[bridge] 活跃 ${s.active} 连接 / 累计 ${s.conns} / 拒绝 ${s.denied}`);
    }, 10000).unref();
  }
  return null;   // 常驻
}

const isMain = process.argv[1] && fileURLToPath(import.meta.url) === process.argv[1];
if (isMain) {
  main().then((rc) => { if (rc !== null && rc !== undefined) process.exit(rc); })
        .catch((e) => { console.error('[bridge] 启动失败:', e); process.exit(1); });
}
