/**
 * sim.js — 内存假臂（离线开发 / CI，不需要任何硬件）
 *
 * 复刻固件的关键行为，尤其是那些**会静默坑人**的：
 *   · MOVE_JOINTS 目标角被静默钳制（回 err=0，但值被改了）
 *   · PARAM_WRITE 值被静默钳制（同上）
 *   · STREAM_JOINTS 单帧跳变 >25° ⇒ **整帧拒收，且基准不更新**（自锁）
 *   · 读失败哨兵 -32768 ⇒ 该轴 null，read_ok=0
 *   · CMD >= 0x80 的包**故意不回**（固件防死循环）
 *   · 查询类响应 = 请求码 | 0x80；控制类回 ACK/NACK
 *   · 丢包 / 延迟可配
 *
 * 2026-09-23 扩展（全指令覆盖，见 docs/网口指令扩展设计_2026-09-23.md）：
 *   · 危险命令强制 MAGIC；`zr` 没有单轴变体 ⇒ ZERO_RESET 只收 flags
 *   · CALIB_ENTER **一律 NACK**（交互式向导从网口进会卡死主循环）
 *   · TEXT_EXEC/TEXT_FETCH = 把整个控制台搬上网口；闸门语义与固件一致
 *     （E-Stop 下只放 s/h；单独 `c` 被拒；11 条抢 ETH 引脚的命令由
 *      ProcessCommand 自己接住 —— 那不算闸门拒绝）
 *
 * 与 Python 版 `arm_sdk/sim.py` 同语义。它的价值：**没有机械臂也能把上位机写完**，
 * 而且能主动注入故障，这在真机上很难复现。
 */

import {
  CMD, RSP, ErrCode, PARAM_TABLE, RAW_READ_FAIL, StreamRc,
  packLogical, unpackLogical, ProtocolError,
  decodeStatus, i16Bytes, readI16, readU16, u16Bytes, i32Bytes,
  AP_PARAM_COUNT,
  // ---- 2026-09-23 扩展所需 ----
  MAGIC, TEXT_BUF_SZ, TEXT_CHUNK, TEXT_EXEC_MAX,
  TEXT_FLAG_TRUNC, TEXT_FLAG_GATE,
  TEACH_READ_MAX, PARAM_LIST_MAX,
  TEMP_LOST, HWERR_LOST, utf8Encoder, utf8Decoder,
} from './protocol.js';
import { Transport, RxEvent } from './transport.js';
// kinematics.js 不依赖任何传输层（纯函数），静态引入安全且与 Python 版一致：
// sim 的 IK_QUERY 必须走**同一个** ikChecked，否则假臂和真固件判据会分叉。
import { ikChecked } from './kinematics.js';

/** 固件硬门限：STREAM_JOINTS 单帧单轴跳变超过它就整帧拒收。 */
export const FW_MAX_JUMP_DEG = 25.0;
/** 固件流看门狗：断流超过这个时间就退出流模式（默认只在 watchdogMs>0 时启用）。 */
export const FW_STREAM_TIMEOUT_MS = 500;

export class SimTransport extends Transport {
  /**
   * @param {object} [opts]
   * @param {number} [opts.latencyMs=0.8]   每个响应的延迟（≈ 实测网口 p50）
   * @param {number} [opts.loss=0]          丢包率 0~1
   * @param {boolean} [opts.estop=false]    急停
   * @param {number[]} [opts.failJoints=[]] 读失败的轴（0 基），如 [2] = J3
   * @param {number[]} [opts.minRaw]        各轴行程下限（raw 0~4095），默认全 0
   * @param {number[]} [opts.maxRaw]        各轴行程上限（raw 0~4095），默认全 4095
   *                                        ↑ 想要"运行时限位"效果就传这俩，
   *                                          MOVE_JOINTS 超限会被静默钳制（复刻固件）
   * @param {boolean} [opts.unreachable=false] MOVE_CART 一律回 err=3
   * @param {number} [opts.watchdogMs=0]    流看门狗（0=关；设 500 复刻固件）
   * @param {number[]} [opts.offset]        舵机零点
   * @param {number[]} [opts.dir]           关节方向
   * @param {(s:string)=>void} [opts.onEvent]
   */
  constructor(opts = {}) {
    super();
    this.latencyMs = opts.latencyMs ?? 0.8;
    this.loss = opts.loss ?? 0;
    this.estop = !!opts.estop;
    this.failJoints = [...(opts.failJoints || [])];
    this.unreachable = !!opts.unreachable;
    this.watchdogMs = opts.watchdogMs ?? 0;
    this.onEvent = opts.onEvent || null;

    this.servoDeg = [180, 180, 180, 180, 180, 180];
    this.targetDeg = [180, 180, 180, 180, 180, 180];
    this.torque = [1, 1, 1, 1, 1, 1];
    this.params = Object.fromEntries(
      Object.entries(PARAM_TABLE).map(([k, v]) => [Number(k), v.def]));
    this.gravityK = 0;
    this.corr = { enabled: true, threshold: 2.0, maxRounds: 10 };
    this.logLevel = 2;
    this.protoVer = 1;
    this.runId = 0x5A;

    this.offset = [...(opts.offset || [180, 180, 180, 180, 180, 180])];
    this.dir = [...(opts.dir || [-1, 1, 1, 1, 1, 1])];
    this.minRaw = opts.minRaw ? [...opts.minRaw] : new Array(6).fill(0);
    this.maxRaw = opts.maxRaw ? [...opts.maxRaw] : new Array(6).fill(4095);

    this.streamActive = false;
    this.streamBase = null;
    this.streamLast = null;
    this.streamFrames = 0;
    this.streamRejects = 0;
    this.streamClamps = 0;
    this.streamLastAt = 0;

    this.trajSpeed = 25;
    this._trajAt = 0;
    this._trajActive = false;

    /* ---- 2026-09-23 扩展：新命令需要的状态（与 sim.py 逐字段对应）---- */
    this.posRaw = new Array(6).fill(2048);      // GET_LIMITS 的"当前/中位" raw
    this.zeroRaw = new Array(6).fill(0);        // 0 = 未做 Home 零点校准（同固件）
    this.busMode = 0;                           // 0=正常 1=全A 2=全B
    this.pid = Object.fromEntries(Array.from({ length: 6 }, (_, j) => [j, [32, 0, 1]]));
    this.corrPending = 0;
    this.build = 0;                             // 0=net 1=can 2=both 3=uart
    this.linkMode = 0;                          // 启动冻结的链路模式
    this.fwTag = 'SIM-NET-0923A';
    this.flagsExtra = 0;                        // 额外状态位（示教录制/回放…）
    this.sysResetCount = 0;
    // 示教文件：idx -> { name, valid, pts:[[6×u16 raw], ...] }
    this.teach = {
      0: { name: 'demo', valid: true,
           pts: Array.from({ length: 12 }, () => new Array(6).fill(2048)) },
      1: { name: 'gone', valid: false, pts: [new Array(6).fill(1024)] },
    };
    this.teachCapacity = 32;
    this.teachFree = 60 * 1024;
    // 文本透传的输出缓冲（复刻固件 2048 B 捕获缓冲；按字节存）
    this._textBuf = '';
    this._textTrunc = false;
    this._textGate = false;

    this._rx = [];
    this._opened = false;
    this._rng = 12345;
    this._t0 = Date.now();
    this._txCount = 0;
  }

  get link() { return 'sim://'; }
  get supportsText() { return false; }

  async open() { this._opened = true; this._t0 = Date.now(); }
  async close() { this._opened = false; }
  get isOpen() { return this._opened; }

  reset() { this._rx = []; }

  /* ---- 随机数（可复现，便于测试）---- */
  _rand() {
    this._rng = (this._rng * 1103515245 + 12345) & 0x7FFFFFFF;
    return this._rng / 0x7FFFFFFF;
  }

  /* ---- 时间 ---- */
  _now() { return (Date.now() - this._t0) / 1000; }

  /* ======================================================================
   * Transport 接口
   * ==================================================================== */

  send(seq, cmd, payload = new Uint8Array(0)) {
    if (!this._opened) throw new Error('SimTransport 未打开');
    this._txCount++;

    // 顺带验证编解码自洽（发出去的东西必须能解回来）
    let parsed;
    try {
      parsed = unpackLogical(packLogical(seq, cmd, payload));
    } catch (e) {
      if (cmd < 0x80) {
        this._schedule(seq, RSP.NACK, Uint8Array.from([cmd & 0xFF, ErrCode.CRC]));
      }
      return null;
    }

    if (this.loss > 0 && this._rand() < this.loss) return null;   // 丢包

    const resp = this._handle(parsed.seq, parsed.cmd, parsed.payload);
    if (resp === null) return null;                               // 故意不回
    this._schedule(parsed.seq, resp[0], resp[1]);
    return packLogical(seq, cmd, payload);
  }

  _schedule(seq, cmd, payload) {
    const push = () => this._rx.push(new RxEvent('packet', { seq, cmd, payload }));
    if (this.latencyMs > 0) setTimeout(push, this.latencyMs);
    else queueMicrotask(push);
  }

  drain() {
    const out = this._rx;
    this._rx = [];
    return out;
  }

  /* ======================================================================
   * 故障注入（测试用）
   * ==================================================================== */

  setJointFail(joints) { this.failJoints = [...joints]; }
  setEstop(on) { this.estop = !!on; }
  setUnreachable(on) { this.unreachable = !!on; }
  setLoss(p) { this.loss = Math.max(0, Math.min(1, p)); }
  /** 让实际位置立刻等于目标（跳过插值，测试里省时间）。 */
  settle() { this.servoDeg = [...this.targetDeg]; }

  streamStats() {
    return {
      frames: this.streamFrames, rejects: this.streamRejects, clamps: this.streamClamps,
      active: this.streamActive,
      base: this.streamBase ? [...this.streamBase] : null,
      last: this.streamLast ? [...this.streamLast] : null,
      tx: this._txCount,
    };
  }

  /* ======================================================================
   * 内部：轨迹插值 / 限位钳制
   * ==================================================================== */

  _stepTraj() {
    const now = this._now();
    const dt = now - this._trajAt;
    this._trajAt = now;
    if (dt <= 0) return;
    const maxStep = Math.max(1.0, this.trajSpeed) * dt;
    let moving = false;
    for (let j = 0; j < 6; j++) {
      const d = this.targetDeg[j] - this.servoDeg[j];
      if (Math.abs(d) > 0.05) {
        moving = true;
        this.servoDeg[j] += Math.max(-maxStep, Math.min(maxStep, d));
      }
    }
    this._trajActive = moving;
  }

  _clampServo(deg) {
    const out = [];
    for (let j = 0; j < 6; j++) {
      let r = Math.round((deg[j] / 360) * 4095) % 4096;
      if (r < this.minRaw[j]) r = this.minRaw[j];
      if (r > this.maxRaw[j]) r = this.maxRaw[j];
      out.push((r / 4095) * 360);
    }
    return out;
  }

  /** 流看门狗：断流超时 ⇒ 退出流模式（复刻固件）。 */
  _checkWatchdog() {
    if (this.watchdogMs <= 0 || !this.streamActive) return;
    if ((Date.now() - this.streamLastAt) > this.watchdogMs) {
      this.streamActive = false;
      this.streamBase = null;
      if (this.onEvent) this.onEvent('sim: 流看门狗触发，已退出流模式');
    }
  }

  /* ======================================================================
   * 内部：命令处理
   * ==================================================================== */

  _handle(seq, cmd, pl) {
    // 响应码 >= 0x80：固件故意不处理、不回（防死循环）
    if (cmd >= 0x80) return null;

    /* ---- 0x01 PING ---- */
    if (cmd === CMD.PING) {
      return [RSP.PING, Uint8Array.from([this.protoVer, this.runId])];
    }

    /* ---- 0x02 GET_STATUS ---- */
    if (cmd === CMD.GET_STATUS) {
      this._stepTraj();
      this._checkWatchdog();
      const body = new Uint8Array(21);
      for (let j = 0; j < 6; j++) {
        const v = this.failJoints.includes(j)
          ? RAW_READ_FAIL
          : Math.round((this.torque[j] ? this.targetDeg[j] : this.servoDeg[j]) * 10);
        body[2 * j] = i16Bytes(v)[0];
        body[2 * j + 1] = i16Bytes(v)[1];
      }
      // 位姿
      const pose = this._fk();
      const pv = [
        Math.round(pose.x * 10), Math.round(pose.y * 10),
        Math.round(pose.z * 10), Math.round(pose.pitchDeg * 10),
      ];
      pv.forEach((v, i) => {
        const b = i16Bytes(v);
        body[12 + 2 * i] = b[0];
        body[12 + 2 * i + 1] = b[1];
      });
      let flags = 0;
      if (this.estop) flags |= 0x01;
      if (this._trajActive) flags |= 0x02;
      if (this.corr.enabled) flags |= 0x04;
      if (this.failJoints.length === 0) flags |= 0x08;
      body[20] = flags;
      return [RSP.STATUS, body];
    }

    /* ---- 0x03 LOG_SET_LEVEL ---- */
    if (cmd === CMD.LOG_SET_LEVEL) {
      if (pl.length < 1) return [RSP.NACK, Uint8Array.from([cmd, ErrCode.PARAM])];
      this.logLevel = Math.min(4, pl[0]);
      return [RSP.ACK, Uint8Array.from([cmd, 0])];
    }

    /* ---- 0x10 MOVE_JOINTS ---- */
    if (cmd === CMD.MOVE_JOINTS) {
      if (pl.length < 16) return [RSP.NACK, Uint8Array.from([cmd, ErrCode.PARAM])];
      if (this.estop) return [RSP.NACK, Uint8Array.from([cmd, ErrCode.ESTOP])];
      const tgt = [];
      for (let j = 0; j < 6; j++) tgt.push(readI16(pl, j * 2) / 10);
      const v = readU16(pl, 12) / 10;
      this.trajSpeed = v >= 5 ? v : 25;
      this.targetDeg = this._clampServo(tgt);       // ⚠ 静默钳制
      this.torque = [1, 1, 1, 1, 1, 1];             // MOVE_JOINTS 自动上力矩
      this.streamActive = false;                    // 轨迹与流互斥
      this._trajAt = this._now();
      return [RSP.ACK, Uint8Array.from([cmd, 0])];
    }

    /* ---- 0x11 STOP ---- */
    if (cmd === CMD.STOP) {
      this.targetDeg = [...this.servoDeg];
      this.streamActive = false;
      return [RSP.ACK, Uint8Array.from([cmd, 0])];
    }

    /* ---- 0x12 SET_TORQUE ---- */
    if (cmd === CMD.SET_TORQUE) {
      if (pl.length < 2) return [RSP.NACK, Uint8Array.from([cmd, ErrCode.PARAM])];
      const jn = pl[0], en = pl[1];
      if (jn === 0xFF) this.torque = new Array(6).fill(en ? 1 : 0);
      else if (jn >= 0 && jn <= 5) this.torque[jn] = en ? 1 : 0;
      else return [RSP.NACK, Uint8Array.from([cmd, ErrCode.PARAM])];
      return [RSP.ACK, Uint8Array.from([cmd, 0])];
    }

    /* ---- 0x13 STREAM_JOINTS ---- */
    if (cmd === CMD.STREAM_JOINTS) {
      if (pl.length < 12) return [RSP.NACK, Uint8Array.from([cmd, ErrCode.PARAM])];
      if (this.estop) return [RSP.NACK, Uint8Array.from([cmd, ErrCode.ESTOP])];
      const logical = [];
      for (let j = 0; j < 6; j++) logical.push(readI16(pl, j * 2) / 10);
      const tgt = logical.map((l, j) => {
        const v = (this.offset[j] + this.dir[j] * l) % 360;
        return v < 0 ? v + 360 : v;
      });
      const base = this.streamBase ?? this.servoDeg;
      // ⚠ 任一首超 25° ⇒ 整帧丢弃，且基准不动（会一直拒 = 自锁）
      for (let j = 0; j < 6; j++) {
        if (Math.abs(tgt[j] - base[j]) > FW_MAX_JUMP_DEG) {
          this.streamRejects++;
          return [RSP.NACK, Uint8Array.from([cmd, ErrCode.LIMIT])];
        }
      }
      const clamped = this._clampServo(tgt);
      if (clamped.some((c, j) => Math.abs(c - tgt[j]) > 1e-6)) this.streamClamps++;
      this.streamBase = [...tgt];                   // 成功 ⇒ 基准前移
      this.streamLast = [...tgt];
      this.servoDeg = [...clamped];
      this.targetDeg = [...clamped];
      this.streamActive = true;
      this.streamFrames++;
      this.streamLastAt = Date.now();
      return [RSP.ACK, Uint8Array.from([cmd, 0])];
    }

    /* ---- 0x20 MOVE_CART ---- */
    if (cmd === CMD.MOVE_CART) {
      if (pl.length < 12) return [RSP.NACK, Uint8Array.from([cmd, ErrCode.PARAM])];
      if (this.estop) return [RSP.NACK, Uint8Array.from([cmd, ErrCode.ESTOP])];
      if (this.unreachable) return [RSP.NACK, Uint8Array.from([cmd, ErrCode.UNREACH])];
      this.streamActive = false;
      return [RSP.ACK, Uint8Array.from([cmd, 0])];
    }

    /* ---- 0x21 FK_QUERY ---- */
    if (cmd === CMD.FK_QUERY) {
      this._stepTraj();
      if (this.failJoints.length) {
        // 读失败 ⇒ read_ok=0，位姿字段无效（这里填 0，和固件一致）
        const body = new Uint8Array(21);
        for (let j = 0; j < 6; j++) {
          const v = this.failJoints.includes(j) ? RAW_READ_FAIL : Math.round(this.targetDeg[j] * 10);
          const b = i16Bytes(v);
          body[8 + 2 * j] = b[0];
          body[8 + 2 * j + 1] = b[1];
        }
        body[20] = 0;
        return [RSP.FK, body];
      }
      const pose = this._fk();
      const body = new Uint8Array(21);
      [pose.x, pose.y, pose.z, pose.pitchDeg].forEach((v, i) => {
        const b = i16Bytes(Math.round(v * 10));
        body[2 * i] = b[0];
        body[2 * i + 1] = b[1];
      });
      for (let j = 0; j < 6; j++) {
        const b = i16Bytes(Math.round(this.targetDeg[j] * 10));
        body[8 + 2 * j] = b[0];
        body[8 + 2 * j + 1] = b[1];
      }
      body[20] = 1;                                 // ⚠ 裸 0/1，不是位域
      return [RSP.FK, body];
    }

    /* ---- 0x30 SET_GRAVITY ---- */
    if (cmd === CMD.SET_GRAVITY) {
      if (pl.length < 2) return [RSP.NACK, Uint8Array.from([cmd, ErrCode.PARAM])];
      this.gravityK = readI16(pl, 0) / 1000;
      return [RSP.ACK, Uint8Array.from([cmd, 0])];
    }

    /* ---- 0x31 SET_CORRECTION ---- */
    if (cmd === CMD.SET_CORRECTION) {
      if (pl.length < 4) return [RSP.NACK, Uint8Array.from([cmd, ErrCode.PARAM])];
      this.corr = { enabled: !!pl[0], threshold: readU16(pl, 1) / 100, maxRounds: pl[3] };
      return [RSP.ACK, Uint8Array.from([cmd, 0])];
    }

    /* ---- 0x40 PARAM_READ ---- */
    if (cmd === CMD.PARAM_READ) {
      if (pl.length < 1) return [RSP.NACK, Uint8Array.from([cmd, ErrCode.PARAM])];
      const id = pl[0];
      if (!(id in PARAM_TABLE)) return [RSP.NACK, Uint8Array.from([cmd, ErrCode.PARAM])];
      const v = Math.round(this.params[id] * 1000) | 0;
      return [RSP.PARAM_READ, Uint8Array.from([id,
        v & 0xFF, (v >>> 8) & 0xFF, (v >>> 16) & 0xFF, (v >>> 24) & 0xFF])];
    }

    /* ---- 0x41 PARAM_WRITE ---- */
    if (cmd === CMD.PARAM_WRITE) {
      if (pl.length < 5) return [RSP.NACK, Uint8Array.from([cmd, ErrCode.PARAM])];
      const id = pl[0];
      if (!(id in PARAM_TABLE)) return [RSP.NACK, Uint8Array.from([cmd, ErrCode.PARAM])];
      const val = ((pl[1] | (pl[2] << 8) | (pl[3] << 16) | (pl[4] << 24)) | 0) / 1000;
      const spec = PARAM_TABLE[id];
      // ⚠ 静默钳制：超范围**仍然回 err=0**，但值被钳过 ⇒ 必须回读确认
      this.params[id] = Math.max(spec.min, Math.min(spec.max, val));
      return [RSP.ACK, Uint8Array.from([cmd, 0])];
    }

    /* ---- 0x42 PARAM_COUNT ---- */
    if (cmd === CMD.PARAM_COUNT) {
      return [RSP.PARAM_COUNT, Uint8Array.from([AP_PARAM_COUNT])];
    }

    /* ======================================================================
     * 2026-09-23 扩展：全指令覆盖
     * 基准 docs/网口指令扩展设计_2026-09-23.md，与 sim.py 逐条对应。
     * ==================================================================== */

    /* ---- 0x04 GET_JOINT_DETAIL ---- */
    if (cmd === CMD.GET_JOINT_DETAIL) {
      if (pl.length < 1) return [RSP.NACK, Uint8Array.from([cmd, ErrCode.PARAM])];
      const j = pl[0];
      let js;
      if (j === 0xFF) js = [0, 1, 2, 3, 4, 5];
      else if (j <= 5) js = [j];
      else return [RSP.NACK, Uint8Array.from([cmd, ErrCode.PARAM])];
      this._stepTraj();
      const body = [js.length, js[0]];
      for (const k of js) {
        if (this.failJoints.includes(k)) {
          // 本轴整组丢读：pos/speed/load 无意义 ⇒ 0；temp/hw_err 回哨兵
          body.push(0, 0, 0, 0, 0, 0);
          body.push(0, TEMP_LOST, HWERR_LOST, 0);
        } else {
          const raw = Math.round((this.servoDeg[k] / 360) * 4095) % 4096;
          body.push(...u16Bytes(raw));          // pos_raw
          body.push(...u16Bytes(0));            // speed (i16 0)
          body.push(...u16Bytes(0));            // load  (i16 0)
          body.push(120, 42, 0, 1);             // 12.0V, 42°C, hw_err=0, ok=1
        }
      }
      return [RSP.JOINT_DETAIL, Uint8Array.from(body)];
    }

    /* ---- 0x05 GET_LIMITS ---- */
    if (cmd === CMD.GET_LIMITS) {
      const body = [];
      for (let k = 0; k < 6; k++) {
        body.push(...u16Bytes(this.minRaw[k]));
        body.push(...u16Bytes(this.posRaw[k]));
        body.push(...u16Bytes(this.maxRaw[k]));
        body.push(...u16Bytes(this.zeroRaw[k]));
      }
      return [RSP.LIMITS, Uint8Array.from(body)];
    }

    /* ---- 0x07 GET_INFO ---- */
    if (cmd === CMD.GET_INFO) {
      const tag = utf8Encoder.encode(this.fwTag).slice(0, 16);
      const body = [tag.length, ...tag];
      while (body.length < 1 + 16) body.push(0);
      body.push(this.build & 0xFF, this.linkMode & 0xFF, AP_PARAM_COUNT & 0xFF);
      return [RSP.INFO, Uint8Array.from(body)];
    }

    /* ---- 0x0A GET_FLAGS（只读变量，不碰总线）---- */
    if (cmd === CMD.GET_FLAGS) {
      let f = 0;
      if (this.estop) f |= 0x01;
      if (this._trajActive) f |= 0x02;
      if (this.streamActive) f |= 0x04;
      f |= this.flagsExtra & 0xFF;
      return [RSP.FLAGS, Uint8Array.from([f, this.runId & 0xFF])];
    }

    /* ---- 0x14 MOVE_JOINT_ONE ---- */
    if (cmd === CMD.MOVE_JOINT_ONE) {
      if (pl.length < 7) return [RSP.NACK, Uint8Array.from([cmd, ErrCode.PARAM])];
      if (this.estop) return [RSP.NACK, Uint8Array.from([cmd, ErrCode.ESTOP])];
      const j = pl[0];
      if (j > 5) return [RSP.NACK, Uint8Array.from([cmd, ErrCode.PARAM])];
      // 固件先读全场快照、只改这一轴 ⇒ **任一轴丢读就整条拒**（拿不准就不动）
      if (this.failJoints.length) return [RSP.NACK, Uint8Array.from([cmd, ErrCode.READ])];
      const deg = readI16(pl, 1) / 10;
      const v = readU16(pl, 3) / 10;
      if (v >= 5.0) this.trajSpeed = v;
      const tgt = [...this.targetDeg];
      tgt[j] = deg;
      this.targetDeg = this._clampServo(tgt);   // ⚠ 静默钳制
      this.torque = [1, 1, 1, 1, 1, 1];
      this.streamActive = false;
      this._trajAt = this._now();
      return [RSP.ACK, Uint8Array.from([cmd, 0])];
    }

    /* ---- 0x17 SET_BUS_MODE ---- */
    if (cmd === CMD.SET_BUS_MODE) {
      if (pl.length < 1 || pl[0] > 2) return [RSP.NACK, Uint8Array.from([cmd, ErrCode.PARAM])];
      this.busMode = pl[0];
      return [RSP.ACK, Uint8Array.from([cmd, 0])];
    }

    /* ---- 0x18 SERVO_INIT（会重写 PID / 限位 / 中位）---- */
    if (cmd === CMD.SERVO_INIT) {
      if (pl.length < 1) return [RSP.NACK, Uint8Array.from([cmd, ErrCode.PARAM])];
      const j = pl[0];
      if (j !== 0xFF && j > 5) return [RSP.NACK, Uint8Array.from([cmd, ErrCode.PARAM])];
      if (this.estop) return [RSP.NACK, Uint8Array.from([cmd, ErrCode.ESTOP])];
      this.offset = [180, 180, 180, 180, 180, 180];   // 中位回 180°
      return [RSP.ACK, Uint8Array.from([cmd, 0])];
    }

    /* ---- 0x22 IK_QUERY（只解不算，不发指令）---- */
    if (cmd === CMD.IK_QUERY) {
      if (pl.length < 8) return [RSP.NACK, Uint8Array.from([cmd, ErrCode.PARAM])];
      const x = readI16(pl, 0) / 10;
      const y = readI16(pl, 2) / 10;
      const z = readI16(pl, 4) / 10;
      const p = readI16(pl, 6) / 10;
      if (this.unreachable) return [RSP.NACK, Uint8Array.from([cmd, ErrCode.UNREACH])];
      let r;
      try {
        r = ikChecked(x, y, z, p, {
          minRaw: this.minRaw, maxRaw: this.maxRaw,
          servoOffset: this.offset, keepJ5J6ServoDeg: this.servoDeg,
        });
      } catch {
        // 与 _fk 同一条纪律：求解器异常**不许**兜底成"成功"
        return [RSP.NACK, Uint8Array.from([cmd, ErrCode.UNREACH])];
      }
      if (!r.ok) {
        const rc = { unreachable: ErrCode.UNREACH, joint_limit: ErrCode.LIMIT,
                     raw_limit: ErrCode.LIMIT, fk_verify: ErrCode.FK_VERIFY,
                     pitch_not_met: ErrCode.FK_VERIFY }[r.reason] ?? ErrCode.UNREACH;
        return [RSP.NACK, Uint8Array.from([cmd, rc])];
      }
      const body = [];
      for (let k = 0; k < 6; k++) body.push(...i16Bytes(Math.round(r.servoDeg[k] * 10)));
      return [RSP.IK_QUERY, Uint8Array.from(body)];
    }

    /* ---- 0x32 / 0x33 补偿的"读"侧 ---- */
    if (cmd === CMD.GET_GRAVITY) {
      const k = this.params[32] ?? 0.03;
      return [RSP.GRAVITY, Uint8Array.from(i16Bytes(Math.round(k * 1000)))];
    }

    if (cmd === CMD.GET_CORRECTION) {
      const en = this.params[33] ? 1 : 0;
      const thr = this.params[34] ?? 2.0;
      const rounds = Math.round(this.params[35] ?? 6.0);
      return [RSP.CORRECTION, Uint8Array.from([
        en, ...u16Bytes(Math.round(thr * 100)),
        rounds & 0xFF, this.corrPending ? 1 : 0])];
    }

    /* ---- 0x36 GET_PID（协议顺序固定 p/i/d）---- */
    if (cmd === CMD.GET_PID) {
      if (pl.length < 1 || pl[0] > 5) return [RSP.NACK, Uint8Array.from([cmd, ErrCode.PARAM])];
      const [jp, ji, jd] = this.pid[pl[0]];
      const ee = this.failJoints.includes(pl[0]) ? 0 : 1;
      return [RSP.PID, Uint8Array.from([jp, ji, jd, ee])];
    }

    /* ---- 0x43 PARAM_SAVE（等价文本 `F`）---- */
    if (cmd === CMD.PARAM_SAVE) {
      if (pl.length < 4 || this._u32(pl) !== MAGIC) {
        return [RSP.NACK, Uint8Array.from([cmd,
          pl.length < 4 ? ErrCode.PARAM : ErrCode.MAGIC])];
      }
      return [RSP.ACK, Uint8Array.from([cmd, 0])];
    }

    /* ---- 0x44 PARAM_LIST（只列元数据，值用 PARAM_READ 取）---- */
    if (cmd === CMD.PARAM_LIST) {
      if (pl.length < 1) return [RSP.NACK, Uint8Array.from([cmd, ErrCode.PARAM])];
      const ids = Object.keys(PARAM_TABLE).map(Number).sort((a, b) => a - b);
      const page = ids.slice(pl[0], pl[0] + PARAM_LIST_MAX);
      const body = [ids.length & 0xFF, page.length];
      for (const pid of page) {
        const spec = PARAM_TABLE[pid];
        body.push(pid & 0xFF, 0);                       // type：假臂不细分
        body.push(...i32Bytes(Math.round(spec.min * 1000)));
        body.push(...i32Bytes(Math.round(spec.max * 1000)));
      }
      return [RSP.PARAM_LIST, Uint8Array.from(body)];
    }

    /* ---- 0x50 ~ 0x55 示教 / 轨迹存储 ---- */
    if (cmd === CMD.TEACH_LIST) {
      if (pl.length < 1) return [RSP.NACK, Uint8Array.from([cmd, ErrCode.PARAM])];
      const idx = pl[0];
      const f = this.teach[idx];
      if (!f) return [RSP.NACK, Uint8Array.from([cmd, ErrCode.PARAM])];
      const name = utf8Encoder.encode(f.name).slice(0, 11);
      const body = [Object.keys(this.teach).length & 0xFF, idx, f.valid ? 1 : 0,
                    ...name];
      while (body.length < 3 + 12) body.push(0);
      body.push(...i32Bytes(f.pts.length));
      return [RSP.TEACH_LIST, Uint8Array.from(body)];
    }

    if (cmd === CMD.TEACH_READ) {
      if (pl.length < 4) return [RSP.NACK, Uint8Array.from([cmd, ErrCode.PARAM])];
      const idx = pl[0];
      const off = readU16(pl, 1);
      const n = Math.min(pl[3], TEACH_READ_MAX);
      const tr = this.teach[idx];
      if (!tr) return [RSP.NACK, Uint8Array.from([cmd, ErrCode.PARAM])];
      const page = tr.pts.slice(off, off + n);          // off 越界 ⇒ 空页（正常读完）
      const body = [page.length];
      for (const pt of page) for (let k = 0; k < 6; k++) body.push(...u16Bytes(pt[k]));
      return [RSP.TEACH_READ, Uint8Array.from(body)];
    }

    if (cmd === CMD.TEACH_DELETE) {
      if (pl.length < 1 || !this.teach[pl[0]]) {
        return [RSP.NACK, Uint8Array.from([cmd, ErrCode.PARAM])];
      }
      this.teach[pl[0]].valid = false;                  // 写 tombstone
      return [RSP.ACK, Uint8Array.from([cmd, 0])];
    }

    if (cmd === CMD.TEACH_COMPACT) {
      for (const k of Object.keys(this.teach).map(Number)) {
        if (!this.teach[k].valid) delete this.teach[k];
      }
      return [RSP.ACK, Uint8Array.from([cmd, 0])];
    }

    if (cmd === CMD.TEACH_TOTAL) {
      let tot = 0;
      for (const v of Object.values(this.teach)) tot += v.pts.length;
      const body = [Object.keys(this.teach).length & 0xFF, this.teachCapacity & 0xFF,
                    ...i32Bytes(tot), ...i32Bytes(this.teachFree)];
      return [RSP.TEACH_TOTAL, Uint8Array.from(body)];
    }

    /* ---- 0x70 ~ 0x73 危险操作（强制 MAGIC；与固件一样走文本路径）---- */
    if (cmd === CMD.CENTER_CALIB) {
      if (pl.length < 4 || this._u32(pl) !== MAGIC) {
        return [RSP.NACK, Uint8Array.from([cmd,
          pl.length < 4 ? ErrCode.PARAM : ErrCode.MAGIC])];
      }
      this._execTextLine('M a');                        // 与固件同一条路径
      return [RSP.ACK, Uint8Array.from([cmd, 0])];
    }

    if (cmd === CMD.ZERO_RESET) {
      if (pl.length < 5 || this._u32(pl, 1) !== MAGIC) {
        return [RSP.NACK, Uint8Array.from([cmd,
          pl.length < 5 ? ErrCode.PARAM : ErrCode.MAGIC])];
      }
      // 文本 `zr` 没有单轴变体 —— 不猜，直接报参数错（与固件一致）
      if (pl[0] > 1) return [RSP.NACK, Uint8Array.from([cmd, ErrCode.PARAM])];
      this._execTextLine(pl[0] ? 'zr save' : 'zr');
      return [RSP.ACK, Uint8Array.from([cmd, 0])];
    }

    if (cmd === CMD.CALIB_ENTER) {
      if (pl.length < 4 || this._u32(pl) !== MAGIC) {
        return [RSP.NACK, Uint8Array.from([cmd,
          pl.length < 4 ? ErrCode.PARAM : ErrCode.MAGIC])];
      }
      // 固件**一律拒绝**：校准向导是交互式 for(;;)，从网口进会卡死主循环
      return [RSP.NACK, Uint8Array.from([cmd, ErrCode.PARAM])];
    }

    if (cmd === CMD.SYS_RESET) {
      if (pl.length < 4 || this._u32(pl) !== MAGIC) {
        return [RSP.NACK, Uint8Array.from([cmd,
          pl.length < 4 ? ErrCode.PARAM : ErrCode.MAGIC])];
      }
      // 真板子：回 ACK 后 150 ms 复位。假臂不复位自己，只记账 + 清运行态。
      this.sysResetCount++;
      this.streamActive = false;
      this.targetDeg = [...this.servoDeg];
      return [RSP.ACK, Uint8Array.from([cmd, 0])];
    }

    /* ---- 0x0B / 0x0C 文本命令透传 ---- */
    if (cmd === CMD.TEXT_EXEC) {
      if (pl.length < 5) return [RSP.NACK, Uint8Array.from([cmd, ErrCode.PARAM])];
      if (this._u32(pl) !== MAGIC) return [RSP.NACK, Uint8Array.from([cmd, ErrCode.MAGIC])];
      let raw = pl.slice(4);
      const nul = raw.indexOf(0);
      if (nul >= 0) raw = raw.slice(0, nul);
      raw = raw.slice(0, TEXT_EXEC_MAX);
      const line = utf8Decoder.decode(raw);
      const { out, gated } = this._textRun(line, true);
      this._setTextBuf(out, gated);
      return [RSP.TEXT_OUT, this._textPage(0, TEXT_CHUNK)];
    }

    if (cmd === CMD.TEXT_FETCH) {
      if (pl.length < 3 || pl[2] === 0) {
        return [RSP.NACK, Uint8Array.from([cmd, ErrCode.PARAM])];
      }
      return [RSP.TEXT_FETCH, this._textPage(readU16(pl, 0),
                                             Math.min(pl[2], TEXT_CHUNK))];
    }

    // 未知命令
    return [RSP.NACK, Uint8Array.from([cmd, ErrCode.PARAM])];
  }

  /* ======================================================================
   * 内部：文本命令透传（复刻固件 ArmMain_TextGate + ArmMain_ExecText）
   *
   * 这是一个**够用**的小解释器，只覆盖自检用到的那些命令。真板子跑的是完整的
   * 124 条 ProcessCommand —— 这里不追求全，但**闸门语义必须一致**，
   * 那才是上位机真正要防的坑。
   * ==================================================================== */

  static _u32(b, o = 0) {
    return ((b[o] | (b[o + 1] << 8) | (b[o + 2] << 16) | (b[o + 3] << 24)) >>> 0);
  }
  _u32(b, o = 0) { return SimTransport._u32(b, o); }

  /** 复刻 0x8B / 0x8C 的分页负载：u16 total, u8 flags, u16 off, u8 n, text[n]
   *
   * ⚠ 偏移与总长都是**字节**。控制台的中文提示是 UTF-8 字节 ⇒ 必须按字节
   *   分页，否则一页正好切在字符中间时上位机拼出来就是坏字符。
   */
  _textPage(off, n) {
    const data = utf8Encoder.encode(this._textBuf);
    const chunk = data.slice(off, off + n);
    let flags = 0;
    if (this._textTrunc) flags |= TEXT_FLAG_TRUNC;
    if (this._textGate) flags |= TEXT_FLAG_GATE;
    const body = [...u16Bytes(data.length), flags, ...u16Bytes(off), chunk.length,
                  ...chunk];
    return Uint8Array.from(body);
  }

  /** 把一段输出放进捕获缓冲（**按字节**截到 TEXT_BUF_SZ，与固件一致）。 */
  _setTextBuf(out, gated) {
    const raw = utf8Encoder.encode(out);
    this._textTrunc = raw.length > TEXT_BUF_SZ;
    this._textBuf = utf8Decoder.decode(raw.slice(0, TEXT_BUF_SZ));
    this._textGate = !!gated;
  }

  /** 给 CENTER_CALIB / ZERO_RESET / PARAM_SAVE 复用的"执行一行文本"入口。 */
  _execTextLine(line) {
    const { out } = this._textRun(line, false);
    this._setTextBuf(out, false);
    return out;
  }

  /**
   * 执行一行控制台文本，返回 { out, gated }。
   *
   * gate 语义（与固件 `ArmMain_TextGate` 一致）：
   *   · fromLink 且 E-Stop ⇒ 只放 `s` / `h`（其余 gated）
   *   · fromLink 且 空 / `#` / 单独 `c`（交互式会卡死主循环）⇒ gated
   *   11 条抢 ETH 引脚的命令**不算 gate 拒绝** —— 那是 ProcessCommand 自己
   *   接住并给原因，上位机要能区分"被策略拒"和"这条命令在当前链路不可用"。
   */
  _textRun(line, fromLink = false) {
    const t = String(line == null ? '' : line).trim();
    if (fromLink && this.estop && t !== 's' && t !== 'h') {
      return { out: "  [GATE] E-Stop active: only 's' and 'h' pass over the link.\r\n",
               gated: true };
    }
    if (fromLink && (!t || t[0] === '#' || this.constructor.TEXT_DENY_BLOCKING.includes(t))) {
      return { out: '  [GATE] refused over link: interactive/blocking command.\r\n',
               gated: true };
    }
    const tok = t.length ? t.split(/\s+/) : [];
    if (!tok.length) return { out: "  Unknown command. Type 'h' for help.\r\n", gated: false };
    const t0 = tok[0];

    if (this.constructor.TEXT_DENY_PINS.includes(t0)) {
      // 与 main.c 里那段 #if ARM_LINK_NET 拦截的文案同义（不是闸门拒绝，
      // 是 ProcessCommand 自己的"接住 + 说明"）
      return { out:
        `  [NET] '${t0}' 在当前链路(网口)下已停用 —— 它会启用 CAN。\r\n` +
        '  原因: 网口占用了 PB12/PB13(ETH_RXD1/ETH_TXD1),\r\n' +
        '        而 CAN2 正是同一对脚 ⇒ 允许切换/扫描会直接掐断网口。\r\n' +
        '  要用 CAN: `link can` → 再 `link save` → RESET 后生效。\r\n',
        gated: false };
    }
    if (t0 === 's') {
      this._stepTraj();
      const cols = this.servoDeg.map((v) => v.toFixed(1).padStart(7)).join(' ');
      return { out: '  J1       J2       J3       J4       J5       J6\r\n' +
                    `  ${cols}\r\n`, gated: false };
    }
    if (t0 === 'h') {
      // ⚠ 固件这里吐 5406 B（会被 2048 B 缓冲截断）。假臂只给摘要 ——
      //   保持"短"更利于自检（要验证截断另有用例）。
      return { out:
        '  === ArmControl_HAL console (simulated) ===\r\n' +
        '  s        status / joint angles\r\n' +
        '  zi/zr    Home zero check / reset\r\n' +
        '  F        save params to flash\r\n' +
        '  M a      center-calibrate all six\r\n' +
        '  link ... link preference (can/net/off, save)\r\n' +
        '  (real firmware lists all 124 commands here)\r\n', gated: false };
    }
    if (t0 === 'zi') {
      const zr = this.zeroRaw.map((v) => String(v).padStart(4)).join(' ');
      const ok = this.zeroRaw.every((v) => v !== 0);
      return { out:
        '\r\n  ===== zi: Home 零点 / IK 限位 状态 =====\r\n' +
        '  calib_zero_raw (0=未校准 ⇒ 出厂中位 180°)\r\n' +
        `     [${zr}]\r\n  ` +
        (ok ? '已校准。' : '⚠ 有轴为 0 ⇒ 未校准, 量纲基准是脆的。') + '\r\n\r\n',
        gated: false };
    }
    if (t0 === 'F') return { out: '  Parameters saved to flash.\r\n', gated: false };
    if (t0 === 'M' && tok.length > 1 && tok[1] === 'a') {
      this.posRaw = new Array(6).fill(2048);
      return { out:
        '  J1 (bus1 id1): writing 40=128... pos=2048 OK (≈2048)\r\n' +
        '  ... (six joints) ...\r\n' +
        "  All 6 centered. WARNING: re-run 'c' Phase1+2 and save now!\r\n",
        gated: false };
    }
    if (t0 === 'zr') {
      const save = tok.length > 1 && tok[1] === 'save';
      this.zeroRaw = new Array(6).fill(0);
      return { out:
        '\r\n  ===== zr: Home 零点已复位为出厂中位 (SERVO_OFFSET=180°) =====\r\n  ' +
        (save ? '已写入 Flash (重启保持)。'
              : '仅本次运行有效 — 重启会恢复 Flash 里的旧值; 要固化请敲 `zr save`。') +
        '\r\n', gated: false };
    }
    if (t0 === 'link') {
      if (tok.length >= 2 && ['can', 'net', 'off'].includes(tok[1])) {
        return { out: `  Link preference -> ${tok[1]} (not saved yet; run \`link save\`, ` +
                      'then RESET)\r\n', gated: false };
      }
      if (tok.length >= 2 && tok[1] === 'save') {
        return { out: '  Link preference saved to flash.\r\n', gated: false };
      }
      return { out: '  Link preference: net (saved) / active: net\r\n', gated: false };
    }
    if (t0 === 'lk') {
      return { out: '  Link UART4 (PA0/PA1 @115200): ' +
                    (this.streamActive ? 'STREAMING' : 'idle') + '\r\n', gated: false };
    }
    if (t0 === 'f') {
      const p = this._fk();
      return { out: `  FK: x=${p.x.toFixed(1)} y=${p.y.toFixed(1)} ` +
                    `z=${p.z.toFixed(1)} pitch=${p.pitchDeg.toFixed(2)}\r\n`,
               gated: false };
    }
    return { out: `  Unknown command '${t0[0]}'. Type 'h' for help.\r\n`, gated: false };
  }

  /* ---- FK（懒加载 kinematics，避免循环依赖）---- */
  _fk() {
    try {
      // 同步 require 不可行（ESM），所以这里用一个内置的简化 FK：
      // 只在自检/示例里要个"像样"的数字，精确值请用 kinematics.js 自己算。
      const { fkFromJointDeg } = SimTransport._kin || {};
      if (fkFromJointDeg) {
        const logical = this.servoDeg.map((s, j) =>
          ((this.dir[j] * (s - this.offset[j]) + 180) % 360 + 360) % 360 - 180);
        const p = fkFromJointDeg(logical);
        return { x: p.x, y: p.y, z: p.z, pitchDeg: (p.pitch * 180) / Math.PI };
      }
    } catch { /* 落到下面的占位 */ }
    return { x: 0, y: 0, z: 0, pitchDeg: 0 };
  }

  /** 注入 kinematics 模块（由 index.js 在加载时调用），让 sim 的 FK 有真值。 */
  static installKinematics(kin) { SimTransport._kin = kin; }
}

SimTransport._kin = null;

/** 交互式命令：单独敲 `c` 会进 `for(;;)` 等人在**本机串口**输入 ⇒ 从网口进
 *  主循环永不返回。而本工程**没有看门狗**（红线 5）⇒ 板子彻底失联。 */
SimTransport.TEXT_DENY_BLOCKING = ['c'];

/** 会抢 ETH 引脚的命令（NET 构建）：PB12/PB13 = ETH_RXD1/TXD1 = CAN2_RX/TX。
 *  敲一次就把网口掐死 ⇒ 固件在 ProcessCommand 开头直接停用它们。 */
SimTransport.TEXT_DENY_PINS = ['cx', 'cp', 'cy', 'cz', 'cw', 'ps', 'can',
                               'mg', 'mp', 'mo', 'ml'];

/* ==========================================================================
 * 自检
 * ========================================================================== */

export async function selftest() {
  let n = 0;
  const fails = [];
  const chk = (name, cond, extra = '') => {
    n++;
    if (!cond) { fails.push(name + ' ' + extra); console.log('  FAIL ' + name + '  ' + extra); }
  };
  const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

  // 小工具：发一条、等一条
  const roundTrip = async (s, seq, cmd, pl) => {
    const before = s.drain().length;
    s.send(seq, cmd, pl);
    await sleep(20);
    const evs = s.drain().filter((e) => e.kind === 'packet');
    return evs.length ? evs[evs.length - 1] : null;
  };

  const s = new SimTransport();
  await s.open();

  // PING
  let r = await roundTrip(s, 1, CMD.PING, new Uint8Array(0));
  chk('PING 回 0x81', r && r.cmd === RSP.PING, r ? r.cmd.toString(16) : 'null');
  chk('PING 负载 2 字节', r && r.payload.length === 2);
  chk('proto_ver=1', r && r.payload[0] === 1);

  // 查询类响应码 = 请求码 | 0x80
  r = await roundTrip(s, 2, CMD.GET_STATUS, new Uint8Array(0));
  chk('GET_STATUS → 0x82 (= 0x02|0x80)', r && r.cmd === 0x82);
  chk('GET_STATUS 负载 21 字节', r && r.payload.length === 21);
  const st = decodeStatus(r.payload);
  chk('flags 含 read_ok(0x08)（无故障轴）', st.read_ok === true, st.flags.toString(16));
  chk('初值全 180', st.joints.every((v) => v === 180), JSON.stringify(st.joints));

  // 控制类回 ACK
  r = await roundTrip(s, 3, CMD.STOP, new Uint8Array(0));
  chk('STOP → ACK 0x7F', r && r.cmd === RSP.ACK, r ? r.cmd.toString(16) : 'null');
  chk('ACK 负载 = [cmd, err=0]',
      r && r.payload[0] === CMD.STOP && r.payload[1] === ErrCode.OK);

  // CMD >= 0x80 故意不回
  const q0 = s.drain().length;
  s.send(4, 0x82, new Uint8Array(0));
  await sleep(20);
  chk('CMD>=0x80 故意不回（防死锁）', s.drain().filter((e) => e.kind === 'packet').length === 0);

  // 读失败哨兵
  s.setJointFail([2]);
  r = await roundTrip(s, 5, CMD.GET_STATUS, new Uint8Array(0));
  const st2 = decodeStatus(r.payload);
  chk('J3 读失败 → null', st2.joints[2] === null, String(st2.joints[2]));
  chk('J3 读失败 → read_ok=0', st2.read_ok === false);
  chk('failed_joints=[3]', JSON.stringify(
      st2.joints.map((v, i) => (v === null ? i + 1 : null)).filter((v) => v !== null)) === '[3]');
  s.setJointFail([]);

  // FK_QUERY 偏移 20 是裸值
  r = await roundTrip(s, 6, CMD.FK_QUERY, new Uint8Array(0));
  chk('FK_QUERY → 0xA1', r && r.cmd === RSP.FK);
  chk('FK read_ok 裸值 === 1', r && r.payload[20] === 1, r ? String(r.payload[20]) : 'null');

  // 静默钳制：MOVE_JOINTS 超范围回 ACK，但真值被钳
  const { payloadMoveJoints, payloadStreamJoints, payloadParamWrite } =
    await import('./protocol.js');
  r = await roundTrip(s, 7, CMD.MOVE_JOINTS, payloadMoveJoints([999, 999, 999, 999, 999, 999], 50));
  chk('MOVE_JOINTS 超范围仍回 ACK(err=0)', r && r.cmd === RSP.ACK);
  await sleep(300);
  r = await roundTrip(s, 8, CMD.GET_STATUS, new Uint8Array(0));
  const st3 = decodeStatus(r.payload);
  chk('MOVE_JOINTS 值被静默钳制（!=999）', st3.joints[0] !== 999, String(st3.joints[0]));
  chk('MOVE_JOINTS 自动上力矩', s.torque.every((t) => t === 1));

  // 流模式：**用一台干净的假臂** —— 上面 MOVE_JOINTS 把臂推到了 360°，
  // 直接接着发逻辑角 0（= 舵机 180°）会被判成 180° 跳变而拒，那是测试自身的问题。
  const ss = new SimTransport();
  await ss.open();
  r = await roundTrip(ss, 9, CMD.STREAM_JOINTS, payloadStreamJoints([0, 0, 0, 0, 0, 0]));
  chk('STREAM_JOINTS 正常帧 ACK', r && r.cmd === RSP.ACK,
      r ? r.cmd.toString(16) + '/' + r.payload[1] : 'null');
  chk('流计数 +1', ss.streamFrames === 1, String(ss.streamFrames));

  // 25° 跳变整帧拒收
  r = await roundTrip(ss, 10, CMD.STREAM_JOINTS, payloadStreamJoints([0, 0, 0, 0, 0, 60]));
  chk('>25° 跳变 → NACK err=4(LIMIT)',
      r && r.cmd === RSP.NACK && r.payload[1] === ErrCode.LIMIT,
      r ? r.cmd.toString(16) + '/' + r.payload[1] : 'null');
  chk('拒帧计数 +1', ss.streamRejects === 1, String(ss.streamRejects));

  // 自锁：拒帧后基准**不更新** ⇒ 只要还发那个大目标，就永远被拒（冻结）
  r = await roundTrip(ss, 11, CMD.STREAM_JOINTS, payloadStreamJoints([0, 0, 0, 0, 0, 60]));
  chk('拒帧后基准不更新（同一个大目标仍被拒 = 自锁）',
      r && r.cmd === RSP.NACK && r.payload[1] === ErrCode.LIMIT);
  chk('自锁期间流帧数不增', ss.streamFrames === 1, String(ss.streamFrames));
  chk('拒帧计数累加到 2', ss.streamRejects === 2, String(ss.streamRejects));

  // 但只要改成小步递进，就能正常过 —— 所以发送端必须自己限速，别指望固件帮渐进
  await roundTrip(ss, 12, CMD.STREAM_JOINTS, payloadStreamJoints([0, 0, 0, 0, 0, 5]));
  chk('改小步（5°）就能过', ss.streamFrames === 2, String(ss.streamFrames));
  await roundTrip(ss, 13, CMD.STREAM_JOINTS, payloadStreamJoints([0, 0, 0, 0, 0, 10]));
  chk('基准随成功帧前移', ss.streamFrames === 3, String(ss.streamFrames));
  await ss.close();

  // PARAM_WRITE 静默钳制
  r = await roundTrip(s, 12, CMD.PARAM_WRITE, payloadParamWrite(3, 1e9));
  chk('PARAM_WRITE 超范围仍回 ACK(err=0)',
      r && r.cmd === RSP.ACK && r.payload[1] === 0);
  r = await roundTrip(s, 13, CMD.PARAM_READ, Uint8Array.from([3]));
  const back = ((r.payload[1] | (r.payload[2] << 8) | (r.payload[3] << 16) |
                 (r.payload[4] << 24)) | 0) / 1000;
  chk('回读 != 输入（静默钳制）', back !== 1e9, String(back));
  chk('回读 == 上限 10.0', Math.abs(back - 10.0) < 1e-6, String(back));

  // 非法参数 ID
  r = await roundTrip(s, 14, CMD.PARAM_READ, Uint8Array.from([200]));
  chk('PARAM_READ 非法 ID → err=1', r && r.payload[1] === ErrCode.PARAM);

  // 非法关节号
  r = await roundTrip(s, 15, CMD.SET_TORQUE, Uint8Array.from([6, 1]));
  chk('SET_TORQUE J6(=6) → err=1（0 基，6 非法）', r && r.payload[1] === ErrCode.PARAM);

  // PARAM_COUNT
  r = await roundTrip(s, 16, CMD.PARAM_COUNT, new Uint8Array(0));
  chk('PARAM_COUNT → 0xC2', r && r.cmd === RSP.PARAM_COUNT);
  chk('PARAM_COUNT = 24', r && r.payload[0] === 24, r ? String(r.payload[0]) : 'null');

  // 急停
  s.setEstop(true);
  r = await roundTrip(s, 17, CMD.MOVE_JOINTS, payloadMoveJoints([180, 180, 180, 180, 180, 180]));
  chk('急停时 MOVE_JOINTS → err=5(ESTOP)', r && r.payload[1] === ErrCode.ESTOP);
  r = await roundTrip(s, 18, CMD.STREAM_JOINTS, payloadStreamJoints([0, 0, 0, 0, 0, 0]));
  chk('急停时 STREAM → err=5', r && r.payload[1] === ErrCode.ESTOP);
  s.setEstop(false);

  // IK 不可达
  s.setUnreachable(true);
  const { payloadMoveCart } = await import('./protocol.js');
  r = await roundTrip(s, 19, CMD.MOVE_CART, payloadMoveCart(9999, 0, 0));
  chk('unreachable → err=3(UNREACH)', r && r.payload[1] === ErrCode.UNREACH);
  s.setUnreachable(false);

  // 丢包
  const s2 = new SimTransport({ loss: 1.0 });
  await s2.open();
  r = await roundTrip(s2, 1, CMD.PING, new Uint8Array(0));
  chk('loss=1 时收不到回包（超时语义）', r === null);
  await s2.close();

  // 看门狗
  const s3 = new SimTransport({ watchdogMs: 60 });
  await s3.open();
  await roundTrip(s3, 1, CMD.STREAM_JOINTS, payloadStreamJoints([0, 0, 0, 0, 0, 0]));
  chk('看门狗开启后流激活', s3.streamActive === true);
  await sleep(120);
  await roundTrip(s3, 2, CMD.GET_STATUS, new Uint8Array(0));
  chk('断流 >watchdog ⇒ 退出流模式', s3.streamActive === false);
  await s3.close();

  await s.close();

  /* ======================================================================
   * 2026-09-23 扩展：新命令自检
   * 用一台**干净的**假臂，避免被上面的 MOVE_JOINTS/estop 残留污染。
   * ==================================================================== */
  const {
    payloadGetJointDetail, payloadIkQuery, payloadGetPid, payloadSetBusMode,
    payloadServoInit, payloadParamList, payloadTeachList, payloadTeachRead,
    payloadTeachDelete, payloadMagic, payloadZeroReset, payloadTextExec,
    payloadTextFetch, payloadMoveJointOne,
    decodeJointDetail, decodeLimits, decodeInfo, decodeFlags, decodeTeachTotal,
    decodeParamList, decodeTextOut,
  } = await import('./protocol.js');

  const g = new SimTransport();
  await g.open();

  // ---- 0x04 GET_JOINT_DETAIL ----
  r = await roundTrip(g, 1, CMD.GET_JOINT_DETAIL, payloadGetJointDetail(0xFF));
  chk('GET_JOINT_DETAIL 六轴 → 0x84 / 62 B',
      r && r.cmd === RSP.JOINT_DETAIL && r.payload.length === 62,
      r ? r.cmd.toString(16) + '/' + r.payload.length : 'null');
  const jd = decodeJointDetail(r.payload);
  chk('六轴 n=6 且 joint 递增', jd.length === 6 && jd.every((d, i) => d.joint === i));
  chk('温度 = 42（正常拍）', jd[0].temp === 42 && jd[0].ok === true);
  r = await roundTrip(g, 2, CMD.GET_JOINT_DETAIL, payloadGetJointDetail(2));
  chk('GET_JOINT_DETAIL 单轴 → 12 B / n=1',
      r && r.payload.length === 12 && r.payload[0] === 1);
  g.setJointFail([2]);
  r = await roundTrip(g, 3, CMD.GET_JOINT_DETAIL, payloadGetJointDetail(0xFF));
  const jd2 = decodeJointDetail(r.payload);
  chk('丢读轴 temp_lost 用哨兵（不是 0 度）',
      jd2[2].temp_lost === true && jd2[2].hw_err_lost === true && jd2[2].ok === false);
  g.setJointFail([]);
  r = await roundTrip(g, 4, CMD.GET_JOINT_DETAIL, Uint8Array.from([9]));
  chk('GET_JOINT_DETAIL 非法关节 → ERR_PARAM', r && r.payload[1] === ErrCode.PARAM);

  // ---- 0x05 GET_LIMITS ----
  r = await roundTrip(g, 5, CMD.GET_LIMITS, new Uint8Array(0));
  chk('GET_LIMITS → 0x85 / 48 B',
      r && r.cmd === RSP.LIMITS && r.payload.length === 48,
      r ? r.cmd.toString(16) + '/' + r.payload.length : 'null');
  const lim = decodeLimits(r.payload);
  chk('zero_raw 全 0 ⇒ calibrated 全 false（量纲基准脆）',
      lim.calibrated.every((c) => c === false));
  chk('pos_raw 默认 2048（中位）', lim.pos_raw.every((v) => v === 2048));

  // ---- 0x07 GET_INFO ----
  r = await roundTrip(g, 6, CMD.GET_INFO, new Uint8Array(0));
  chk('GET_INFO → 0x87 / 20 B',
      r && r.cmd === RSP.INFO && r.payload.length === 20,
      r ? r.cmd.toString(16) + '/' + r.payload.length : 'null');
  const inf = decodeInfo(r.payload);
  chk('fw_tag = SIM-NET-0923A', inf.fw_tag === 'SIM-NET-0923A', inf.fw_tag);
  chk('build_name = net', inf.build_name === 'net', inf.build_name);
  chk('n_params = 24', inf.n_params === 24, String(inf.n_params));

  // ---- 0x0A GET_FLAGS ----
  r = await roundTrip(g, 7, CMD.GET_FLAGS, new Uint8Array(0));
  chk('GET_FLAGS → 0x8A / 2 B',
      r && r.cmd === RSP.FLAGS && r.payload.length === 2);
  const fl = decodeFlags(r.payload);
  chk('初态 flags 全 0 / run_id=0x5A', fl.raw === 0 && fl.run_id === 0x5A);
  g.setEstop(true);
  r = await roundTrip(g, 8, CMD.GET_FLAGS, new Uint8Array(0));
  chk('急停 ⇒ flags bit0', decodeFlags(r.payload).estop === true);
  g.setEstop(false);

  // ---- 0x14 MOVE_JOINT_ONE ----
  r = await roundTrip(g, 9, CMD.MOVE_JOINT_ONE, payloadMoveJointOne(2, 100, 30));
  chk('MOVE_JOINT_ONE → ACK', r && r.cmd === RSP.ACK && r.payload[1] === ErrCode.OK);
  chk('MOVE_JOINT_ONE 只改 J3 目标（其余留在中位）',
      Math.abs(g.targetDeg[2] - 100) < 1 && g.targetDeg.filter((v) => Math.abs(v - 180) > 0.1).length === 1,
      JSON.stringify(g.targetDeg));
  g.setJointFail([4]);
  r = await roundTrip(g, 10, CMD.MOVE_JOINT_ONE, payloadMoveJointOne(2, 90));
  chk('任一轴丢读 ⇒ 整条 ERR_READ(7)（拿不准就不动）',
      r && r.payload[1] === ErrCode.READ, r ? String(r.payload[1]) : 'null');
  g.setJointFail([]);

  // ---- 0x17 SET_BUS_MODE ----
  r = await roundTrip(g, 11, CMD.SET_BUS_MODE, Uint8Array.from([2]));
  chk('SET_BUS_MODE 2 → ACK 且记账', r && r.payload[1] === 0 && g.busMode === 2);
  r = await roundTrip(g, 12, CMD.SET_BUS_MODE, Uint8Array.from([3]));
  chk('SET_BUS_MODE 3（裸包）→ ERR_PARAM（固件只允许 0/1/2）',
      r && r.payload[1] === ErrCode.PARAM);
  chk('SET_BUS_MODE 3 经 SDK 在**客户端**就被拦住（不发帧）',
      (() => { try { payloadSetBusMode(3); return false; } catch { return true; } })());

  // ---- 0x18 SERVO_INIT ----
  r = await roundTrip(g, 13, CMD.SERVO_INIT, payloadServoInit(0xFF));
  chk('SERVO_INIT → ACK', r && r.payload[1] === 0);
  r = await roundTrip(g, 14, CMD.SERVO_INIT, Uint8Array.from([6]));
  chk('SERVO_INIT J7(=6)（裸包）→ ERR_PARAM', r && r.payload[1] === ErrCode.PARAM);
  chk('SERVO_INIT 6 经 SDK 在客户端被拦住',
      (() => { try { payloadServoInit(6); return false; } catch { return true; } })());

  // ---- 0x22 IK_QUERY ----
  r = await roundTrip(g, 15, CMD.IK_QUERY, payloadIkQuery(200, 0, 150, 0));
  chk('IK_QUERY 可解 → 0xA2 / 12 B',
      r && r.cmd === RSP.IK_QUERY && r.payload.length === 12,
      r ? r.cmd.toString(16) + '/' + r.payload.length : 'null');
  r = await roundTrip(g, 16, CMD.IK_QUERY, payloadIkQuery(2000, 2000, 2000, 0));
  chk('IK_QUERY 远超臂展 → ERR_UNREACH(3)',
      r && r.cmd === RSP.NACK && r.payload[1] === ErrCode.UNREACH,
      r ? String(r.payload[1]) : 'null');
  r = await roundTrip(g, 17, CMD.IK_QUERY, new Uint8Array(4));
  chk('IK_QUERY 负载过短 → ERR_PARAM', r && r.payload[1] === ErrCode.PARAM);

  // ---- 0x32 / 0x33 / 0x36 读回 ----
  r = await roundTrip(g, 18, CMD.GET_GRAVITY, new Uint8Array(0));
  chk('GET_GRAVITY → 0xB2', r && r.cmd === RSP.GRAVITY && r.payload.length === 2);
  r = await roundTrip(g, 19, CMD.GET_CORRECTION, new Uint8Array(0));
  chk('GET_CORRECTION → 0xB3 / 5 B',
      r && r.cmd === RSP.CORRECTION && r.payload.length === 5);
  r = await roundTrip(g, 20, CMD.GET_PID, payloadGetPid(0));
  chk('GET_PID → 0xB6 / 4 B（p/i/d/from_eeprom）',
      r && r.cmd === RSP.PID && r.payload.length === 4 && r.payload[3] === 1);
  r = await roundTrip(g, 21, CMD.GET_PID, Uint8Array.from([6]));
  chk('GET_PID J7(=6)（裸包）→ ERR_PARAM', r && r.payload[1] === ErrCode.PARAM);
  chk('GET_PID 6 经 SDK 在客户端被拦住',
      (() => { try { payloadGetPid(6); return false; } catch { return true; } })());

  // ---- 0x43 PARAM_SAVE / 0x44 PARAM_LIST ----
  r = await roundTrip(g, 22, CMD.PARAM_SAVE, payloadMagic());
  chk('PARAM_SAVE 带 MAGIC → ACK', r && r.payload[1] === 0);
  r = await roundTrip(g, 23, CMD.PARAM_SAVE, Uint8Array.from([1, 2, 3, 4]));
  chk('PARAM_SAVE MAGIC 错 → ERR_MAGIC(8)', r && r.payload[1] === ErrCode.MAGIC,
      r ? String(r.payload[1]) : 'null');
  r = await roundTrip(g, 24, CMD.PARAM_SAVE, new Uint8Array(2));
  chk('PARAM_SAVE 负载过短 → ERR_PARAM', r && r.payload[1] === ErrCode.PARAM);

  let listed = [];
  let offP = 0;
  for (let guard = 0; guard < 10; guard++) {
    r = await roundTrip(g, 25, CMD.PARAM_LIST, payloadParamList(offP));
    chk('PARAM_LIST → 0xC4（页码 ' + offP + '）', r && r.cmd === RSP.PARAM_LIST);
    const { total, items } = decodeParamList(r.payload);
    chk('PARAM_LIST 单页 ≤6 条（62 B 上限）', items.length <= 6 && r.payload.length === 2 + items.length * 10);
    if (!items.length) break;
    listed.push(...items);
    offP += items.length;
    if (offP >= total) break;
  }
  chk('PARAM_LIST 翻页列全 24 条', listed.length === 24, String(listed.length));
  chk('PARAM_LIST[0].id = 1', listed[0].id === 1, String(listed[0].id));
  chk('PARAM_LIST 用 i32 存范围（值域可能超 i16）', listed[0].vmax === 5.0,
      String(listed[0].vmax));

  // ---- 0x50 ~ 0x55 示教存储 ----
  r = await roundTrip(g, 30, CMD.TEACH_TOTAL, new Uint8Array(0));
  chk('TEACH_TOTAL → 0xD5 / 10 B',
      r && r.cmd === RSP.TEACH_TOTAL && r.payload.length === 10);
  const tt = decodeTeachTotal(r.payload);
  chk('TEACH_TOTAL n_files=2 cap=32', tt.n_files === 2 && tt.capacity === 32,
      JSON.stringify(tt));
  r = await roundTrip(g, 31, CMD.TEACH_LIST, payloadTeachList(0));
  chk('TEACH_LIST → 0xD0 / 19 B', r && r.cmd === RSP.TEACH_LIST && r.payload.length === 19);
  chk('TEACH_LIST #0 = demo(12 样本)',
      r.payload[2] === 1 && String.fromCharCode(r.payload[3], r.payload[4], r.payload[5], r.payload[6]) === 'demo');
  r = await roundTrip(g, 32, CMD.TEACH_LIST, payloadTeachList(9));
  chk('TEACH_LIST 越界 idx → ERR_PARAM', r && r.payload[1] === ErrCode.PARAM);
  r = await roundTrip(g, 33, CMD.TEACH_READ, payloadTeachRead(0, 0, 5));
  chk('TEACH_READ → 0xD1 / 1+5×12=61 B',
      r && r.cmd === RSP.TEACH_READ && r.payload.length === 61,
      r ? String(r.payload.length) : 'null');
  r = await roundTrip(g, 34, CMD.TEACH_READ, payloadTeachRead(0, 100, 5));
  chk('TEACH_READ off 越界 → n=0（正常读完，不是错误）',
      r && r.cmd === RSP.TEACH_READ && r.payload[0] === 0);
  r = await roundTrip(g, 35, CMD.TEACH_DELETE, payloadTeachDelete(1));
  chk('TEACH_DELETE → ACK', r && r.payload[1] === 0);
  chk('TEACH_DELETE 写 tombstone（idx 1 valid=0）', g.teach[1].valid === false);
  r = await roundTrip(g, 36, CMD.TEACH_DELETE, payloadTeachDelete(9));
  chk('TEACH_DELETE 越界 → ERR_PARAM', r && r.payload[1] === ErrCode.PARAM);
  r = await roundTrip(g, 37, CMD.TEACH_COMPACT, new Uint8Array(0));
  chk('TEACH_COMPACT → ACK', r && r.payload[1] === 0);
  chk('COMPACT 只清 tombstone（有效文件仍留）',
      Object.keys(g.teach).length === 1 && g.teach[0] !== undefined,
      JSON.stringify(Object.keys(g.teach)));
  await roundTrip(g, 38, CMD.TEACH_DELETE, payloadTeachDelete(0));
  await roundTrip(g, 39, CMD.TEACH_COMPACT, new Uint8Array(0));
  chk('全部删光 + COMPACT ⇒ 0 个文件',
      Object.keys(g.teach).length === 0, JSON.stringify(Object.keys(g.teach)));

  // ---- 0x70 ~ 0x73 危险操作 ----
  r = await roundTrip(g, 40, CMD.CENTER_CALIB, payloadMagic());
  chk('CENTER_CALIB 带 MAGIC → ACK', r && r.payload[1] === 0);
  chk('CENTER_CALIB 把 pos_raw 推回 2048', g.posRaw.every((v) => v === 2048));
  r = await roundTrip(g, 41, CMD.CENTER_CALIB, Uint8Array.from([9, 9, 9, 9]));
  chk('CENTER_CALIB MAGIC 错 → ERR_MAGIC(8)', r && r.payload[1] === ErrCode.MAGIC);
  r = await roundTrip(g, 42, CMD.CALIB_ENTER, payloadMagic());
  chk('CALIB_ENTER **一律 NACK(ERR_PARAM)**（交互式会卡死主循环）',
      r && r.cmd === RSP.NACK && r.payload[1] === ErrCode.PARAM,
      r ? r.cmd.toString(16) + '/' + r.payload[1] : 'null');
  g.zeroRaw = new Array(6).fill(2048);            // 先造出"已校准"状态
  const ziBefore = await g._textRun('zi', false);
  chk('`zi` 在已校准状态给出"已校准。"',
      ziBefore.out.includes('已校准。'), ziBefore.out.slice(0, 60));
  r = await roundTrip(g, 43, CMD.ZERO_RESET, payloadZeroReset(true));
  chk('ZERO_RESET(save) → ACK', r && r.payload[1] === 0);
  chk('ZERO_RESET 把全部六轴 zero_raw 清 0', g.zeroRaw.every((v) => v === 0));
  const ziOut = await g._textRun('zi', false);
  chk('ZERO_RESET 后 `zi` 转报警（量纲基准脆）',
      ziOut.out.includes('⚠ 有轴为 0'), ziOut.out.slice(0, 60));
  r = await roundTrip(g, 44, CMD.ZERO_RESET, payloadZeroReset());
  chk('ZERO_RESET(no save) → ACK', r && r.payload[1] === 0);
  r = await roundTrip(g, 45, CMD.ZERO_RESET,
                      Uint8Array.from([0xFF, 90, 90, 90, 90]));
  chk('ZERO_RESET flags 非法(0xFF) → ERR_PARAM（`zr` 没有单轴变体）',
      r && r.payload[1] === ErrCode.PARAM);
  const nResetBefore = g.sysResetCount;
  r = await roundTrip(g, 46, CMD.SYS_RESET, payloadMagic());
  chk('SYS_RESET 带 MAGIC → ACK', r && r.payload[1] === 0);
  chk('SYS_RESET 记账 +1', g.sysResetCount === nResetBefore + 1);
  r = await roundTrip(g, 47, CMD.SYS_RESET, Uint8Array.from([1, 2]));
  chk('SYS_RESET 负载过短 → ERR_PARAM', r && r.payload[1] === ErrCode.PARAM);

  // ---- 0x0B / 0x0C 文本透传 ----
  r = await roundTrip(g, 50, CMD.TEXT_EXEC, payloadTextExec('s'));
  chk('TEXT_EXEC → 0x8B', r && r.cmd === RSP.TEXT_OUT, r ? r.cmd.toString(16) : 'null');
  let to = decodeTextOut(r.payload);
  chk('TEXT_EXEC 分页头自洽（off=0，total≥n）',
      to.off === 0 && to.total >= to.raw.length);
  chk('TEXT_EXEC `s` 含 J1 表头', to.text.includes('J1'));
  chk('TEXT_EXEC `s` 未截断未拒绝', to.truncated === false && to.gated === false);

  r = await roundTrip(g, 51, CMD.TEXT_EXEC, payloadTextExec('c'));
  to = decodeTextOut(r.payload);
  chk('TEXT_EXEC `c` 被闸门拒 ⇒ flags.gated + 原因正文',
      to.gated === true && to.text.includes('GATE'), to.text.trim());

  r = await roundTrip(g, 52, CMD.TEXT_EXEC, payloadTextExec('cx'));
  to = decodeTextOut(r.payload);
  chk('TEXT_EXEC `cx` **不算闸门拒绝**（是 ProcessCommand 自己接住）',
      to.gated === false && to.text.includes('已停用'), to.text.slice(0, 40));
  chk('TEXT_EXEC `cx` 输出是 UTF-8 多字节（total 远大于 ASCII 行数）',
      to.total > 200, String(to.total));

  // 翻页：把 `cx` 的 288 B 全文拼回来
  const parts = [to.raw];
  let off2 = to.off + to.raw.length;
  while (off2 < to.total && parts.length < 20) {
    r = await roundTrip(g, 53, CMD.TEXT_FETCH, payloadTextFetch(off2, 58));
    if (!r || r.cmd !== RSP.TEXT_FETCH) break;
    const pg = decodeTextOut(r.payload);
    if (!pg.raw.length) break;
    parts.push(pg.raw);
    off2 += pg.raw.length;
  }
  const joined = new Uint8Array(parts.reduce((a, p) => a + p.length, 0));
  let ww = 0;
  for (const p of parts) { joined.set(p, ww); ww += p.length; }
  const full = utf8Decoder.decode(joined);
  chk('TEXT_FETCH 逐页拼回 = 完整 288 B 文本',
      joined.length === to.total && full.includes('PB12/PB13') && full.includes('link can'),
      joined.length + '/' + to.total);
  chk('跨页 UTF-8 拼接无坏字符（不含 U+FFFD）', !full.includes('\uFFFD'));

  r = await roundTrip(g, 54, CMD.TEXT_FETCH, Uint8Array.from([0, 0, 0]));
  chk('TEXT_FETCH n=0 → ERR_PARAM（防死循环）', r && r.payload[1] === ErrCode.PARAM);

  r = await roundTrip(g, 55, CMD.TEXT_EXEC, Uint8Array.from([1, 2, 3, 4, 5]));
  chk('TEXT_EXEC MAGIC 错 → ERR_MAGIC(8)', r && r.payload[1] === ErrCode.MAGIC);
  r = await roundTrip(g, 56, CMD.TEXT_EXEC, new Uint8Array(3));
  chk('TEXT_EXEC 负载过短 → ERR_PARAM', r && r.payload[1] === ErrCode.PARAM);

  // 急停闸门：只放 s / h
  g.setEstop(true);
  r = await roundTrip(g, 57, CMD.TEXT_EXEC, payloadTextExec('s'));
  chk('急停时 TEXT_EXEC `s` 仍放行',
      decodeTextOut(r.payload).gated === false);
  r = await roundTrip(g, 58, CMD.TEXT_EXEC, payloadTextExec('zi'));
  to = decodeTextOut(r.payload);
  chk('急停时 TEXT_EXEC `zi` 被闸门拒（只放 s/h）',
      to.gated === true && to.text.includes('E-Stop'), to.text.trim());
  g.setEstop(false);

  await g.close();

  console.log('\n  sim.js 自检：' + n + ' 项，' +
              (fails.length ? '失败 ' + fails.length + ' 项' : '全部通过'));
  return fails.length ? 1 : 0;
}

// ⚠ 用 basename 精确比较，别用 endsWith：
//   否则 `02_offline_sim.mjs` 这种名字会 endsWith('sim.mjs') → 一 import 就误跑自检。
const _script = (typeof process !== 'undefined' && process.argv && process.argv[1]) || '';
const _base = _script.split(/[\\/]/).pop();
const isMain = _base === 'sim.js' || _base === 'sim.mjs';
if (isMain) {
  selftest().then((rc) => process.exit(rc));
}
