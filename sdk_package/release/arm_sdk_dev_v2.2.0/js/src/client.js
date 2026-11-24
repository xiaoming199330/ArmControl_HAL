/**
 * client.js — 高层客户端 ArmClient（SEQ 配对 + 超时 + 重试 + 异步流）
 *
 * 上层只跟这个类打交道。它负责：
 *   · 包 SEQ 配对（RX 泵把响应投递到对应请求的等待槽）
 *   · 超时重发：**只对幂等命令**重发，运动命令（MOVE_JOINTS/MOVE_CART/STREAM_JOINTS/
 *     SET_TORQUE）绝不自动重发 —— 重发一次就是多动一次
 *   · 把 ACK/NACK/查询响应统一成 `Reply`（**不抛异常**，要抛用 `.raiseIfFailed()`）
 *   · 流帧的 ACK 单独收（不回给调用方，避免阻塞）
 *
 * 与 Python 版 `arm_sdk/client.py` 同构。
 */

import {
  CMD, RSP, ErrCode, ERR_DESC,
  cmdName, decodeStatus, decodeFk, decodePing,
  decodeParamRead, decodeParamCount, failedJoints,
  payloadMoveJoints, payloadStreamJoints, payloadMoveCart, payloadSetTorque,
  payloadSetGravity, payloadSetCorrection, payloadParamRead, payloadParamWrite,
  payloadLogSetLevel,
  // ---- 2026-09-23 扩展 ----
  TEXT_BUF_SZ, TEXT_CHUNK, TEACH_READ_MAX, utf8Decoder,
  payloadGetJointDetail, payloadIkQuery, payloadGetPid, payloadSetBusMode,
  payloadServoInit, payloadParamList, payloadTeachList, payloadTeachRead,
  payloadTeachDelete, payloadMagic, payloadZeroReset,
  payloadTextExec, payloadTextFetch, payloadMoveJointOne,
  decodeJointDetail, decodeLimits, decodeInfo, decodeFlags, decodeIkQuery,
  decodeGravity, decodeCorrection, decodePid, decodeParamList,
  decodeTeachList, decodeTeachRead, decodeTeachTotal, decodeTextOut,
} from './protocol.js';
import { makeTransport } from './transport.js';

/** 这些命令**不自动重发**：重发等于多动一次（或把不可逆动作做两遍）。 */
export const NON_IDEMPOTENT = new Set([
  CMD.MOVE_JOINTS, CMD.MOVE_CART, CMD.STREAM_JOINTS, CMD.SET_TORQUE, CMD.STOP,
  CMD.MOVE_JOINT_ONE, CMD.SERVO_INIT, CMD.TEACH_COMPACT,
  CMD.PARAM_SAVE, CMD.CENTER_CALIB, CMD.CALIB_ENTER, CMD.ZERO_RESET, CMD.SYS_RESET,
  CMD.TEXT_EXEC,                      // 等于整个控制台，可能改基准/松力矩
]);

/** 查询类命令 → 期望的响应码（= 请求码 | 0x80）。
 *  ⚠ 漏登记一条的后果：响应码对不上 ⇒ 被当成"意外的响应码"报错。 */
const QUERY_RSP = {
  [CMD.PING]: RSP.PING,
  [CMD.GET_STATUS]: RSP.STATUS,
  [CMD.FK_QUERY]: RSP.FK,
  [CMD.PARAM_READ]: RSP.PARAM_READ,
  [CMD.PARAM_COUNT]: RSP.PARAM_COUNT,
  /* 2026-09-23 扩展 */
  [CMD.GET_JOINT_DETAIL]: RSP.JOINT_DETAIL,
  [CMD.GET_LIMITS]: RSP.LIMITS,
  [CMD.GET_INFO]: RSP.INFO,
  [CMD.GET_FLAGS]: RSP.FLAGS,
  [CMD.TEXT_EXEC]: RSP.TEXT_OUT,
  [CMD.TEXT_FETCH]: RSP.TEXT_FETCH,
  [CMD.IK_QUERY]: RSP.IK_QUERY,
  [CMD.GET_GRAVITY]: RSP.GRAVITY,
  [CMD.GET_CORRECTION]: RSP.CORRECTION,
  [CMD.GET_PID]: RSP.PID,
  [CMD.PARAM_LIST]: RSP.PARAM_LIST,
  [CMD.TEACH_LIST]: RSP.TEACH_LIST,
  [CMD.TEACH_READ]: RSP.TEACH_READ,
  [CMD.TEACH_TOTAL]: RSP.TEACH_TOTAL,
};

export class ArmCommandError extends Error {
  constructor(reply) {
    super(`${reply.label} 失败：${reply.desc}` +
          (reply.err !== null ? ` (err=${reply.err}/${reply.errName})` : ''));
    this.name = 'ArmCommandError';
    this.reply = reply;
  }
}

/** 一条命令的结果。**不抛异常**，看 `.ok` / `.err`。 */
export class Reply {
  constructor(fields = {}) {
    this.seq = fields.seq ?? -1;
    this.reqCmd = fields.reqCmd ?? -1;
    this.cmd = fields.cmd ?? -1;
    this.payload = fields.payload ?? new Uint8Array(0);
    this.ok = !!fields.ok;
    this.err = fields.err ?? null;
    this.errName = fields.errName ?? '';
    this.desc = fields.desc ?? '';
    this.latencyMs = fields.latencyMs ?? 0;
    this.attempts = fields.attempts ?? 1;
    this.timeout = !!fields.timeout;
    this.label = fields.label ?? '';
    this.dryRun = !!fields.dryRun;
  }

  raiseIfFailed() {
    if (!this.ok) throw new ArmCommandError(this);
    return this;
  }

  toString() {
    const ms = this.latencyMs.toFixed(1);
    if (this.dryRun) return `${this.label} (dry-run 未发送)`;
    if (this.timeout) return `✗ ${this.label} 超时 (${ms}ms)`;
    return `${this.ok ? '✓' : '✗'} ${this.label} (${ms}ms)` +
           (this.ok ? '' : `  ${this.desc}` + (this.err !== null ? ` [err=${this.err}]` : ''));
  }
}

export class ClientStats {
  constructor() {
    this.sent = 0; this.recv = 0; this.timeout = 0; this.retries = 0;
    this.bad = 0; this.streamAcks = 0;
  }
  summary() {
    return `sent=${this.sent} recv=${this.recv} timeout=${this.timeout} ` +
           `retries=${this.retries} bad=${this.bad} streamAck=${this.streamAcks}`;
  }
}

class Slot {
  constructor() {
    this.reply = null;
    this.waiters = [];
  }
  resolve(r) {
    this.reply = r;
    const w = this.waiters;
    this.waiters = [];
    for (const fn of w) fn(r);
  }
}

/* ==========================================================================
 * ArmClient
 * ========================================================================== */

export class ArmClient {
  /**
   * @param {import('./transport.js').Transport} transport
   * @param {{timeout?:number, retries?:number, dryRun?:boolean,
   *          verbose?:boolean, onEvent?:(s:string)=>void}} [opts]
   */
  constructor(transport, opts = {}) {
    this.transport = transport;
    this.timeout = opts.timeout ?? 0.3;          // 秒（跟 Python 版一致）
    this.retries = opts.retries ?? 1;
    this.dryRun = !!opts.dryRun;
    this.verbose = !!opts.verbose;
    this.onEvent = opts.onEvent || null;

    this.stats = new ClientStats();
    this._slots = new Map();
    this._streamAcks = [];
    this._seq = 0;
    this._pumpTimer = null;
    this._pumping = false;
    this._open = false;
  }

  async open() {
    if (this._open) return this;
    await this.transport.open?.();
    this._open = true;
    this._startPump();
    return this;
  }

  async close() {
    this._stopPump();
    try { await this.transport.close?.(); } catch { /* 忽略 */ }
    this._open = false;
  }

  get isOpen() { return this._open; }
  get link() { return this.transport.link || 'unknown'; }

  /* ---- RX 泵 ---- */
  _startPump() {
    this._pumping = true;
    const tick = () => {
      if (!this._pumping) return;
      try {
        for (const ev of this.transport.drain()) this._onEvent(ev);
      } catch (e) {
        if (this.onEvent) this.onEvent('RX 泵异常: ' + e.message);
      }
      this._pumpTimer = setTimeout(tick, 1);
      // ⚠ 必须 unref：否则这个 1ms 循环会把 Node 事件循环**永远吊住**，
      //   脚本跑完不退出（表现为 `node xxx.js` 卡住不回命令行）。
      this._pumpTimer.unref?.();
    };
    this._pumpTimer = setTimeout(tick, 1);
    this._pumpTimer.unref?.();
  }

  _stopPump() {
    this._pumping = false;
    if (this._pumpTimer) clearTimeout(this._pumpTimer);
    this._pumpTimer = null;
  }

  _onEvent(ev) {
    if (ev.kind === 'packet') {
      this.stats.recv++;
      const slot = this._slots.get(ev.seq);
      if (slot) {
        slot.resolve(new Reply({
          seq: ev.seq, cmd: ev.cmd, payload: ev.payload,
        }));
      } else {
        // 没人等的包：多半是流帧 ACK
        this._streamAcks.push(new Reply({ seq: ev.seq, cmd: ev.cmd, payload: ev.payload }));
        if (this._streamAcks.length > 4096) this._streamAcks.splice(0, 2048);
      }
    } else if (ev.kind === 'bad') {
      this.stats.bad++;
      if (this.verbose && this.onEvent) this.onEvent('坏包: ' + ev);
    } else if (ev.kind === 'text') {
      if (this.onEvent) this.onEvent('文本: ' + ev.line);
    }
  }

  nextSeq() {
    this._seq = (this._seq + 1) & 0xFF;
    return this._seq;
  }

  /** 发一条不等响应的包（流模式用）。 */
  sendAsync(cmd, payload = new Uint8Array(0)) {
    const seq = this.nextSeq();
    this.transport.send(seq, cmd, payload);
    this.stats.sent++;
    return seq;
  }

  /**
   * 发一条命令并等响应。
   * @param {number} cmd
   * @param {Uint8Array} [payload]
   * @param {{timeout?:number, retries?:number, label?:string, idempotent?:boolean}} [opts]
   * @returns {Promise<Reply>}
   */
  async request(cmd, payload = new Uint8Array(0), opts = {}) {
    const timeout = (opts.timeout ?? this.timeout) * 1000;
    const label = opts.label || cmdName(cmd);
    const idempotent = opts.idempotent !== undefined
      ? opts.idempotent : !NON_IDEMPOTENT.has(cmd);
    const tries = idempotent ? (opts.retries ?? this.retries) + 1 : 1;

    if (this.dryRun) {
      return new Reply({ seq: -1, reqCmd: cmd, cmd: -1, ok: true, err: 0,
                        errName: 'DRY', desc: 'dry-run 未发送', label, dryRun: true });
    }
    if (!this._open) throw new Error('ArmClient 未 open()');

    let last = new Reply({ seq: -1, reqCmd: cmd, cmd: -1, ok: false, timeout: true, label });

    for (let attempt = 1; attempt <= tries; attempt++) {
      const seq = this.nextSeq();
      const slot = new Slot();
      this._slots.set(seq, slot);
      const t0 = Date.now();

      try {
        this.transport.send(seq, cmd, payload);
        this.stats.sent++;
      } catch (e) {
        this._slots.delete(seq);
        return new Reply({ seq, reqCmd: cmd, ok: false, timeout: false,
                           desc: '发送失败: ' + e.message, label, attempts: attempt });
      }

      // ⚠ 超时定时器收到响应后必须 clear：否则每个"按时返回"的请求都会
      //   在事件循环里留一颗活雷，跑几千次请求的脚本收尾时会多挂一阵。
      //   ⚠ 但**不能 unref**：请求在途时，这颗定时器是"还有活没干完"的唯一凭据；
      //   RX 泵的 1ms 定时器是 unref 的，一旦把超时也 unref，事件循环会在
      //   响应投递之前就空掉 → 顶层 await 永远等不到 → Node 以退出码 13 收场。
      let timer = null;
      const got = await Promise.race([
        new Promise((res) => slot.waiters.push(res)),
        new Promise((res) => { timer = setTimeout(() => res(null), timeout); }),
      ]);
      if (timer) clearTimeout(timer);
      const dt = Date.now() - t0;
      this._slots.delete(seq);

      if (!got) {
        this.stats.timeout++;
        last = new Reply({
          seq, reqCmd: cmd, ok: false, timeout: true, latencyMs: dt,
          attempts: attempt, label,
          desc: '超时（查：目的端口 5000 / 报文长度是否 = 5+LEN / 链路是否开）',
        });
        if (attempt < tries) { this.stats.retries++; continue; }
        return last;
      }

      const r = got;
      r.reqCmd = cmd;
      r.label = label;
      r.latencyMs = dt;
      r.attempts = attempt;

      const exp = QUERY_RSP[cmd];
      if (r.cmd === exp) {
        r.ok = true; r.err = 0; r.errName = 'OK'; r.desc = 'OK';
      } else if (r.cmd === RSP.ACK && r.payload.length >= 2) {
        r.ok = true;
        r.err = r.payload[1];
        r.errName = errName(r.err);
        r.desc = ERR_DESC[r.err] || '?';
      } else if (r.cmd === RSP.NACK && r.payload.length >= 2) {
        r.ok = false;
        r.err = r.payload[1];
        r.errName = errName(r.err);
        r.desc = ERR_DESC[r.err] || '?';
      } else {
        r.ok = false;
        r.desc = `意外的响应码 0x${r.cmd.toString(16).toUpperCase()}` +
                 `（期望 ${exp ? '0x' + exp.toString(16).toUpperCase() : 'ACK/NACK'}）`;
      }
      // 只有超时才重发；明确 NACK 不重发
      if (r.ok || !r.timeout) return r;
      last = r;
    }
    return last;
  }

  /** 取走流帧的 ACK（异步流模式用，不回给业务层）。 */
  pollStreamAcks(drain = true) {
    const out = this._streamAcks;
    if (drain) this._streamAcks = [];
    return out;
  }

  /* ======================================================================
   * 高层 API（14 条命令全覆盖）
   * ==================================================================== */

  /** 0x01 PING → {proto_ver, run_id} 或 null。 */
  async ping(opts = {}) {
    const r = await this.request(CMD.PING, new Uint8Array(0), { ...opts, label: 'PING' });
    if (!r.ok) return null;
    const info = decodePing(r.payload);
    info.latencyMs = r.latencyMs;
    info.reply = r;
    return info;
  }

  /** 0x02 GET_STATUS → 状态对象（**不兜底**：读失败轴是 null）。 */
  async getStatus(opts = {}) {
    const r = await this.request(CMD.GET_STATUS, new Uint8Array(0),
                                 { ...opts, label: 'GET_STATUS' });
    if (!r.ok) return null;
    const st = decodeStatus(r.payload);
    st.latencyMs = r.latencyMs;
    st.reply = r;
    st.failedJoints = failedJoints(st);
    return st;
  }

  /** 0x21 FK_QUERY → 位姿 + 关节（注意与 GET_STATUS 顺序相反）。 */
  async fkQuery(opts = {}) {
    const r = await this.request(CMD.FK_QUERY, new Uint8Array(0), { ...opts, label: 'FK_QUERY' });
    if (!r.ok) return null;
    const fk = decodeFk(r.payload);
    fk.latencyMs = r.latencyMs;
    fk.reply = r;
    return fk;
  }

  /** 0x03 LOG_SET_LEVEL */
  async logSetLevel(level, opts = {}) {
    return this.request(CMD.LOG_SET_LEVEL, payloadLogSetLevel(level),
                        { ...opts, label: 'LOG_SET_LEVEL' });
  }

  /** 0x10 MOVE_JOINTS ⚠ 参数是**舵机度 0~360**，且会被静默钳制。 */
  async moveJoints(servoDeg, v = 0, a = 0, opts = {}) {
    return this.request(CMD.MOVE_JOINTS, payloadMoveJoints(servoDeg, v, a),
                        { ...opts, label: 'MOVE_JOINTS' });
  }

  /**
   * 0x13 STREAM_JOINTS ⚠ 参数是**逻辑角**（相对 Home）。
   *
   * @param {number[]} logicalDeg
   * @param {boolean} wait  true=等 ACK；false=只发不等（高速流推荐 false）
   */
  async streamJoints(logicalDeg, wait = true, opts = {}) {
    const pl = payloadStreamJoints(logicalDeg);
    if (!wait) { this.sendAsync(CMD.STREAM_JOINTS, pl); return null; }
    return this.request(CMD.STREAM_JOINTS, pl,
                        { ...opts, label: 'STREAM_JOINTS', timeout: opts.timeout ?? 0.2 });
  }

  /** 0x11 STOP */
  async stop(opts = {}) {
    return this.request(CMD.STOP, new Uint8Array(0), { ...opts, label: 'STOP' });
  }

  /** 0x12 SET_TORQUE。joint: 0~5 或 0xFF（全部）。 */
  async setTorque(joint = 0xFF, enable = true, opts = {}) {
    return this.request(CMD.SET_TORQUE, payloadSetTorque(joint, enable),
                        { ...opts, label: 'SET_TORQUE' });
  }

  /** 0x20 MOVE_CART（mm / deg）。 */
  async moveCart(x, y, z, pitch = 0, v = 0, a = 0, opts = {}) {
    return this.request(CMD.MOVE_CART, payloadMoveCart(x, y, z, pitch, v, a),
                        { ...opts, label: 'MOVE_CART' });
  }

  /** 0x30 SET_GRAVITY */
  async setGravity(kDegPerMm, opts = {}) {
    return this.request(CMD.SET_GRAVITY, payloadSetGravity(kDegPerMm),
                        { ...opts, label: 'SET_GRAVITY' });
  }

  /** 0x31 SET_CORRECTION（三参数一次写全） */
  async setCorrection(enable = true, thresholdDeg = 2.0, maxRounds = 10, opts = {}) {
    return this.request(CMD.SET_CORRECTION,
                        payloadSetCorrection(enable, thresholdDeg, maxRounds),
                        { ...opts, label: 'SET_CORRECTION' });
  }

  /** 0x40 PARAM_READ → 数值 或 null。 */
  async paramRead(paramId, opts = {}) {
    const r = await this.request(CMD.PARAM_READ, payloadParamRead(paramId),
                                 { ...opts, label: `PARAM_READ(${paramId})` });
    if (!r.ok) return null;
    return decodeParamRead(r.payload).value;
  }

  /** 0x41 PARAM_WRITE ⚠ 超范围**仍回 err=0**，但值被钳过。 */
  async paramWrite(paramId, value, opts = {}) {
    return this.request(CMD.PARAM_WRITE, payloadParamWrite(paramId, value),
                        { ...opts, label: `PARAM_WRITE(${paramId})` });
  }

  /**
   * 写参数并**回读确认**（因为 PARAM_WRITE 会静默钳制）。
   * @returns {Promise<{ok:boolean, want:number, got:number|null, clamped:boolean, reply:Reply}>}
   */
  async paramWriteVerified(paramId, value, opts = {}) {
    const w = await this.paramWrite(paramId, value, opts);
    if (!w.ok) return { ok: false, want: value, got: null, clamped: false, reply: w };
    const got = await this.paramRead(paramId, opts);
    const clamped = got === null || Math.abs(got - value) > 1e-6;
    return { ok: !clamped, want: value, got, clamped, reply: w };
  }

  /** 0x42 PARAM_COUNT */
  async paramCount(opts = {}) {
    const r = await this.request(CMD.PARAM_COUNT, new Uint8Array(0),
                                 { ...opts, label: 'PARAM_COUNT' });
    if (!r.ok) return null;
    return decodeParamCount(r.payload);
  }

  /** 读六轴**逻辑角**（STREAM 用的量纲）。读失败返回 null，不兜底。 */
  async logicalAngles(calib, opts = {}) {
    const st = await this.getStatus(opts);
    if (!st) return null;
    if (!st.read_ok || st.joints.some((v) => v === null)) return null;
    return calib.toLogical(st.joints);
  }

  /** 从上一条 GET_STATUS 判断读是否正常。 */
  async checkReadOk(verbose = false) {
    const st = await this.getStatus();
    if (!st) { if (verbose) console.log('GET_STATUS 无响应'); return false; }
    if (st.read_ok) return true;
    if (verbose) console.log('读失败轴(J编号)：' + JSON.stringify(st.failedJoints));
    return false;
  }

  /* ======================================================================
   * 2026-09-23 扩展：全指令覆盖（27 个方法）
   *
   * 命名约定：会改硬件基准 / 会松力矩 / 会改 Flash 的，方法名带 `Danger`
   * 后缀 —— 让调用点在 review 时一眼可见。
   * ⚠ 读取类方法约定：**失败一律抛 ArmCommandError**（不返回 null）。调用方
   *   不必再判 null，也就不会把 null 当成合法结果往下算。
   * ==================================================================== */

  /** 0x04 GET_JOINT_DETAIL → 关节遥测数组。
   *
   * ⚠ 总线是**逐字段瞬时丢读**的（实测 Angle≈5% / Temp≈20%，各字段独立），
   *   所以别看单次结果下结论：
   *     · d.ok          本轴整组是否读成功
   *     · d.temp_lost   温度这一拍丢了（不是 0 度，也不是故障）
   *     · d.hw_err_lost 硬件错误寄存器读失败（≠ 硬件真的报错）
   *   要稳值就多读几次取多数。 */
  async getJointDetail(joint = 0xFF, opts = {}) {
    const r = await this.request(CMD.GET_JOINT_DETAIL, payloadGetJointDetail(joint),
                                 { ...opts, label: 'GET_JOINT_DETAIL' });
    if (!r.ok) throw new ArmCommandError(r);
    return decodeJointDetail(r.payload);
  }

  /** 0x05 GET_LIMITS → raw 限位与 Home 零点（来自 Flash 校准记录）。
   *
   * ⚠ 这是**校准记录**，不是当前运行时限位（运行时限位是 JOINT_MIN/JOINT_MAX，
   *   本协议未开放读取）。
   * ⚠ zero_raw[k] === 0 表示该轴没做过 Home 零点校准 —— 固件回落 180°/raw 2048，
   *   量纲基准是脆的，别在上面做精度动作。 */
  async getLimits(opts = {}) {
    const r = await this.request(CMD.GET_LIMITS, new Uint8Array(0),
                                 { ...opts, label: 'GET_LIMITS' });
    if (!r.ok) throw new ArmCommandError(r);
    return decodeLimits(r.payload);
  }

  /** 0x07 GET_INFO → fw_tag / build / link_mode / n_params。
   *
   * ⚠ link_mode 是**启动冻结值**（不是 Flash 偏好）—— 「改了偏好但还没 RESET」
   *   时两者会分叉，这正是判断"到底跑在哪条链路上"的依据。 */
  async getInfo(opts = {}) {
    const r = await this.request(CMD.GET_INFO, new Uint8Array(0), { ...opts, label: 'GET_INFO' });
    if (!r.ok) throw new ArmCommandError(r);
    return decodeInfo(r.payload);
  }

  /** 0x0A GET_FLAGS → 运行状态位图。**不碰舵机总线** ⇒ 适合 10~50Hz 轮询做 UI 状态灯。 */
  async getFlags(opts = {}) {
    const r = await this.request(CMD.GET_FLAGS, new Uint8Array(0), { ...opts, label: 'GET_FLAGS' });
    if (!r.ok) throw new ArmCommandError(r);
    return decodeFlags(r.payload);
  }

  /** 0x14 MOVE_JOINT_ONE → 单轴移动，其余五轴保持。
   *
   * ⚠ 角度是**舵机度**（0~360），不是 STREAM_JOINTS 的逻辑角。
   * ⚠ 固件先读全场快照、只改这一轴 —— 任一轴丢读则回 ERR_READ(7)。这是有意的：
   *   拿不准位置时不动，比动错强。 */
  async moveJointOne(joint, servoDeg, v = 0, a = 0, opts = {}) {
    return this.request(CMD.MOVE_JOINT_ONE, payloadMoveJointOne(joint, servoDeg, v, a),
                        { ...opts, label: `MOVE_JOINT_ONE J${joint + 1}` });
  }

  /** 0x17 SET_BUS_MODE：0=正常 A:AABBB / 1=全A / 2=全B。调试用，别放进生产路径。 */
  async setBusMode(mode, opts = {}) {
    return this.request(CMD.SET_BUS_MODE, payloadSetBusMode(mode),
                        { ...opts, label: `SET_BUS_MODE ${mode}` });
  }

  /** 0x18 SERVO_INIT ⚠ 会重写该轴 PID/限位/中位并 RESET（同文本命令 `i`）。 */
  async servoInit(joint = 0xFF, opts = {}) {
    return this.request(CMD.SERVO_INIT, payloadServoInit(joint), { ...opts, label: 'SERVO_INIT' });
  }

  /** 0x22 IK_QUERY → 六关节舵机度。**只解不算，不启动运动**。
   *
   * 失败抛 ArmCommandError，错误码区分原因：UNREACH(3) 不可达 /
   * LIMIT(4) 超关节限位 / FK_VERIFY(6) 回代超差。
   * 用于「先问能不能到，再决定动不动」—— 比 moveCart 撞失败更省事。 */
  async ikQuery(x, y, z, pitch = 0, opts = {}) {
    const r = await this.request(CMD.IK_QUERY, payloadIkQuery(x, y, z, pitch),
                                 { ...opts, label: 'IK_QUERY' });
    if (!r.ok) throw new ArmCommandError(r);
    return decodeIkQuery(r.payload);
  }

  /** 0x32 GET_GRAVITY → 重力前馈增益 K（deg/mm）。 */
  async getGravity(opts = {}) {
    const r = await this.request(CMD.GET_GRAVITY, new Uint8Array(0),
                                 { ...opts, label: 'GET_GRAVITY' });
    if (!r.ok) throw new ArmCommandError(r);
    return decodeGravity(r.payload);
  }

  /** 0x33 GET_CORRECTION → 自适应修正配置（SET_CORRECTION 的反向）。 */
  async getCorrection(opts = {}) {
    const r = await this.request(CMD.GET_CORRECTION, new Uint8Array(0),
                                 { ...opts, label: 'GET_CORRECTION' });
    if (!r.ok) throw new ArmCommandError(r);
    return decodeCorrection(r.payload);
  }

  /** 0x36 GET_PID → 单轴 PID（读自舵机 EEPROM，权威值）。
   *
   * from_eeprom=false 表示**读失败**，p/i/d 不可信（是那个从机没答，
   * 不是"PID 就是 0"）。 */
  async getPid(joint, opts = {}) {
    const r = await this.request(CMD.GET_PID, payloadGetPid(joint),
                                 { ...opts, label: `GET_PID J${joint + 1}` });
    if (!r.ok) throw new ArmCommandError(r);
    return decodePid(r.payload);
  }

  /** 0x43 PARAM_SAVE（等价文本 `F`）：当前参数（PID/修正/速度/补偿…）落盘 Flash。 */
  async paramSave(opts = {}) {
    return this.request(CMD.PARAM_SAVE, payloadMagic(), { ...opts, label: 'PARAM_SAVE' });
  }

  /** 0x44 PARAM_LIST → 全部参数**元数据**（内部按 6 条/页自动翻页）。
   *
   * ⚠ 只有元数据（id/type/范围），**值要逐个 paramRead 取** —— 带上 i32 值会让
   *   响应帧超过 64B 上限，所以固件刻意拆开了。
   * 返回顺序 = 固件参数表顺序，数组索引即 PARAM_LIST 的 offset。 */
  async paramList(opts = {}) {
    const out = [];
    let offset = 0;
    for (;;) {
      const r = await this.request(CMD.PARAM_LIST, payloadParamList(offset),
                                   { ...opts, label: `PARAM_LIST @${offset}` });
      if (!r.ok) break;
      const { total, items } = decodeParamList(r.payload);
      if (!items.length) break;
      out.push(...items);
      offset += items.length;
      if (offset >= total) break;
    }
    return out;
  }

  /** 0x55 TEACH_TOTAL → 存储总览：文件数 / 容量 / 总样本 / 剩余字节。 */
  async teachTotal(opts = {}) {
    const r = await this.request(CMD.TEACH_TOTAL, new Uint8Array(0),
                                 { ...opts, label: 'TEACH_TOTAL' });
    if (!r.ok) throw new ArmCommandError(r);
    return decodeTeachTotal(r.payload);
  }

  /** 0x50 TEACH_LIST → 第 idx 个文件的信息（名字 / 样本数 / 是否已删）。 */
  async teachList(idx, opts = {}) {
    const r = await this.request(CMD.TEACH_LIST, payloadTeachList(idx),
                                 { ...opts, label: `TEACH_LIST #${idx}` });
    if (!r.ok) throw new ArmCommandError(r);
    return decodeTeachList(r.payload);
  }

  /** 列举全部文件（按 teachTotal 的文件数逐条取）。 */
  async teachListAll(opts = {}) {
    const tot = await this.teachTotal(opts);
    const out = [];
    for (let i = 0; i < tot.n_files; i++) {
      try { out.push(await this.teachList(i, opts)); }
      catch { /* 空槽位跳过 */ }
    }
    return out;
  }

  /** 0x51 TEACH_READ → 一页样本（n 上限 5 = 60B）。n=0 表示 off 越界 = 正常读完。 */
  async teachRead(idx, off, n = TEACH_READ_MAX, opts = {}) {
    const r = await this.request(CMD.TEACH_READ, payloadTeachRead(idx, off, n),
                                 { ...opts, label: `TEACH_READ #${idx} @${off}` });
    if (!r.ok) throw new ArmCommandError(r);
    return decodeTeachRead(r.payload);
  }

  /** ⭐ 把一整条轨迹取下来（内部按 5 样本/页翻页）。
   *
   * 1000 点轨迹 = 200 次请求；网口 p50 1.61 ms ⇒ 约 0.3 s 取完。
   * maxSamples 是安全上限 —— 防 samples 字段异常导致死循环。 */
  async teachTrace(idx, maxSamples = 20000, opts = {}) {
    const out = [];
    let off = 0;
    while (out.length < maxSamples) {
      const page = await this.teachRead(idx, off, TEACH_READ_MAX, opts);
      if (!page.length) break;                       // n === 0 ⇒ 正常读完
      out.push(...page);
      off += page.length;
      if (page.length < TEACH_READ_MAX) break;       // 尾页不满 = 到底了
    }
    return out;
  }

  /** 0x53 TEACH_DELETE → 删除文件（写 tombstone，空间用 teachCompact 回收）。 */
  async teachDelete(idx, opts = {}) {
    return this.request(CMD.TEACH_DELETE, payloadTeachDelete(idx),
                        { ...opts, label: `TEACH_DELETE #${idx}` });
  }

  /** 0x54 TEACH_COMPACT ⚠ 回收 tombstone 空间。会擦写 Flash，耗时且不该频繁调。 */
  async teachCompact(opts = {}) {
    return this.request(CMD.TEACH_COMPACT, new Uint8Array(0),
                        { ...opts, label: 'TEACH_COMPACT' });
  }

  /** ⚠ 0x70 CENTER_CALIB：等价文本 `M a`，六轴推到 raw 2048 中位。
   *  **会改变量纲基准**，不可逆。 */
  async centerCalibDanger(opts = {}) {
    return this.request(CMD.CENTER_CALIB, payloadMagic(), { ...opts, label: 'CENTER_CALIB' });
  }

  /** ⚠ 0x71 CALIB_ENTER：相位校准向导（文本 `c`）。**固件一律回 NACK(ERR_PARAM)**。
   *
   * 这条**设计上不可用**，不是实现漏了：cmdCalibrate() 是 `for(;;)` 等人在
   * **本机串口**敲 '0'/'s'/'q'。从网口进 = 主循环永不返回，而本工程**没有看门狗**
   * （红线 5）⇒ 板子彻底失联但 USB 还在枚举，最难查的一种死法。它本来就要求
   * 人手摆位，是控制台专属命令。
   * 保留这个方法是为了让"不可用"在 API 上**可见**。要校准请接 COM11。 */
  async calibEnterDanger(opts = {}) {
    return this.request(CMD.CALIB_ENTER, payloadMagic(), { ...opts, label: 'CALIB_ENTER' });
  }

  /** ⚠ 0x72 ZERO_RESET：等价文本 `zr` / `zr save`，Home 零点重置回出厂中位。
   *
   * ⚠ 一次复位**全部六轴** —— 文本 `zr` 没有单轴变体，所以这里也没有关节号。
   * save=true 写 Flash 固化（重启保持）；false 仅本次运行有效。 */
  async zeroResetDanger(save = false, opts = {}) {
    return this.request(CMD.ZERO_RESET, payloadZeroReset(save),
                        { ...opts, label: `ZERO_RESET${save ? ' save' : ''}` });
  }

  /** ⚠ 0x73 SYS_RESET 软复位。**先回 ACK、150 ms 后才真正复位**（固件刻意如此：
   *  顺序反了上位机只会看到超时，分不清"复位成功"和"包丢了"）。
   *
   * 复位后按 **Flash 里的链路偏好**重建 —— 若偏好指向另一条链路（`link can`
   * 且已 `link save`），这台臂就不会再在网口上回话。这正是远程切链路的完整序列：
   * `textExec('link can')` → `textExec('link save')` → 本方法。 */
  async sysResetDanger(opts = {}) {
    return this.request(CMD.SYS_RESET, payloadMagic(), { ...opts, label: 'SYS_RESET' });
  }

  /** 0x0B TEXT_EXEC：在臂上执行**任意一条控制台命令**，返回它的输出（自动翻页）。
   *
   * 这是"所有指令都支持网口下发"的入口：`arm.textExec('s')` 与在 COM11 上敲 `s`
   * 是**同一条** ProcessCommand，连回显文本都一样。
   *
   * 能到什么程度：
   *   · 124 条控制台命令里 **113 条**可用；
   *   · 11 条抢 ETH 引脚的命令（cx/cp/cy/cz/cw/ps/can/mg/mp/mo/ml）会被固件拦下
   *     并把**原因当输出回给你**（不是静默失败）；
   *   · `c`（校准向导）被安全闸门拒绝 —— 交互式，会卡死主循环；
   *   · 急停时只放行 `s` / `h`。
   *
   * ⚠ 命令最长 58 字符（TEXT_EXEC_MAX）；超长在客户端就抛 ProtocolError，
   *   不让固件把命令钳断静默执行半条（半条往往是**另一条命令**）。
   * ⚠ 输出上限 2048 B。超了会在末尾追加 [截断] 提示 —— `h`（帮助表 5406 B）
   *   就是这种情况。
   * ⚠ 会改基准/松力矩的命令（`M a`/`zr`/`i`/`Y`）请走**带 Danger 后缀的类型化
   *   方法**，不要在这里随手拼字符串。 */
  async textExec(command, opts = {}) {
    const r = await this.request(CMD.TEXT_EXEC, payloadTextExec(command),
                                 { ...opts, label: `TEXT_EXEC ${JSON.stringify(command)}` });
    if (!r.ok) throw new ArmCommandError(r);

    const first = decodeTextOut(r.payload);
    if (first.gated) {
      // 闸门拒绝：正文是**原因说明**，不是命令输出。原样返回更有用。
      return first.text;
    }

    // ⚠ 按**字节**拼接，最后统一 decode 一次 —— 分页边界可能正好切在一个多字节
    //   UTF-8 字符中间（固件控制台的中文提示是 UTF-8 字节），逐页 decode 会在
    //   缝上拼出坏字符。
    // ⚠ 不用 Buffer.concat：那是 Node 专有 API，浏览器（走 ws:// 边车）里没有。
    const parts = [first.raw];
    let off = first.off + first.raw.length;
    let truncated = first.truncated;
    const guard = Math.floor(TEXT_BUF_SZ / TEXT_CHUNK) + 2;   // 防固件异常时死循环
    while (off < first.total && parts.length < guard) {
      const r2 = await this.request(CMD.TEXT_FETCH, payloadTextFetch(off, TEXT_CHUNK),
                                    { ...opts, label: `TEXT_FETCH @${off}` });
      if (!r2.ok) break;
      const page = decodeTextOut(r2.payload);
      if (!page.raw.length) break;
      parts.push(page.raw);
      off += page.raw.length;
      truncated = truncated || page.truncated;
    }

    const all = new Uint8Array(parts.reduce((n, p) => n + p.length, 0));
    let w = 0;
    for (const p of parts) { all.set(p, w); w += p.length; }
    let text = utf8Decoder.decode(all);
    if (truncated) {
      text += `\n[TEXT_EXEC] 输出超过 ${TEXT_BUF_SZ} B 已截断。这条命令的回显太长, ` +
              '请改用结构化命令(如 GET_JOINT_DETAIL / GET_INFO / TEACH_*)。';
    }
    return text;
  }

  /** 同 textExec，但只取**第一页**并保留分页元信息（total/truncated/gated）。
   *
   * 给需要自己控节奏的调用方：比如只要 `s` 的前几行，或想区分"被闸门拒绝"
   * 与"命令正常执行但没输出"。 */
  async textOut(command, opts = {}) {
    const r = await this.request(CMD.TEXT_EXEC, payloadTextExec(command),
                                 { ...opts, label: `TEXT_EXEC ${JSON.stringify(command)}` });
    if (!r.ok) throw new ArmCommandError(r);
    return decodeTextOut(r.payload);
  }

  describe() {
    return `ArmClient(${this.link}) timeout=${this.timeout}s retries=${this.retries}` +
           (this.dryRun ? ' DRY-RUN' : '');
  }
}

export function errName(code) {
  for (const [k, v] of Object.entries(ErrCode)) if (v === code) return k;
  return 'ERR' + code;
}

/* ==========================================================================
 * 工厂：open_arm("net://ip:5000")
 * ========================================================================== */

/**
 * @param {string} uri  net://ip:port | udp://ip:port | ws://host:port | webserial:// | sim://
 * @param {{timeout?:number, retries?:number, dryRun?:boolean, open?:boolean}} [opts]
 * @returns {Promise<ArmClient>}
 */
export async function openArm(uri, opts = {}) {
  const t = await makeTransport(uri, opts);
  const c = new ArmClient(t, opts);
  if (opts.open !== false) await c.open();
  return c;
}

export { QUERY_RSP };
